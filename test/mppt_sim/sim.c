#define _USE_MATH_DEFINES
#define _GNU_SOURCE
#include <stdio.h>
#include <math.h>
#include <stdint.h>
#include "targets.h"
#include "mppt.h"
extern uint16_t ADC_raw_volts, ADC_raw_current, VOLTAGE_DIVIDER, input;
extern volatile uint32_t zero_crosses;
extern volatile char armed; extern uint8_t running; extern char prop_brake_active;
extern int h_all_off;   /* set by the harness when mppt.c calls allOff() */
extern int e_com_time;
extern char play_tone_flag;

/* ~12 V Voc array, sized to what EGAN_JUWI_L431 can actually measure. */
/* SunPower back-contact cells, the usual RC choice: 0.71 V Voc and 0.62 V
 * Vmpp per cell, which implies a 28.9 mV per-cell diode voltage. The plant
 * must model the SAME cell the firmware assumes, or the beta target is
 * derived for one panel and verified against another. */
#ifndef CELLS
#define CELLS   19
#endif
#ifndef VOC_STC
#define VOC_STC (CELLS * 0.710)
#endif
#ifndef ISC_STC
#define ISC_STC  0.120
#endif
#ifndef VTH
#define VTH     (CELLS * 0.0289)
#endif
#ifndef CBUS
#define CBUS   470e-6
#endif
#ifndef R_M
#define R_M 0.30
#endif
#ifndef L_M
#define L_M 200e-6
#endif
#ifndef KE
#define KE  0.00955
#endif
#ifndef J_M
#define J_M 2.0e-5
#endif
#ifndef POLE_PAIRS
#define POLE_PAIRS 7
#endif
#ifndef KQ
#define KQ  2.62e-7
#endif
static double IS_SAT;
static double ipv(double v,double G){ return ISC_STC*G - IS_SAT*(exp(v/VTH)-1.0); }
static double PMTAB[1001]; static int ok=0;
static double pmax_raw(double G){ double b=0; for(int k=0;k<600;k++){ double v=VOC_STC*1.05*k/599.0;
    double i=ipv(v,G); if(i<0)i=0; double p=v*i; if(p>b)b=p;} return b; }
static double pmax_of_G(double G){ if(!ok){for(int k=0;k<=1000;k++)PMTAB[k]=pmax_raw(k/1000.0);ok=1;}
    int k=(int)(G*1000+0.5); if(k<0)k=0; if(k>1000)k=1000; return PMTAB[k]; }
/* exact inverses of AM32's own scaling, with this board's constants */
static uint16_t adc_v(double v){ double r=v*100.0*4095.0/(3300.0*(TARGET_VOLTAGE_DIVIDER/100.0));
    if(r<0)r=0; if(r>4095)r=4095; return (uint16_t)(r+0.5); }
static uint16_t adc_i(double a){ double r=(a*100.0*HW_MVA + CURRENT_OFFSET*100.0)*41.0/3300.0;
    if(r<0)r=0; if(r>4095)r=4095; return (uint16_t)(r+0.5); }
static double irradiance(double t){
    double bank=0, roll=100.0*M_PI/180.0;
    if(t<0.60) bank=0; else if(t<1.30) bank=roll*(t-0.60);
    else if(t<1.80) bank=roll*0.70; else if(t<2.50) bank=roll*0.70-roll*(t-1.80); else bank=0;
    if(bank<0)bank=0; double G=cos(bank); if(G<0.05)G=0.05;
    if(t>2.70&&t<2.90){ double e=(t-2.70)/0.025; if(e>1)e=1; G*=(1.0-0.88*e); }
    if(t>=2.90&&t<3.00){ double e=(t-2.90)/0.025; if(e>1)e=1; G*=(0.12+0.88*e); }
    if(G<0.04)G=0.04; return G;
}
#ifndef TSIM
#define TSIM 3.4
#endif
/* SHADE_AT>0 drops irradiance to 4% for SHADE_MS, to force a bus collapse and
 * exercise the RECOVER / absolute-minimum unload path. */
/* Overall irradiance level, 1.0 = full sun. THE axis that matters: a large
 * array in low light behaves like a small array in full sun, so there is no
 * such thing as a "small panel case" separate from a "low light case". */
#ifndef G_SCALE
#define G_SCALE 1.0
#endif
#ifndef SHADE_AT
#define SHADE_AT 0.0
#endif
#ifndef SHADE_MS
#define SHADE_MS 400.0
#endif
#ifndef SHADE_G
#define SHADE_G 0.04
#endif
int main(void){
    IS_SAT = ISC_STC/(exp(VOC_STC/VTH)-1.0);
    double dt=2e-6, T=TSIM; long N=(long)(T/dt);
    double tick_dt = 1.0/(double)MPPT_TICK_HZ; long td=(long)(tick_dt/dt);
    double v=VOC_STC, im=0, w=0, zcacc=0;
    uint16_t am32=0, d_prev=0, pv=adc_v(VOC_STC), pi=adc_i(0);
    VOLTAGE_DIVIDER=TARGET_VOLTAGE_DIVIDER; armed=0; running=0; input=0; prop_brake_active=0;
    ADC_raw_volts=pv; ADC_raw_current=pi; mppt_init();
    double harv=0, avail=0, vmin=1e9, dutymin=1e9, regen=0, wmin=1e9, wref=0; long bo=0;
    for(long n=0;n<N;n++){
        double t=n*dt, G=irradiance(t)*G_SCALE;
        if(SHADE_AT>0.0 && t>SHADE_AT && t<SHADE_AT+SHADE_MS/1000.0) G=SHADE_G;
        if(t>0.05){armed=1;running=1;input=2047;}
        zcacc += 6.0*7.0*(w/(2*M_PI))*dt; if(!armed||!running) zcacc=0;
        zero_crosses=(uint32_t)zcacc;
        {   /* electrical revolution period in us, AM32's e_com_time units.
             * Below ~24 zero-crossings main.c reports the 65408 sentinel. */
            double erps = w/(2*M_PI)*POLE_PAIRS;
            e_com_time = (zero_crosses<24 || erps<1.0) ? 65408 : (int)(1e6/erps + 0.5);
        }
        if(n%td==0){ ADC_raw_volts=pv; ADC_raw_current=pi; mppt_1khz_update();
            /* Bus current is the APPLIED duty times motor current. Sampling
             * am32 here instead - AM32's unclamped intent, which ramps to
             * 2000 and stays - fed the firmware ~6x the real bus current and
             * made every absolute-current threshold meaningless. d_prev is
             * last tick's applied duty, which is what the shunt actually saw. */
            pv=adc_v(v); pi=adc_i((d_prev/2000.0)*im);
            if(armed&&running){ if(am32<2000) am32+=20; } else am32=0; }
        h_all_off=0;
        uint16_t d=am32; mppt_apply_duty(&d); d_prev=d; double D=d/2000.0;
        double ib, dim;
        if(h_all_off){
            /* COASTING - all six FETs off. The winding is open, so no current
             * and no braking torque; only aerodynamic drag slows the prop.
             * (Body-diode rectification back into the bus is neglected, which
             * makes this the pessimistic case for coasting.) */
            im = 0.0; dim = 0.0; ib = 0.0;
        } else {
            /* DRIVEN. Note that at D=0 with complementary PWM this is a SHORT,
             * not a coast: di/dt = (-R*i - KE*w)/L drives i negative and the
             * -KE*w term brakes the rotor. That is the hardware behaviour the
             * coast path exists to avoid, and it must stay modelled here. */
            dim = (D*v - R_M*im - KE*w)/L_M;
            ib  = D*im;
        }
        v += ((ipv(v,G)-ib)/CBUS)*dt;
        im += dim*dt;
        w  += ((KE*im - KQ*w*w)/J_M)*dt;
        if(v<0.1)v=0.1; if(w<0)w=0;
        if(t>0.35){ double p=v*ib; harv+=p*dt; avail+=pmax_of_G(G)*dt;
            if(v<vmin)vmin=v; if(v<MPPT_V_ABSOLUTE_MIN/100.0)bo++;
            if(t>1.0&&d<dutymin)dutymin=d; if(ib<regen)regen=ib;
            if(SHADE_AT>0.0){ if(t<SHADE_AT) wref=w; else if(w<wmin) wmin=w; } }
    }
    fprintf(stderr,"T=%4.1fs eff=%6.2f%%  vmin=%5.2fV  brownout=%ld  min_duty(t>1s)=%4.0f  worst_regen=%6.3fA  collapses=%u",
        T,100.0*harv/avail,vmin,bo,dutymin,regen,mppt.collapse_events);
    if(SHADE_AT>0.0) fprintf(stderr,"  rpm_kept=%5.1f%% coast_events=%u", wmin<1e8?100.0*wmin/wref:100.0, mppt.coast_events);
    fprintf(stderr,"\n");
    return 0;
}
