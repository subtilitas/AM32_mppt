# AM32 — solar MPPT fork

> ## ⚠️ This fork builds MPPT firmware only. It is useless on a battery-powered ESC.
>
> Every board target here is built with a maximum-power-point tracker
> **force-enabled**. The tracker continuously regulates the ESC's duty cycle to
> hold the *input* bus at a solar panel's maximum-power voltage, and it will
> clamp or shed throttle whenever the bus sags.
>
> On a battery that behaviour is wrong in every direction. A battery is a stiff
> voltage source, so there is no maximum-power point to find; the tracker will
> read the pack's sag under load as a collapsing panel and throttle back, and
> the safety thresholds that keep the MCU alive on an unbuffered solar bus will
> cut power at voltages a normal pack reaches routinely.
>
> **Do not flash these binaries onto a battery-powered aircraft, rover or boat.**
> For normal use, take upstream AM32:
> <https://github.com/AlkaMotors/AM32-MultiRotor-ESC-firmware>

This is a fork of AM32 for **direct-drive solar aircraft**: PV array → ESC →
BLDC motor, with **no battery anywhere**. The MCU's own logic supply sits on the
same bus as the panel, so a bus collapse is a loss of the aircraft — most of the
design is about preventing that rather than about the last percent of power.

## The MPPT tracker

Full detail in [doc/MPPT.md](doc/MPPT.md). In brief:

A cascaded, all-integer control loop with no FPU use, no `float`, and no
dynamic allocation:

```
tenKhzRoutine()      10/20 kHz   duty ceiling + bus-collapse clamp
  └─ 1 kHz block     ~950 Hz     sample, filter, PI voltage loop
       └─ tracker    ~48 Hz      move the setpoint
```

The inner PI holds the panel at `vref`; the outer tracker decides where `vref`
should be. Two trackers exist, one compiled in per board:

| `MPPT_TRACKER` | How it finds the MPP | Needs current? | Steady state |
|---|---|---|---|
| `MPPT_TRACKER_BETA` | regulates `β = ln(I/V) − c·V` to its known MPP value | yes, absolute | 0.000 V p-p |
| `MPPT_TRACKER_RPM` | perturb-and-observe on commutation period | no | dithers ±0.21 V |

β is a *regulator*, not a hill-climber: it computes where the peak is from the
present operating point instead of searching for it, so nothing perturbs and
nothing dithers. Its target value is very nearly irradiance-invariant, which is
what makes that possible — measured across a 3:1 irradiance range it moved by
three parts in ten thousand.

Which tracker a board gets is decided by `test/mppt_targets.sh` from
`Inc/targets.h`: boards that declare their own `MILLIVOLT_PER_AMP` have a real
current-sense chain and get β (112 boards); the rest inherit AM32's fabricated
`20` fallback and get the rpm tracker (136 boards).

Measured against a PV + bus-capacitor + BLDC + propeller plant model, 60 s:

| irradiance | β | rpm |
|---|---|---|
| 100% | **99.91%** | 99.64% |
| 35% | **99.72%** | 98.30% |
| 20% | 87.35% | **97.56%** |
| 10% | 88.87% | **98.62%** |

Cost on EGAN_MPPT_L431: ~3.4 kB flash, 111 bytes RAM.

## Before you fly it

Only `EGAN_MPPT_L431` has been characterised. For any other board:

- `MPPT_BETA_MPP_Q8` and `MPPT_BETA_VT` are **panel**-specific. Find peak rpm by
  hand and read `mppt.beta` off telemetry; that number is the constant.
- `MPPT_V_COLLAPSE` and `MPPT_V_ABSOLUTE_MIN` are **placeholders**. With no
  battery on the bus they are the only thing keeping the MCU alive — set them
  from your board's 3.3 V regulator dropout plus margin.
- Keep the bus capacitor in roughly 100–1000 µF. 2200 µF is measured unstable.

Follow the bench-up order in [doc/MPPT.md](doc/MPPT.md) §8.

## Testing

```bash
./test/mppt_sim/run.sh test      # directed tests, UBSan + ASan
./test/mppt_sim/run.sh 60        # plant model, 60 s
G=0.35 ./test/mppt_sim/run.sh 60 # at 35% irradiance
./test/mppt_targets.sh --report  # which board gets which tracker
```

CI additionally cross-builds every board with MPPT enabled — see
`.github/workflows/mppt_build.yml`.

---

# Upstream AM32 documentation

## AM32-MultiRotor-ESC-firmware
Firmware for ARM based speed controllers
<p align="left">
  <a href="/LICENSE"><img src="https://img.shields.io/badge/license-GPL--3.0-brightgreen" alt="GitHub license" /></a>
</p>

The AM32 firmware is designed for STM32 ARM processors to control a brushless motor (BLDC).
The firmware is intended to be safe and fast with smooth fast startups and linear throttle. It is meant for use with multiple vehicle types and a flight controller. The firmware can also be built with support for crawlers. For crawler usage please read this wiki page [Crawler Hardware](https://github.com/AlkaMotors/AM32-MultiRotor-ESC-firmware/wiki/Crawler-Hardware-and-AM32)

### Features

AM32 has the following features:

- Firmware upgradable via betaflight passthrough, single wire serial or arduino
- Servo PWM, Dshot(300, 600) motor protocol support
- Bi-directional Dshot
- KISS standard ESC telemetry
- Variable PWM frequency
- Sinusoidal startup mode, which is designed to get larger motors up to speed
### Build instructions
Download and install Keil community edition. Open the Keil project for the mcu you want in the "Keil projects" folder. Install any mcu packs if prompted. 
Select the build target from the drop down box and build project 

### Firmware Release & Configuration Tool

The latest release of the firmware can be found [here](https://am32.ca/downloads).

The primary configurator is the [AM32 Configurator](https://am32.ca)
which supports web browser based configuration and firmware update.

You can also use a desktop configurator which you can download from here:

[WINDOWS](https://drive.google.com/file/d/16kaPek9umz7fQFunzBeW4pp2LgT6_E5o/view?usp=drive_link)
[LINUX](https://drive.google.com/file/d/1QtSKwp3RT6sncPADsPkmdasGqNIk68HH/view?usp=sharing)

Alternately you can use the [Online-ESC Configurator](https://esc-configurator.com/) to flash or change settings with any web browser that supports web serial.



### Hardware
AM32 currently has support for STSPIN32F0, STM32F051, STM32G071, GD32E230, AT32F415 and AT32F421.
The CKS32F051 is not recommended due to too many random issues.
Target compatibility List can be found [here](https://github.com/am32-firmware/AM32/blob/main/Inc/targets.h)


### Installation & Bootloader

To use AM32 firmware on a blank ESC, a bootloader must first be installed using an ST-LINK, GD-LINK , CMIS-DAP or AT-LINK.  THe bootloader will be dependant on the MCU used ont he esc . Choose the bootloader that matches the MCU type and signal input pin of the ESC.
The compatibility chart has the bootloader pinouts listed.
Current bootloaders can be found [here](https://github.com/am32-firmware/AM32-bootloader).

After the bootloader has been installed the main firmware from can be installed either with the configuration tools and a Betaflight flight controller or a direct connection with a usb serial adapter modified for one wire.

To update an existing AM32 bootloader an update tool can be found [here](https://github.com/am32-firmware/AM32-unlocker).

### Support and Developers Channel
There are two ways you can get support or participate in improving am32.
We have a discord server here:

https://discord.gg/h7ddYMmEVV

Etiquette: Please wait around long enough for a reply - sometimes people are out flying, asleep or at work and can't answer immediately. 

If you wish to support the project please join the Patreon.

https://www.patreon.com/user?u=44228479


### Sponsors
The AM32 project would not have made this far without help from the following sponsors:

Holmes Hobbies - https://holmeshobbies.com/ - The project would not be where it is today without the support of HH. Check out the Crawlmaster V2 for the best am32 experience!

Repeat Robotics - https://repeat-robotics.com/ - Bringing Am32 esc's to the fighting robot community!

Quaternium - https://www.quaternium.com/ - Firmware development support and hardware donations

Airbot - Many hardware donations

NeutronRC - For hardware, am32 promotion and schematics 

Aikon - Hardware donations and schematics\
Skystars  - For hardware and taking a chance on the first commercial am32 esc's\
Diatone - Hardware donations\
T-motor - Motor and Hardware donations\
HLGRC  - Hardaware donations


### Contributors
A big thanks to all those who contributed time, advice and code to the AM32 project.\
Un!t\
Hugo Chiang (Dusking)\
Micheal Keller (Mikeller)\
ColinNiu\
Jacob Walser

And for feedback from pilots and drivers:\
Jye Smith\
Markus Gritsch\
Voodoobrew

(and many more)

