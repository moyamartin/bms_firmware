# Battery Management System (BMS) - Firmware

This is the firmware dev section of the [BMS project](https://github.com/moyamartin/bms_unr), as the [hardware section](https://github.com/moyamartin/bms_hardware) states, the brain of the BMS is based on a [STM32F407VGTx](https://www.st.com/en/microcontrollers-microprocessors/stm32f407vg.html) microcontroller unit.

It's mainly based on STM32Cube HAL (Hardware Abstraction Layer) and the CMSIS
(Cortex Microcontroller Software Interface Standard).

To clone this repo run:

```
git clone --recursive -j8 https://github.com/moyamartin/bms_firmware.git
```

## Features

The firmware has to be able to accomplish the following goals:

* Estimate the state of charge of the battery using a Kalman Filter and an
  electrical model of a 18650PF (Work in progress [WIP]).
* Device drivers for the peripherals of the BMS in order to get the following information:
    * Discharge/charge current
    * Cell voltage
* Communicate the current status of the battery pack via CAN bus or
  RS232 protocol (WIP)
* Handle failure interruptions and trigger the load switch for protection (WIP)
* Handle the battery charger indicators (WIP)
* Unbalanced cell detector (WIP)
* Finite-state machine for handling charge/discharge phases (WIP)

## Requirements

### Software

Before building and loading the firmware into the board, first you'll need to
install the [GNU ARM embedded toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads) and OpenOCD (On-Chip Debugger) or you can install the [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) and import it as this proyect is compatible with that development environment.

### Hardware

You will need a [STLink/V2 in-circuit debugger/programmer](https://www.st.com/en/development-tools/st-link-v2.html) in order to loadand debug the firmware on the board.

NOTE: if you have a [STM32F4discovery board](https://www.st.com/en/evaluation-tools/stm32f4discovery.html) or another known board that has a STLink programmer with accessible pins  you won't need to acquire an stand-alone programmer in order to load the firmware to the board, you only need to connect the SWDIO pins.

## Build and Run

### Linux

Open a terminal and run the following steps

1 - Go to the cloned repo's folder

```
cd root_path/bms_unr 
```

2 - Run `make` to build the firmware . This will create a `build` folder containing the binaries of the
firmware (`.bin`, `.hex` and `.elf`). 

NOTE: to build the firmware without debug symbols modify the `Makefile` and set
the `DEBUG` variable to 0. (WIP, run `make debug` or `make prod` to avoid
changing this variable manually)

2- To load the firmware to the microcontroller run the
following command

```
openocd -d0 -f board/stm32f4discovery.cfg -c "init;targets;halt;flash write_image erase bms_firmware.hex;shutdown"
```

3 - To debug the firmware you first have to run an `openocd` server on a
terminal running (Make sure that you are building the project with debug symbols, otherwise you won't be able to see C code in GDB)

```
openocd -f board/stm32f4discovery.cfg
```

and on a separate terminal run

```
arm-none-eabi-gdb -f root_path/bms_firmare/build/bms_firmare.elf -x init.gdb
```

### Windows

## Testing

We are running some local test for specific algorithms using [Cpputest](http://cpputest.github.io/). To run them first build and configure the framework running:

```
cd path_to_repo/Test/cpputest/cpputest_build && autoreconf .. -i && ../configure && make && mkdir ../lib && cp ./lib/* ../lib/
```

And them, to execute the tests run:

```
cd Test && make -B gcov
```

This will make a test coverage html file saying which tests passed or failed

