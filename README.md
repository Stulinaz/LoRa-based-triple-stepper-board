<!-- For .md file development refers to https://docs.github.com/en -->
# LoRa Wireless Triple Stepper Controller

Designed to remote control up to three axis mechatronic system.
Build with [MIPOT](https://www.mipot.com) [32001506DEU](https://mipot.com/en/products/mip-series/dual-core/32001506deu/) LoRa Open Core module based on 
[STM32WL55JC](https://www.st.com/en/microcontrollers-microprocessors/stm32wl55cc.html) dual core microcontroller.

![img0](https://github.com/Stulinaz/LoRa-based-triple-stepper-board/blob/master/img/tmc2130_mip_HWREV1_ISO.png)

## Top Side of the board:

![img1](https://github.com/Stulinaz/LoRa-based-triple-stepper-board/blob/master/img/tmc2130_mip_HWREV1_TOP.png)

## Bottom Side of The board:

![img2](https://github.com/Stulinaz/LoRa-based-triple-stepper-board/blob/master/img/tmc2130_mip_HWREV1_BOT.png)

## Board with 4 layers for EMI Compliance:

![img3](https://github.com/Stulinaz/LoRa-based-triple-stepper-board/blob/master/img/SCH_REV1_LAYERS.png)

## Build with Kicad:

![img4](https://github.com/Stulinaz/LoRa-based-triple-stepper-board/blob/master/img/SCH_REV1_FULL.png)

## Main firwmare files for a quick view:

![img5](https://github.com/Stulinaz/LoRa-based-triple-stepper-board/blob/master/img/firmware_files.png)

## Pcb and stencil order on pcbway:

![img6](https://github.com/Stulinaz/LoRa-based-triple-stepper-board/blob/master/img/pcbway_order.png)

## Mode of Operation

![img7](https://github.com/Stulinaz/LoRa-based-triple-stepper-board/blob/master/img/operation_mode.png)

## Electrical characteristics

| Parameter                | Description | Min        | Typ     | Max     |  Unit  | Test Consitions |    
| ---                      |    ---      | ---        | ---     | ---     | ---    | ---             |   
|**Input Voltage**         | -           | 6.25 Vdc   | 12 Vdc  | 45 Vdc  | -      | -               |   
|**Operating temperature** | -           | 0 \*C      | 25 \*C  | 55 \*C  | -      | -               |
|**Power consumption**     | -           | -          | -       | -       | -      | -               |

## Folder content and description

- **technical_docs:** Datasheet, Application notes and manual of the on-board electronic components.
- **firmware:** Source code of the microcontroller written in C.
	- **Core:** The application files, system and peripheral configuration.
	- **freertos:** The Freertos kernel. *Keep read only*.
	- **freertos_cli:** The Freertos command line interface source code. *Keep read only*.
	- **wl-lib:** STM32 HAL and LL Drivers. *Keep read only*.
	- **trinamic_lib:** Driver of TMC2130 stepper controller.
	- **miplib** Driver of MIPOT LoRa Module 32001506DEU. *Keep read only*.
- **img:** Development and test images of the project.
- **kicad:** hardware design files.Source files of schematic and PCB.
	- **lib:** Symbols and footprints of the components.
- **production:** Production files. Gerbers, BOM and Stencil.
- **doc:**  User manual of the board and Schematic. PDF format.


## Bll of materials

Interactive [BOM](https://github.com/Stulinaz/LoRa-based-triple-stepper-board/blob/master/production/ibom.html).


## Firmware characteristics

Based on [Freertos](https://www.freertos.org) with simple and clean development.
![img8](https://github.com/Stulinaz/LoRa-based-triple-stepper-board/blob/master/img/fw_freertos.png).


## Software needed for development

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) **1.17.0**
- [KICAD](https://www.kicad.org/) **8.0.5**
- [GIT](https://git-scm.com/) **2.46.1.windows.1**
- [Tera Term](https://teratermproject.github.io/index-en.html) **5.3 x86**


## Tools needed for development

Programmer [STLINK](https://www.st.com/en/development-tools/st-link-v2.html).


## License

Shown in the LICENSE.md file


## Developers

Hardware developer: [@ts-manuel](https://github.com/ts-manuel)

Firmware developer: [@Stulinaz](https://github.com/Stulinaz)