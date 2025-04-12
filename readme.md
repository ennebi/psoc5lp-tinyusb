# TinyUSB on PSOC&trade; 5LP

This project contains a tinyusb library port for Infineon&reg; (formerly Cypress&trade;) PSOC&trade; 5LP microcontrollers.
We decided to release this internal project to the public in the hope the extended flexibility of TinyUSB over the block implementation could be useful to other developers working on the same platform.

Due to the reconfigurable nature of the microcontroller (especially the interrupt routing mechanism), we were unable to provide a port that could compile within the library, together with proper example code that would fit in the building scheme.
What's more, the PSOC&trade; Creator IDE does not support cmake and we needed to configure the IDE accordingly.

Note: Infineon&reg; and PSOC&reg; are registered trademarks of Infineon Corporation AG in United States and other countries.

## Project structure

We provide a minimal example project for reference. The TinyUSB library is a submodule, and the MCU port is handled outside the TinyUSB source tree.

If a different class demo is needed, replacing the example folder in the build tree should be enough.
We did not test all the classes and some of them may not be supported (see [Functionality](#functionality)).

### Configuration

The Top Design schematic contains an input PIN (button) and an output pin (LED), that you may need to reconfigure according to your board.

The following preprocessor definitions are required for proper compilation (already added to the build configuration in the example project):

- `CFG_TUSB_MCU=OPT_MCU_CY8C5` (just a placeholder, the port is added out of tree)
- `TUD_ENDPOINT_ONE_DIRECTION_ONLY` (the MPU supports one endpoint per direction)
- `CFG_TUD_EP1_ACTIVE` to `CFG_TUD_EP8_ACTIVE` (enable or disable EP interrupts. Since each interrupt consumes block resources, we did not enable all of them at once. You may need to enable each of the used endpoints editing the USBFS component and adding the corresponding definitions to the project configuration)
- `CFG_TUD_ENDPOINT0_SIZE=8` (EP0 size is 8 bytes)
- `TUP_DCD_ENDPOINT_MAX=8` (processor allows up to 8 endpoints)

### Caveats

During development, we found that some defintions used by TinyUSB may conflict with generated API. In this case, after the headers are generated, it's sufficient to tick "Suppress API generation" in the Top Design USBFS component, under "Advanced" tab.

## Functionality

Available in the port:
- Configuration endpoints (EP0)
- Bulk endpoints (note that IN and OUT EP pairs shall use different EP address, i.e. 1 and 2)
- FIFO transfer (untested)
- Interrupt EP (untested)

Not (yet) supported:
- Isochronous endpoints
- DMA transfers

## Support

This project is independently provided AS-IS, without warranty of any kind, by [Ennebi Elettronica](https://ennebielettronica.com). See [License](license.md) for more information.

We can provide commercial support on request, additional features, bug fixes, hardware and software design services. To reach us out, visit [our website](https://ennebielettronica.com) or write an email to info (at) ennebielettronica.com.
