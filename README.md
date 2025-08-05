# PicoJoybus

This repo contains simple implementation of Joybus protocol (device only) written in PIO assembly and some supporting C code.
Joybus protocol was used in the Nintendo 64 and GameCube game consoles.

It also contains example usage with BTstack connecting to the wireless Xbox Series ONE/S/X gamepad (BLE versions only).
Controller's hardware address is hardcoded within `src/main.c` and you should update the `xbox_address` variable before building this project.

## Controller button mapping

The default mapping is as follows:

| Xbox | N64 |
|:--|:--|
| DL | DL |
| DR | DR |
| DU | DU |
| DD | DD |
| LB | CL |
| RB | CR |
| Y | CU |
| B | CD |
| A | A | 
| X | B | 
| Left stick button | L |
| RT | R |
| LT | Z |
| menu / option | Start |

### Extras
- The right joystick may also be used for C button directions.
- The Xbox button is a macro (currently used for resetting the RGB mod)
- The upload button resets the controller joysticks to zero.

## Obtaining the "xbox_address" on Windows
- Connect the controller over bluetooth.
- Open Device Manager (search for it in Windows). 
- Expand "Bluetooth". 
- Find the Wireless Controller (it might be listed as "Bluetooth LE XINPUT compatible input device"). 
- Right-click on it, choose "Properties," then the "Details" tab. 
- In the "Property" dropdown, select "Hardware Ids

## Building
- Run the dev container.
- Press the `Build` button on the bottom toolbar.
- The binary's will be added to the project root `build` folder.

### Clean
Delete the contents of the build folder.

## Pico Pinout
WARNING: VSYS should not be connected when powered (debugging) by USB.
- VSYS -> N64 joypad power VCC+.
- GPIO28 -> N64 joypad IO.
- AGND -> N64 joypad GND.
