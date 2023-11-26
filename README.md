# PicoJoybus

This repo contains simple implementation of Joybus protocol (device only) written in PIO assembly and some supporting C code.
Joybus protocol was used in the Nintendo 64 and GameCube game consoles.

It also contains example usage with BTstack connecting to the wireless Xbox Series S/X gamepad (BLE versions only).
Controller's MAC address is hardcoded and should be filled in `xbox_address` variable before building this project.
