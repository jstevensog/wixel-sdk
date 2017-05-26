dexbridge
=========

**NOTE: This version includes a package queue that saves up to 64 packets on the wixel and transmits them to a phone (running xDrip+) as soon as a bluetooth connection can be established. So if you are out of range of your phone you will not miss the packets but instead receive them once you are back in range.**

The handling of the LEDs also changed compared to the original version:
RED LED: Dex packet, blinking 1/sec while trying to capture a packet, blinking 2/sec once a packet has been captured.
YELLOW LED: Bluetooth status, blinking 1/sec while trying to connect to the phone, blinking 2/sec once a BLE connection has been established.

Disclaimer: This version is my personal research version - so code in the repository will often not work as expected. It also requires a modified version of xDrip to be able to correctly handle the modified packet format.

Pololu Wixel SDK, with dexbridge code.
This repository contains Adrien de Croy's polou wixel SDK, his original dexterity code, and a modified version of that code called dexbridge.
To build dexbridge, please download this entire repository.

All thanks to Adrien de Croy for all his work on developing dexterity, which has made this code possible.



