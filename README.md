# xBridge SDK


## Pololu Wixel SDK, with xBridge code.
This repository contains Adrien de Croy's polou wixel SDK, his original dexterity code, and a modified version of that code called xBridge2.
To build xBridge2, please download this entire repository. Or download and install the xBridge2.wxl file from apps/xBridge2 onto your wixel.

All thanks to Adrien de Croy for all his work on developing dexterity, which has made this code possible.

## xBridge Version History

Version | Changes 
--- | ---
2.47e | Adds support for HM-16/17 BLE modules. Simplified circuitry. Greater packet capture and sleep timing stability
2.46 | Increased LED indications as requested by Kate Farnsworth.
2.43 | Improved power consumption. Fixed power drain through UART.

# Road Map

1. Implement a packet queue, a la savek-cc's version.  (target 2.48 or 3.0x).  Requires a protocol change that will impact users of xDrip and older versions of xDrip+.  Hence my reluctance to implement in v2.xx.  v3 will only be supported in xDrip+, and this can be better managed.
2. Channel Scanning (target v3.xx).  This will remove the reliance on sleep time being to within a 100ms accuracy.  However, attempts to write the correct code into the radio mac has so far proven unsuccessful.

# Important documentation to be completed
1. Test Plan - So that every developer can regression test their code prior to doing PRs.
2. Stand alone code repository, not dependent on the wixel-sdk.
3. Release Process - So far this has been a minimal process relying solely on jstevensog to manage.  This needs to change so that others can contribute to getting changes through development, testing, beta testing and release without impacting the non-tehcnical user.
