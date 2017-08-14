# xBridge1

Pololu Wixel SDK, with xBridge2 code.
This repository contains Adrien de Croy's polou wixel SDK, his original dexterity code, and a modified version of that code called xBridge2.
To build dexbridge, please download this entire repository.

All thanks to Adrien de Croy for all his work on developing dexterity, which has made this code possible.

# Road Map

1. Stabilise sleep timing (target v2.48).  Currently the LPRCOSC is used during sleep timing to get the most battery life.  However, this is not that stable, as the oscillator drifts with temperature changes.  There are two possible solutions, one reliant on the other.
  1. Determine the maximum sleep time for stability after a calculation, and wake that often to recalibrate.  First attmpt was in v2.47, making the wixel wake every 10 seconds to perform the calibration.  This could severely impact battery life.
  2. Determine how long the wixel was in PM1/3.  This should be done using the WORTIME counter, but so far I have not been able to reliably read this value in the ST ISR and store the values in variabls for this calculation.  It cannot be used for timing in active mode, as ST interrupts can be missed, but reading it in the ISR should give a reasonably accurate (to within 10ms) idea of the sleep time.  Will be in concert with the first option.
2. Implement a packet queue, a la savek-cc's version.  (target 2.48 or 3.0x).  Requires a protocol change that will impact users of xDrip and older versions of xDrip+.  Hence my reluctance to implement in v2.xx.  v3 will only be supported in xDrip+, and this can be better managed.
3. Channel Scanning (target v3.xx).  This will remove the reliance on sleep time being to within a 100ms accuracy.  However, attempts to write the correct code into the radio mac has so far proven unsuccessful.

# Important documentation to be completed
1. Test Plan - So that every developer can regression test their code prior to doing PRs.
2. Stand alone code repository, not dependent on the wixel-sdk.
3. Release Process - So far this has been a minimal process relying solely on jstevensog to manage.  This needs to change so that others can contribute to getting changes through development, testing, beta testing and release without impacting the non-tehcnical user.
