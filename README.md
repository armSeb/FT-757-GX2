# FT-757-GX2
A CPU replacement for the FT-757-GX2 based on the atmega644p microcontroller. 

This is an early version, with an home made cpu adaptor. Please note there is a lot of optimizations to do.

Why this project? Because I purchased a non-working rig with a defect CPU, and because this component is not available anymore.
This is also a very good excercise, to learn how this device works internally. 
And... maybe a good way to improve this rig (i.e more memories ?).

Whats is working?

- VFO A / B
- Memories
- Clarifier
- Split
- VFO KNOB
- Display
- Ham / Gen button
- Band / mode change buttons, Fast button
- EEPROM to save state and frequencies (there is no need for backup battery anymore)
- Firmware update via the CAT connector (Optiboot bootloader).
- Mic Up / Down buttons
- Factory reset by powering on while pressing the h/g button
- VFO Lock 

TODO : 

- Implement different increment / decrement steps according to the user manual
- Implement SCAN Frequencies / Memories
- Implement memories pages (to increase mems from 10 to 100 positions) ?
- Maybe make an optimized schematic and PCB to fit more easily on the local unit.

![adaptor_top](https://github.com/armSeb/FT-757-GX2/assets/12976342/83bf0b96-5a5e-4aee-bec3-9193282dfbc8)
![adaptor_bottom](https://github.com/armSeb/FT-757-GX2/assets/12976342/1931da4f-d532-43c9-a7dd-eafc4d1aa3a8)
![installed_cpu](https://github.com/armSeb/FT-757-GX2/assets/12976342/a5e87364-992d-4f5f-966c-068ec9f8eef4)
