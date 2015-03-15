# MarlinKimbra4due 3D Printer Firmware

### THIS IS A BETA VERSION

### New features are:
* Steprates up to 320.000 steps/s (top for 1/128 stepper driver)
 
### known issues:
  * External DAC (ALLIAGTOR BOARD) is not well written
  * Watchdog doesn't work, don't activate it
  * Now only 1 extruder
  * 

---
# MarlinKimbra4due 3D Printer Firmware
  * [Configuration & Compilation](/Documentation/Compilation.md)
  * Supported
    * [Features](/Documentation/Features.md)
    * [Hardware](/Documentation/Hardware.md)
    * [GCodes](/Documentation/GCodes.md)
  * Notes
    * [Auto Bed Leveling](/Documentation/BedLeveling.md)
    * [Filament Sensor](/Documentation/FilamentSensor.md)
    * [Ramps Servo Power](/Documentation/RampsServoPower.md)


## Quick Information

This version of Marlin was made to accommodate some requests made by the community RepRap Italy http://forums.reprap.org/index.php?349
The new features are:
A single Firmware for all types of printers; Cartesian, Delta, SCARA, CoreXY.
The possibility of having only one hotend independently from the extruders that you have.
The addition of the 4th extruder.
System Management MKr4 for 4 extruders with just two drivers or two extruders with a driver only.
Management Multyextruder NPr2, 4/6 extruders with only two engines.
Adding commands to facilitate purging of hotend. 
Step per unit varied for each extruder as well as the feedrate.
The addition of a different feedrate for retraction. 
Adding Debug Dryrun used by repetier.

## Credits

The current MarlinKimbra dev team consists of:

 - MagoKimbra (https://github.com/MagoKimbra)
  

More features have been added by:
  - 