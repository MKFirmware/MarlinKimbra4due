# MarlinKimbra4due 3D Printer Firmware

### Special thanks
* Wurstnase
* all Marlin8bit-developers.

### THIS IS A BETA VERSION

### New features are:
* Steprates up to 320.000 steps/s (top for 1/128 stepper driver)
* smoother temperature (median of last 5 values)
 
### known issues:
  * Watchdog doesn't work, don't activate it
  * Servo not tested
  * LCD not tested

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
    * [LCD Language - Font - System](Documentation/LCDLanguageFont.md)



## Quick Information

This version of Marlin was made to accommodate some requests made by the community RepRap Italy http://forums.reprap.org/index.php?349
The new features are:
A single Firmware for all types of printers; Cartesian, Delta, SCARA, CoreXY.
The possibility of having only one hotend independently from the extruders that you have.
The addition of the 4th extruder.
System Management MKr4 for 4 extruders with a driver only.
Management Multyextruder NPr2, 4/6 extruders with only two engines.
Adding commands to facilitate purging of hotend. 
Step per unit varied for each extruder as well as the feedrate.
The addition of a different feedrate for retraction. 
Adding Debug Dryrun used by repetier.

## Credits

The current MarlinKimbra dev team consists of:

 - MagoKimbra (https://github.com/MagoKimbra)

More features have been added by:
  - Nico [@wurstnase],
  - Lampmaker,
  - Bradley Feldman,
  - simone97 (https://github.com/simone97),
  - and others...

## License

Marlin is published under the [GPL license](/Documentation/COPYING.md) because I believe in open development.
Please do not use this code in products (3D printers, CNC etc) that are closed source or are crippled by a patent.