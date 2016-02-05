#ifndef WATCHDOG_H
#define WATCHDOG_H

#if ENABLED(USE_WATCHDOG)
  // Initialize watchdog with a 4 second interrupt time
  void watchdog_init();
  // pad the dog/reset watchdog. MUST be called at least every second after the first watchdog_init or AVR will go into emergency procedures..
  void watchdog_reset();
#else
  //If we do not have a watchdog, then we can have empty functions which are optimized away.
  FORCE_INLINE void watchdog_init() {};
  FORCE_INLINE void watchdog_reset() {};
#endif

#endif
