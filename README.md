# Interrupt Example

This is an example program that produces a PWM signal. Every time the PWM output (pin 9.6) falls it triggers an interrupt. The interrupt service routine (ISR) toggles LED3 (pin 6.3). You can do whatever else needs to be done during the ISR so long as it doesn't take very much time.

