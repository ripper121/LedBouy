// set timer to 0.016s
// WDTCR |= (0<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);
// set timer to 0.032s
// WDTCR |= (0<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);
// set timer to 0.064s
// WDTCR |= (0<<WDP3) | (0<<WDP2) | (1<<WDP1) | (0<<WDP0);
// set timer to 0.125s
// WDTCR |= (0<<WDP3) | (0<<WDP2) | (1<<WDP1) | (1<<WDP0);
// set timer to 0.250s
// WDTCR |= (0<<WDP3) | (1<<WDP2) | (0<<WDP1) | (0<<WDP0);
// set timer to 0.5s
// WDTCR |= (0<<WDP3) | (1<<WDP2) | (0<<WDP1) | (1<<WDP0);
//set timer to 2 sec
//WDTCR |= (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);
//set timer to 4 sec
//WDTCR |= (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);
//set timer to 8 sec
//WDTCR |= (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>
#include <Vcc.h>
byte power_count = 0;
const float VccMin   = 2.0;           // Minimum expected Vcc level, in Volts.
const float VccMax   = 3.0;           // Maximum expected Vcc level, in Volts.
const float VccCorrection = 1.0 / 1.0; // Measured Vcc by multimeter divided by reported Vcc
Vcc vcc(VccCorrection);

ISR(WDT_vect) {
  power_count++;
}

void setup() {
  byte p = vcc.Read_Perc(VccMin, VccMax);
  if (p > 100)
    p = 100;
  DDRB |= (1 << DDB0) | (1 << DDB1) | (1 << DDB4);
  for (byte i = 0; i < p / 10; i++) {
    PORTB |= (1 << DDB0) | (1 << DDB1) | (1 << DDB4);
    _delay_ms(250);
    PORTB = 0x00;
    _delay_ms(250);
  }
  // set timer to 8s
  WDTCR |= (1 << WDP3) | (0 << WDP2) | (0 << WDP1) | (1 << WDP0) | (1 << WDIE) | (0 << WDE);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
}


void loop() {
  //-----POWER OFF START-----
  //PortB Input LOW
  DDRB = 0x00;
  PORTB = 0x00;
  ADCSRA &= ~(1 << ADEN);
  power_usi_disable();
  power_adc_disable();
  power_all_disable();
  sleep_mode();
  //-----POWER OFF END  -----
  if (power_count > 0) {
    power_count = 0;
    DDRB |= (1 << DDB0) | (1 << DDB1) | (1 << DDB4);
    PORTB |= (1 << DDB0) | (1 << DDB1) | (1 << DDB4);
    _delay_ms(100);
  }
}

















