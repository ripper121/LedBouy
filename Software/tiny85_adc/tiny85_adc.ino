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
//set timer to 1 sec
//WDTCR |= (0 << WDP3) | (1 << WDP2) | (1 << WDP1) | (0 << WDP0);
//set timer to 2 sec
//WDTCR |= (0<<WDP3) | (1<<WDP2) | (1<<WDP1) | (1<<WDP0);
//set timer to 4 sec
//WDTCR |= (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP0);
//set timer to 8 sec
//WDTCR |= (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP1);
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#define DARKVAL 200
#define DEVIATION 1
#define HYST 6
byte power_counter = 1;
int adc_in = 1023;
int adc_in_pref = 1023;
int adc_dev = 1023;
int addr = 0;
ISR(WDT_vect) {
  power_counter--;
}

void setup() {
  // set timer to 0.5s
  WDTCR |= (0 << WDP3) | (1 << WDP2) | (0 << WDP1) | (1 << WDP0) | (1 << WDIE) | (0 << WDE);
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
}

void loop() {
  //-----POWER OFF START-----
  //PortB Input LOW
  DDRB = 0x00;
  PORTB = 0x00;
  power_all_disable();
  //Go Sleep
  sleep_enable();
  sleep_mode();
  sleep_disable();
  //-----POWER OFF END  -----

  //-----READ ADC START-----
  //Save old value
  adc_in_pref = adc_in;
  //PB2 Input Pullup
  DDRB &= ~(1 << DDB2);
  PORTB |= (1 << PB2);
  power_adc_enable();
  // Set the ADC input to PB2/ADC1
  ADMUX |= (1 << MUX0) | (1 << ADLAR);
  // Set the prescaler to clock/128 & enable ADC
  ADCSRA |= (1 << ADPS1) | (1 << ADPS0) | (1 << ADEN) | (1 << ADSC);
  // Wait for it to finish
  while (ADCSRA & (1 << ADSC));
  //adc_in = (ADCH << 8) | ADCL;
  adc_in = ADCH;
  //Turn off ADC
  ACSR |= _BV(ACD);                         //disable the analog comparator
  ADCSRA &= ~_BV(ADEN);                     //disable ADC
  power_adc_disable();
  //-----READ ADC END  -----
  if (power_counter < 1 | power_counter > HYST) {
    power_counter = 1;
    //Get DEVIATION
    if (adc_in > adc_in_pref)
      adc_dev = adc_in - adc_in_pref;
    else
      adc_dev = adc_in_pref - adc_in;

    //Check DEVIATION and DARKVAL
    if (adc_dev > DEVIATION && adc_in > DARKVAL) {
      DDRB |= (1 << DDB0) | (1 << DDB1) | (1 << DDB4);
      for (byte i = 0; i < 10; i++) {
        PORTB ^= (1 << PB0) | (1 << PB1) | (1 << PB4);
        _delay_ms(100);
      }
      power_counter = HYST;
    }
  }
}

















