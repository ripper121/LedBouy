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
//WDTCR |= (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP1);
#include <avr/interrupt.h>
#include <avr/sleep.h>
byte power_count = 0;
ISR(WDT_vect) {
  power_count++;
}

void setup() {
  WDTCR |= (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (0<<WDP1);

  // Set watchdog timer in interrupt mode
  WDTCR |= (1 << WDIE);
  WDTCR |= (0 << WDE);

  sei(); // Enable global interrupts
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
}


void loop() {
  power_off();
  if (power_count > 0) {
    power_count = 0;
    pinMode(PB1, OUTPUT);
    digitalWrite(PB1, HIGH);
    delay(250);
  }
}

void power_off() {
  pinMode(PB0, INPUT);
  pinMode(PB1, INPUT);
  pinMode(PB2, INPUT);
  pinMode(PB3, INPUT);
  pinMode(PB4, INPUT);
  digitalWrite(PB0, LOW);
  digitalWrite(PB1, LOW);
  digitalWrite(PB2, LOW);
  digitalWrite(PB3, LOW);
  digitalWrite(PB4, LOW);
  ADCSRA &= ~(1 << ADEN);   //Turn off ADC
  ACSR |= _BV(ACD);         //Disable analog comparator
  sleep_mode();             //Go Sleep
}



















