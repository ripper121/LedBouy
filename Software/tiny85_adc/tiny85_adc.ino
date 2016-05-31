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
byte power_counter = 0;
byte adc_counter = 0;
int adc_in = 1024;

ISR(WDT_vect) {
  power_counter++;
  adc_counter++;
}

void setup() {

  WDTCR |= (1 << WDP3) | (0 << WDP2) | (0 << WDP1) | (0 << WDP0);
  // Set watchdog timer in interrupt mode
  WDTCR |= (1 << WDIE);
  WDTCR |= (0 << WDE);
  sei(); // Enable global interrupts
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
}


void loop() {
  power_off();
  if (power_counter > 0) {
    power_counter = 0;
    adc_in = read_LDR();
    if (adc_in > 30) { //Higher Value = Darker
      pinMode(PB0, OUTPUT);
      pinMode(PB1, OUTPUT);
      pinMode(PB4, OUTPUT);
      digitalWrite(PB0, HIGH);
      digitalWrite(PB1, HIGH);
      digitalWrite(PB4, HIGH);
      delay(250);
    }
  }
}





int read_LDR(void)
{
  int adc_val = 0;
  //enable pullup on ADC Pin
  pinMode(PB2, INPUT);
  digitalWrite(PB2, HIGH);
  // Set the ADC input to PB2/ADC1
  ADMUX |= (1 << MUX0);
  ADMUX |= (1 << ADLAR);
  // Set the prescaler to clock/128 & enable ADC
  ADCSRA |= (1 << ADPS1) | (1 << ADPS0) | (1 << ADEN);
  for (byte i = 0; i < 3; i++) {
    // Start the conversion
    ADCSRA |= (1 << ADSC);
    // Wait for it to finish
    while (ADCSRA & (1 << ADSC));
    adc_val += ADCH;
  }
  return adc_val / 3;
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
















