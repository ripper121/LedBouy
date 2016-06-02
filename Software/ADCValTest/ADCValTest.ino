/*
  TrinketKeyboard example
  For Trinket by Adafruit Industries
*/

#include <TrinketKeyboard.h>
#include <EEPROMex.h>
float sensorValue = 0;

void setup()
{
  TrinketKeyboard.begin();

}

void loop()
{
  TrinketKeyboard.poll();
  //-----READ ADC START-----
  //PB2 Input Pullup
  DDRB &= ~(1 << DDB2);
  PORTB |= (1 << PB2);
  // Set the ADC input to PB2/ADC1
  ADMUX |= (1 << MUX0) | (1 << ADLAR);
  // Set the prescaler to clock/128 & enable ADC
  ADCSRA |= (1 << ADPS1) | (1 << ADPS0) | (1 << ADEN) | (1 << ADSC);
  // Wait for it to finish
  while (ADCSRA & (1 << ADSC));
  uint16_t combined=ADC;
  //Turn off ADC
  ACSR |= _BV(ACD);                         //disable the analog comparator
  ADCSRA &= ~_BV(ADEN);                     //disable ADC
  //-----READ ADC END  -----

  TrinketKeyboard.println(combined,DEC);
  delay(2);
}
