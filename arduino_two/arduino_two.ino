#include <SPI.h>

#define LM35_PIN 1 // LM35 connected to Analog pin A1 (ADC1)
#define LEDpin 2

volatile boolean received = false;
volatile byte Slavereceived, Slavesend;

void customDigitalWrite(uint8_t pin, uint8_t value) {
    if (pin < 8) { // PORTD
        if (value == HIGH) {
            PORTD |= (1 << pin); 
        } else {
            PORTD &= ~(1 << pin);
        }
    } else if (pin < 14) { // PORTB
        pin -= 8; // Adjust pin number for PORTB
        if (value == HIGH) {
            PORTB |= (1 << pin); 
        } else {
            PORTB &= ~(1 << pin); 
        }
    } else if (pin < 20) { // (A0-A5) belong to PORTC
        pin -= 14; // Adjust pin number for PORTC
        if (value == HIGH) {
            PORTC |= (1 << pin);  
        } else {
            PORTC &= ~(1 << pin); 
        }
    }
}

void customPinMode(uint8_t pin, uint8_t mode) {
    if (pin < 8) { // PORTD
        if (mode == OUTPUT) {
            DDRD |= (1 << pin); 
        } else if (mode == INPUT) {
            DDRD &= ~(1 << pin); 
            PORTD &= ~(1 << pin); // Disable pull-up
        } else if (mode == INPUT_PULLUP) {
            DDRD &= ~(1 << pin); 
            PORTD |= (1 << pin); // Enable pull-up
        }
    } else if (pin < 14) { // PORTB
        pin -= 8; // Adjust for PORTB pins
        if (mode == OUTPUT) {
            DDRB |= (1 << pin);
        } else if (mode == INPUT) {
            DDRB &= ~(1 << pin); 
            PORTB &= ~(1 << pin); // Disable pull-up
        } else if (mode == INPUT_PULLUP) {
            DDRB &= ~(1 << pin); 
            PORTB |= (1 << pin); // Enable pull-up
        }
    } else if (pin < 20) { // PORTC (Analog pins A0-A5)
        pin -= 14; // Adjust for PORTC pins
        if (mode == OUTPUT) {
            DDRC |= (1 << pin); 
        } else if (mode == INPUT) {
            DDRC &= ~(1 << pin); 
            PORTC &= ~(1 << pin); // Disable pull-up
        } else if (mode == INPUT_PULLUP) {
            DDRC &= ~(1 << pin); 
            PORTC |= (1 << pin); // Enable pull-up
        }
    }
}



void setup() {
  DDRC &= ~(1 << LM35_PIN); // Configure ADC1 (A1) as input
  ADMUX = (1 << REFS0);     // AVcc as reference voltage
  ADMUX |= LM35_PIN;        // Select ADC1 (A1)
  ADCSRA = (1 << ADEN)      // Enable ADC
         | (1 << ADPS2)     // Prescaler = 64
         | (1 << ADPS1);

  customPinMode(MISO, OUTPUT);      // Set MISO as OUTPUT
  SPCR |= _BV(SPE);           // Enable SPI in Slave Mode
  SPI.attachInterrupt();      // Enable SPI interrupt
  
  customPinMode(LEDpin, OUTPUT);
  customDigitalWrite(LEDpin, LOW);

  Serial.begin(115200);
}

ISR(SPI_STC_vect) {
  Slavereceived = SPDR; // Store the byte received from the Master
  SPDR = Slavesend; // Send the current byte
  received = true;
}

void loop() {
  // Read LM35 sensor
  ADCSRA |= (1 << ADSC);      // Start ADC conversion
  while (ADCSRA & (1 << ADSC)); // Wait for conversion to complete
  uint16_t adcValue = ADC;

  // Calculate temperature
  float voltage = adcValue * 5.0 / 1023.0;
  float tempC = voltage * 100.0;
  float tempF = (tempC * 9.0 / 5.0) + 32;

  // Populate data to send
  Slavesend = tempC; // First byte: Integer part of tempC

  Serial.println(Slavesend);

  // LED Indicator
  if (tempC > 30.0) {
    customDigitalWrite(LEDpin, HIGH);
  } else {
    customDigitalWrite(LEDpin, LOW);
  }

  // Debugging
  if (received) {
    Serial.println("Master requested data.");
    received = false;
  }

  delay(1000); // Delay for 1 second
}
