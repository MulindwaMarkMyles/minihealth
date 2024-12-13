#include <SPI.h>

#define LM35_PIN 0
#define LEDpin 2
#define BUZZERpin 3

volatile boolean received = false;
volatile byte Slavereceived, Slavesend;


// Timer1 Registers as variables with direct register addresses
volatile uint8_t* _TCCR1A = (volatile uint8_t*) 0x80;
volatile uint8_t* _TCCR1B = (volatile uint8_t*) 0x81;
volatile uint16_t* _TCNT1 = (volatile uint16_t*) 0x84;
volatile uint16_t* _OCR1A = (volatile uint16_t*) 0x88;
volatile uint8_t* _TIFR1 = (volatile uint8_t*) 0x36;

volatile uint8_t *_PORTB = 0x25;
volatile uint8_t *_DDRB = 0x24;
volatile uint8_t *_PORTC = 0x28;
volatile uint8_t *_DDRC = 0x27;
volatile uint8_t *_PORTD = 0x2B;
volatile uint8_t *_DDRD = 0x2A;

// Function to create a delay using Timer1
void customTimerDelay(uint16_t milliseconds) {
    // Clear Timer/Counter1 registers
    *_TCCR1A = 0x00;  // Normal mode
    *_TCCR1B = 0x00;  // No clock source
    *_TCNT1 = 0x0000; // Clear counter value

    // Assuming 16 MHz clock and prescaler of 64
    uint16_t compareValue = (milliseconds * 250) - 1;
    *_OCR1A = compareValue;

    // Configure Timer1 for CTC mode with prescaler of 64
    *_TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);

    // Wait for the timer to reach the compare match value
    while (!(*_TIFR1 & (1 << OCF1A))) {
    }

    // Clear the OCF1A flag by writing 1 to it
    *_TIFR1 |= (1 << OCF1A);

    // Stop the timer
    *_TCCR1B = 0x00;
}


void customDigitalWrite(uint8_t pin, uint8_t value) {
    if (pin < 8) { // _PORTD
        if (value == HIGH) {
            *_PORTD |= (1 << pin); 
        } else {
            *_PORTD &= ~(1 << pin);
        }
    } else if (pin < 14) { // _PORTB
        pin -= 8; // Adjust pin number for _PORTB
        if (value == HIGH) {
            *_PORTB |= (1 << pin); 
        } else {
            *_PORTB &= ~(1 << pin); 
        }
    } else if (pin < 20) { // (A0-A5) belong to _PORTC
        pin -= 14; // Adjust pin number for _PORTC
        if (value == HIGH) {
            *_PORTC |= (1 << pin);  
        } else {
            *_PORTC &= ~(1 << pin); 
        }
    }
}

void customPinMode(uint8_t pin, uint8_t mode) {
    if (pin < 8) { // _PORTD
        if (mode == OUTPUT) {
            *_DDRD |= (1 << pin); 
        } else if (mode == INPUT) {
            *_DDRD &= ~(1 << pin); 
            *_PORTD &= ~(1 << pin); // Disable pull-up
        } else if (mode == INPUT_PULLUP) {
            *_DDRD &= ~(1 << pin); 
            *_PORTD |= (1 << pin); // Enable pull-up
        }
    } else if (pin < 14) { // _PORTB
        pin -= 8; // Adjust for _PORTB pins
        if (mode == OUTPUT) {
            *_DDRB |= (1 << pin);
        } else if (mode == INPUT) {
            *_DDRB &= ~(1 << pin); 
            *_PORTB &= ~(1 << pin); // Disable pull-up
        } else if (mode == INPUT_PULLUP) {
            *_DDRB &= ~(1 << pin); 
            *_PORTB |= (1 << pin); // Enable pull-up
        }
    } else if (pin < 20) { // _PORTC (Analog pins A0-A5)
        pin -= 14; // Adjust for _PORTC pins
        if (mode == OUTPUT) {
            *_DDRC |= (1 << pin); 
        } else if (mode == INPUT) {
            *_DDRC &= ~(1 << pin); 
            *_PORTC &= ~(1 << pin); // Disable pull-up
        } else if (mode == INPUT_PULLUP) {
            *_DDRC &= ~(1 << pin); 
            *_PORTC |= (1 << pin); // Enable pull-up
        }
    }
}



void setup() {
  *_DDRC &= ~(1 << LM35_PIN); // Configure ADC1 (A1) as input
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
  customPinMode(BUZZERpin, OUTPUT);
  customDigitalWrite(BUZZERpin, LOW);

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
    customDigitalWrite(BUZZERpin, HIGH);
  } else {
    customDigitalWrite(LEDpin, LOW);
    customDigitalWrite(BUZZERpin, LOW);
  }

  // Debugging
  if (received) {
    Serial.println("Master requested data.");
    received = false;
  }

  customTimerDelay(1000); // Delay for 1 second
}
