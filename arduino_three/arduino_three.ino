#include <SPI.h>
#include <PulseSensorPlayground.h>

const int PULSE_SENSOR_PIN = 0;  
const int LED_PIN = 13;          
const int THRESHOLD = 550; 

volatile boolean received = false;
volatile byte Slavereceived, Slavesend;
int currentBPM = 0;

PulseSensorPlayground pulseSensor;

// Timer1 Registers as variables with direct register addresses
volatile uint8_t* _TCCR1A = (volatile uint8_t*) 0x80;
volatile uint8_t* _TCCR1B = (volatile uint8_t*) 0x81;
volatile uint16_t* _TCNT1 = (volatile uint16_t*) 0x84;
volatile uint16_t* _OCR1A = (volatile uint16_t*) 0x88;
volatile uint8_t* _TIFR1 = (volatile uint8_t*) 0x36;

volatile uint8_t * _PORTB = 0x25;
volatile uint8_t * _DDRB = 0x24;
volatile uint8_t * _PORTC = 0x28;
volatile uint8_t * _DDRC = 0x27;
volatile uint8_t * _PORTD = 0x2B;
volatile uint8_t * _DDRD = 0x2A;

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
  customPinMode(MISO, OUTPUT);      // Set MISO as OUTPUT
  SPCR |= _BV(SPE);           // Enable SPI in Slave Mode
  SPI.attachInterrupt();      // Enable SPI interrupt

  pulseSensor.analogInput(PULSE_SENSOR_PIN);
  pulseSensor.blinkOnPulse(LED_PIN);
  pulseSensor.setThreshold(THRESHOLD);
 
  if (pulseSensor.begin()) 
  {
    Serial.println("PulseSensor object created successfully!");
  }

  Serial.begin(115200);
}

ISR(SPI_STC_vect) {
  Slavereceived = SPDR; // Store the byte received from the Master
  SPDR = Slavesend; // Send the current byte
  received = true;
}

void loop() {
  currentBPM = pulseSensor.getBeatsPerMinute();
 
  // Check if a heartbeat is detected
  if (pulseSensor.sawStartOfBeat()) 
  {
    Serial.println("♥ A HeartBeat Happened!");
    Serial.print("BPM: ");
    Serial.println(currentBPM);
  }
 
  // Populate data to send
  Slavesend = currentBPM; 

  Serial.print("The data sent: ");
  Serial.println(Slavesend);

  // Debugging
  if (received) {
    Serial.println("Master requested data.");
    received = false;
  }

  // customTimerDelay(1000); // Causing bugs
  delay(1000);
}
