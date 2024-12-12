#include <EEPROM.h>
#include <SPI.h>

volatile bool dataReady = false;
volatile uint8_t receivedBPM = 0;
volatile uint8_t receivedTemp = 0;
volatile uint8_t byteCount = 0;

volatile uint8_t *_PORTB = 0x25;
volatile uint8_t *_DDRB = 0x24;

volatile uint8_t *_PORTC = 0x28;
volatile uint8_t *_DDRC = 0x27;

volatile uint8_t *_PORTD = 0x2B;
volatile uint8_t *_DDRD = 0x2A;

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
    } else if (pin < 20) { // _PORTC (A0-A5)
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


volatile bool dataReceived = false;
volatile uint8_t receivedData; 
volatile uint8_t dataIndex = 0;

void setup() {
    // Set up SPI as Slave
    customPinMode(MISO, OUTPUT);
    SPCR |= _BV(SPE); // Enable SPI

    // Attach interrupt to handle incoming SPI data
    SPI.attachInterrupt();
    Serial.begin(9600);
    Serial.println("Slave 3 ready");
}

ISR(SPI_STC_vect) {
    // SPI Serial Transfer Complete Interrupt
    receivedData = SPDR; // Read data from SPI Data Register
    SPDR = 1; 
    dataReceived = true;
}

void loop() {
    if (dataReceived) {
        // Once data is fully received, process it

        if (receivedData < 60) {

          int temp = receivedData;

          EEPROM.write(1, temp);

          Serial.print("Temp backed up: ");
          Serial.println(temp);

          dataReceived = false; // Reset flag
        } else {
          int bpm = receivedData;

          EEPROM.write(0, bpm);

          Serial.print("BPM backed up: ");
          Serial.println(bpm);

          dataReceived = false; // Reset flag          
        }
    }
}
