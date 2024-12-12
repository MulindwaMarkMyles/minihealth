#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <EEPROM.h> // Include EEPROM library

// Register definitions using pointers
volatile uint8_t *_PORTB = 0x25;
volatile uint8_t *_DDRB = 0x24;

volatile uint8_t *_PORTC = 0x28;
volatile uint8_t *_DDRC = 0x27;

volatile uint8_t *_PORTD = 0x2B;
volatile uint8_t *_DDRD = 0x2A;

uint8_t resetCount = 0;

#define SS1 10
#define SS2 9
#define SS3 8
#define LEDpin 1

LiquidCrystal_I2C lcd(0x27, 16, 2);

// Timer1 Registers as variables with direct register addresses
volatile uint8_t* _TCCR1A = (volatile uint8_t*) 0x80;
volatile uint8_t* _TCCR1B = (volatile uint8_t*) 0x81;
volatile uint16_t* _TCNT1 = (volatile uint16_t*) 0x84;
volatile uint16_t* _OCR1A = (volatile uint16_t*) 0x88;
volatile uint8_t* _TIFR1 = (volatile uint8_t*) 0x36;

// Function to create a delay using Timer1
void customTimerDelay(uint16_t milliseconds) {
    *_TCCR1A = 0x00;  // Normal mode
    *_TCCR1B = 0x00;  // No clock source
    *_TCNT1 = 0x0000; // Clear counter value

    uint16_t compareValue = (milliseconds * 250) - 1;
    *_OCR1A = compareValue;

    *_TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);

    while (!(*_TIFR1 & (1 << OCF1A))) {}

    *_TIFR1 |= (1 << OCF1A);
    *_TCCR1B = 0x00;
}

// Custom digital write
void customDigitalWrite(uint8_t pin, uint8_t value) {
    if (pin < 8) {
        if (value == HIGH) {
            *_PORTD |= (1 << pin); 
        } else {
            *_PORTD &= ~(1 << pin);
        }
    } else if (pin < 14) {
        pin -= 8;
        if (value == HIGH) {
            *_PORTB |= (1 << pin); 
        } else {
            *_PORTB &= ~(1 << pin); 
        }
    } else if (pin < 20) {
        pin -= 14;
        if (value == HIGH) {
            *_PORTC |= (1 << pin);  
        } else {
            *_PORTC &= ~(1 << pin); 
        }
    }
}

// Custom pin mode
void customPinMode(uint8_t pin, uint8_t mode) {
    if (pin < 8) {
        if (mode == OUTPUT) {
            *_DDRD |= (1 << pin); 
        } else if (mode == INPUT) {
            *_DDRD &= ~(1 << pin); 
            *_PORTD &= ~(1 << pin);
        } else if (mode == INPUT_PULLUP) {
            *_DDRD &= ~(1 << pin); 
            *_PORTD |= (1 << pin);
        }
    } else if (pin < 14) {
        pin -= 8;
        if (mode == OUTPUT) {
            *_DDRB |= (1 << pin);
        } else if (mode == INPUT) {
            *_DDRB &= ~(1 << pin); 
            *_PORTB &= ~(1 << pin);
        } else if (mode == INPUT_PULLUP) {
            *_DDRB &= ~(1 << pin); 
            *_PORTB |= (1 << pin);
        }
    } else if (pin < 20) {
        pin -= 14;
        if (mode == OUTPUT) {
            *_DDRC |= (1 << pin); 
        } else if (mode == INPUT) {
            *_DDRC &= ~(1 << pin); 
            *_PORTC &= ~(1 << pin);
        } else if (mode == INPUT_PULLUP) {
            *_DDRC &= ~(1 << pin); 
            *_PORTC |= (1 << pin);
        }
    }
}

void updateLCD(const char* label, int value, int row, int col) {
    lcd.setCursor(col, row);
    lcd.print(label);
    lcd.print(value);
}

void communicateWithSlave(uint8_t slavePin, const char* slaveName) {
    Serial.print("Communicating with ");
    Serial.print(slaveName);
    Serial.print(": ");
    
    if (strcmp(slaveName, "Board 1") == 0) {
        customDigitalWrite(slavePin, LOW);
        byte receivedData = SPI.transfer(1);
        customDigitalWrite(slavePin, HIGH);
        
        Serial.print("TempC: ");
        Serial.print(receivedData);
        Serial.println("\u00B0C");

        updateLCD("Temp(C): ", receivedData, 1, 0);

        customDigitalWrite(SS3, LOW);
        byte _receivedData = SPI.transfer(receivedData);
        customDigitalWrite(SS3, HIGH);

    } else if (strcmp(slaveName, "Board 2") == 0) {
        customDigitalWrite(slavePin, LOW);
        byte receivedData = SPI.transfer(1);
        customDigitalWrite(slavePin, HIGH);
        
        Serial.print("BPM: ");
        Serial.println(receivedData);

        updateLCD("BPM: ", receivedData, 1, 0);

        customDigitalWrite(SS3, LOW);
        byte _receivedData = SPI.transfer(receivedData);
        customDigitalWrite(SS3, HIGH);
        
    } else {
        updateLCD("Standby..", 0, 1, 0);
    }

    customTimerDelay(1000);
}

void setup() {
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV8);

    customPinMode(SS1, OUTPUT);
    customPinMode(SS2, OUTPUT);
    customPinMode(SS3, OUTPUT);
    customPinMode(LEDpin, OUTPUT);

    customDigitalWrite(SS1, HIGH);
    customDigitalWrite(SS2, HIGH);
    customDigitalWrite(SS3, HIGH);

    lcd.begin();
    lcd.backlight();

    lcd.setCursor(1, 0);
    lcd.print("GROUP 6");
    lcd.setCursor(1, 1);
    lcd.print("MINI - HEALTH");
    customTimerDelay(1500);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("MINI - HEALTH");

    Serial.begin(115200);
    Serial.println("System Initialized");

    resetCount = EEPROM.read(0);
    resetCount = (resetCount + 1) % 2;
    EEPROM.write(0, resetCount);

    lcd.setCursor(0, 0);
    lcd.print("Mode: ");
    lcd.print(resetCount == 0 ? "Board 1" : "Board 2");
}

void loop() {
    resetCount = EEPROM.read(0);
    
    if (resetCount == 0) {
        communicateWithSlave(SS1, "Board 1");
    } else if (resetCount == 1) {
        communicateWithSlave(SS2, "Board 2");
    }

    customTimerDelay(1500);
}
