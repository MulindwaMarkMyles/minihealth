#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>

// Register definitions using pointers
volatile uint8_t *_PORTB = 0x25;
volatile uint8_t *_DDRB = 0x24;

volatile uint8_t *_PORTC = 0x28;
volatile uint8_t *_DDRC = 0x27;

volatile uint8_t *_PORTD = 0x2B;
volatile uint8_t *_DDRD = 0x2A;

#define SS1 10
#define SS2 9
#define SS3 8
#define LEDpin 1

LiquidCrystal_I2C lcd(0x27, 16, 2);

void customDelay(unsigned long delay) {
  unsigned long start_time = millis();
  while ((millis() - start_time) < delay) {}
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
    } else if (pin < 20) { // _PORTC (A0-A5)
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

void communicateWithSlave(uint8_t slavePin, const char* slaveName) {
  customDigitalWrite(slavePin, LOW);              // Select the slave
  byte receivedData; // Variable to store received bytes

  receivedData = SPI.transfer(1); // Send request and read response
  
  customDigitalWrite(slavePin, HIGH);             // Deselect the slave

  // Process and display the received data
  Serial.print("TempC: ");
  Serial.print(receivedData);
  Serial.print("°C \n");

  lcd.setCursor(6, 1); // Move cursor to the position of the temperature value
  lcd.print("     ");  // Clear the previous value (if needed)
  lcd.setCursor(6, 1); // Move cursor back to the start of the temperature value
  lcd.print(receivedData); // Print the new temperature
  
  delay(1000); // Delay for 1 second
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
  customDelay(1500);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MINI - HEALTH");
  lcd.setCursor(0, 1);
  lcd.print("TempC: 0");

  Serial.begin(115200);
  Serial.println("System Initialized");
}

void loop() {
  communicateWithSlave(SS1, "Slave 1");
  communicateWithSlave(SS2, "Slave 2");
  communicateWithSlave(SS3, "Slave 3");
}
