//THIS IS THE MAIN ARDUINO
// - It controls all the others.

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>

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

void communicateWithSlave(uint8_t slavePin, const char* slaveName) {
  customDigitalWrite(slavePin, LOW);              // Select the slave
  byte receivedData; // Variable to store received bytes

  receivedData = SPI.transfer(1); // Send request and read response
  
  customDigitalWrite(slavePin, HIGH);             // Deselect the slave

  // Process and display the received data
  Serial.print("TempC: ");
  Serial.print(receivedData);
  Serial.print("°C \n");

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("MINI - HEALTH");
  lcd.setCursor(0, 1);
  lcd.print("TempC: ");
  lcd.print(receivedData);
  
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

  Serial.begin(115200);
  Serial.println("System Initialized");
}

void loop() {
  communicateWithSlave(SS1, "Slave 1");
  communicateWithSlave(SS2, "Slave 2");
  communicateWithSlave(SS3, "Slave 3");
}

