//THIS IS THE MAIN ARDUINO
// - It controls all the others.

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

void customDelay(unsigned long delay){
  unsigned long start_time = millis();
  while ((millis() - start_time) < delay)
  {}
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
int LEDpin = 13;


void setup()
{

  customPinMode(13, OUTPUT);
  customDigitalWrite(LEDpin, LOW);
  // initialize the LCD
	lcd.begin();
  // Turn on the blacklight and print a message.
	lcd.backlight();

  //Initialize the serial port
  Serial.begin(115200);
  Serial.println("Initialing the system..");

	lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Initializing");
  lcd.setCursor(1, 1);
  lcd.print("the system..");

  customDelay(2000);
  lcd.clear();

  lcd.setCursor(1, 0);
	lcd.print("GROUP 6");
  lcd.setCursor(1, 1);
  lcd.print("MINI - HEALTH");
  customDelay(3000);

  lcd.clear();

  lcd.setCursor(1, 0);
  lcd.print("Initialization");
  lcd.setCursor(1, 1);
  lcd.print("done.");
  customDelay(1500);
  customDigitalWrite(LEDpin, HIGH);
  
}

void loop()
{
  lcd.clear();
	lcd.setCursor(1, 0);
	lcd.print("GROUP 6");
  lcd.setCursor(1, 1);
  lcd.print("MINI - HEALTH");
  customDelay(3000);
}
