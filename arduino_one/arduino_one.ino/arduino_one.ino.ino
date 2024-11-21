//THIS IS THE MAIN ARDUINO
// - It controls all the others.

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

void custom_delay(unsigned long delay){
  unsigned long start_time = millis();
  while ((millis() - start_time) < delay)
  {}
}

void setup()
{

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

  custom_delay(2000);
  lcd.clear();

  lcd.setCursor(1, 0);
	lcd.print("GROUP 6");
  lcd.setCursor(1, 1);
  lcd.print("MINI - HEALTH");
  custom_delay(3000);

  lcd.clear();

  lcd.setCursor(1, 0);
  lcd.print("Initialization");
  lcd.setCursor(1, 1);
  lcd.print("done.");
  custom_delay(1500);
  
}

void loop()
{
  lcd.clear();
	lcd.setCursor(1, 0);
	lcd.print("GROUP 6");
  lcd.setCursor(1, 1);
  lcd.print("MINI - HEALTH");
  custom_delay(3000);
}
