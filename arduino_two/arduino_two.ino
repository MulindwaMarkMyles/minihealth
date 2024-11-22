#define SENSOR_PIN 1 // LM35 connected to Analog pin A1 (ADC1)

float tempC = 0; // Store temperature in Celsius
float tempF = 0; // Store temperature in Fahrenheit


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

void setup() {
  // Configure ADC1 (A1) as input using DDRC register
  DDRC &= ~(1 << SENSOR_PIN); // Clear DDC1 bit to set A1 as input
  
  // Configure ADC
  ADMUX = (1 << REFS0);        // AVcc as reference voltage
  ADMUX |= SENSOR_PIN;         // Select ADC1 (A1) as input channel
  ADCSRA = (1 << ADEN)         // Enable ADC
         | (1 << ADPS2)        // Prescaler 64 for 16 MHz clock (250 kHz ADC clock)
         | (1 << ADPS1);       // (Prescaler = 64)

  customPinMode(LEDpin, OUTPUT);
  customDigitalWrite(LEDpin, LOW);
}

void loop() {
  // Start ADC conversion
  ADCSRA |= (1 << ADSC);
  
  // Wait for conversion to complete
  while (ADCSRA & (1 << ADSC));
  
  // Read ADC value (10 bits)
  uint16_t adcValue = ADC;
  
  // Convert ADC value to voltage
  float voltage = adcValue * 5.0 / 1023.0;
  
  // Convert voltage to temperature (10 mV per °C)
  tempC = voltage * 100.0;
  
  // Convert Celsius to Fahrenheit
  tempF = (tempC * 9.0 / 5.0) + 32;
  
  // Print results
  Serial.begin(9600);
  Serial.println(String(tempC) + "°C, " + String(tempF) + "°F");
  if (tempC > 30.0){
    customDigitalWrite(LEDpin, HIGH);
  } else {
    customDigitalWrite(LEDpin, LOW);
  }

  
  customDelay(1000); // Delay for 1 second
}

