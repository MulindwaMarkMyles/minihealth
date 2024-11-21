#define SENSOR_PIN 1 // LM35 connected to Analog pin A1 (ADC1)

float tempC = 0; // Store temperature in Celsius
float tempF = 0; // Store temperature in Fahrenheit

void customDelay(unsigned long delay){
  unsigned long start_time = millis();
  while ((millis() - start_time) < delay)
  {}
}

void setup() {
  // Configure ADC1 (A1) as input using DDRC register
  DDRC &= ~(1 << SENSOR_PIN); // Clear DDC1 bit to set A1 as input
  
  // Configure ADC
  ADMUX = (1 << REFS0);        // AVcc as reference voltage
  ADMUX |= SENSOR_PIN;         // Select ADC1 (A1) as input channel
  ADCSRA = (1 << ADEN)         // Enable ADC
         | (1 << ADPS2)        // Prescaler 64 for 16 MHz clock (250 kHz ADC clock)
         | (1 << ADPS1);       // (Prescaler = 64)
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
  // Serial.print(" °C, ");
  // Serial.print(tempF);
  // Serial.println(" °F");
  
  customDelay(5000); // Delay for 1 second
}

