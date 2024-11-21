

---

# Vital Sign Monitoring System with Arduino On a Budget 

## Project Overview  
This project is designed to measure vital signs, such as heart rate and temperature, and send alerts when readings are outside safe limits. The system is built on a network of four Arduinos, each assigned specific tasks, and communicates using serial protocols. One Arduino acts as the main controller, while the others handle individual sensors and data logging tasks.  

## Features  
- **Real-time Monitoring:** Measures heart rate and temperature.  
- **Alerts and Notifications:** Triggers alerts for abnormal readings.  
- **Data Logging:** Records vital sign data for analysis.  
- **Serial Communication:** Ensures smooth communication between Arduinos.  
- **Customizable Thresholds:** Allows user-defined safe ranges stored in EEPROM.  

---

## System Architecture  

### Task Breakdown Across Arduinos  

#### 1. **Arduino A - Main Controller and Data Aggregator**  
- **Function:**  
  - Aggregates data from other Arduinos.  
  - Displays readings on an OLED.  
  - Triggers alerts when readings exceed safe thresholds.  
- **Key Components:**  
  - LCD Display  
  - Buzzer or Speaker  
- **Responsibilities:**  
  - Collects heart rate and temperature data via serial communication.  
  - Displays readings and alert messages.  
  - Stores and manages threshold values in EEPROM.  
  - Handles reset functionality with switch debouncing.  

#### 2. **Arduino B - Heart Rate Monitoring**  
- **Function:**  
  - Measures heart rate using a dedicated sensor.  
  - Sends readings to Arduino A.  
- **Key Components:**  
  - Heart Rate Sensor (e.g., Pulse Sensor)  
- **Responsibilities:**  
  - Reads analog data using ADC.  
  - Calculates BPM using counters and interrupts.  
  - Transmits heart rate data via serial communication.  

#### 3. **Arduino C - Temperature Monitoring**  
- **Function:**  
  - Measures body temperature.  
  - Sends readings to Arduino A.  
- **Key Components:**  
  - Temperature Sensor (e.g., LM35 or DS18B20)  
- **Responsibilities:**  
  - Reads temperature data using ADC.  
  - Regularly transmits data via serial communication.  
  - Stores critical thresholds in EEPROM.  

#### 4. **Arduino D - Data Logger and Serial Communication Hub**  
- **Function:**  
  - Logs all sensor data for long-term storage.  
  - Acts as a communication hub if needed.  
- **Key Components:**  
  - SD Card Module or External EEPROM  
- **Responsibilities:**  
  - Logs data with timestamps.  
  - Relays logged data to Arduino A or an external device.  

---

## Data Flow and Communication  
- **Serial Communication:**  
  - Arduino B and Arduino C periodically send sensor data to Arduino A.  
  - Arduino D logs and optionally relays data to Arduino A or a computer.  
- **Error Handling:**  
  - Implements error-checking mechanisms for reliable communication.  

---

## Core Functionalities  

1. **ADC:** Reads analog signals from sensors (Heart Rate Sensor on Arduino B and Temperature Sensor on Arduino C).  
2. **Interrupts:** Detects heart rate pulses for accurate BPM calculation.  
3. **Timers:** Controls measurement intervals (e.g., temperature every few seconds).  
4. **EEPROM:** Stores user-defined thresholds and settings.  
5. **Switch Debouncing:** Stabilizes button inputs for resetting and clearing alerts.  

---

## Hardware Requirements  
- **Arduino Boards:** 4 units  
- **Sensors:**  
  - Heart Rate Sensor (Pulse Sensor or similar)  
  - Temperature Sensor (LM35, DS18B20, etc.)  
- **Display:** LCD Display  
- **Data Storage:** SD Card Module or External EEPROM  
- **Other Components:**  
  - Buzzer/Speaker  
  - Push Buttons for Reset  
  - Power Supply (e.g., Battery Pack)  

---

## Getting Started  

### 1. **Setup**  
- Connect the components to their respective Arduino boards.  
- Ensure proper wiring for serial communication.  

### 2. **Programming**  
- Upload the individual sketches to each Arduino based on their assigned roles.  

### 3. **Operation**  
- Power on the system.  
- Monitor data on the LCD display.  
- Alerts will trigger if readings exceed thresholds.  

---

## Future Improvements  
- Integration with IoT for remote monitoring.  
- Adding more sensors (e.g., SpO2).  
- Enhancing error-handling mechanisms.  

---

## Contribution  
Feel free to contribute to the project by submitting pull requests or reporting issues.  

--- 

