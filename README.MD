# MINIHEALTH

**MINIHEALTH** is a modular, Arduino-based project designed to measure vital signs, such as heart rate and temperature, and send alerts if readings fall outside safe limits. The system utilizes multiple Arduinos communicating via SPI, with each board dedicated to a specific task.

---

## Features

- **Heart Rate Monitoring**: Real-time measurement and transmission of heart rate data.
- **Temperature Monitoring**: Regular body temperature readings sent to the main controller.
- **Data Logging**: Long-term storage of readings for analysis.
- **System Status Display**: Aggregated data and system health displayed on an LCD.
- **Modular Design**: Independent operation of each Arduino for better fault tolerance.

---

## System Architecture

The system is divided across four Arduino boards, each with a specific function:

### 1. Arduino A - Main Controller and Data Aggregator
- **Function**: Gathers data from other Arduinos and displays it on an LCD.
- **Components**:
  - LCD Display for real-time data visualization.
  - Status LED for system health.
- **Responsibilities**:
  - Collects data from other Arduinos via SPI.
  - Displays heart rate, temperature, and system status.
  - Tracks communication with other boards using EEPROM.
  - Allows board selection through a reset button.

### 2. Arduino B - Heart Rate Monitoring
- **Function**: Measures heart rate using a sensor and sends readings to Arduino A.
- **Components**:
  - Heart Rate Sensor.
- **Responsibilities**:
  - Reads analog data from the heart rate sensor using ADC.
  - Periodically transmits heart rate data to Arduino A via SPI.

### 3. Arduino C - Temperature Monitoring
- **Function**: Measures body temperature and sends data to Arduino A.
- **Components**:
  - Temperature Sensor.
- **Responsibilities**:
  - Reads temperature data using ADC.
  - Uses timers for periodic measurements.
  - Transmits temperature data to Arduino A via SPI.

### 4. Arduino D - Data Logger and Serial Communication Hub
- **Function**: Logs data received from other Arduinos and coordinates serial communication.
- **Components**:
  - External EEPROM.
- **Responsibilities**:
  - Logs heart rate and temperature data at regular intervals.
  - Sends data logs to Arduino A or a connected computer for analysis.
  - Debounces button presses for controls (e.g., clearing logs).

---

## Data Flow and Communication

- **Heart Rate and Temperature Data**: Sent from Arduino B and Arduino C to Arduino A periodically via SPI.
- **Logged Data**: Stored on Arduino D, retrievable by Arduino A or a connected computer.
- **Main Display**: Arduino A aggregates and displays all data on the LCD.

---

## Key Features and Technology

1. **Analog-to-Digital Conversion (ADC)**:
   - Arduino B for heart rate sensor.
   - Arduino C for temperature sensor.
2. **General Purpose Input/Output (GPIOs)**:
   - Arduino A for controlling the buzzer and display.
3. **Interrupts**:
   - Used for inter-Arduino communication.
4. **Timers and Counters**:
   - Control measurement intervals (e.g., temperature readings).
5. **EEPROM**:
   - Arduino D stores the latest heart rate and temperature readings.
6. **Serial Communication (SPI)**:
   - Facilitates data sharing between Arduinos.
7. **Debouncing**:
   - Ensures reliable button operations on Arduino D.

---

## Additional Notes

- Modular design ensures uninterrupted operation if one Arduino is disconnected.
- SPI communication protocol enables efficient data transfer between boards.
- Reset button on Arduino A simplifies board selection during operations.

---

## How to Use

1. **Setup**:
   - Connect the Arduinos as per their designated roles.
   - Ensure proper wiring for SPI communication and sensor integration.
2. **Operation**:
   - Power on the system.
   - Monitor real-time data on the LCD connected to Arduino A.
3. **Maintenance**:
   - Regularly clear logs using the button on Arduino D.
   - Verify sensor connections and functionality periodically.

---

## Future Improvements

- Integration of additional sensors (e.g., SpO2).
- Wireless communication between Arduinos for enhanced modularity.
- Enhanced UI for data visualization.

---

## License

This project is licensed under the MIT License. See `LICENSE` for details.

