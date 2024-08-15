# TPMSV-1I1
#An EOL Tester for TPMS board

This project is an end-of-line tester for a TPMS (Tire Pressure Monitoring System) receiver board that connects to the vehicle's CAN network. This project is implemented using the STM32F103C8T6 microcontroller.

Features
CAN Network Connection: Communication with the vehicle's CAN network to receive TPMS data.
Current Measurement: Using the INA219 sensor with I2C interface to measure the board's current.
Results Transmission: Transmitting test results via UART with a specific format.
Required Hardware
STM32F103C8T6 microcontroller
INA219 sensor
CAN module (like MCP2515)
UART module (if needed)
Necessary connections and cables
Required Software
STM32CubeIDE or any other IDE that supports STM32.
STM32 libraries for CAN and I2C.
UART library for data transmission.
Installation and Setup

Open the project: Open the project in STM32CubeIDE.
Hardware configuration: Configure the settings for CAN, I2C, and UART in STM32CubeMX.
Compile and load: Compile the project and load it onto the microcontroller.
Usage
After setting up the hardware and loading the code, the tester is ready to use. By sending data from the TPMS sensor, you can observe the current and sensor status via UART.
Contribution
If you would like to contribute to this project, we are happy to hear your suggestions and feedback.

Contact 
For any questions or support, you can reach me at:
Email: mirakimojtaba7@gmail.com
