# Arduino Mobile Robot_Arm
Arduino program for driving the mobile robot with a robotic arm.

The mobile robot is built using the following:
1. 6 degrees of freedom robot arm 
https://www.hobbytronics.co.za/Content/external/1084/6-DOF-Robot-Arm-Installation-Diagram.pdf

2. Robotic Tank Chassis 
https://github.com/SmartArduino/XPT/blob/master/SR14.doc

3. Arduino UNO 
https://store.arduino.cc/arduino-uno-rev3	

4. PWM Servo Driver IIC interface PCA9685
http://wiki.sunfounder.cc/index.php?title=PCA9685_16_Channel_12_Bit_PWM_Servo_Driver

5. L298N Dual H Bridge Stepper Motor Driver
https://tronixlabs.com.au/news/tutorial-l298n-dual-motor-controller-module-2a-and-arduino/

6. HC-06 Wireless Serial 4 Pin Bluetooth RF Transceiver 
https://www.instructables.com/id/Tutorial-Using-HC06-Bluetooth-to-Serial-Wireless-U-1/

7. 4 AA Battery pack – 2 sets
https://www.amazon.co.uk/KEESIN-Battery-plastic-Fastening-4-Solts-PCS/dp/B07333LC7N

The 6 motors of the robotic arm are connected to the PCA9685, and the PCA9685 is connected to the Arduino Uno via the A4 (SDA), A5 (SCL), GND and 3.3V. The V+ and power of the PCA9685 and the 5V of the Arduino Uno are connected to the 6V power of the battery pack, and all GNDs are connected to the battery pack ground as well.
Please note that the 2 battery packs can be connected in parallel to provide more battery power to the robot.
The 2 motors of the Robotic Tank Chassis are connected and driven by the L298N Dual H Bridge Stepper Motor Driver, and the L298N is connected to the Arduino Uno via I/O port 2, 3, 4, and 5 to control the direction of movements of each motor.
The HC06 is connected to the Arduino Uno to enable wireless bluetooth connection. The HC06 is connected to the Arduino Uno via port 8 (TXD) and port 9 (RXD).

This program implements the interface to the PCA9685 to control the robotic arm, and it also enables the control of the I/O ports 2-5 to control the movement of the motors of the Robotic tank Chassis. In addition, it interfaces with the HC06 to receive remote commands wirelessly via bluetooth. 
