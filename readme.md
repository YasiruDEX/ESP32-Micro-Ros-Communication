# **ESP32 Micro-ROS**  

This repository contains a **Micro-ROS** boilerplate for ESP32, enabling communication with a ROS 2 system over Wi-Fi. The project implements:  

## **Features**
- **ROS 2 Topics & Messages:**  
  - Publishes `std_msgs/msg/Int16` and `std_msgs/msg/Int16MultiArray`.  
  - Subscribes to `std_msgs/msg/Bool` for LED control and `std_msgs/msg/Int16MultiArray` for data exchange.  
- **Timers & Executors:**  
  - Uses a **50ms timer** to publish sensor data.  
  - Handles message callbacks efficiently with the **rclc executor**.  
- **Wi-Fi Transport:**  
  - Uses **Wi-Fi with Micro-ROS agent** for wireless connectivity.  
- **State Management:**  
  - Implements a state machine to handle connection stability (`WAITING_AGENT`, `AGENT_AVAILABLE`, `AGENT_CONNECTED`, `AGENT_DISCONNECTED`).  

## **Hardware Requirements**  
- **ESP32** (Tested on ESP32-WROOM)  
- **Micro-ROS Agent** running on a ROS 2 system  
- **Wi-Fi connectivity**  

## **Setup & Uploading Code**  
1. Install the **Arduino IDE** and add ESP32 board support.  
2. Install **Micro-ROS for Arduino** (`micro_ros_arduino` library).  
3. Update the Wi-Fi credentials and **ROS 2 agent IP address** in `setup()`.  
4. Connect the ESP32 via USB and **upload the `.ino` file** using the Arduino IDE.  
5. Run the Micro-ROS agent on a ROS 2 system:  
   ```sh
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

	6.	Open the Serial Monitor to check logs.

Uploading Code to ESP32

To flash the provided .ino file to the ESP32, follow these steps:
	1.	Connect ESP32 to your PC via USB.
	2.	Open Arduino IDE.
	3.	Install the ESP32 board support package from the Board Manager.
	4.	Select ESP32 Dev Module under Tools > Board.
	5.	Install the Micro-ROS Arduino library.
	6.	Modify the Wi-Fi credentials in setup():

set_microros_wifi_transports("YourSSID", "YourPassword", "ROS_AGENT_IP", 8888);


	7.	Select the correct COM port under Tools > Port.
	8.	Click Upload (⏫).
	9.	Open Serial Monitor (115200 baud rate) to verify the connection.
