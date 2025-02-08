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

    # **Setting Up Micro-ROS Agent on ROS 2 Humble**  

Follow these steps to set up the **Micro-ROS Agent** on **ROS 2 Humble**, enabling communication with an **ESP32 running Micro-ROS**.  

---

## **1️⃣ Source ROS 2 Environment**  
Make sure your **ROS 2 Humble** environment is set up:  
```sh
source /opt/ros/$ROS_DISTRO/setup.bash

2️⃣ Create and Build the Micro-ROS Workspace

mkdir uros_ws && cd uros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash

3️⃣ Build the Micro-ROS Agent

ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh

4️⃣ Run the Micro-ROS Agent (UDP over IPv6)

To launch the Micro-ROS Agent using UDP6 on port 8888, run:

ros2 run micro_ros_agent micro_ros_agent udp6 -p 8888

📌 Notes
	•	Ensure your ESP32 Micro-ROS node is configured to communicate via UDP6.
	•	The Micro-ROS Agent bridges the ESP32 and ROS 2 Humble, enabling topic communication.
	•	If using a different network setup, you may need to modify the transport settings in your ESP32 code.
	•	Use IPv4 (udp4) instead of IPv6 (udp6) if your network does not support IPv6.

🚀 You’re now ready to communicate between ESP32 and ROS 2!


