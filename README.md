# ros2_imu

---

## 1. Set Up the ROS2 Environment
Before proceeding with the ESP32 and Micro-ROS setup, ensure that your ROS2 environment is properly configured. Follow these steps if you haven't done so already.

### 1.1 Install ROS2
Follow the official [ROS2 installation guide](https://docs.ros.org/en/rolling/Installation.html) for your operating system. </br>
For example, for Ubuntu 22.04, the commands would be:
```bash
sudo apt update
sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://get.rvm.io | bash -s stable --rails
sudo apt install ros-humble-desktop
```

### 1.2 Set Up ROS2 Workspace
Create a new workspace in your home directory if you don’t have one already:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```
Then build the workspace:
```bash
colcon build
source install/setup.bash
```

## 2. Install Micro-ROS on ESP32
### 2.1 Install the ESP32 Board Support in Arduino IDE
- Open Arduino IDE.
- Go to **File → Preferences** and add the ESP32 board manager URL:
``` https://dl.espressif.com/dl/package_esp32_index.json```
- Go to **Tools → Board → Board Manager**, search for **ESP32** by **Espressif Systems** and install it.
2.2 Install Required Libraries
You will need several libraries for the Micro-ROS and sensor communication:
- **Micro-ROS Arduino:** This enables communication between ESP32 and ROS2.
  - Install via Arduino IDE Library Manager: **Tools → Manage Libraries → Search for "micro_ros_arduino".**
- **MPU9250 Library:** For interfacing with the MPU9250 IMU sensor.
  - Install via Arduino IDE Library Manager: **Tools → Manage Libraries → Search for "MPU9250".**

## 3. Upload Code to ESP32
### 3.1 Code Overview
The provided code initializes the MPU9250 sensor, reads the IMU data, and publishes it via Micro-ROS to the ROS2 system.
- The code uses Micro-ROS for communication and MPU9250 for IMU data.
- It publishes IMU data, magnetometer data, temperature data, and heading information.
  
### 3.2 Modify the Code for Your Setup
Ensure that you modify the following in the code:
- Update the MPU9250 sensor I2C address (0x68 by default).
- Change the declination angle to your location's magnetic declination.
- Use appropriate topics for your data in the ROS2 system.
### 3.3 Upload Code
Upload the Arduino code to your ESP32 using the Arduino IDE:
1. Select your ESP32 board from the Tools → Board menu.
2. Select the correct port under Tools → Port.
3. Click Upload.

## 5. Build Your ROS2 Workspace
Once the code and launch file are ready, it’s time to build the workspace.
### 5.1 Build the Workspace
Go to the workspace root and build the workspace:
```bash
cd ~/ros2_ws/src
```
### 5.2 Clone the repository
```bash
git clone <repository-url>
```
### 5.3 Install dependencies 
For the project, we can check the README.md or package.xml file in the cloned repository for specific dependencies. Usually, it's done with:
```bash
rosdep install --from-paths src --ignore-src -r -y
```
### 5.4 Build the workspace
```bash
cd ~/ros2_ws
colcon build --symlink-install
```
### 5.5 Source the workspace
```bash
source ~/ros2_ws/install/setup.bash
```

## 6. Run the System
### 6.1 Run the Launch File
Start your system by launching the ROS2 nodes:
```bash
ros2 launch your_package imu_launch.py
```
This will launch the necessary transforms and data publishers for the IMU, magnetometer, temperature, and heading information.
### 6.2 Check the Data
You can check the topics to verify that data is being published correctly:
```bash
ros2 topic list
```
For example, you should see topics like:
```
- /imu/data
- /mag/data
- /temp
- /headingros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

## 7. Troubleshooting
### 7.1 Common Issues
- I2C Communication Failure:** If you see I2C errors, ensure that the ESP32 and MPU9250 are correctly wired, and the I2C address is properly configured in the code.
- **No Data on Topics:** If no data is being published, check the serial monitor for any error messages. Ensure that Micro-ROS is set up correctly and the ESP32 is properly communicating with ROS2.

## 8. Additional Notes
### 8.1 Micro-ROS
- Micro-ROS is a powerful library that allows microcontrollers (like ESP32) to communicate with ROS2 systems. It helps in reducing the complexity of integrating embedded systems with ROS2, providing a simple yet efficient way to exchange sensor data.
- You can find more information on [Micro-ROS](https://micro.ros.org/).
