#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>
#include "eeprom_utils.h"
#include <micro_ros_arduino.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/temperature.h>
#include <std_msgs/msg/float32.h>

MPU9250 mpu;

// ROS2 entities
rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_timer_t timer;

// Macros for checking return of ROS2 functions and entering an infinite error loop in case of error
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
// Infinite error loop function. If something fails, the device will get stuck here

void error_loop() {
  while(1) {
    delay(100);
  }
}

// Publishers
rcl_publisher_t imu_pub;
rcl_publisher_t mag_pub;
rcl_publisher_t temp_pub;
rcl_publisher_t heading_pub;

// Messages
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__MagneticField mag_msg;
sensor_msgs__msg__Temperature temp_msg;
std_msgs__msg__Float32 heading_msg;

// Constants
const float declination_rad = 0.02234021; // Karachi declination
const float G_TO_MS2 = 9.80665;
const float UT_TO_T = 1e-6;

// Fix 1: Use existing DEG_TO_RAD definition from Arduino.h
// Remove the custom DEG_TO_RAD definition

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    (void)last_call_time;
    if (timer != NULL) {
        int64_t time_stamp = rmw_uros_epoch_millis();
        
        // IMU Message
        imu_msg.header.stamp.sec = time_stamp / 1000;
        imu_msg.header.stamp.nanosec = (time_stamp % 1000) * 1e6;
        
        // Fix 2: Proper string assignment for frame_id
        static const char frame_id[] = "imu_link";
        imu_msg.header.frame_id.data = (char*)frame_id;
        imu_msg.header.frame_id.size = sizeof(frame_id);
        imu_msg.header.frame_id.capacity = sizeof(frame_id);

        // Quaternion
        imu_msg.orientation.x = mpu.getQuaternionX();
        imu_msg.orientation.y = mpu.getQuaternionY();
        imu_msg.orientation.z = mpu.getQuaternionZ();
        imu_msg.orientation.w = mpu.getQuaternionW();

        // Angular velocity (convert deg/s to rad/s)
        imu_msg.angular_velocity.x = mpu.getGyroX() * DEG_TO_RAD;
        imu_msg.angular_velocity.y = mpu.getGyroY() * DEG_TO_RAD;
        imu_msg.angular_velocity.z = mpu.getGyroZ() * DEG_TO_RAD;

        // Linear acceleration (convert g to m/s²)
        imu_msg.linear_acceleration.x = mpu.getLinearAccX() * G_TO_MS2;
        imu_msg.linear_acceleration.y = mpu.getLinearAccY() * G_TO_MS2;
        imu_msg.linear_acceleration.z = mpu.getLinearAccZ() * G_TO_MS2;

        // Covariances (unknown)
        memset(imu_msg.orientation_covariance, 0, sizeof(imu_msg.orientation_covariance));
        memset(imu_msg.angular_velocity_covariance, 0, sizeof(imu_msg.angular_velocity_covariance));
        memset(imu_msg.linear_acceleration_covariance, 0, sizeof(imu_msg.linear_acceleration_covariance));

        // Fix 3: Use proper error checking macro
        RCCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));

        // Magnetometer (convert uT to T)
        float mx = mpu.getMagX();
        float my = mpu.getMagY();
        mag_msg.header = imu_msg.header;
        mag_msg.magnetic_field.x = mx * UT_TO_T;
        mag_msg.magnetic_field.y = my * UT_TO_T;
        mag_msg.magnetic_field.z = mpu.getMagZ() * UT_TO_T;
        memset(mag_msg.magnetic_field_covariance, 0, sizeof(mag_msg.magnetic_field_covariance));

        RCCHECK(rcl_publish(&mag_pub, &mag_msg, NULL));
        RCCHECK(rcl_publish(&mag_pub, &mag_msg, NULL));

        // Temperature
        temp_msg.header = imu_msg.header;
        temp_msg.temperature = mpu.getTemperature();
        temp_msg.variance = 0.0;

        RCCHECK(rcl_publish(&temp_pub, &temp_msg, NULL));

        // Heading calculation
        float heading_rad = atan2(my, mx) + declination_rad;
        if (heading_rad < 0) heading_rad += 2 * M_PI;
        heading_msg.data = heading_rad * (180.0 / M_PI);

        RCCHECK(rcl_publish(&heading_pub, &heading_msg, NULL));
    }
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    Wire.setClock(400000);  // I2C fast mode (400kHz)
    delay(2000);

    if (!mpu.setup(0x68)) {
        Serial.println("MPU9250 connection failed");
        while(1);
    }

    loadCalibration();

    // Initialize micro-ROS
    set_microros_transports();
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, "mpu9250_node", "", &support));

    // Create publishers
    RCCHECK(rclc_publisher_init_default(
        &imu_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), 
        "imu/data"
    ));

    RCCHECK(rclc_publisher_init_default(
        &mag_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField), 
        "mag/data"
    ));

    RCCHECK(rclc_publisher_init_default(
        &temp_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature), 
        "temp"
    ));

    RCCHECK(rclc_publisher_init_default(
        &heading_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), 
        "heading"
    ));

    // Create timersetDlpfBandwidth
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(10),
        timer_callback
    ));

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    Serial.println("ROS2 MPU9250 node initialized!");
}

void loop() {
    mpu.update();
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    // delay(10);
}