#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <IMU.h>
#include <trajectory_msgs/JointTrajectory.h> // Include the correct message type

ros::NodeHandle nh; // Create a NodeHandle

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu/data", &imu_msg);

// Replace the existing JointState subscriber with JointTrajectory subscriber
void jointTrajectoryCallback(const trajectory_msgs::JointTrajectory& msg); // Callback function prototype
ros::Subscriber<trajectory_msgs::JointTrajectory> joint_trajectory_sub("/joint_group_position_controller/command", &jointTrajectoryCallback);

// Define JointState message and publisher
sensor_msgs::JointState joint_msg;
ros::Publisher joint_state_pub("joint_states", &joint_msg);

cIMU imu;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150
#define SERVOMAX  600
#define NUM_JOINTS 12
#define PI 3.14159265358979323846

float joint_angles[NUM_JOINTS] = {0}; // Array to store joint angles

// Function to convert radians to degrees and round to the nearest whole number
int radiansToDegrees(float radians) {
    return round(radians * (180.0 / PI)); // Convert to degrees and round
}

// Function to set servo angles with new pin mapping
void setServoAngle(int servo_num, int angle) {
    int pulse_length = map(angle, 0, 180, SERVOMIN, SERVOMAX);
   
    // Map the servo numbers to the correct PWM pin assignments
    int pwm_pin;
    if (servo_num >= 0 && servo_num <= 2) {
        pwm_pin = servo_num;
    } else if (servo_num >= 3 && servo_num <= 5) {
        pwm_pin = servo_num + 1;
    } else if (servo_num >= 6 && servo_num <= 8) {
        pwm_pin = servo_num + 2;
    } else if (servo_num >= 9 && servo_num <= 11) {
        pwm_pin = servo_num + 3;
    } else {
        return; // Invalid servo number
    }
    pwm.setPWM(pwm_pin, 0, pulse_length); // Set PWM for mapped pin
}

void jointTrajectoryCallback(const trajectory_msgs::JointTrajectory& msg) {
    if (msg.points_length > 0) {
        for (int i = 0; i < NUM_JOINTS; i++) {
            if (i < msg.points[0].positions_length) {
                float angle = radiansToDegrees(msg.points[0].positions[i]);
                joint_angles[i] = constrain(angle, 0, 180);
            }
        }
    }
}

// Joint names
char* joint_names[12] = {
    "lf_hip_joint", "lf_lower_leg_joint", "lf_upper_leg_joint",
    "lh_hip_joint", "lh_lower_leg_joint", "lh_upper_leg_joint",
    "rf_hip_joint", "rf_lower_leg_joint", "rf_upper_leg_joint",
    "rh_hip_joint", "rh_lower_leg_joint", "rh_upper_leg_joint"
};

// Callback for trajectory_msgs/JointTrajectory
void trajectoryCallback(const trajectory_msgs::JointTrajectory& msg) {
    if (msg.points) {
        const auto& positions = msg.points->positions;

        // Update joint_msg with the positions from the trajectory message
        joint_msg.header.stamp = nh.now();
        joint_msg.position = positions;
        joint_msg.name = joint_names; // Use predefined C-style string array
    }
}

// Initialize subscriber
ros::Subscriber<trajectory_msgs::JointTrajectory> trajectory_sub("/joint_group_position_controller/command", &trajectoryCallback);

void setup() {
    nh.initNode();
    nh.advertise(imu_pub);
    nh.subscribe(joint_trajectory_sub);
    nh.advertise(joint_state_pub);
    imu.begin();
    Serial.begin(57600);
    Serial.println("Adafruit 16 channel PWM/Servo test!");
    pwm.begin();
    pwm.setPWMFreq(60);
    delay(2);
}

void loop() {
    static uint32_t pre_time;
    imu.update();

    // Update servos based on joint angles
    for (int i = 0; i < NUM_JOINTS; i++) {
        setServoAngle(i, joint_angles[i]);
    }

    // Update and publish joint states at a regular interval
    if (millis() - pre_time >= 1000) {
        pre_time = millis();

        // Update IMU data
        imu_msg.header.stamp = nh.now();
        imu_msg.header.frame_id = "imu_link";
        imu_msg.angular_velocity.x = imu.gyroData[0];
        imu_msg.angular_velocity.y = imu.gyroData[1];
        imu_msg.angular_velocity.z = imu.gyroData[2];
        imu_msg.linear_acceleration.x = imu.accData[0];
        imu_msg.linear_acceleration.y = imu.accData[1];
        imu_msg.linear_acceleration.z = imu.accData[2];
        imu_msg.orientation.w = imu.quat[0];
        imu_msg.orientation.x = imu.quat[1];
        imu_msg.orientation.y = imu.quat[2];
        imu_msg.orientation.z = imu.quat[3];
        imu_pub.publish(&imu_msg);

        // Update joint states
        joint_msg.header.stamp = nh.now();
        joint_msg.name = joint_names;

        // Assign current joint angles to the position array
        for (int i = 0; i < NUM_JOINTS; i++) {
            joint_msg.position[i] = joint_angles[i];
        }

        // Publish the joint state message
        joint_state_pub.publish(&joint_msg);
    }

    nh.spinOnce();
    delay(2);
}
