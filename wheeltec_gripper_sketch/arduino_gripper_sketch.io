#include <ros.h>
#include <std_msgs/Float32.h>
#include <AccelStepper.h>

// Ros node handle
ros::NodeHandle nh;

AccelStepper stepperX(1, 8, 9);
#define home_switch 2

const double degreesPerStep = 0.000340909090909091;
long initial_homing = -1;

// ROS Callback function to handle incoming commands
void commandCallback(const std_msgs::Float32& cmd_msg) {
    double radians = cmd_msg.data; // Get the position in radians
    int steps = int(radians / degreesPerStep); // Convert degrees to stepper steps
    stepperX.moveTo(steps);
    stepperX.runToPosition(); // Move stepper to new position

}

// ROS Subscriber
ros::Subscriber<std_msgs::Float32> sub("wheeltec_gripper_position_requested", &commandCallback);

// ROS Publisher
std_msgs::Float32 pos_msg;
ros::Publisher pub("wheeltec_gripper_position_actual", &pos_msg);

void setup() {
    nh.initNode();
    nh.subscribe(sub);
    nh.advertise(pub);

    pinMode(home_switch, INPUT_PULLUP);
    stepperX.setMaxSpeed(1000.0);
    stepperX.setAcceleration(1000.0);

    // Home the stepper at startup
    while (digitalRead(home_switch)) {
        stepperX.moveTo(initial_homing);
        initial_homing--;
        stepperX.run();
        delay(5);
    }
    stepperX.setCurrentPosition(0);

    Serial.begin(9600);
    Serial.println("Setup Complete: Stepper is homed.");
}

void loop() {
    if (stepperX.distanceToGo() == 0) {
        float currentRadians = stepperX.currentPosition() * degreesPerStep;
        pos_msg.data = currentRadians;
        pub.publish(&pos_msg);
    }
    stepperX.run();
    nh.spinOnce();
}