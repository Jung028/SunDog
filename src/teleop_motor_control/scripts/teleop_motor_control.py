#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

void messageCb(const std_msgs::String& msg) {
    Serial.println(msg.data.c_str());
}

ros::Subscriber<std_msgs::String> sub("arduino_topic", &messageCb);

void setup() {
    Serial.begin(9600);
    nh.initNode();
    nh.subscribe(sub);
}

void loop() {
    nh.spinOnce();
    delay(100);
}
