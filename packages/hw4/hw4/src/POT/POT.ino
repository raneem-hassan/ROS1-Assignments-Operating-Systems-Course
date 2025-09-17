#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

ros::NodeHandle nh;

float z = 0.0;
float tolerance = 0.0;

unsigned long lastBlinkTime = 0;
int blinkInterval = 150;  
bool ledState = false;

bool insideTolerance = false;

void messageCb(const std_msgs::Float32MultiArray& data) {
  z = data.data[0];
  tolerance = data.data[1];
  insideTolerance = (z >= -tolerance && z <= +tolerance);
}

ros::Subscriber<std_msgs::Float32MultiArray> sub("altitude", messageCb);

std_msgs::Float32 POT_value;
ros::Publisher chatter("pot", &POT_value);

void setup() {
  pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
}

void loop() {
  POT_value.data = analogRead(A0);
  chatter.publish(&POT_value);
  nh.spinOnce();

  unsigned long currentMillis = millis();

  if (currentMillis - lastBlinkTime >= 150) {
    lastBlinkTime = currentMillis;

    if (insideTolerance) {
      // Continuous flash
      ledState = !ledState;
      digitalWrite(13, ledState);
    } else {
      // Flash ON then OFF every 150ms
      digitalWrite(13, HIGH);
      delay(150);
      digitalWrite(13, LOW);
    }
  }

  delay(10);  // minor delay for stability
}
