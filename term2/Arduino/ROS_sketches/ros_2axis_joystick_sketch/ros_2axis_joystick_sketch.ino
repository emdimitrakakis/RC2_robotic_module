#include <ros.h>
#include <geometry_msgs/Point.h>

#define SW 4

geometry_msgs::Point joystick_values;

//define ROS node and publisher
ros::NodeHandle nh;
ros::Publisher pub("read_joystick", &joystick_values);

void setup() {
  //initiate ROS node and publisher  
  nh.initNode();
  nh.advertise(pub);
  
  Serial.begin(9600);
  pinMode(SW,INPUT);
  digitalWrite(SW,HIGH);

}

void loop() {
  Serial.print("VRX_value=");
  int X_value=analogRead(A3);
  Serial.print(X_value);
  Serial.print("\t");

  Serial.print("VRY_value=");
  int Y_value=analogRead(A4);
  Serial.print(Y_value);
  Serial.print("\t");

  Serial.print("Switch_value=");
  int switch_value=digitalRead(SW);
  Serial.print(switch_value);
  Serial.println("\t");

  //transfer the joystic values into the variable to be published
  joystick_values.x = X_value;
  joystick_values.y = Y_value;
  joystick_values.z = switch_value;

  //publish the variable over ROS  
  pub.publish(&joystick_values);
  nh.spinOnce();

  delay(100);

}
