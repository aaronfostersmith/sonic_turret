#include <Servo.h>
#include <ros.h>
//a header file for each msg type must be included
#include <sonic_turret/Minimal_PointCloud.h>
#include <std_msgs/Bool.h>

/*
  This program publishes a pointcloud msg with four points every time the turret completes a scan at a new angle
*/


//scan parameters
#define ANGLE_MIN -0.78539816339 //rad
#define ANGLE_MAX 0.78539816339 //rad
#define SCAN_RES 24 //scans per 360 deg
#define MOVE_TIME   .20//s
#define RANGE_MIN 0.02//m
#define RANGE_MAX 4//m

//sensor parameters
#define TRIGPIN1 10 //forward
#define TRIGPIN2 9 //right
#define TRIGPIN3 8 //rear
#define TRIGPIN4 7 //left
#define ECHOPIN1 6
#define ECHOPIN2 5
#define ECHOPIN3 4
#define ECHOPIN4 3
const byte trigpin[4] = {TRIGPIN1, TRIGPIN2, TRIGPIN3, TRIGPIN4}; //make it easy to loop through the trig pins
const byte echopin[4] = {ECHOPIN1, ECHOPIN2, ECHOPIN3, ECHOPIN4};
//servo parameters
#define SERVOPIN 2
Servo myservo;


//instantiate a handle for the node
ros::NodeHandle turretNode;
//publisher
sonic_turret::Minimal_PointCloud cloud_msg;
ros::Publisher cloud_pub("/sweeper/TurretCloud", &cloud_msg);
//subscriber
//callback for on/off boolean
bool onoff = false;
void onoff_cb(const std_msgs::Bool& state_msg) {
  onoff = state_msg.data;
}
ros::Subscriber<std_msgs::Bool> sub_onoff("/sweeper/Turret_On", &onoff_cb );

void setup() {


  //initialize modules
  initComms();

  myservo.attach(SERVOPIN);
  //reset servo
  myservo.write(0);
  delay(2000);
  for (byte y = 0; y < 4; y++) {
    pinMode(trigpin[y], OUTPUT);
    pinMode(echopin[y], INPUT);
  }
}

//initialize ros nodes and advertise Publishers, subcribe to topics
void initComms() {
  turretNode.getHardware() -> setBaud(115200);
  turretNode.initNode();
  ros::Time begin = turretNode.now();

  //advertise publishers
  turretNode.advertise(cloud_pub);
  //subscribe to
  turretNode.subscribe(sub_onoff);
  turretNode.spinOnce();
}



//main loop
void loop() {
  long scanStart = millis();
  //publish to rosserial
  if (onoff) {
    scan_and_publish();
  } else {
    myservo.write(0);
  }
  turretNode.spinOnce(); //communication callbacks are handled here

  //spin until the next scan
  while (millis() - scanStart < MOVE_TIME * 1000) {}

}

void scan_and_publish() {
  cloud_msg.header.frame_id = "/odom";
  cloud_msg.header.stamp = turretNode.now();
  get_Scan();
  cloud_pub.publish(&cloud_msg); //mousePose_msg is published to topic mouse_odo

}


//get scan from all four sensors at turret_angle servo position
float pos = 0;
short dir = 1;
void get_Scan() {

  float m = (RANGE_MIN - RANGE_MAX) / (RANGE_MAX * (RANGE_MAX - RANGE_MIN)); //slope of intensity calculation

  //get a result from each sensor and put it into the scan array
  for (int i = 0; i < 4 ; i++) {

    long duration;
    float distance;

    //query the sensor

    long start = micros();
    digitalWrite(trigpin[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(trigpin[i], LOW);
    delayMicroseconds(10);
    duration = pulseIn(echopin[i], HIGH, 23310);
    //timeout if the duration is too long

    while (micros() - start < 23310) {}

    if (duration > 117)
    {
      distance = duration * 0.0001716; //duration [us]* 343.2 [m/s] /(2 *1000000 [us/s])  [m]
    }
    else distance = 0;

    //calculate the x,y position from angle and distance
    cloud_msg.x[i] = distance * cos(pos + i * 1.5708);
    cloud_msg.y[i] = distance * sin(pos + i * 1.5708);
    if (distance > 0)cloud_msg.intensity[i] = m * (distance - RANGE_MIN) + 1;
    else cloud_msg.intensity[i] = 0;

  }


  //move servo to new position
  pos = pos + dir * (ANGLE_MAX - ANGLE_MIN) / (SCAN_RES / 4);
  pos = constrain(pos, ANGLE_MIN, ANGLE_MAX);
  myservo.write((int)((pos + 0.78539816339) * 57.2957795129));

  //switch direction if we have reached the end point
  if (pos == ANGLE_MAX || pos == ANGLE_MIN) {
    dir = -1 * dir;
  }

}

