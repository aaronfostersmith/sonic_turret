#include <Servo.h>
#include <ros.h>
//a header file for each msg type must be included
#include <sensor_msgs/PointCloud.h>

/*
  This program publishes a pointcloud msg with four points every time the turret completes a scan at a new angle
*/


//scan parameters
#define ANGLE_MIN -0.78539816339 //rad
#define ANGLE_MAX 0.78539816339 //rad
#define SCAN_RES 24 //scans per 360 deg
#define MOVE_TIME  1//s
#define RANGE_MIN 0.02//m
#define RANGE_MAX 4//m

//sensor parameters
#define TRIGPIN1 2
#define TRIGPIN2 3
#define TRIGPIN3 4
#define TRIGPIN4 5
#define ECHOPIN 6

//servo parameters
#define SERVOPIN 7
Servo myservo;


//instantiate a handle for the node
ros::NodeHandle turretNode;
//publisher
sensor_msgs::PointCloud cloud_msg;
ros::Publisher cloud_pub("/sweeper/TurretCloud", &cloud_msg);

void setup() {


  //initialize modules
  initComms();


  myservo.attach(SERVOPIN);
  //reset servo
  myservo.write(0);
  delay(2000);

  pinMode(TRIGPIN1, OUTPUT);
  pinMode(ECHOPIN, INPUT);

}

//initialize ros nodes and advertise Publishers, subcribe to topics
void initComms() {
  turretNode.getHardware() -> setBaud(115200);
  turretNode.initNode();
  ros::Time begin = turretNode.now();

  //advertise publishers
  turretNode.advertise(cloud_pub);

  turretNode.spinOnce();
}

//main loop
void loop() {
  long scanStart = millis();
  //publish to rosserial

  scan_and_publish();

  //spin until the next scan
  while (millis() - scanStart < MOVE_TIME * 1000) {}

}

void scan_and_publish() {
  cloud_msg.header.frame_id = "/odom";
  cloud_msg.header.stamp = turretNode.now();
  get_Scan();
  cloud_pub.publish(&cloud_msg); //mousePose_msg is published to topic mouse_odo
  turretNode.spinOnce(); //communication callbacks are handled here
}


//get scan from all four sensors at turret_angle servo position
float pos = 0;
short dir = 1;
void get_Scan() {

  //get a result from each sensor and put it into the scan array
  for (int i = 0; i < 4 ; i++) {

    long duration;
    float distance;

    //query the sensor
    digitalWrite(TRIGPIN1, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGPIN1, LOW);
    duration = pulseIn(ECHOPIN, HIGH); //TODO REPLACE THIS WITH A SWITCH ON i TO SELECT CORRECT PIN

    //timeout if the duration is too long
    long start = micros();
    while (micros() - start < 23310) {}

    if (duration > 117)
    {
      distance = duration * 0.0001716; //duration [us]* 343.2 [m/s] /(2 *1000000 [us/s])  [m]
    }
    else distance = 0;

    //calculate the x,y position from angle and distance
    cloud_msg.points[i].x = distance * cos(pos);
    cloud_msg.points[i].y = distance * sin(pos);
    cloud_msg.points[i].z = 0;

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

