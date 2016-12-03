#include <Servo.h>
#include <ros.h>
//a header file for each msg type must be included
#include <sensor_msgs/LaserScan.h>

/*
  This program publishes a laserscan msg with a full 360 deg scan from the turret
*/


//scan parameters
#define ANGLE_MIN 0 //rad
#define ANGLE_MAX 6.2831853 //rad
#define SCAN_RES 24 //scans per 360 deg
#define TIME_INCREMENT  0.025//s
#define SCAN_TIME 0.2//s
#define MOVE_TIME 0.1 //s
#define RANGE_MIN 0.02//m
#define RANGE_MAX 4//m

//sensor parameters
#define TRIGPIN1 2
#define ECHOPIN1 3

//servo parameters
#define SERVOPIN 4
Servo myservo;

void setup() {

  //initialize modules
  initComms();

  myservo.attach(SERVOPIN);
  pinMode(TRIGPIN1, OUTPUT);
  pinMode(ECHOPIN1, INPUT);

}



//main loop
void loop() {
  long scanStart = millis();
  //publish to rosserial

  scan_and_publish();

  //spin until the next scan
  while (millis() - scanStart < (SCAN_TIME + MOVE_TIME) * 1000) {}

}

//get scan from all four sensors at turret_angle servo position
void get_Scan(float *scan) {


  //for each position needed
  for (int posnum = 0; posnum < SCAN_RES / 4; posnum++) {


    //get a result from each sensor and put it into the scan array
    for (int i = 0; i < 4 ; i++) {
      long duration = 0;
      float distance;
      //query the sensor
      digitalWrite(TRIGPIN1, HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIGPIN1, LOW);
      duration = pulseIn(ECHOPIN1, HIGH); //TODO REPLACE THIS WITH A SWITCH ON i TO SELECT CORRECT PIN

      //timeout if the duration is too long
      long start = micros();
      while (micros() - start < 23310) {}

      if (duration > 117)
      {
        distance = duration * 0.0001716; //duration [us]* 343.2 [m/s] /(2 *1000000 [us/s])  [m]
      }
      else distance = 0;

      scan[posnum + i * SCAN_RES / 4] = distance;


    }
    //move servo to new position
    myservo.write(posnum * (ANGLE_MAX / SCAN_RES) * (180 / 3.1415));
    delay(MOVE_TIME * 1000);
  }
  return scan;
}



//instantiate a handle for the node
ros::NodeHandle turretNode;


//publisher
sensor_msgs::LaserScan scan_msg;
ros::Publisher scan_pub("/sweeper/Range", &scan_msg);

//initialize ros nodes and advertise Publishers, subcribe to topics
void initComms() {
  turretNode.getHardware() -> setBaud(115200);
  turretNode.initNode();
  ros::Time begin = turretNode.now();

  //advertise publishers
  turretNode.advertise(scan_pub);

  turretNode.spinOnce();
}


void scan_and_publish() {
  float thisScan[SCAN_RES];

  scan_msg.header.frame_id = "/odom";
  scan_msg.header.stamp = turretNode.now();
  scan_msg.angle_min = ANGLE_MIN;
  scan_msg.angle_max = ANGLE_MAX;
  scan_msg.range_min = RANGE_MIN;
  scan_msg.range_max = RANGE_MAX;
  scan_msg.angle_increment = ANGLE_MAX / SCAN_RES;
  scan_msg.time_increment = TIME_INCREMENT;
  scan_msg.scan_time = SCAN_TIME + MOVE_TIME;
  scan_msg.ranges_length = SCAN_RES;

  get_Scan(thisScan);
  scan_msg.ranges = thisScan;

  scan_pub.publish(&scan_msg); //mousePose_msg is published to topic mouse_odo

  turretNode.spinOnce(); //communication callbacks are handled here

}



