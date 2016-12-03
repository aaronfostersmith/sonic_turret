#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sonic_turret/Minimal_PointCloud.h>


class MapToPointCloud
{
public:
  MapToPointCloud()
  {
    //creat publisher
    pub_ = n_.advertise<sensor_msgs::PointCloud>("/sweeper/TurretCloudFull", 100);

    //subscriber
    sub_ = n_.subscribe("/sweeper/TurretCloud", 1, &MapToPointCloud::callback, this);
  }

  void callback(const sonic_turret::Minimal_PointCloud &input)
  {
    sensor_msgs::PointCloud output;
   
	  //map the Minimal_PointCloud input message to a sensor_msgs/PointCloud msg
	unsigned int num_pts = 4;
	
	  //populate header of output
	output.header.stamp = ros::Time::now();
	output.header.frame_id = "/base_link";
	
	  //set sizes and names
	output.points.resize(num_pts);
	output.channels.resize(1); //only one channel: intensity
	output.channels[0].values.resize(num_pts);
	output.channels[0].name = "intensities";
	  
	  //pull data from input and slap that shit into output
	for(int i = 0; i<num_pts; i++){
		output.points[i].x=input.x[i];
		output.points[i].y=input.y[i];
		output.points[i].z=0;
		output.channels[0].values[i]= input.intensity[i];
	}	
    pub_.publish(output);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class MapToPointCloud



int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "min2PointCloud");

  //Create an object of class SubscribeAndPublish that will take care of everything
  MapToPointCloud CTPCobj;

  ros::spin();

  return 0;
}
