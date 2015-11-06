#include <ros/ros.h>
#include <geometry_msgs/Pose.h> 
#include <math.h>

#define _USE_MATH_DEFINES

int main(int argc, char **argv) {
        //Initializes ROS, and sets up a node
        ros::init(argc, argv, "pose");
        ros::NodeHandle nh;

        //Ceates the publisher, and tells it to publish
        ros::Publisher pub=nh.advertise<geometry_msgs::Pose>("pose/linear", 100);

        //Sets the loop to publish at a rate of 2Hz
	ros::Rate rate(2);
	
	double x=0.0;
	double y=0.0;
	double z=0.0;
	double th=0.0;
	double v=0.1;
	double vx=0.0;
	double vz=0.0;

        while(ros::ok()) 
	{
                //Declares the message to be sent
                geometry_msgs::Pose msg;
		
		
                msg.position.x=x;
                msg.position.y=y;
                msg.position.z=z;
       		//Publish the message
        	pub.publish(msg);

		//Generating new position
		if (10.0 < x && x <= 10.0+10*cos(M_PI/4))
		{
			th=M_PI/4;
		}
		else if (10.0+10*cos(M_PI/4) < x && sqrt(x*x+z*z) <= 20.0+10*cos(M_PI/4))
		{
			th=0;
		}
		else if (20.0+10*cos(M_PI/4) < x && x <= 20.0+20*cos(M_PI/4))
		{
			th=-M_PI/4;
		}
		else if (20.0+20*cos(M_PI/4) < sqrt(x*x+z*z))
		{
			v=0.0;
		}

		vx=v*cos(th);
	        vz=v*sin(th);

		x+=vx*0.5;
		z+=vz*0.5;
		
		//y+=0.1;
		
        	//Delays untill it is time to send another message
        	rate.sleep();
	}
}
