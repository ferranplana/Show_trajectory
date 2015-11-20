#include <ros/ros.h>
#include <geometry_msgs/Pose.h> 
#include <math.h>
#include <stdlib.h> 

#define _USE_MATH_DEFINES

int main(int argc, char **argv) {
        //Initializes ROS, and sets up a node
        ros::init(argc, argv, "pose");
        ros::NodeHandle nh;

        //Ceates the publisher, and tells it to publish
        ros::Publisher pub=nh.advertise<geometry_msgs::Pose>("pose/linear", 100);
	
        //Sets the loop to publish at a rate of freq Hz
	float freq = 1.0;
	ros::Rate rate(freq);
	
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
	double x2 = 0.0;
	double y2 = 0.0;
	double z2 = 0.0;
	double pitch = 0.0, yaw = 0.0;
	double v = 0.2;
	double vx, vy, vz;
	int dist = 3;

        while(ros::ok()) 
	{
		std::srand(std::time(NULL));
                //Declares the message to be sent
                geometry_msgs::Pose msg;
		
		msg.position.x = x;
                msg.position.y = y;
                msg.position.z = z;
	

       		//Publish the message
        	pub.publish(msg);

		//Generating new position
		if (dist <= sqrt(x2*x2+y2*y2+z2*z2))
		{
			pitch = ((float(rand()) / float(RAND_MAX)) * (M_PI/2)) - M_PI/4;
			yaw = ((float(rand()) / float(RAND_MAX)) * (M_PI/2)) - M_PI/4;

			dist = rand() % 10 + 1;
			

			x2 = 0;
			y2 = 0;
			z2 = 0;
		}


		vx = v*cos(pitch);
	        vz = v*sin(pitch);
		vy = v*sin(yaw);

		x += vx*freq;
		z += vz*freq;
		y += vy*freq;

		x2 += vx*freq;
		y2 += vy*freq;
		z2 += vz*freq;

		//ROS_INFO("\nPitch = [%f]\nYaw = [%f]\nDist = [%f]",pitch,yaw,dist);
	
		
        	//Delays untill it is time to send another message
        	rate.sleep();
	}
}
