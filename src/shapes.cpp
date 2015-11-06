#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h> 
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <vector>

//Global Variables
double pos_x=0.0;
double pos_z=0.0;
double pos_y=0.0;

void poseCallback(geometry_msgs::Pose msg)
{
  //ROS_INFO("I heard: \nx = [%f]\ny = [%f]\nz = [%f]", msg.position.x, msg.position.y, msg.position.z);
  pos_x=msg.position.x;
  pos_y=msg.position.y;
  pos_z=msg.position.z;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "shapes");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  ros::Subscriber sub = n.subscribe("pose/linear", 1000, poseCallback);

  ros::Rate r(30);

///////////////////////////////////////////////////
//  Initializing Markers
/////////////////////////////////////////////////// 
  // Points initialization
  visualization_msgs::Marker points;
  points.header.frame_id = "/frame";
  points.header.stamp = ros::Time::now();
  points.ns = "traj";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  // Tube initialization
  visualization_msgs::Marker tube;
  tube.header.frame_id = "/frame";
  tube.header.stamp = ros::Time::now();
  tube.ns = "tube";
  tube.action = visualization_msgs::Marker::ADD;
  tube.pose.orientation.w = 1.0;
  tube.id = 1;
  tube.type = visualization_msgs::Marker::CYLINDER;

  // 1 = 1 meter
  points.scale.x = 0.05;
  points.scale.y = 0.05;
  tube.scale.x = 0.3;
  tube.scale.y = 0.3;
  tube.scale.z = 1.0;
  // Color
  points.color.g = 1.0f;
  points.color.a = 1.0;
  tube.color.b = 1.0f;
  tube.color.a = 0.5;

///////////////////////////////////////////////////
//  Initializing Variables
///////////////////////////////////////////////////
  int k = 0;
  int j = 0;
  int cont = 1;
  double list_x[10000];
  double list_y[10000];
  double list_z[10000];
  double tube_list[10000][8];
  double v1[3];
  double v2[3];
  double angle;
  double old_angle = 0.0;
  double sum_angle = 0.0;
  double dist_ini = sqrt(pos_x*pos_x+pos_y*pos_y+pos_z*pos_z);
  double dist_fi = dist_ini;
  double p_ini[3] = {0.0,0.0,0.0};
  double p_fi[3] = {0.0,0.0,0.0};
  bool turn = false;
  bool plot = false;
  // Tube Marker Orientation (set up initial rotation)
  tf::Quaternion q;
  q=tf::createQuaternionFromRPY(0.0,M_PI/2,0.0);
  tube_list[0][4] = q[0];
  tube_list[0][5] = q[1];
  tube_list[0][6] = q[2];
  tube_list[0][7] = q[3];

// First tube orientation

  while (ros::ok())
  {
///////////////////////////////////////////////////
//  List of trajectory points
///////////////////////////////////////////////////    
    if (j == 10001) j = 0;
    
    list_x[j]=pos_x;
    list_y[j]=pos_y;
    list_z[j]=pos_z;

    geometry_msgs::Point p;

    for (int i=0;i<j;i++)
    {
        p.x = list_x[i];
        p.y = list_y[i];
        p.z = list_z[i];
        points.points.push_back(p);
    }
    marker_pub.publish(points);

///////////////////////////////////////////////////
//  Deteting a turn
///////////////////////////////////////////////////
/* The idea comes from considering the last tree obtained points,
 * and mount two vectors with them. Once we have both vectors, the angle
 * between them is computed, so, if the resutlant is 0, there is no turn, 
 * otherwise, there is turn 
*/
    if (j > 1 && turn == false)
    {
        v1[0] = list_x[j]-list_x[j-1];
        v1[1] = list_y[j]-list_y[j-1];
	v1[2] = list_z[j]-list_z[j-1];
	v2[0] = list_x[j-1]-list_x[j-2];
	v2[1] = list_y[j-1]-list_y[j-2];
	v2[2] = list_z[j-1]-list_z[j-2];
	angle = acos((v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2])/(sqrt(v1[0]*v1[0]+v1[1]*v1[1]+v1[2]*v1[2])*sqrt(v2[0]*v2[0]+v2[1]*v2[1]+v2[2]*v2[2])));
	
	ROS_INFO("\nAngle = [%f]", angle);
	if (fabs(angle) == 0.0 && fabs(old_angle) > 0.01)
	{
	    turn = true;
	    old_angle = 0;
	    ROS_INFO("ARA SI");
	}
	else if (fabs(angle) > 0.01) 
	{
	    cont++;
	    old_angle = angle;
	    sum_angle += angle; 
	}
    }
///////////////////////////////////////////////////
//  Getting Tube Marker varaiables
///////////////////////////////////////////////////
/* The tube is genereted every meter. Variables used for this porpous:
 *		
 *	(1)dist_ini
 *	(2)dist_fi
 * 
 * Variable(1) save the distance from the last turn
 * Variable(2) save the total distance traveled
 *
 * The points representing the initial and final part of the tube are:
 *      
 *      p_ini
 *      p_fi
 *
 * Usin this informati on we can obtain the center point and 
 * the scale on z axis of the tube. This variables are updated
 * every meter.
 */
    if (dist_fi-dist_ini >= 1.0 && turn == false) 
    {
	dist_ini = sqrt(pos_x*pos_x+pos_y*pos_y+pos_z*pos_z);
	dist_fi = dist_ini;
	p_fi[0] = list_x[j];
	p_fi[1] = list_y[j];
	p_fi[2] = list_z[j];
	tube_list[k][0] = (p_fi[0]+p_ini[0])/2;
	tube_list[k][1] = (p_fi[1]+p_ini[1])/2;
	tube_list[k][2] = (p_fi[2]+p_ini[2])/2;
	tube_list[k][3] = sqrt(pow(p_fi[0]-p_ini[0],2)+pow(p_fi[1]-p_ini[1],2)+pow(p_fi[2]-p_ini[2],2));
	plot = true;
    }

/* Once a turn is found, then finalization of last tube and initializating variables for a new one
 * It is taken the last point of the line, so the point j-1 of the list. With this point and the 
 * initial point, it is computed the central point of the tube. Finally the length of the tube
 * (tube.scale.z) is computed using teh initial and final point. 
*/
    else if (turn == true)
    {
	tube_list[k][0]	= (list_x[j-cont]+p_ini[0])/2;
	tube_list[k][1]	= (list_y[j-cont]+p_ini[1])/2;
	tube_list[k][2]	= (list_z[j-cont]+p_ini[2])/2;
	tube_list[k][3] = sqrt(pow(list_x[j-cont]-p_ini[0],2)+pow(list_y[j-cont]-p_ini[1],2)+pow(list_z[j-cont]-p_ini[2],2));
	// Next Tube orientation
	q=tf::createQuaternionFromRPY(0.0,sum_angle,0.0);
	tube_list[k+1][4] = q[0];
	tube_list[k+1][5] = q[1];
	tube_list[k+1][6] = q[2];
	tube_list[k+1][7] = q[3];
	dist_ini = sqrt(list_x[j-cont]*list_x[j-cont]+list_y[j-cont]*list_y[j-cont]+list_z[j-cont]*list_z[j-cont]);
	dist_fi = dist_ini;
	p_ini[0] = list_x[j-cont];
	p_ini[1] = list_y[j-cont];
	p_ini[2] = list_z[j-cont];


        plot = true;
	cont = 1;
    }
    else dist_fi = sqrt(pos_x*pos_x+pos_y*pos_y+pos_z*pos_z);

//  Publishing last tub
    if(plot == true)
    {
	tube.id = k+1;
	tube.pose.position.x = tube_list[k][0];
    	tube.pose.position.y = tube_list[k][1];
    	tube.pose.position.z = tube_list[k][2];
    	tube.scale.z = tube_list[k][3];
	tube.pose.orientation.x = tube_list[k][4];
    	tube.pose.orientation.y = tube_list[k][5];
    	tube.pose.orientation.z = tube_list[k][6];
    	tube.pose.orientation.w = tube_list[k][7];
    	marker_pub.publish(tube);
	plot = false;
    }
    
//  If a turn have occured, it is increased the number of tubes (k) and turn becomes false again.
    if (turn == true){k++; turn = false;}
//  Increasing nuber of points of the trajectory (j)
    j++;

    ROS_INFO("\nx = [%f]\ny = [%f]\nz = [%f]", list_x[j],list_y[j],list_z[j]);
    sleep(1);

    if (!ros::ok())
    {
        return 0;
    }
    ros::spinOnce();
  }
}
