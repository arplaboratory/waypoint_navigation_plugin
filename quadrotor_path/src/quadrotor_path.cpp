#include <ros/ros.h>
#include <Eigen/Geometry>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <memory>

using namespace std;
int BUFFER_SIZE = 100;
nav_msgs::Odometry  * odom_buffer;
int Head = 0;
int Tail = 0;
int current_size = 0;
std::string frame_id="simulator"; //frame id 
nav_msgs::Odometry last_point;
void outputListiner(const nav_msgs::Odometry &msg){
	static bool not_first_read = false;
	//if buffer isn't full check if the last value has the same time stamp
	//if the time stamps are the same then remove it redundent call backs bad. 
	int index = Head -1;
	last_point = msg;
	if(index < 0){
		index +=BUFFER_SIZE;
	}
	// Check distance of last unit 
	if(not_first_read){
		double distance = 0.0;
		nav_msgs::Odometry last_pt = odom_buffer[index];
		distance +=pow((last_pt.pose.pose.position.x- msg.pose.pose.position.x),2);
		distance +=pow((last_pt.pose.pose.position.y- msg.pose.pose.position.y),2);
		distance +=pow((last_pt.pose.pose.position.z- msg.pose.pose.position.z),2);
		if (distance < 0.01){
			return;
		}
		current_size+=1;
	}
	//std::cout << Head <<std::endl;
	odom_buffer[Head] = msg;
	Head=(Head+1)%BUFFER_SIZE;
	if(Head==Tail){
		Tail = (Tail+1)%BUFFER_SIZE;
	}
	not_first_read = true;
	if(current_size > BUFFER_SIZE){
		current_size = BUFFER_SIZE;
	}
}

nav_msgs::Path navmsgsPath(){
	nav_msgs::Path msg; 
	if (current_size < 2){
		return msg;
	}
	msg.header.frame_id = frame_id;
	msg.header.stamp = ros::Time::now();
	geometry_msgs::Quaternion rot;
	rot.x = 0;
	rot.y = 0;
	rot.z = 0;
	rot.w = 1;
	int index = Tail; 
	for (int k = 0; k < current_size-2; k++) {
			geometry_msgs::PoseStamped ps;
			geometry_msgs::Pose pose;
			geometry_msgs::Point point;
			nav_msgs::Odometry odom = odom_buffer[index];
			point.x = odom.pose.pose.position.x;
			point.y = odom.pose.pose.position.y;
			point.z = odom.pose.pose.position.z;
			pose.position = point;
			pose.orientation = rot;
			ps.pose = pose;
			ps.header.frame_id = frame_id;
			ps.header.stamp = ros::Time::now();
			ps.header.seq = k;
			msg.poses.push_back(ps);
			index+=1;
			if(index >= BUFFER_SIZE){
				index -=BUFFER_SIZE;
			}
	}
	return msg;
}

int main(int argc, char **argv)
{
	std::string odom_frame="/quadrotor/odom";

    ros::init(argc,argv,"quadrotor_path");
    ros::NodeHandle n("~");
	n.getParam("/LENGTH", BUFFER_SIZE);
	n.getParam("/frame_name", frame_id);
	n.getParam("/odom_topic", odom_frame);
	odom_buffer = new nav_msgs::Odometry[BUFFER_SIZE];
    ros::Publisher quad_path = n.advertise<nav_msgs::Path> ("quadrotor_path_VIO", 1);
	ros::Publisher current_pos = n.advertise<nav_msgs::Odometry> ("current_pos", 1);

    ros::Subscriber pub_cloudsub = n.subscribe(odom_frame, 10, outputListiner);

	while(ros::ok()) {
		ros::spinOnce();
		// wait for atleast two messages to be published before starting
		nav_msgs::Odometry point_now =  last_point;
		point_now.header.frame_id = frame_id;
		current_pos.publish(point_now);
		if (current_size  > 2){
			quad_path.publish(navmsgsPath());

		}
	}
    return 0;
}
