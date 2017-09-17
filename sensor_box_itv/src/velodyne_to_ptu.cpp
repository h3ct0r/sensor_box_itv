#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
   
int main(int argc, char** argv){
	ros::init(argc, argv, "velodyne_to_ptu");
	ros::NodeHandle node;
    
	tf::TransformBroadcaster br;
	tf::Transform transform;
   
	ros::Rate rate(10.0);
	while (node.ok()){
		transform.setOrigin( tf::Vector3(0.0, 0.0, 0.03) ); // velodyne is 3 cm at the top of the ptu_mount_link
		transform.setRotation( tf::Quaternion(0, 0, -0.6816388, 0.7316889) ); // the color pointcloud of the velodyne is aligned -90 degress in Z
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "ptu_mount_link", "velodyne"));
		rate.sleep();
	}
	return 0;
};