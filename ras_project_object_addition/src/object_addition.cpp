#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ras_msgs/RAS_Evidence.h>
#include <ras_project_camera/StringStamped.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <std_msgs/Bool.h>
class ObjectAddition
{
	private:
		ros::NodeHandle n;
		ros::Subscriber object_position_sub;
		ros::Subscriber trap_position_sub;
		ros::Subscriber battery_position_sub;
		ros::Subscriber classification_sub;
		ros::Subscriber image_sub;
		ros::Publisher evidence_pub;
		ros::Publisher occupgrid_pub;
		ros::Time last_position_stamp;
		std::list<ras_msgs::RAS_Evidence> waiting_objects;
		std::list<float[4]> classified_objects;
		std::string filename;

	public:
	ObjectAddition()
	{
		n = ros::NodeHandle();
    		object_position_sub = n.subscribe("/map/objectCoord", 10, &ObjectAddition::objectPositionCallback,this);
		trap_position_sub = n.subscribe("/map/trapCoord", 10, &ObjectAddition::trapPositionCallback, this);
		battery_position_sub = n.subscribe("/map/batteryCoord", 10, &ObjectAddition::batteryPositionCallback, this);
    		classification_sub = n.subscribe("/camera/object_class", 10, &ObjectAddition::classificationCallback, this);
		image_sub = n.subscribe("/camera/object_detected_image", 10, &ObjectAddition::imageCallback, this);
		evidence_pub = n.advertise<ras_msgs::RAS_Evidence>("/evidence", 1);
		occupgrid_pub = n.advertise<geometry_msgs::PoseStamped>("/object_add", 1);
		filename = "classified_objects.txt";
	}
		void objectPositionCallback(const geometry_msgs::TransformStamped::ConstPtr& msg);
		void trapPositionCallback(const geometry_msgs::TransformStamped::ConstPtr& msg);
		void batteryPositionCallback(const geometry_msgs::TransformStamped::ConstPtr& msg);
		void classificationCallback(const ras_project_camera::StringStamped::ConstPtr& msg);
		void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
		void object_add(void);
		int classification_string_to_int(char* classification);
		float classification_to_radius(int classification);
};

void ObjectAddition::objectPositionCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
	ros::Time timestamp = msg->header.stamp;
	std::list<ras_msgs::RAS_Evidence>::iterator it;
	for (it = waiting_objects.begin(); it != waiting_objects.end(); it++)
	{
		if (timestamp == it->stamp)
			break;
	}
	for (std::list<float[4]>::iterator it_bis = classified_objects.begin(); it_bis != classified_objects.end(); it_bis++)	
	{
		if (std::sqrt(std::pow(msg->transform.translation.x - (*it_bis)[0],2) + std::pow(msg->transform.translation.y - (*it_bis)[1], 2)) < classification_to_radius((*it_bis)[3]) && std::abs(msg->transform.translation.z - (*it_bis)[2]) < 0.02)
		{
			if (it != waiting_objects.end())
			{
				delete(&(*it));
				waiting_objects.erase(it);
			}
			return;
		}
	}
	if (it == waiting_objects.end())
	{
		ras_msgs::RAS_Evidence object;
		object.stamp = timestamp;
		object.object_location = *msg; // May not work, need maybe to clone instead
		object.group_number = 1;
		waiting_objects.push_back(object);
	}
	else
	{
		while (timestamp != waiting_objects.front().stamp)
		{
			delete(&waiting_objects.front()); // Not sure if necessary, may cause error
			waiting_objects.erase(waiting_objects.begin());
		}
		waiting_objects.front().object_location = *msg;
		if (waiting_objects.front().image_evidence && waiting_objects.front().object_id)
		{
			evidence_pub.publish(waiting_objects.front());
			float object[] = new float[4];
			object[0] = waiting_objects.front().object_location.transform.translation.x;
			object[1] = waiting_objects.front().object_location.transform.translation.y;
			object[2] = waiting_objects.front().object_location.transform.translation.z;
			object[3] = classification_string_to_int(waiting_objects.front().object_id);
			classified_objects.push_back(object);
			delete(&waiting_objects.front());
			waiting_objects.erase(waiting_objects.begin());
			geometry_msgs::PoseStamped object_map;
			object_map.header.stamp = ros::Time::now();
			object_map.pose.position.x = object[0];
			object_map.pose.position.y = object[1];
			object_map.pose.position.z = 1;
			occupgrid_pub.publish(object_map);
		}
	}
	return;
}

void ObjectAddition::trapPositionCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
	ros::Time timestamp = msg->header.stamp;
	std::list<ras_msgs::RAS_Evidence>::iterator it;
	for (it = waiting_objects.begin(); it != waiting_objects.end(); it++)
	{
		if (timestamp == it->stamp)
			break;
	}
	for (std::list<float[4]>::iterator it_bis = classified_objects.begin(); it_bis != classified_objects.end(); it_bis++)	
	{
		if (std::sqrt(std::pow(msg->transform.translation.x - (*it_bis)[0],2) + std::pow(msg->transform.translation.y - (*it_bis)[1], 2)) < classification_to_radius((*it_bis)[3]))
		{
			if (it != waiting_objects.end())
			{
				delete(&(*it));
				waiting_objects.erase(it);
			}
			return;
		}
	}
	if (it != waiting_objects.end())
	{
		delete(&(*it));
		waiting_objects.erase(it);
	}
	float object[] = new float[4];
	object[0] = msg->transform.translation.x;
	object[1] = msg->transform.translation.y;
	object[2] = msg->transform.translation.z;
	object[3] = 18.0;
	classified_objects.push_back(object);
	geometry_msgs::PoseStamped object_map;
	object_map.header.stamp = ros::Time::now();
	object_map.pose.position.x = object[0];
	object_map.pose.position.y = object[1];
	object_map.pose.position.z = 2;
	occupgrid_pub.publish(object_map);
	return;
}

void ObjectAddition::batteryPositionCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
	ros::Time timestamp = msg->header.stamp;
	std::list<ras_msgs::RAS_Evidence>::iterator it;
	for (it = waiting_objects.begin(); it != waiting_objects.end(); it++)
	{
		if (timestamp == it->stamp)
			break;
	}
	for (std::list<float[4]>::iterator it_bis = classified_objects.begin(); it_bis != classified_objects.end(); it_bis++)	
	{
		if (std::sqrt(std::pow(msg->transform.translation.x - (*it_bis)[0],2) + std::pow(msg->transform.translation.y - (*it_bis)[1], 2)) < classification_to_radius((*it_bis)[3]) && std::abs(msg->transform.translation.z - (*it_bis)[2]) < 0.02)
		{
			if (it != waiting_objects.end())
			{
				delete(&(*it));
				waiting_objects.erase(it);
			}
			return;
		}
	}
	if (it != waiting_objects.end())
	{
		delete(&(*it));
		waiting_objects.erase(it);
	}
	float object[] = new float[4];
	object[0] = msg->transform.translation.x;
	object[1] = msg->transform.translation.y;
	object[2] = msg->transform.translation.z;
	object[3] = 17.0;
	classified_objects.push_back(object);
	geometry_msgs::PoseStamped object_map;
	object_map.header.stamp = ros::Time::now();
	object_map.pose.position.x = object[0];
	object_map.pose.position.y = object[1];
	object_map.pose.position.z = 3;
	occupgrid_pub.publish(object_map);
	return;
}

void ObjectAddition::classificationCallback (const std_msgs::StringStamped::ConstPtr& msg) {
	ros::Time timestamp = msg->header.stamp;
	std::list<ras_msgs::RAS_Evidence>::iterator it;
	for (it = waiting_objects.begin(); it != waiting_objects.end(); it++)
	{
		if (timestamp == it->stamp)
			break;
	}
	if (it == waiting_objects.end())
	{
		ras_msgs::RAS_Evidence object;
		object.stamp = timestamp;
		object.object_id = msg->data; // May not work, need maybe to clone instead
		object.group_number = 1;
		waiting_objects.push_back(object);
	}
	else
	{
		while (timestamp != waiting_objects.front().stamp)
		{
			delete(&waiting_objects.front()); // Not sure if necessary, may cause error
			waiting_objects.erase(waiting_objects.begin());
		}
		waiting_objects.front().object_id = msg->data;
		if (waiting_objects.front().image_evidence && waiting_objects.front().object_location)
		{
			evidence_pub.publish(waiting_objects.front());
			float object[] = new float[4];
			object[0] = waiting_objects.front().object_location.transform.translation.x;
			object[1] = waiting_objects.front().object_location.transform.translation.y;
			object[2] = waiting_objects.front().object_location.transform.translation.z;
			object[3] = classification_string_to_int(waiting_objects.front().object_id);
			classified_objects.push_back(object);
			delete(&waiting_objects.front());
			waiting_objects.erase(waiting_objects.begin());
			geometry_msgs::PoseStamped object_map;
			object_map.header.stamp = ros::Time::now();
			object_map.pose.position.x = object[0];
			object_map.pose.position.y = object[1];
			object_map.pose.position.z = 1;
			occupgrid_pub.publish(object_map);
		}
	}
	return;
}

void ObjectAddition::imageCallback (const sensor_msgs::Image::ConstPtr& msg) {
	std::list<ras_msgs::RAS_Evidence>::iterator it;
	for (it = waiting_objects.begin(); it != waiting_objects.end(); it++)
	{
		if (timestamp == it->stamp)
			break;
	}
	if (it == waiting_objects.end())
	{
		ras_msgs::RAS_Evidence object;
		object.stamp = timestamp;
		object.image_evidence = *msg; // May not work, need maybe to clone instead
		object.group_number = 1;
		waiting_objects.push_back(object);
	}
	else
	{
		while (timestamp != waiting_objects.front.stamp)
		{
			delete(&waiting_objects.front()); // Not sure if necessary, may cause error
			waiting_objects.erase(waiting_objects.begin());
		}
		waiting_objects.front.image_evidence = *msg;
		if (waiting_objects.front().image_evidence && waiting_objects.front().object_location)
		{
			evidence_pub.publish(waiting_objects.front());
			float object[] = new float[4];
			object[0] = waiting_objects.front().object_location.transform.translation.x;
			object[1] = waiting_objects.front().object_location.transform.translation.y;
			object[2] = waiting_objects.front().object_location.transform.translation.z;
			object[3] = classification_string_to_int(waiting_objects.front().object_id);
			classified_objects.push_back(object);
			delete(&waiting_objects.front());
			waiting_objects.erase(waiting_objects.begin());
			geometry_msgs::PoseStamped object_map;
			object_map.header.stamp = ros::Time::now();
			object_map.pose.position.x = object[0];
			object_map.pose.position.y = object[1];
			object_map.pose.position.z = 1;
			occupgrid_pub.publish(object_map);
		}
	}
	return;
}

int ObjectAddition::classification_string_to_int(char* classification)
{
	if (classification == "An object")
		return 1;
	else if (classification == "Red Cube")
		return 2;
	else if (classification == "Red Hollow Cube")
		return 3;
	else if (classification == "Blue Cube")
		return 4;
	else if (classification == "Green Cube")
		return 5;
	else if (classification == "Yellow Cube")
		return 6;
	else if (classification == "Yellow Ball")
		return 7;
	else if (classification == "Red Ball")
		return 8;
	else if (classification == "Red Cylinder")
		return 9;
	else if (classification == "Green Cylinder")
		return 10;
	else if (classification == "Green Hollow Cube")
		return 11;
	else if (classification == "Blue Triangle")
		return 12;
	else if (classification == "Purple Cross")
		return 13;
	else if (classification == "Purple Star")
		return 14;
	else if (classification == "Orange Cross")
		return 15;
	else if (classification == "Patric")
		return 16;
	else if (classification == "Battery")
		return 17;
	else if (classification == "Booby Trap")
		return 18;
	else
		return 0;
}

float ObjectAddition::classification_to_radius(int classification)
{
	switch(classification)
	{
		case 2:
		case 4:
		case 5:
		case 6:
			return 0.055/2.0;
		case 3:
		case 11:
			return 0.052/2.0;
		case 7:
		case 8:
			return 0.045/2.0;
		case 9:
		case 10:
			return 0.046/2.0;
		case 12:
			return 0.048/2.0;
		case 13:
		case 14:
		case 15:
		case 16:
			return 0.05/2.0;
		case 17:
			return 0.15/2.0;
		case 18:
			return 0.1/2.0;
		default:
			return 0.0;
	}
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_addition");
    PathFollowing P;
    ros::Rate r(100);
    while(ros::ok()) {
	ros::spinOnce();
	r.sleep();
    }
    return 0;
}

