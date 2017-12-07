#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <ras_msgs/RAS_Evidence.h>
#include <ras_project_camera/StringStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <ras_project_brain/ObjPickup_Update.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include <math.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <fstream>
class ObjectAddition
{
	private:
		ros::NodeHandle n;
		ros::Subscriber object_position_sub;
		ros::Subscriber trap_position_sub;
		ros::Subscriber battery_position_sub;
		ros::Subscriber classification_sub;
		ros::Subscriber image_sub;
		ros::Subscriber pickup_sub;
		ros::Subscriber save_sub;
		ros::Subscriber load_sub;
		ros::Publisher load_number_pub;
		ros::Publisher load_coordinates_pub;
		ros::Publisher coord_update_pub;
		ros::Publisher saved_pub;
		ros::Publisher evidence_pub;
		ros::Publisher occupgrid_pub;
		float exit_x_area;
		float exit_y_area;
		std::list<ras_msgs::RAS_Evidence> waiting_objects;
		std::list< std::vector<float> > classified_objects;
		std::string filename;

	public:
	ObjectAddition(): n(ros::NodeHandle())
	{
		//n = ros::NodeHandle();
    		object_position_sub = n.subscribe("/map/objectCoord", 10, &ObjectAddition::objectPositionCallback,this);
		trap_position_sub = n.subscribe("/map/trapCoord", 10, &ObjectAddition::trapPositionCallback, this);
		battery_position_sub = n.subscribe("/map/batteryCoord", 10, &ObjectAddition::batteryPositionCallback, this);
    		classification_sub = n.subscribe("/camera/object_class", 10, &ObjectAddition::classificationCallback, this);
		image_sub = n.subscribe("/camera/object_detected_image", 10, &ObjectAddition::imageCallback, this);
		pickup_sub = n.subscribe("/map/pickupSuccess", 10, &ObjectAddition::pickupCallback, this);
		save_sub = n.subscribe("/object/listSave", 10, &ObjectAddition::saveCallback, this);
		load_sub = n.subscribe("/object/listLoad", 10, &ObjectAddition::loadCallback, this);
		load_number_pub = n.advertise<std_msgs::Int32>("/object/numberLoad", 1);
		load_coordinates_pub = n.advertise<geometry_msgs::Point>("/object/coordinates", 1);
		coord_update_pub = n.advertise<std_msgs::Bool>("/map/coordUpdate", 1);
		saved_pub = n.advertise<std_msgs::Bool>("/object/listSaved", 1);
		evidence_pub = n.advertise<ras_msgs::RAS_Evidence>("/evidence", 1);
		occupgrid_pub = n.advertise<geometry_msgs::PoseStamped>("/object_add", 1);
		filename = "/home/ras11/catkin_ws/src/ras_project/ras_project_object_addition/classified_objects.txt";
		exit_x_area = 0.3;
		exit_y_area = 0.3;
	}
		void objectPositionCallback(const geometry_msgs::TransformStamped::ConstPtr& msg);
		void trapPositionCallback(const geometry_msgs::TransformStamped::ConstPtr& msg);
		void batteryPositionCallback(const geometry_msgs::TransformStamped::ConstPtr& msg);
		void classificationCallback(const ras_project_camera::StringStamped::ConstPtr& msg);
		void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
		void object_add(void);
		int classification_string_to_int(std::string classification);
		float classification_to_radius(int classification);
		void pickupCallback(const ras_project_brain::ObjPickup_Update::ConstPtr& msg);
		void saveCallback(const std_msgs::Bool::ConstPtr& msg);
		void loadCallback(const std_msgs::Bool::ConstPtr& msg);
};

void ObjectAddition::pickupCallback(const ras_project_brain::ObjPickup_Update::ConstPtr& msg)
{
	std::list< std::vector<float> >::iterator it;
	for(it = classified_objects.begin(); it != classified_objects.end(); it++)
	{
		if ((*it)[0] == msg->coord.x && (*it)[1] == msg->coord.y && (*it)[2] == msg->coord.z && msg->pickedUp)
		{
			geometry_msgs::PoseStamped pose;
			pose.header.stamp = ros::Time::now();
			pose.pose.position.x = (*it)[0];
			pose.pose.position.y = (*it)[1];
			pose.pose.position.z = -1.0;
			occupgrid_pub.publish(pose);
			classified_objects.erase(it);
			break;
		}
	}
	return;
}

void ObjectAddition::saveCallback(const std_msgs::Bool::ConstPtr& msg)
{
	std::ofstream file;
	file.open(filename.c_str());
	if (file)
	{
		std::list< std::vector<float> >::iterator it;
		for (it = classified_objects.begin(); it != classified_objects.end(); it++)
		{
			if ((*it)[3] < 17.0)
			{
				file.seekp(0,std::ios::end);
				file<<(*it)[0]<<" "<<(*it)[1]<<" "<<(*it)[2]<<" "<<(*it)[3]<<"\n";
			}
		}
		file.close();
		std_msgs::Bool msg;
		msg.data = true;
		saved_pub.publish(msg);
	}
	else
	{
		std_msgs::Bool msg;
		msg.data = false;
		saved_pub.publish(msg);
	}
	return;
}

void ObjectAddition::loadCallback(const std_msgs::Bool::ConstPtr& msg)
{
	std::ifstream file;
	file.open(filename.c_str());
	if (file)
	{
		std::vector<float> object(4);
		std::string line;
		while(getline(file, line))
		{
			std::stringstream linestream(line);
			linestream >> object[0] >> object[1] >> object[2] >> object[3];
			classified_objects.push_back(object);
		}
	}
	std_msgs::Int32 nmbr_objects;
	nmbr_objects.data = classified_objects.size();
	load_number_pub.publish(nmbr_objects);
	std_msgs::Bool update;
	update.data = true;
	coord_update_pub.publish(update);
	std::list< std::vector<float> >::iterator it;
	for (it = classified_objects.begin(); it != classified_objects.end(); it++)
	{
		geometry_msgs::Point obj;
		obj.x = (*it)[0];
		obj.y = (*it)[1];
		obj.z = (*it)[2];
		load_coordinates_pub.publish(obj);
	}
	update.data = false;
	coord_update_pub.publish(update);
	return;
}

void ObjectAddition::objectPositionCallback(const geometry_msgs::TransformStamped::ConstPtr& msg) {
	//ROS_INFO("Position");
	ros::Time timestamp = msg->header.stamp;
	std::list<ras_msgs::RAS_Evidence>::iterator it;
	for (it = waiting_objects.begin(); it != waiting_objects.end(); it++)
	{
		if (timestamp == it->stamp)
			break;
	}
	if (msg->transform.translation.x <= exit_x_area && msg->transform.translation.y <= exit_y_area)
	{
		if (it != waiting_objects.end())
			waiting_objects.erase(it);
		return;
	}
	for (std::list< std::vector<float> >::iterator it_bis = classified_objects.begin(); it_bis != classified_objects.end(); it_bis++)
	{

		if (std::sqrt(std::pow(msg->transform.translation.x - (*it_bis)[0],2) + std::pow(msg->transform.translation.y - (*it_bis)[1], 2)) < classification_to_radius((*it_bis)[3]) && std::abs(msg->transform.translation.z - (*it_bis)[2]) < 0.02)
		{
			if (it != waiting_objects.end())
			{
				//delete(&(*it));
				waiting_objects.erase(it);
			}
			return;
		}
	}
	if (waiting_objects.front().object_location.header.stamp.toSec() != 0.0 && std::sqrt(std::pow(msg->transform.translation.x - waiting_objects.front().object_location.transform.translation.x,2) + std::pow(msg->transform.translation.y - waiting_objects.front().object_location.transform.translation.y,2)) < classification_to_radius(classification_string_to_int(waiting_objects.front().object_id) && std::abs(msg->transform.translation.z - waiting_objects.front().object_location.transform.translation.z) < 0.02))
	{
		if (it != waiting_objects.end())
			waiting_objects.erase(it);
		return;
	}
	else if (it == waiting_objects.end())
	{
		ras_msgs::RAS_Evidence object;
		object.stamp = timestamp;
		object.object_location = *msg; // May not work, need maybe to clone instead
		object.group_number = 1;
		waiting_objects.push_back(object);
	}
	else
	{
		(*it).object_location = *msg;
		std_msgs::Bool update;
		if (waiting_objects.front().image_evidence.header.stamp.toSec() != 0.0 && waiting_objects.front().object_id != "")
		{
			while (timestamp != waiting_objects.front().stamp)
				waiting_objects.erase(waiting_objects.begin());
			waiting_objects.front().stamp = ros::Time::now();
			update.data = true;
			coord_update_pub.publish(update);
			std::vector<float> object(4);
			object[0] = waiting_objects.front().object_location.transform.translation.x;
			object[1] = waiting_objects.front().object_location.transform.translation.y;
			object[2] = waiting_objects.front().object_location.transform.translation.z;
			object[3] = classification_string_to_int((std::string) waiting_objects.front().object_id);
			classified_objects.push_back(object);
			evidence_pub.publish(waiting_objects.front());
			//delete(&waiting_objects.front());
			waiting_objects.erase(waiting_objects.begin());
			geometry_msgs::PoseStamped object_map;
			object_map.header.stamp = ros::Time::now();
			object_map.pose.position.x = object[0];
			object_map.pose.position.y = object[1];
			object_map.pose.position.z = 1;
			occupgrid_pub.publish(object_map);
		}
		update.data = false;
		coord_update_pub.publish(update);
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
	if (msg->transform.translation.x <= exit_x_area && msg->transform.translation.y <= exit_y_area)
	{
		if (it != waiting_objects.end())
			waiting_objects.erase(it);
		return;
	}
	for (std::list< std::vector<float> >::iterator it_bis = classified_objects.begin(); it_bis != classified_objects.end(); it_bis++)	
	{
		if (std::sqrt(std::pow(msg->transform.translation.x - (*it_bis)[0],2) + std::pow(msg->transform.translation.y - (*it_bis)[1], 2)) < classification_to_radius((*it_bis)[3]))
		{
			if (it != waiting_objects.end())
			{
				//delete(&(*it));
				waiting_objects.erase(it);
			}
			return;
		}
	}
	if (it != waiting_objects.end())
	{
		//delete(&(*it));
		waiting_objects.erase(it);
	}
	std::vector<float> object(4);
	object[0] = msg->transform.translation.x;
	object[1] = msg->transform.translation.y;
	object[2] = msg->transform.translation.z;
	object[3] = 18.0;
	classified_objects.push_back(object);
	geometry_msgs::PoseStamped object_map;
	object_map.header.stamp = ros::Time::now();
	object_map.pose.position.x = object[0];
	object_map.pose.position.y = object[1];
	object_map.pose.position.z = 2.0;
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
	if (msg->transform.translation.x <= exit_x_area && msg->transform.translation.y <= exit_y_area)
	{
		if (it != waiting_objects.end())
			waiting_objects.erase(it);
		return;
	}
	for (std::list< std::vector<float> >::iterator it_bis = classified_objects.begin(); it_bis != classified_objects.end(); it_bis++)	
	{
		if (std::sqrt(std::pow(msg->transform.translation.x - (*it_bis)[0],2) + std::pow(msg->transform.translation.y - (*it_bis)[1], 2)) < classification_to_radius((*it_bis)[3]) && std::abs(msg->transform.translation.z - (*it_bis)[2]) < 0.02)
		{
			if (it != waiting_objects.end())
			{
				//delete(&(*it));
				waiting_objects.erase(it);
			}
			return;
		}
	}
	if (it != waiting_objects.end())
	{
		//delete(&(*it));
		waiting_objects.erase(it);
	}
	std::vector<float> object(4);
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

void ObjectAddition::classificationCallback (const ras_project_camera::StringStamped::ConstPtr& msg) {
	//ROS_INFO("Classification");
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
		(*it).object_id = msg->data;
		std_msgs::Bool update;
		if (waiting_objects.front().image_evidence.header.stamp.toSec() != 0.0 && waiting_objects.front().object_location.header.stamp.toSec() != 0.0)
		{
			while (timestamp != waiting_objects.front().stamp)
				waiting_objects.erase(waiting_objects.begin());
			update.data = true;
			coord_update_pub.publish(update);
			waiting_objects.front().stamp = ros::Time::now();
			std::vector<float> object(4);
			object[0] = waiting_objects.front().object_location.transform.translation.x;
			object[1] = waiting_objects.front().object_location.transform.translation.y;
			object[2] = waiting_objects.front().object_location.transform.translation.z;
			object[3] = classification_string_to_int((std::string) waiting_objects.front().object_id);
			classified_objects.push_back(object);
			//delete(&waiting_objects.front());
			evidence_pub.publish(waiting_objects.front());
			waiting_objects.erase(waiting_objects.begin());
			geometry_msgs::PoseStamped object_map;
			object_map.header.stamp = ros::Time::now();
			object_map.pose.position.x = object[0];
			object_map.pose.position.y = object[1];
			object_map.pose.position.z = 1;
			occupgrid_pub.publish(object_map);
		}
		update.data = false;
		coord_update_pub.publish(update);
	}
	return;
}

void ObjectAddition::imageCallback (const sensor_msgs::Image::ConstPtr& msg) {
	//ROS_INFO("Image");
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
		object.image_evidence = *msg; // May not work, need maybe to clone instead
		object.group_number = 1;
		waiting_objects.push_back(object);
	}
	else
	{
		(*it).image_evidence = *msg;
		std_msgs::Bool update;
		if (waiting_objects.front().object_id != "" && waiting_objects.front().object_location.header.stamp.toSec() != 0.0)
		{
			while (timestamp != waiting_objects.front().stamp)
				waiting_objects.erase(waiting_objects.begin());
			update.data = true;
			coord_update_pub.publish(update);
			waiting_objects.front().stamp = ros::Time::now();
			std::vector<float> object(4);
			object[0] = waiting_objects.front().object_location.transform.translation.x;
			object[1] = waiting_objects.front().object_location.transform.translation.y;
			object[2] = waiting_objects.front().object_location.transform.translation.z;
			object[3] = classification_string_to_int((std::string) waiting_objects.front().object_id);
			classified_objects.push_back(object);
			//delete(&waiting_objects.front());
			evidence_pub.publish(waiting_objects.front());
			waiting_objects.erase(waiting_objects.begin());
			geometry_msgs::PoseStamped object_map;
			object_map.header.stamp = ros::Time::now();
			object_map.pose.position.x = object[0];
			object_map.pose.position.y = object[1];
			object_map.pose.position.z = 1;
			occupgrid_pub.publish(object_map);
		}
		update.data = false;
		coord_update_pub.publish(update);
	}
	return;
}

int ObjectAddition::classification_string_to_int(std::string classification)
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
    ObjectAddition object;
    ros::Rate r(100);
    while(ros::ok()) {
	ros::spinOnce();
	r.sleep();
    }
    return 0;
}

