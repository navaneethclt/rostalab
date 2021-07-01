#include <cmath>
#include <algorithm>
#include <smb_highlevel_controller/SmbHighlevelController.hpp>
#include <geometry_msgs/Twist.h>
namespace smb_highlevel_controller {

SmbHighlevelController::SmbHighlevelController(ros::NodeHandle &nodeHandle) :
		nodeHandle_(nodeHandle), subscriberQueueSize_(10), scanTopic_("/scan"),count_(0){
	if (!readParameters()) {
		ROS_ERROR("Could not read parameters.");
		ros::requestShutdown();
	}
	
	scanSubscriber_ = nodeHandle_.subscribe(scanTopic_, subscriberQueueSize_,
			&SmbHighlevelController::scanCallback, this);
	EulerPosSubscriber_ = nodeHandle_.subscribe("/EulerPos", subscriberQueueSize_,
			&SmbHighlevelController::EulerPosCallback, this);
	cmdPublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel",1000);
	
}

SmbHighlevelController::~SmbHighlevelController() {
}

void SmbHighlevelController::EulerPosCallback(const geometry_msgs::Twist& msg){
this->pos_x = msg.linear.x;
this->pos_y = msg.linear.y;
this->pos_z = msg.linear.z;
this->pos_yaw = msg.angular.z;
this->pos_pitch = msg.angular.x;
this->pos_roll = msg.angular.y;
}

bool SmbHighlevelController::readParameters() {
	bool success = true;
	success &= nodeHandle_.getParam(
			"/smb_highlevel_controller/scan_subscriber_topic_name", scanTopic_);
	success &= nodeHandle_.getParam(
			"/smb_highlevel_controller/scan_subscriber_queue_size",
			subscriberQueueSize_);
	return success;
}



void SmbHighlevelController::scanCallback(
		const sensor_msgs::LaserScan::ConstPtr &msg) {
	double min = msg->range_max;
	for (int i = 0; i < msg->ranges.size(); ++i) {
		if (msg->ranges[i] < min)
			min = msg->ranges[i];
	}
        geometry_msgs::Twist cmdmsg;
        if(this->count_==0 && min >1)
        {cmdmsg.linear.x = 0.05;
        cmdmsg.angular.z = 0.0;}
        
        if(this->count_==0 && min<1)
        {cmdmsg.linear.x = 0.0;
        cmdmsg.angular.z = 0.0;
        this->count_ = this->count_+ 1;
        }
        
         if(this->count_==1 && (abs(this->pos_yaw-1.57)>0.2))
        {cmdmsg.linear.x = 0.0;
        cmdmsg.angular.z = 0.05;
        
        }
        
        if(this->count_==1 && (abs(this->pos_yaw-1.57)<0.2))
        {cmdmsg.linear.x = 0.0;
        cmdmsg.angular.z = 0.0;
        this->count_ = this->count_+ 1;
        }
        
        if(this->count_==2 && (abs(this->pos_y-1.2)>0.2))
        {cmdmsg.linear.x = 0.05;
        cmdmsg.angular.z = 0.0;
        }
        
        if(this->count_==2 && (abs(this->pos_y-1.2)<0.2))
        {cmdmsg.linear.x = 0.0;
        cmdmsg.angular.z = 0.0;
        this->count_ = this->count_+ 1;
        }
        
        if(this->count_==3 && (abs(this->pos_yaw-0)>0.2))
        {cmdmsg.linear.x = 0.0;
        cmdmsg.angular.z = -0.05;
        
        }
        
        if(this->count_==3 && (abs(this->pos_yaw-0)<0.2))
        {cmdmsg.linear.x = 0.0;
        cmdmsg.angular.z = 0.0;
        this->count_ = this->count_+ 1;
        }
        
        if(this->count_==4 )
        {cmdmsg.linear.x = 0.1;
        cmdmsg.angular.z = 0;
        
        }
        cmdPublisher_.publish(cmdmsg);
        
        //this->count_ = this->count_+ 1;
        //std::cout<< this->count_<<std::endl;
        //ROS_INFO_STREAM_THROTTLE(2.0,"Count: " << this->count_);
	//ROS_INFO_STREAM_THROTTLE(2.0,"Minimum Range: " << min);
	
	
}

}  // namespace smb_highlevel_controller
