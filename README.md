# Object Avoidance and Navigation High Level Controller

## **1. README.md** (Project Documentation)
```markdown
# Mobile Robot Control  

## Overview  
This project implements a motion control system for a ROS-enabled mobile robot using a Jetson Xavier NX processor. The robot follows a trajectory while avoiding obstacles using an Intel RealSense depth camera.

## Setup  
### Installation  
```bash
cd catkin_ws2
catkin_make
source devel/setup.bash
```

### Running the Controller  
```bash
roslaunch highlevel_controller highlevel_controller.launch
```

## Control Implementation  
- **Motion Control:** Skid-steer system for independent left/right motor control  
- **Obstacle Avoidance:** Uses depth camera measurements  
- **PID Control:** Ensures accurate position tracking  

## Data Visualization  
```bash
cd ws_plotjuggler
source devel/setup.bash
rosrun plotjuggler plotjuggler
```
```

## **2. HighlevelController.cpp** (Main Control Code)
```cpp
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

class HighlevelController {
public:
    HighlevelController(ros::NodeHandle &nh) {
        cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel/managed", 10);
        scan_sub = nh.subscribe("/scan", 10, &HighlevelController::scanCallback, this);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
        double min_dist = *std::min_element(msg->ranges.begin(), msg->ranges.end());
        geometry_msgs::Twist cmd;
        
        if (min_dist > 0.5) {
            cmd.linear.x = 0.3; // Move forward
            cmd.angular.z = 0.0;
        } else {
            cmd.linear.x = 0.0;
            cmd.angular.z = 1.0; // Turn left if obstacle detected
        }
        cmd_pub.publish(cmd);
    }

private:
    ros::Publisher cmd_pub;
    ros::Subscriber scan_sub;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "highlevel_controller");
    ros::NodeHandle nh;
    HighlevelController controller(nh);
    ros::spin();
    return 0;
}
```

## **3. highlevel_controller.launch** (Launch File for ROS)
```xml
<launch>
    <node pkg="highlevel_controller" type="highlevel_controller" name="controller" output="screen" />
</launch>
```

## **4. CMakeLists.txt**
```cmake
cmake_minimum_required(VERSION 3.0.2)
project(highlevel_controller)
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
add_executable(highlevel_controller src/HighlevelController.cpp)
target_link_libraries(highlevel_controller ${catkin_LIBRARIES})
```
## Video Demonstration  
[![Watch the video](https://img.youtube.com/vi/dzT1IAFJenI/0.jpg)](https://www.youtube.com/shorts/dzT1IAFJenI)
```

---

This setup will allow you to implement, build, and run your mobile robot control system. 
