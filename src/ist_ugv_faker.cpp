#include "ros/ros.h"
#include <ros/package.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib_tutorials/FibonacciAction.h>

#include "mission_planner/RequestMobileChargingStationAction.h"
#include "mission_planner/DoCloserInspectionAction.h"

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_tutorials/FibonacciAction.h>

#include "geographic_msgs/GeoPoseStamped.h"

#include <cmath>

class ISTugvFaker{
  protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<mission_planner::RequestMobileChargingStationAction> mobile_station_as_;
    actionlib::SimpleActionServer<mission_planner::DoCloserInspectionAction> closer_inspection_as_;

    mission_planner::RequestMobileChargingStationResult mobile_station_result_;
    mission_planner::DoCloserInspectionResult closer_inspection_result_;

    // UGV Pose Fakers Publishers
    ros::Publisher atrvjr_pose_pub_;
    ros::Publisher jackal_pose_pub_;
    geographic_msgs::GeoPoseStamped atrvjr_pose_; 
    geographic_msgs::GeoPoseStamped jackal_pose_;
    geographic_msgs::GeoPoseStamped north_position_;
    geographic_msgs::GeoPoseStamped south_position_;
    geographic_msgs::GeoPoseStamped center_;
    float latitude_increment_;
    float angular_velocity_;
    float angle_increment_;
    float current_angle_;
    ros::Rate pose_rate_;
    std::string atrvjr_direction_;
    std::string jackal_direction_;

  public:
    ISTugvFaker() : pose_rate_(0.2), // 5 seconds
      mobile_station_as_(nh_, "/jackal0/cooperation_use/request_mobile_charging_station", boost::bind(&ISTugvFaker::mobileStationCB, this, _1), false),
      closer_inspection_as_(nh_, "/atrvjr/cooperation_use/do_closer_inspection", boost::bind(&ISTugvFaker::closerInspectionCB, this, _1), false) {
        mobile_station_as_.start();
        closer_inspection_as_.start();

        atrvjr_pose_pub_ = nh_.advertise<geographic_msgs::GeoPoseStamped>("/atrvjr/geopose", 1);
        jackal_pose_pub_ = nh_.advertise<geographic_msgs::GeoPoseStamped>("/jackal0/geopose", 1);

        // Initial atrvjr and jackal movement directions
        atrvjr_direction_ = "right";
        jackal_direction_ = "up";

        // Define north and south points for jackal trajectory
        // Latitude  - y
        // Longitude - x
        // Altitude  - z
        north_position_.pose.position.latitude = 38.54121161489917;
        north_position_.pose.position.longitude = -7.961711602856107;
        north_position_.pose.position.altitude = 230.1384181595744;
        south_position_.pose.position.latitude = 38.54106489217978;
        south_position_.pose.position.longitude = -7.961711602856107;
        south_position_.pose.position.altitude = 230.1384181595744;

        // Define center coordinates for atrvjr circular trajectory
        center_.pose.position.latitude = 38.5410856346297;
        center_.pose.position.longitude = -7.961538763126298;
        center_.pose.position.altitude = 230.0598269717272;

        // Compute the circular trajectory parameters
        angular_velocity_ = 2 * 3.141592654 / 30; // w = 2 * pi / T
        angle_increment_ = angular_velocity_ * 5; // AO = w * At
        current_angle_ = 0;

        // Compute the latitude increment per position update interval
        // 15 meters, let's say 7'5 seconds to travel those 15 meters, 0'2 seconds per positions update
        // 38 updates per direction
        latitude_increment_ = (north_position_.pose.position.latitude - south_position_.pose.position.latitude) / 38;

        // Position: solar pannel row 14 column 2
        // atrvjr_pose_.pose.position.latitude = 38.54143688611777;
        // atrvjr_pose_.pose.position.longitude = -7.961302628908529;
        // atrvjr_pose_.pose.position.altitude = 227.61875915527344;

        //Initial position: south
        jackal_pose_.pose.position.latitude = south_position_.pose.position.latitude;
        jackal_pose_.pose.position.longitude = south_position_.pose.position.longitude;
        jackal_pose_.pose.position.altitude = south_position_.pose.position.altitude;

        atrvjr_pose_.pose.position.altitude = center_.pose.position.altitude;

        pose_rate_.reset();
        while(ros::ok())
        {
          ros::spinOnce();
          UGVPositionUpdater();
          atrvjr_pose_pub_.publish(atrvjr_pose_);
          jackal_pose_pub_.publish(jackal_pose_);
          pose_rate_.sleep();
        }
      }

    ~ISTugvFaker(void){}

    void UGVPositionUpdater() {
      // Jackal will move in a vertical line in the central path at the north of the origin
      // Point 1: x==0, y==10: -7.961712761304593,38.54106489217978,230.1931775972878
      // Point 2: x==0, y==25: -7.961711602856107,38.54121161489917,230.1384181595744
      if (jackal_direction_ == "up") {
        jackal_pose_.pose.position.latitude = jackal_pose_.pose.position.latitude + latitude_increment_;
        if (jackal_pose_.pose.position.latitude > north_position_.pose.position.latitude) {
          jackal_direction_ = "down";
        }
      }
      else {
        jackal_pose_.pose.position.latitude = jackal_pose_.pose.position.latitude - latitude_increment_;
        if (jackal_pose_.pose.position.latitude < south_position_.pose.position.latitude) {
          jackal_direction_ = "down";
        }
      }

      // atrvjr will move in a square around the building
      // Corner 1: x==0,  y==30:   -7.961710021346433,38.54128355993739,229.9654475267011
      // Corner 2: x==20, y==30:   -7.961515416900085,38.54128629006915,229.5583745517417
      // Corner 3: x==20, y==47.5: -7.961515302176315,38.54142284054371,229.1205587457127
      // Corner 4: x==0,  y==47.5: -7.961710530654779,38.54142927235737,229.567600993049

      // atrvjr will move in a circle around a coordinate center point
      current_angle_ += angle_increment_;
      if (current_angle_ > 2 * 3.141592654)
        current_angle_ = 0;
      atrvjr_pose_.pose.position.latitude = center_.pose.position.latitude   + 0.0001 * sin(current_angle_);
      atrvjr_pose_.pose.position.longitude = center_.pose.position.longitude + 0.0001 * cos(current_angle_);
    }

    void mobileStationCB(const mission_planner::RequestMobileChargingStationGoalConstPtr &goal) {
      mobile_station_result_.success = true;
      ROS_INFO("Requested mobile charging station. Returning SUCCESS");
      mobile_station_as_.setSucceeded(mobile_station_result_);
    }

    void closerInspectionCB(const mission_planner::DoCloserInspectionGoalConstPtr &goal) {
      closer_inspection_result_.success = true;
      ROS_INFO("Requested closer inspection. Returning SUCCESS");
      closer_inspection_as_.setSucceeded(closer_inspection_result_);
    }
};


int main(int argc, char** argv){
  ros::init(argc, argv, "ist_ugv_faker");

  ISTugvFaker ist_ugv_faker;
  ROS_INFO("Ending");

  return 0;
}
