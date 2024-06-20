#include "puma_local_planner.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <base_local_planner/goal_functions.h>

#define CONST_ANALOG_TO_RAD 0.006135742
#define PI 3.14159265
#define ZERO_POSITION 395
#define D2R 0.0174532925  

using namespace std;

// Register planner
PLUGINLIB_EXPORT_CLASS(puma_local_planner::PumaLocalPlanner, nav_core::BaseLocalPlanner);

namespace puma_local_planner {

  PumaLocalPlanner::PumaLocalPlanner() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

  PumaLocalPlanner::PumaLocalPlanner(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) : costmap_ros_(NULL), tf_(NULL), initialized_(false) {
    
  }
  
  PumaLocalPlanner::~PumaLocalPlanner() {}

  void PumaLocalPlanner::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) {
    if(!initialized_) {
      ros::NodeHandle nh = ros::NodeHandle("~" + name);
      tf_ = tf;

      // Subs and pubs
      status_arduino_sub = nh.subscribe("/puma/arduino/status", 1000, &PumaLocalPlanner::statusArduinoCallback, this);
      odometry_sub = nh.subscribe("/odometry/filtered", 1000, &PumaLocalPlanner::odometryCallback, this);
      ackerman_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/puma/control/ackermann/command", 10);
      
      // Config PID
      pid_linear.p = 0.7;
      pid_linear.i = 5.0;
      pid_linear.d = 3.0;
      pid_linear.integral_error = pid_linear.prev_error = 0.0;
      pid_angular.p = 0.3;
      pid_angular.i = 2.0;
      pid_angular.d = 2.0;
      pid_angular.integral_error = pid_angular.prev_error = 0.0;

      dt = 0.1;

      relocate_orientation = false;

      costmap_ros_ = costmap_ros;
      // initialize the copy of the costmap the controller will use
      costmap_ = costmap_ros_->getCostmap();
    
      ROS_INFO("Puma Local planner iniciado!");
      initialized_ = true;
    } else {
      ROS_WARN("Puma Local planner ya se encuentra iniciado!");
    }
  }

  void PumaLocalPlanner::statusArduinoCallback(const puma_arduino_msgs::StatusArduino::ConstPtr &status){
    current_analog_direction = status->current_position_dir;
    current_angle_direction = (current_analog_direction - ZERO_POSITION)* CONST_ANALOG_TO_RAD;

    //ROS_INFO("Angulo(rads) actual: %f", current_angle_direction);
  }

  void PumaLocalPlanner::odometryCallback(const nav_msgs::Odometry::ConstPtr &odom) {
    current_vel_x = odom->twist.twist.linear.x;
    current_angular_z = odom->twist.twist.angular.z;

    current_position.x = odom->pose.pose.position.x;
    current_position.y = odom->pose.pose.position.y;
    current_position.z = odom->pose.pose.position.z;

    // Create variables for orientation odom
    double x, y, z, w = 0;
    x = odom->pose.pose.orientation.x;
    y = odom->pose.pose.orientation.y;
    z = odom->pose.pose.orientation.z;
    w = odom->pose.pose.orientation.w;
    tf2::Quaternion quat_odom(x,y,z,w);
    // Create variables for convert in RPY
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat_odom).getRPY(roll, pitch, yaw);

    current_position.az = yaw;
  }


  bool PumaLocalPlanner::setPlan( const std::vector<geometry_msgs::PoseStamped>& orig_global_plan ) {
    if(!initialized_) {
      ROS_ERROR("Este planificador no ha sido iniciado");
      return false;
    }
    // Init count
    count = 1;

    // Set plan
    plan = orig_global_plan;
    length = (plan).size();
    goal_reached = false;

    setNextPart();
    return true;
  }

  void PumaLocalPlanner::setNextPart(){
    next_position.x = plan[count].pose.position.x;
    next_position.y = plan[count].pose.position.y;
    //ROS_INFO("Angle part position: %f ", plan[count].pose.orientation.z);
  }

  void PumaLocalPlanner::setErrorNow(){
    double delta_theta;

    // Calculate error position between next and current
    error_position.x = next_position.x - current_position.x;
    error_position.y = next_position.y - current_position.y;

    // We have arrived to goal
    if (error_position.x == 0 & error_position.y == 0){
      delta_theta = current_position.az;
    } else {
    // Calculate tan
      delta_theta = std::atan2(error_position.y, error_position.x);
    }
   
    //Calculate distance
    distance = std::sqrt( pow(error_position.x,2) + pow(error_position.y,2) );
    error_position.az = delta_theta - current_position.az ;


    // Choose smallest angle  
    // Always smaller than abs(PI)
    // if ( fabs(error_position.az) > PI ) { relocate_orientation = true; }
    // else { relocate_orientation = false; }
    //if ( error_position.az < -PI ) { error_position.az += 2*PI; }
  }

  double PumaLocalPlanner::computePid(pid pid_controller, double desired, double actual) {
    double error = desired - actual;
    pid_controller.integral_error += error * dt;
    double derivative = (error - pid_controller.prev_error) / dt;
    pid_controller.prev_error = error;

    return (pid_controller.p * error) + (pid_controller.i * pid_controller.integral_error) + (pid_controller.d * derivative);
  }

  void PumaLocalPlanner::setVelocity(){
    double desired_steering_angle = error_position.az;
    double angle_control = computePid(pid_angular, desired_steering_angle, current_angle_direction);
    acker_cmd.drive.steering_angle = angle_control;
    acker_cmd.drive.speed = 0.6;
  }

  void PumaLocalPlanner::setRotation(){
    
      if (fabs(error_position.az) > 30*D2R ) {
        // Wheen error > limit direction
        //ROS_INFO("Error angular mayor a la direccion limite. Error: %f", error_position.az);
        acker_cmd.drive.steering_angle = error_position.az;
        acker_cmd.drive.speed = 0.0;

        if (fabs(current_angle_direction) >= 29*D2R) {
          // When direction is in limit
          acker_cmd.drive.speed = 0.3;
        }
      } else {
        // Wheen error is on limit of direction
        //ROS_INFO("Error angular dentro de los limites de la direccion");
        double desired_steering_angle = error_position.az;
        double angle_control = computePid(pid_angular, desired_steering_angle, current_angle_direction);
        acker_cmd.drive.steering_angle = angle_control;
        acker_cmd.drive.speed = 0.0;

        if ( fabs(error_position.az) <= (current_angle_direction + 2*D2R) & 
          fabs(error_position.az) >= (current_angle_direction - 2*D2R)) {
          acker_cmd.drive.speed = 0.5;
        }
      
    }
  }

  void PumaLocalPlanner::setCountPlan(){
    if (count < (length-1)) {
      // If left part of global plan
      if((length - count) < 10){ 
        // When few parts of the plan are missing
        count = length - 1;
      } else if ((length - count) < 20){ 
        // When 20 parts of the plan are missing
        count += 10;
      } else {
        // When many parts of the plan are missing
        count += 20; 
      }
      setNextPart();
    } else {
      ROS_INFO("Llegada al destino");
      goal_reached = true;
      // Ackermann
      acker_cmd.drive.speed = 0.0;
      acker_cmd.drive.steering_angle = 0.0;
      // Reset pid
      pid_angular.prev_error = pid_angular.integral_error = 0.0;
      pid_linear.prev_error = pid_linear.integral_error = 0.0;
    }
  }

  bool PumaLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
    if(!initialized_) {
      ROS_ERROR("Este planificador no ha sido iniciado");
      return false;
    }
    
    // If have a path created
    if (length != 0 ){
      // Calculate diff position
      setErrorNow();

      if (distance > 0.5) {
        // If have considerable distance
        if(fabs(error_position.az > 2 * D2R)) {
          setRotation();
        } else {
          setVelocity();
        }
      } else {
        //ROS_INFO("Distancia menor a 0.5m del destino");
        setCountPlan();
      }
    }

    ackerman_pub.publish(acker_cmd);
    return true;
  }

  bool PumaLocalPlanner::isGoalReached() {
    if(!initialized_) {
      ROS_ERROR("Este planificador no ha sido iniciado");
      return false;
    }
    return goal_reached;
  }

} // namespace puma_local_planner