#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/Point.h"

#define error_rotation_threshold 0.2//radians
#define error_translation_threshold 0.3// meters

#define rotation_speed_max M_PI/4//radians/s

#define kpr 0.7
#define kir 0
#define kdr 0

#define kpt 0.5
#define kit 0
#define kdt 0

#define safety_distance 0.2

#define safety_factor 0.02

class action_node {
private:

    ros::NodeHandle n;

    // communication with one_moving_person_detector or person_tracker
    ros::Subscriber sub_goal_to_reach;

    // communication with odometry
    ros::Subscriber sub_odometry;

    // communication with obstacle_detection
    ros::Subscriber sub_obstacle_detection;

    // communication with cmd_vel to send command to the mobile robot
    ros::Publisher pub_cmd_vel;

    geometry_msgs::Point goal_to_reach;
    bool new_goal_to_reach;//to check if a new /goal_to_reach is available or not

    bool cond_goal;// boolean to check if we still have to reach the goal or not

    //pid for rotation
    float rotation_to_do, rotation_done;
    float error_rotation;//error in rotation
    bool cond_rotation;//boolean to check if we still have to rotate or not
    float initial_orientation;// to store the initial orientation ie, before starting the pid for rotation control
    float current_orientation;// to store the current orientation: this information is provided by the odometer
    float error_integral_rotation;
    float error_previous_rotation;
    float rotation_speed;

    //pid for translation
    float translation_to_do, translation_done,translation_to_do_r;
    float error_translation;//error in translation
    bool cond_translation;//boolean to check if we still have to translate or not
    geometry_msgs::Point initial_position;// to store the initial position ie, before starting the pid for translation control
    geometry_msgs::Point current_position;// to store the current position: this information is provided by the odometer
    float error_integral_translation;
    float error_previous_translation;
    float translation_speed;

    bool init_odom;
    bool init_obstacle;
    geometry_msgs::Point closest_obstacle;   

public:

action_node() {

    // communication with cmd_vel to command the mobile robot
    pub_cmd_vel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // communication with odometry
    sub_odometry = n.subscribe("odom", 1, &action_node::odomCallback, this);

    // communication with obstacle_detection
    sub_obstacle_detection = n.subscribe("closest_obstacle", 1, &action_node::closest_obstacleCallback, this);

    // communication with datmo
    sub_goal_to_reach = n.subscribe("goal_to_reach", 1, &action_node::goal_to_reachCallback, this);

    cond_goal = false;
    new_goal_to_reach = false;
    init_odom = false;   
    init_obstacle = false;

    //INFINITE LOOP TO COLLECT LASER DATA AND PROCESS THEM
    ros::Rate r(10);// this node will run at 10hz - time sampling 
    while (ros::ok()) {
        ros::spinOnce();//each callback is called once to collect new data: laser + robot_moving
        update();//processing of data
        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }

}

//UPDATE: main processing
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update() {

    if ( init_odom && init_obstacle ) { //wait for the initialization of odometer and detect_obstacle_node

        // we receive a new /goal_to_reach
        if ( new_goal_to_reach )
            init_action();

        //we are performing a rotation and a translation
        if ( cond_goal )
        {
            compute_rotation();
            compute_translation();
            //cond_goal = ... DO NOT FORGET TO UPDATE cond_goal
            cond_goal=false;
            combine_rotation_and_translation();            
            move_robot();
        }
    }
    else
        if ( !init_obstacle )
            ROS_WARN("waiting for obstacle_detection_node");

}// update

void init_action()
{
    new_goal_to_reach = false;
    ROS_INFO("processing the /goal_to_reach received at (%f, %f)", goal_to_reach.x, goal_to_reach.y);

    // we have a rotation and a translation to perform
    // we compute the /translation_to_do
    if (translation_to_do > safety_distance)
    {
        translation_to_do = sqrt( ( goal_to_reach.x * goal_to_reach.x ) + ( goal_to_reach.y * goal_to_reach.y ) ) - safety_distance;
        translation_to_do_r = sqrt( ( goal_to_reach.x * goal_to_reach.x ) + ( goal_to_reach.y * goal_to_reach.y ) ) ;

        if ( translation_to_do )
        {
            cond_goal = true;

            //we compute the /rotation_to_do
            rotation_to_do = acos( goal_to_reach.x / translation_to_do_r );

            if ( goal_to_reach.y < 0 )
                rotation_to_do *=-1;

            //we initialize the pid for the control of rotation
            initial_orientation = current_orientation;
            error_integral_rotation = 0;
            error_previous_rotation = 0;

            //we initialize the pid for the control of translation
            initial_position = current_position;
            error_integral_translation = 0;
            error_previous_translation = 0;

            ROS_INFO("rotation_to_do: %f, translation_to_do: %f", rotation_to_do*180/M_PI, translation_to_do);
            ROS_INFO("initial_position: (%f, %f), initial_orientation: %f provided by odometer", initial_position.x, initial_position.y, initial_orientation*180/M_PI);
        }
        else {
            ROS_WARN("translation_to_do is equal to 0");
            cond_goal = false;
        }
    }
}// init_action

void compute_rotation()
{

    ROS_INFO("current_orientation: %f, initial_orientation: %f", current_orientation*180/M_PI, initial_orientation*180/M_PI);
    rotation_done = current_orientation-initial_orientation;

    //do not forget that rotation_done must always be between -M_PI and +M_PI
    if ( rotation_done > M_PI )
        {
            ROS_WARN("rotation_done > 180 degrees: %f degrees -> %f degrees", rotation_done*180/M_PI, (rotation_done-2*M_PI)*180/M_PI);
            rotation_done -= 2*M_PI;
        }
        else
            if ( rotation_done < -M_PI )
            {
                ROS_WARN("rotation_done < -180 degrees: %f degrees -> %f degrees", rotation_done*180/M_PI, (rotation_done+2*M_PI)*180/M_PI);
                rotation_done += 2*M_PI;
            }

    error_rotation = rotation_to_do - rotation_done;
    ROS_INFO("rotation_to_do: %f, rotation_done: %f, error_rotation: %f", rotation_to_do*180/M_PI, rotation_done*180/M_PI, error_rotation*180/M_PI);

    if (abs(error_rotation) < error_rotation_threshold)  //cond_rotation is used to control if we stop or not the pid - HERE
    	{cond_rotation = false;
    	ROS_INFO("the last error_rotation: %f",error_rotation);}
    else
    	{cond_rotation = true ;}

    rotation_speed = 0;
    if ( cond_rotation )
    {
       
        float error_derivation_rotation = (error_rotation-error_previous_rotation)/0.1;

        error_integral_rotation = error_integral_rotation+ (error_rotation )*0.1;
        
        rotation_speed = kpr*error_rotation + kdr*error_derivation_rotation+ kir*error_integral_rotation;
        ROS_INFO("rotation_speed: %f", rotation_speed*180/M_PI);
        error_previous_rotation = error_rotation;
    }
    else
        ROS_WARN("pid for rotation will stop");

}//compute_rotation

void compute_translation()
{

    ROS_INFO("current_position: (%f, %f), initial_position: (%f, %f)", current_position.x, current_position.y, initial_position.x, initial_position.y);
    translation_done= distancePoints(current_position,initial_position);
    error_translation = translation_to_do - translation_done;

    ROS_INFO("translation_to_do: %f, translation_done: %f, error_translation: %f", translation_to_do, translation_done, error_translation);

    //cond_translation = ...; cond_translation is used to control if we stop or not the pid for translation
    if (abs(error_translation) < error_translation_threshold)  //cond_translation is used to control if we stop or not the pid - HERE
    	{cond_translation = false;
    	ROS_INFO("The last translation error: %f",error_translation);}
    else
    	{cond_translation = true ;}

    translation_speed = 0;

    if ( cond_translation )
    {
        float error_derivation_translation = (error_translation-error_previous_translation)/0.1;

        error_integral_translation = error_integral_translation+error_translation*0.1;
        
        translation_speed = kpt*error_translation + kdt*error_derivation_translation+ kit*error_integral_translation;
        ROS_INFO("translation_speed: %f", translation_speed);
        error_previous_translation = error_translation;
    }

}//compute_translation

void combine_rotation_and_translation()
{

    float coef_rotation=abs(rotation_to_do)/rotation_speed_max;// = ...;
    if ( coef_rotation > 1 )
        coef_rotation = 1;
    float coef_translation = 1 - coef_rotation;

    //translation_speed = ...
    translation_speed = coef_rotation*rotation_speed + coef_translation*translation_speed;
    
    ROS_INFO("coef_rotation: %f, rotation_speed: %f, coef_translation: %f, translation_speed: %f", coef_rotation, rotation_speed*180/M_PI, coef_translation, translation_speed);

    if ( translation_speed < 0 )
    {
        translation_speed = 0;
        ROS_WARN("translation_speed is negative");
    }

}//combine_rotation_and_translation

void move_robot()
{

    // processing of obstacle
    //DO NOT REMOVE - combination
    
    if ( ( fabs(closest_obstacle.x) < safety_distance ) || ( ! cond_translation ) )
    {
        translation_speed = 0;
        if ( !cond_translation )
            ROS_WARN("pid for translation will stop");
        else
            ROS_WARN("obstacle detected: (%f, %f)", closest_obstacle.x, closest_obstacle.y);
    }
    
    //end of processing of obstacle

    geometry_msgs::Twist twist;
    twist.linear.x = translation_speed;
    twist.linear.y = 0;
    twist.linear.z = 0;

    twist.angular.x = 0;
    twist.angular.y = 0;
    //twist.angular.z = 0;    
    twist.angular.z = rotation_speed;

    pub_cmd_vel.publish(twist);

}// move_robot

//CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void odomCallback(const nav_msgs::Odometry::ConstPtr& o) {

    init_odom = true;

    current_position = o->pose.pose.position;
    current_orientation = tf::getYaw(o->pose.pose.orientation);

}

void goal_to_reachCallback(const geometry_msgs::Point::ConstPtr& g) {
// process the goal received from moving_persons detector

    new_goal_to_reach = true;
    goal_to_reach = *g;

}

void closest_obstacleCallback(const geometry_msgs::Point::ConstPtr& obs) {

    init_obstacle = true;
    closest_obstacle = *obs;

}//closest_obstacleCallback

// Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

}

};

int main(int argc, char **argv){

    ros::init(argc, argv, "action_node");

    ROS_INFO("(action_node) waiting for a /goal_to_reach");
    action_node bsObject;

    ros::spin();

    return 0;
}
