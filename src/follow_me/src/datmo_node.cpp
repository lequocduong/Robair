#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <tf/transform_datatypes.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"

#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "message_filters/subscriber.h"
#include "tf/message_filter.h"


#define detection_threshold 0.01//threshold for motion detection 0.2 --> 1.8 - 0.01 --> dynamic point
#define dynamic_threshold 35 //to decide if a cluster is static or dynamic 75 -->35 () --> cluster dynamic or not

//threshold for clustering
#define cluster_threshold 0.2

//used for detection of leg
#define leg_size_min 0.05
#define leg_size_max 0.25
#define legs_distance_max 0.7

//used for frequency
#define frequency_init 5
#define frequency_max 25

//used for uncertainty associated to the tracked person
#define uncertainty_min 0.5
#define uncertainty_max 1
#define uncertainty_inc 0.05

using namespace std;

class datmo {

private:
    ros::NodeHandle n;

    ros::Subscriber sub_scan;
    ros::Subscriber sub_robot_moving;

    ros::Publisher pub_datmo;
    ros::Publisher pub_datmo_marker;

    

    // to store, process and display both laserdata
    bool init_laser;
    int nb_beams;
    float range_min, range_max;
    float angle_min, angle_max, angle_inc;
    float r[1000];
    float theta[1000];
    geometry_msgs::Point current_scan[1000];

    //to perform detection of motion
    bool init_robot;
    bool stored_background;
    float background[1000];// to store the range of each hit
    bool dynamic[1000];//to know if the corresponding hit is dynamic or not

    // to know if the robot is moving or not
    bool current_robot_moving;
    bool previous_robot_moving;

    //to perform clustering
    int nb_cluster;// number of cluster
    int cluster[1000]; //to store for each hit, the cluster it belongs to
    float cluster_size[1000];//to store the size of each cluster
    geometry_msgs::Point cluster_middle[1000];// to store the middle of each cluster
    int cluster_dynamic[1000];//to store the percentage of the cluster that is dynamic    

    //to perform detection of legs and to store them
    int nb_legs_detected;
    geometry_msgs::Point leg_detected[1000];//to store the coordinates a leg
    int leg_cluster[1000];//to store the cluster corresponding to a leg
    bool leg_dynamic[1000];//to know if a leg is dynamic or not

    //to perform detection of a moving person and store it
    int nb_persons_detected;
    geometry_msgs::Point person_detected[1000];//to store the coordinates of a person
    bool person_dynamic[1000];//to know if a person is dynamic or not

    //BOYAN ADD
    geometry_msgs::Point person_detected_leg1[1000];
    geometry_msgs::Point person_detected_leg2[1000];
    /*
    geometry_msgs::Point moving_person_leg1_tracked;
    geometry_msgs::Point moving_person_leg2_tracked;
    */

    //to perform tracking of the moving person
    bool tracking_mode;//to know if we are tracking a moving person or not
    geometry_msgs::Point moving_person_tracked;//to store the coordinates of the moving person that we are tracking
    float uncertainty;
    int frequency;

    // GRAPHICAL DISPLAY
    int nb_pts;
    geometry_msgs::Point display[1000];
    std_msgs::ColorRGBA colors[1000];

public:

datmo() {

    sub_scan = n.subscribe("scan", 1, &datmo::scanCallback, this);
    sub_robot_moving = n.subscribe("robot_moving", 1, &datmo::robot_movingCallback, this);

    // communication with action
    pub_datmo = n.advertise<geometry_msgs::Point>("goal_to_reach", 1);     // Preparing a topic to publish the goal to reach.

    pub_datmo_marker = n.advertise<visualization_msgs::Marker>("datmo", 1); // Preparing a topic to publish our results. This will be used by the visualization tool rviz

    init_laser = false;
    init_robot = false;

    previous_robot_moving = true;

    tracking_mode = false;

    ros::Rate r(10);

    while (ros::ok()) {
        ros::spinOnce();
        update();
        r.sleep();
    }

}

//UPDATE
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update() {

    // we wait for new data of the laser and of the robot_moving_node to perform laser processing
    if ( init_laser && init_robot ) {

        ROS_INFO("\n");
        ROS_INFO("New data of laser received");
        ROS_INFO("New data of robot_moving received");

        nb_pts = 0;

        //if the robot is not moving then we can perform moving person detection
        if ( !current_robot_moving ) {
            ROS_INFO("robot is not moving");
            

            // if the robot was moving previously and now it is not moving then we store the background
            if ( previous_robot_moving ) {
                
                store_background();
                //ros::Duration(0, 5).sleep(); //500ms sleeping
                //detect_motion();
                //getchar();*/
            }
            
        }
        else{
            ROS_INFO("robot is moving");
            
        }
      
        previous_robot_moving = current_robot_moving;

        //we search for moving person in 4 steps
        detect_motion();//to classify each hit of the laser as dynamic or not

        perform_clustering();//to perform clustering
        detect_legs();//to detect moving legs using cluster
        detect_persons();//to detect moving_person using moving legs detected

        if ( !tracking_mode )
           detect_a_moving_person();
        else
            track_a_moving_person();

       
  
    }
    else
    {
//        if ( !init_laser )
//            ROS_INFO("waiting for laser data");
        if ( !init_robot )
            ROS_INFO("waiting for robot_moving_node");
    }

}// update

// DETECTION OF MOTION
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void store_background() {
// store all the hits of the laser in the background table

    ROS_INFO("storing background");

    for (int loop=0; loop<nb_beams; loop++)
        background[loop] = r[loop];

    ROS_INFO("background stored");

}//init_background

void reset_motion() {
// for each hit, compare the current range with the background to detect motion

    ROS_INFO("reset motion");
    for (int loop=0 ; loop<nb_beams; loop++ )
        dynamic[loop] = false;

    ROS_INFO("reset_motion done");

}//reset_motion

void detect_motion() {

    ROS_INFO("detecting motion");

    nb_pts = 0;
     for (int loop=0; loop<nb_beams; loop++ )
      {//loop over all the hits
        // the detect of motion ONLY takes place when the robot is not moving, ie when current_robot_moving is false
        // when current_robot_moving is true, dynamic[loop] is false for all the beams
        if (!current_robot_moving){
                // if the difference between ( the background and the current range ) is higher than "detection_threshold"
                // then
            if ((background[loop] - r[loop]) > detection_threshold)
                   dynamic[loop] = true;//the current hit is dynamic
            else
                dynamic[loop] = false;//else its static

            if ( dynamic[loop] ) {

                //display in blue of hits that are dynamic
                /*
                 display[nb_pts] = current_scan[loop];

                colors[nb_pts].r = 0;
                colors[nb_pts].g = 0;
                colors[nb_pts].b = 1;
                colors[nb_pts].a = 1.0;
                */
                nb_pts++;
                }
        }
        else{

            dynamic[loop] = false; 

        }
    }

    //graphical display of the results
    populateMarkerTopic();

    ROS_INFO("motion detected");

}//detect_motion

// CLUSTERING
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void perform_clustering() {
//store in the table cluster, the cluster of each hit of the laser
//if the distance between the previous hit and the current one is higher than "cluster_threshold"
//then we end the current cluster with the previous hit and start a new cluster with the current hit
//else the current hit belongs to the current cluster

    ROS_INFO("performing clustering");

    nb_cluster = 0;//to count the number of cluster

    //initialization of the first cluster
    int start = 0;// the first hit is the start of the first cluster
    int end;
    int nb_dynamic = 0;// to count the number of hits of the current cluster that are dynamic

    nb_pts = 0;
    /*
    //graphical display of the start of the current cluster in green
    display[nb_pts] = current_scan[start];

    colors[nb_pts].r = 0;
    colors[nb_pts].g = 1;
    colors[nb_pts].b = 0;
    colors[nb_pts].a = 1.0;
    */
    nb_pts++;

    for( int loop=1; loop<nb_beams; loop++ )//loop over all the hits
        if (distancePoints(current_scan[loop], current_scan[loop-1]) > cluster_threshold)
        {//the current hit doesnt belong to the same hit
              cluster[loop-1] = nb_cluster;
              if (dynamic[loop-1])
                nb_dynamic++;

            //   1/ we end the current cluster, so we update:
            //   - end to store the last hit of the current cluster
            end = loop-1;
            
            //   - cluster_size to store the size of the cluster ie, the euclidian distance between the first hit of the cluster and the last one
            cluster_size[nb_cluster] = distancePoints(current_scan[start], current_scan[end]);

            //   - cluster_middle to store the middle of the cluster
            cluster_middle[nb_cluster] = current_scan[(int)floor((start + end)/2)];

            //   - cluster_dynamic to store the percentage of hits of the current cluster that are dynamic
            cluster_dynamic[nb_cluster] = (nb_dynamic /(end-start+1))*100;

            //graphical display of the end of the current cluster in red
            /*
            display[nb_pts] = current_scan[end];
            
            colors[nb_pts].r = 1;
            colors[nb_pts].g = 0;
            colors[nb_pts].b = 0;
            colors[nb_pts].a = 1.0;
            */
            nb_pts++;

            //textual display
             ROS_INFO("cluster[%i]: [%i](%f, %f) -> [%i](%f, %f), size: %f, dynamic: %i"     , nb_cluster
                                                                                            , start
                                                                                             , current_scan[start].x
                                                                                             , current_scan[start].y
                                                                                            , end
                                                                                             , current_scan[end].x
                                                                                             , current_scan[end].y
                                                                                             , cluster_size[nb_cluster]
                                                                                             , cluster_dynamic[nb_cluster]);

            //   2/ we start a new cluster with the current hit
            nb_cluster++;
            start = loop;
            nb_dynamic = 0;// to count the number of hits of the current cluster that are dynamic
            /*
            //graphical display of the start of the current cluster in green
            display[nb_pts] = current_scan[start];
            
            colors[nb_pts].r = 0;
            colors[nb_pts].g = 1;
            colors[nb_pts].b = 0;
            colors[nb_pts].a = 1.0;
            */
            nb_pts++;

        }
        else
        {
            cluster[loop-1] = nb_cluster;
            if (dynamic[loop])
                nb_dynamic++;
        }

    //Dont forget to update the different information for the last cluster
    //...
    end = nb_beams;
    cluster_size[nb_cluster] = distancePoints(current_scan[start], current_scan[end]);
    cluster_middle[nb_cluster] = current_scan[(int)floor((start + end)/2)];
    cluster_dynamic[nb_cluster] = (nb_dynamic / (end-start+1))*100;
    display[nb_pts] = current_scan[end];
    /*
    colors[nb_pts].r = 1;
    colors[nb_pts].g = 0;
    colors[nb_pts].b = 0;
    colors[nb_pts].a = 1.0;
    */
    nb_pts++;

    //textual display
     ROS_INFO("cluster[%i]: [%i](%f, %f) -> [%i](%f, %f), size: %f, dynamic: %i"     , nb_cluster
                                                                                     , start
                                                                                     , current_scan[start].x
                                                                                     , current_scan[start].y
                                                                                     , end
                                                                                     , current_scan[end].x
                                                                                     , current_scan[end].y
                                                                                    , cluster_size[nb_cluster]
                                                                                    , cluster_dynamic[nb_cluster]);

    //graphical display of the results
    populateMarkerTopic();

    ROS_INFO("clustering performed");

}//perform_clustering

// DETECTION OF A MOVING PERSON
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void detect_legs() {
// a leg is a cluster:
// - with a size higher than "leg_size_min";
// - with a size lower than "leg_size_max;
// if more than "dynamic_threshold"% of its hits are dynamic the leg is considered to be dynamic

    ROS_INFO("detecting legs");
    nb_legs_detected = 0;

    // nb_pts = 0;
    for (int loop=0; loop<nb_cluster; loop++)//loop over all the clusters
        // if the size of the current cluster is higher than "leg_size_min" and lower than "leg_size_max"
        // then the current cluster is a leg
        if (cluster_size[loop] > leg_size_min && cluster_size[loop] < leg_size_max){

            // we update:
            // - the leg_detected table to store the middle of the moving leg;
            leg_detected[nb_legs_detected] = cluster_middle[loop];

            // - the leg_cluster to store the cluster corresponding to a leg;
            leg_cluster[nb_legs_detected] = cluster[loop];

            // - the leg_dynamic to know if the leg is dynamic or not.
            if (cluster_dynamic[loop] > dynamic_threshold){
                leg_dynamic[nb_legs_detected] = true;
            }
            else{
                leg_dynamic[nb_legs_detected] = false;
            }

            if ( leg_dynamic[nb_legs_detected] )
            {
                // ROS_INFO("moving leg found: %i -> cluster = %i, (%f, %f), size: %f, dynamic: %i", nb_legs_detected,
                //                                                                                   loop,
                //                                                                                   leg_detected[nb_legs_detected].x,
                //                                                                                   leg_detected[nb_legs_detected].y,
                //                                                                                   cluster_size[loop],
                //                                                                                   cluster_dynamic[loop]);
                 for(int loop2=0; loop2<nb_beams; loop2++)
                    if ( cluster[loop2] == loop ) {

                        // moving legs are yellow
                        
                        display[nb_pts] = current_scan[loop2];
                        
                        colors[nb_pts].r = 1;
                        colors[nb_pts].g = 1;
                        colors[nb_pts].b = 0;
                        colors[nb_pts].a = 1.0;
                        
                        nb_pts++;
                    }
            }
            else
            {
                // ROS_INFO("static leg found: %i -> cluster = %i, (%f, %f), size: %f, dynamic: %i", nb_legs_detected,
                //                                                                                   loop,
                //                                                                                   leg_detected[nb_legs_detected].x,
                //                                                                                   leg_detected[nb_legs_detected].y,
                //                                                                                   cluster_size[loop],
                //                                                                                   cluster_dynamic[loop]);
                 for(int loop2=0; loop2<nb_beams; loop2++)
                    if ( cluster[loop2] == loop ) {

                        // static legs are white
                        
                        display[nb_pts] = current_scan[loop2];
                        
                        colors[nb_pts].r = 1;
                        colors[nb_pts].g = 1;
                        colors[nb_pts].b = 1;
                        colors[nb_pts].a = 1.0;
                        
                        nb_pts++;
                    }
            }

            nb_legs_detected++;

            nb_pts++;
            }
    if ( nb_legs_detected )
        ROS_INFO("%d legs have been detected.\n", nb_legs_detected);

    //graphical display of the results
    populateMarkerTopic();

    ROS_INFO("detecting legs done");

}//detect_legs

void detect_persons() {
// a person has two legs located at less than "legs_distance_max" one from the other
// a moving person (ie, person_dynamic array) has 2 legs that are dynamic

    ROS_INFO("detecting persons");
    nb_persons_detected = 0;

    for (int loop_leg1=0; loop_leg1<nb_legs_detected; loop_leg1++)//loop over all the legs
        for (int loop_leg2=loop_leg1+1; loop_leg2<nb_legs_detected; loop_leg2++)//loop over all the legs
            // if the distance between two legs is lower than "legs_distance_max"
            // then we find a person
            if (distancePoints(leg_detected[loop_leg1], leg_detected[loop_leg2]) < legs_distance_max){

            // we update the person_detected table to store the middle of the person
            person_detected[nb_persons_detected].x = (leg_detected[loop_leg1].x + leg_detected[loop_leg2].x) / 2;
            person_detected[nb_persons_detected].y = (leg_detected[loop_leg1].y + leg_detected[loop_leg2].y) / 2;
            /*
            //BOYAN ADD
            person_detected_leg1[nb_persons_detected].x = leg_detected[loop_leg1].x;
            person_detected_leg1[nb_persons_detected].y = leg_detected[loop_leg1].y;
            person_detected_leg2[nb_persons_detected].x = leg_detected[loop_leg2].x;
            person_detected_leg2[nb_persons_detected].y = leg_detected[loop_leg2].y;
            */

            // we update the person_dynamic table to know if the person is moving or not
            if(leg_dynamic[loop_leg1] && leg_dynamic[loop_leg2])
                person_dynamic[nb_persons_detected] = true;
            else
                person_dynamic[nb_persons_detected] = false;

            if ( person_dynamic[nb_persons_detected] )
            {
                // ROS_INFO("moving person detected: leg[%i]+leg[%i] -> (%f, %f)", loop_leg1,
                //                                                                 loop_leg2,
                //                                                                 person_detected[nb_persons_detected].x,
                //                                                                 person_detected[nb_persons_detected].y);
                //  a moving person detected is green
                /*
                display[nb_pts] = person_detected[nb_persons_detected];
                
                colors[nb_pts].r = 0;
                colors[nb_pts].g = 1;
                colors[nb_pts].b = 0;
                colors[nb_pts].a = 1.0;
                */
                nb_pts++;
            }
            else
            {
                // ROS_INFO("static person detected: leg[%i]+leg[%i] -> (%f, %f)", loop_leg1,
                //                                                                 loop_leg2,
                //                                                                 person_detected[nb_persons_detected].x,
                //                                                                 person_detected[nb_persons_detected].y);
                // a static person detected is red
                /*
                display[nb_pts] = person_detected[nb_persons_detected];
                
                colors[nb_pts].r = 1;
                colors[nb_pts].g = 0;
                colors[nb_pts].b = 0;
                colors[nb_pts].a = 1.0;
                */
                nb_pts++;
            }

            nb_persons_detected++;
        }

    if ( nb_persons_detected ) {
        ROS_INFO("%d persons have been detected.\n", nb_persons_detected);
    }

    //graphical display of the results
    populateMarkerTopic();

    ROS_INFO("persons detected");

}//detect_persons

void detect_a_moving_person() {

    ROS_INFO("detecting a moving person");

    for (int loop=0; loop<nb_persons_detected; loop++)
        if ( person_dynamic[loop] )
        {   /*
            //BOYAN ADD
            moving_person_leg1_tracked = person_detected_leg1[loop];
            moving_person_leg2_tracked = person_detected_leg2[loop];
            */
            //we update moving_person_tracked and publish it
            moving_person_tracked = person_detected[loop];
            pub_datmo.publish(moving_person_tracked);

            //BOYAN add, MAYBE exit loop after one person detected?
            tracking_mode=true;

        }
    ROS_INFO("detecting a moving person done");

}//detect_moving_person

/*
void track_a_group(){

	ROS_INFO("tracking a group: %i people detected", counter);
	
	float group_x = 0.0;
	float group_y = 0.0;
	
	//get center of group
	
	for (int loop = 0; loop < nb_persons_detected; loop ++){
		if (person_dynamic[loop]){
			group_x += person_detected[loop].x;
			group_y += person_detected[loop].y;
		}
	}
	
	//get the center of the group
	group_x /= counter;
	group_y /= counter;
	
	//find middlepoint between group center and my position
	//group_x += 

}*/

void track_a_moving_person() {

    ROS_INFO("tracking a moving person");

    bool associated = false;
    //float distance_min = uncertainty;
    float distance_min = uncertainty_max;
    int index_min;

    //search for person
    for( int loop_detection=0; loop_detection<nb_persons_detected; loop_detection++ )
    {
        float current_dist = distancePoints(moving_person_tracked, person_detected[loop_detection]);
        //ROS_INFO("distance with [%i] = %f", loop_detection, current_dist);
        //we search for the best association

        //BOYAN add
        if(current_dist<distance_min){
            distance_min=current_dist;
            index_min=loop_detection;
            associated=true;
            ROS_INFO("track associated with %i", loop_detection);
        }

    }
    //ROS_INFO("Associated person -%i- at(%f,%f):",associated, moving_person_tracked.x, moving_person_tracked.y);
    /*
    //BOYAN ADD
    bool associated_leg1 = false;
    bool associated_leg2 = false;
    int index_min_leg1;
    int index_min_leg2;
    float distance_min_leg1 = uncertainty;
    float distance_min_leg2 = uncertainty;
    
        for (int loop_leg=0; loop_leg<nb_legs_detected; loop_leg++){
            float current_dist1 = distancePoints(moving_person_leg1_tracked, leg_detected[loop_leg]);
            float current_dist2 = distancePoints(moving_person_leg2_tracked, leg_detected[loop_leg]);
            if(current_dist1<distance_min_leg1){
                distance_min_leg1=current_dist1;
                index_min_leg1=loop_leg;
                associated_leg1=true;
            }
            if(current_dist2<distance_min_leg2){
                distance_min_leg2=current_dist1;
                index_min_leg2=loop_leg;
                associated_leg2=true;
            }
        }
    
     ROS_INFO("Associated leg1 -%i- at(%f,%f):",associated_leg1, moving_person_leg1_tracked.x, moving_person_leg1_tracked.y);
         ROS_INFO("Associated legé -%i- at(%f,%f):",associated_leg2, moving_person_leg2_tracked.x, moving_person_leg2_tracked.y);
    */
    // graphical display
    nb_pts -= nb_persons_detected;
    for( int loop_detection=0; loop_detection<nb_persons_detected; loop_detection++ )
    {
        // detected persons are red
        display[nb_pts] = person_detected[loop_detection];
        
        colors[nb_pts].r = 1;
        colors[nb_pts].g = 0;
        colors[nb_pts].b = 0;
        colors[nb_pts].a = 1.0;
        
        nb_pts++;
    }

    //we update the position, uncertainty and frequency of the moving_person_tracked
    if ( associated ) {
        /*    moving_person_tracked = ...
        frequency = ...
        uncertainty = ...
        pub_datmo.publish(moving_person_tracked);*/
        /*
        //BOYAN add
        moving_person_tracked = person_detected[index_min];
        moving_person_leg1_tracked = person_detected_leg1[index_min];
        moving_person_leg2_tracked = person_detected_leg2[index_min];
        */
        moving_person_tracked=person_detected[index_min];
        pub_datmo.publish(moving_person_tracked);
        if(frequency<frequency_max){
            frequency +=1;
        }
        uncertainty=uncertainty_min;
          // moving person tracked is green
        display[nb_pts] = moving_person_tracked;

        colors[nb_pts].r = 0;
        colors[nb_pts].g = 1;
        colors[nb_pts].b = 0;
        colors[nb_pts].a = 1.0;
        ROS_INFO("Tracking Person: [%i,%i]", moving_person_tracked.x,  moving_person_tracked.y);
    }
    
    else {  
               
        frequency -= 1;
        uncertainty += uncertainty_inc;    

    if(frequency==0){
        tracking_mode=false;
    }
    else{
        tracking_mode=true;
    }

    if ( !tracking_mode )
    {
        ROS_WARN("moving person tracked has been lost");
        moving_person_tracked.x = 0;
        moving_person_tracked.y = 0;
        pub_datmo.publish(moving_person_tracked);
    }

    }
    
    //graphical display of the results
    populateMarkerTopic();

    ROS_INFO("tracking of a moving person done");

}


//CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {

    init_laser = true;
    // store the important data related to laserscanner
    range_min = scan->range_min;
    range_max = scan->range_max;
    angle_min = scan->angle_min;
    angle_max = scan->angle_max;
    angle_inc = scan->angle_increment;
    nb_beams = ((-1 * angle_min) + angle_max)/angle_inc;

    // store the range and the coordinates in cartesian framework of each hit
    float beam_angle = angle_min;
    for ( int loop=0 ; loop < nb_beams; loop++, beam_angle += angle_inc ) {
        if ( ( scan->ranges[loop] < range_max ) && ( scan->ranges[loop] > range_min ) )
            r[loop] = scan->ranges[loop];
        else
            r[loop] = range_max;
        theta[loop] = beam_angle;

        //transform the scan in cartesian framework
        current_scan[loop].x = r[loop] * cos(beam_angle);
        current_scan[loop].y = r[loop] * sin(beam_angle);
        current_scan[loop].z = 0.0;
    }

}//scanCallback

void robot_movingCallback(const std_msgs::Bool::ConstPtr& state) {

    init_robot = true;
     current_robot_moving = state->data;

}//robot_movingCallback

// Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

}

// Draw the field of view and other references
void populateMarkerReference() {

    visualization_msgs::Marker references;

    references.header.frame_id = "laser";
    references.header.stamp = ros::Time::now();
    references.ns = "example";
    references.id = 1;
    references.type = visualization_msgs::Marker::LINE_STRIP;
    references.action = visualization_msgs::Marker::ADD;
    references.pose.orientation.w = 1;

    references.scale.x = 0.02;

    references.color.r = 1.0f;
    references.color.g = 1.0f;
    references.color.b = 1.0f;
    references.color.a = 1.0;
    geometry_msgs::Point v;

    v.x =  0.02 * cos(-2.356194);
    v.y =  0.02 * sin(-2.356194);
    v.z = 0.0;
    references.points.push_back(v);

    v.x =  5.6 * cos(-2.356194);
    v.y =  5.6 * sin(-2.356194);
    v.z = 0.0;
    references.points.push_back(v);

    float beam_angle = -2.356194 + 0.006136;
    // first and last beam are already included
    for (int i=0 ; i< 723; i++, beam_angle += 0.006136){
        v.x =  5.6 * cos(beam_angle);
        v.y =  5.6 * sin(beam_angle);
        v.z = 0.0;
        references.points.push_back(v);
    }

    v.x =  5.6 * cos(2.092350);
    v.y =  5.6 * sin(2.092350);
    v.z = 0.0;
    references.points.push_back(v);

    v.x =  0.02 * cos(2.092350);
    v.y =  0.02 * sin(2.092350);
    v.z = 0.0;
    references.points.push_back(v);

    pub_datmo_marker.publish(references);

}

void populateMarkerTopic(){

    visualization_msgs::Marker marker;

    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time::now();
    marker.ns = "example";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;

    marker.color.a = 1.0;

    //ROS_INFO("%i points to display", nb_pts);
    for (int loop = 0; loop < nb_pts; loop++) {
            geometry_msgs::Point p;
            std_msgs::ColorRGBA c;

            p.x = display[loop].x;
            p.y = display[loop].y;
            p.z = display[loop].z;

            c.r = colors[loop].r;
            c.g = colors[loop].g;
            c.b = colors[loop].b;
            c.a = colors[loop].a;

            //ROS_INFO("(%f, %f, %f) with rgba (%f, %f, %f, %f)", p.x, p.y, p.z, c.r, c.g, c.b, c.a);
            marker.points.push_back(p);
            marker.colors.push_back(c);
        }

    pub_datmo_marker.publish(marker);
    populateMarkerReference();

}

};

int main(int argc, char **argv){

    ros::Time::init();
    ros::init(argc, argv, "datmo");
    
    datmo bsObject;

    ros::spin();

    return 0;
}   
