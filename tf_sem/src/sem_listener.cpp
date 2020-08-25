#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

int id = 0;

void markerInit(visualization_msgs::Marker& marker){
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "sem_markers";
    marker.id = id++;
    marker.action = visualization_msgs::Marker::ADD;
    marker.color.a = 1.0;
    marker.pose.orientation.w = 1.0;
}

void shapeConfig(visualization_msgs::Marker& s, visualization_msgs::Marker& e, visualization_msgs::Marker& m, visualization_msgs::Marker& o_e, visualization_msgs::Marker& o_m){
    s.scale.x = s.scale.y = s.scale.z = 1;
    e.scale.x = e.scale.y = e.scale.z = 0.5;
    m.scale.x = m.scale.y = m.scale.z = 0.3;
    o_e.scale.x = 0.2;
    o_m.scale.x = 0.1;
    s.color.r = 1.0;
    e.color.b = o_e.color.b = 1.0;
    m.color.g = o_m.color.g = 1.0;
}

void poseConfig(visualization_msgs::Marker& marker, tf::StampedTransform& tf, int shape_id){
    switch (shape_id)
    {       
    case 0:
        marker.pose.position.x = tf.getOrigin().x();
        marker.pose.position.y = tf.getOrigin().y();
        marker.pose.position.z = tf.getOrigin().z();
        marker.pose.orientation.x = tf.getRotation().x();
        marker.pose.orientation.y = tf.getRotation().y();
        marker.pose.orientation.z = tf.getRotation().z();
        break;
    
    case 1:
        geometry_msgs::Point p;
        p.x = tf.getOrigin().x();
        p.y = tf.getOrigin().y();
        p.z = tf.getOrigin().z();
        marker.points.push_back(p);
        break;
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "sem_listener");
    ros::NodeHandle node;

    ros::Publisher marker_pub1 = node.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    visualization_msgs::Marker sun, earth, moon, orbit_e, orbit_m;

    sun.type = earth.type = moon.type = visualization_msgs::Marker::SPHERE;
    orbit_e.type = orbit_m.type = visualization_msgs::Marker::LINE_STRIP;

    markerInit(sun);
    markerInit(earth);
    markerInit(moon);
    markerInit(orbit_m);
    markerInit(orbit_e);
    shapeConfig(sun, earth, moon, orbit_e, orbit_m);

    tf::TransformListener listener;

    ros::Rate rate(10);

    while(ros::ok()){
        tf::StampedTransform tf1, tf2;
        // tf::StampedTransform tf3;
        try{
            listener.waitForTransform("world", "earth", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("world", "earth", ros::Time(0), tf1);
            listener.waitForTransform("world", "moon", ros::Time(0), ros::Duration(3.0));
            listener.lookupTransform("world", "moon", ros::Time(0), tf2);            
            // listener.waitForTransform("earth", "moon", ros::Time(0), ros::Duration(3.0));
            // listener.lookupTransform("earth", "moon", ros::Time(0), tf3);  
        }
        catch(tf::TransformException &ex){
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        marker_pub1.publish(sun);

        poseConfig(earth, tf1, 0);
        marker_pub1.publish(earth);
        
        poseConfig(moon, tf2, 0);
        marker_pub1.publish(moon);
        
        poseConfig(orbit_e, tf1, 1);
        marker_pub1.publish(orbit_e); 
        
        poseConfig(orbit_m, tf2, 1);
        marker_pub1.publish(orbit_m);          


        // poseConfig(moon, tf3, 0);
        // marker_pub1.publish(moon);


        rate.sleep();
    }
    
    return 0;
}