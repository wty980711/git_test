#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
#include<cmath>

double x_s = 0, x_e = 6, x_m = 2;
double theta_s, theta_e;
int count = 0;

void rotate(){
    theta_s = 2*M_PI*count/100;
    theta_e = 10*M_PI*count/100;
    count++;
    count %= 100;
}

int main( int argc, char** argv ){
    ros::init(argc, argv, "sem_broadcaster");
    ros::NodeHandle node;

    static tf::TransformBroadcaster br;

    tf::Transform tf1, tf2, tf3;
    tf::Quaternion q;

    ros::Rate rate(10);

    while(ros::ok()){
        rotate();

        tf1.setOrigin(tf::Vector3(x_s, 0 , 0));
        q.setRPY(0, 0, theta_s);
        tf1.setRotation(q);
        br.sendTransform(tf::StampedTransform(tf1, ros::Time::now(), "world", "sun"));

        tf2.setOrigin(tf::Vector3(x_e, 0, 0));
        q.setRPY(0, 0, theta_e);
        // q.setRPY(0, 0, 0);
        tf2.setRotation(q);
        br.sendTransform(tf::StampedTransform(tf2, ros::Time::now(), "sun", "earth"));

        tf3.setOrigin(tf::Vector3(x_m, 0, 0));
        q.setRPY(0, 0, 0);
        tf3.setRotation(q);
        br.sendTransform(tf::StampedTransform(tf3, ros::Time::now(), "earth", "moon"));

        rate.sleep();

    }

    return 0;

}