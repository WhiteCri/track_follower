/*
    IMPORTANT: every calculation is done in body frame.
    1. extract marker(use KF?) and set goal
    2. use pure pursuit to follow goals

    
*/
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <eigen3/Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TwistStamped.h>

class TrackFollower{
template <typename T>
void getParam(ros::NodeHandle& nh, std::string n, T& v){
    if (nh.getParam(n, v) == false) throw std::runtime_error(std::string("param ") + n + " is not set!");
}
public:
    TrackFollower(){
        getParam(nh, "track_follower/param1", param1);
        param1 = param1 / 180 * M_PI; //degree to radian
        getParam(nh, "track_follower/param2", param2);
        getParam(nh, "track_follower/param3", param3);
        getParam(nh, "track_follower/vehicle_velocity", vehicle_velocity);

        lavacons_pos_sub = nh.subscribe("lavacons_pos", 10, &TrackFollower::poseArrayCB, this);
        left_right_pub = nh.advertise<visualization_msgs::Marker>("track_follower/left_right_vis", 10);
        goal_pub = nh.advertise<visualization_msgs::Marker>("track_follower/track_follower_goal_vis", 10);
        twist_pub = nh.advertise<geometry_msgs::TwistStamped>("vehicle_velocity", 10);
    }

    void poseArrayCB(const geometry_msgs::PoseArrayConstPtr& ptr){
        goal = extractLeftandRightLavacons(*ptr);
        calcVelocityWithPurePursuit(goal);
        visualize();
    }

    geometry_msgs::Pose extractLeftandRightLavacons(geometry_msgs::PoseArray lavacons){
        // Assume lavacon searching is reliable. If not, you can use filtering(e.g. Kalman Filter).
        /* 
            Goal Selection
            1. choose lavacons within 150 degree(param1) range according to car's yaw
            2. find lavacons within 6 meter
            3. sort lavacons according to y coordinate
            4. choose best 2 and set goal in the middle of them
        */

        // 1. choose lavacons within 150 degree range according to car's yaw
        std::vector<geometry_msgs::Pose> candidates1;
        for(auto&& lavacon : lavacons.poses){
            double angle = direction(lavacon.position.x, lavacon.position.y); 
            if (angle >= -param1/2 && angle <= param1/2) candidates1.push_back(lavacon);
        }
        std::cout << candidates1.size() << " lavacons[1] : ";
        for(auto&& p : candidates1) std::cout << "(" << p.position.x << ", " << p.position.y << "), "; 
        std::cout << std::endl;
         
        // 2. find lavacons within 6 meter
        std::vector<geometry_msgs::Pose> candidates2;
        for(auto&& lavacon : candidates1){
            double dist = l2norm(lavacon.position.x, lavacon.position.y);
            if (param3 <= dist && dist <= param2) candidates2.push_back(lavacon);
        }
        std::cout << candidates2.size() << " lavacons[2] : ";
        for(auto&& p : candidates2) std::cout << "(" << p.position.x << ", " << p.position.y << "), "; 
        std::cout << std::endl;
        if (candidates2.size() <= 1) {
            ROS_ERROR("can not find left/right lavacon!");
            return goal; //return previous goal
        }

        // 3. sort lavacons according to y coordinate
        std::sort(candidates2.begin(), candidates2.end(), [](auto&& l, auto&& r){
            return l.position.y > r.position.y;
        });
        left = candidates2[0];
        right = candidates2[1];
        
        std::cout << "left : " << "(" << left.position.x << ", " << left.position.y << "), "; std::cout << std::endl;
        std::cout << "right : " << "(" << right.position.x << ", " << right.position.y << "), "; std::cout << std::endl;

        geometry_msgs::Pose g;
        g.position.x = (left.position.x + right.position.x) / 2.0;
        g.position.y = (left.position.y + right.position.y) / 2.0;
        return g;
    }

    void calcVelocityWithPurePursuit(geometry_msgs::Pose goal){
        // l = r * theta
        // v = r * w
        // r is calculated by pure pursuit : https://msc9533.github.io/irl-study-2020/algorithm/2020/05/15/pure_pursuit.html
        double r = l2norm(goal.position.x, goal.position.y) / (2*goal.position.y);
        double w = vehicle_velocity / r;

        geometry_msgs::TwistStamped t;
        t.header.frame_id = "body";
        t.header.stamp = ros::Time::now();
        t.twist.linear.x = vehicle_velocity;
        t.twist.angular.z = w;
        twist_pub.publish(t);
    }

    void visualize(){
        //publish left and right marker
        visualization_msgs::Marker left_and_right_marker;
        int id = 0;
        left_and_right_marker.header.frame_id = "body";
        left_and_right_marker.header.stamp = ros::Time::now();
        left_and_right_marker.ns = "track_follower";
        left_and_right_marker.id = id++;
        left_and_right_marker.type = visualization_msgs::Marker::CUBE_LIST;
        left_and_right_marker.action = visualization_msgs::Marker::ADD;
        left_and_right_marker.scale.x = 1.0;
        left_and_right_marker.scale.y = 1.0;
        left_and_right_marker.scale.z = 1.0;

        std_msgs::ColorRGBA c;
        c.r = 0.5;
        c.g = 0.5;
        c.a = 0.8;

        geometry_msgs::Point p;
        p.x = left.position.x;
        p.y = left.position.y;
        left_and_right_marker.points.push_back(p);
        left_and_right_marker.colors.push_back(c);
        p.x = right.position.x;
        p.y = right.position.y;
        left_and_right_marker.points.push_back(p);
        left_and_right_marker.colors.push_back(c);
        
        left_and_right_marker.lifetime = ros::Duration(0);
        left_right_pub.publish(left_and_right_marker);

        //publish goal marker
        visualization_msgs::Marker goal_marker;
        goal_marker.header.frame_id = "body";
        goal_marker.header.stamp = ros::Time::now();
        goal_marker.ns = "track_follower";
        goal_marker.id = id++;
        goal_marker.type = visualization_msgs::Marker::CUBE;
        goal_marker.action = visualization_msgs::Marker::ADD;
        goal_marker.scale.x = 0.3;
        goal_marker.scale.y = 0.3;
        goal_marker.scale.z = 0.3;
        goal_marker.color.b = 1;
        goal_marker.color.a = 1;
        goal_marker.pose.position.x = goal.position.x;
        goal_marker.pose.position.y = goal.position.y;
        goal_marker.lifetime = ros::Duration(0);
        goal_pub.publish(goal_marker);
    }

    double l2norm(double x, double y){
        return std::sqrt(x*x + y*y);
    }
    double direction(double x, double y){
        return std::atan2(y, x);
    }
private:
    double param1, param2, param3;
    geometry_msgs::Pose left, right, goal;
    double vehicle_velocity;
    
    ros::NodeHandle nh;
    ros::Subscriber lavacons_pos_sub;
    ros::Publisher left_right_pub, goal_pub;
    ros::Publisher twist_pub;
};

int main(int argc, char * argv[]){
    ros::init(argc, argv, "track_follower");

    TrackFollower t;
    ros::spin();
}