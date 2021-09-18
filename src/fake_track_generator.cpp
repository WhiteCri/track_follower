#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <eigen3/Eigen/Dense>

struct Lavacon{
    Lavacon():x(0), y(0) {}
    Lavacon(double x, double y): x(x), y(y) {}
    double x;
    double y;
    double dist(double x_, double y_) {return std::sqrt((x_-x)*(x_-x) + (y_-y)*(y_-y));}
};

class FakeTrackGenerator{
template <typename T>
void getParam(ros::NodeHandle& nh, std::string n, T& v){
    if (nh.getParam(n, v) == false) throw std::runtime_error(std::string("param ") + n + " is not set!");
}
public:
    FakeTrackGenerator() : car_pos_world(0, 0), car_yaw_world(0){
        /* load param */
        std::vector<double> lavacon_vec;
        getParam(nh, "fake_track_generator/lavacons", lavacon_vec);
        for(size_t i = 0 ; i < lavacon_vec.size()/2; ++i)
            lavacons_world.emplace_back(lavacon_vec[2*i], lavacon_vec[2*i+1]);
        std::cout << "n lavacons : " << lavacons_world.size() << std::endl;
        getParam(nh, "fake_track_generator/max_detectable_range", max_detectable_range);

        /* ros members initialization */
        lavacons_all_vis_pub = nh.advertise<visualization_msgs::Marker>("fake_track_generator/lavacon_all_vis_marker", 10);
        lavacons_near_vis_pub = nh.advertise<visualization_msgs::Marker>("fake_track_generator/lavacon_near_vis_marker", 10);
        car_vis_pub = nh.advertise<visualization_msgs::Marker>("fake_track_generator/car_vis_marker", 10);
        lavacon_pos_pub = nh.advertise<geometry_msgs::PoseArray>("lavacons_pos", 10);
        car_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("fake_track_generator/car_pos", 10);
        car_vel_sub = nh.subscribe("vehicle_velocity", 10, &FakeTrackGenerator::velCB, this);
    }

    void run(){
        /* 
            procedure
            1. update car position(2d). velocity of the car is subscribed from track_follower
            2. find lavacons with certain range to mimic lidar
            3. publish found lavacons and updated car position
        */
        constexpr int CALCULATION_FREQUENCY = 20; // hz
        ros::Rate rate(CALCULATION_FREQUENCY); //20hz calculation

        while(ros::ok()){
            ros::spinOnce();

            /* 1. update car position(2d). velocity of the car is subscribed from track_follower */
            double dt = 1.0 / CALCULATION_FREQUENCY;
            
            // calc transformation matrix. See http://planning.cs.uiuc.edu/node108.html
            Eigen::Matrix2d body_world_rotation;
            body_world_rotation << std::cos(car_yaw_world), -std::sin(car_yaw_world)
                                    , std::sin(car_yaw_world), std::cos(car_yaw_world);
            Eigen::Matrix2d world_body_rotation = body_world_rotation.inverse();
            std::cout << world_body_rotation << std::endl;
            
            Eigen::Matrix3d world_body_transform; // for future use
            world_body_transform << world_body_rotation(0), world_body_rotation(1), -car_pos_world(0),
                                world_body_rotation(2), world_body_rotation(3), -car_pos_world(1),
                                0, 0, 1;

            // calculate car's velocity in world frame
            Eigen::Vector2d car_v_body;
            car_v_body << car_twist_body.twist.linear.x, 0;
            std::cout << "car v body : " << car_v_body << std::endl;
            Eigen::Vector2d car_v_world = world_body_rotation * car_v_body;
            std::cout << "car v world : " << car_v_world << std::endl;

            // update car's position in world frame
            car_pos_world(0) += car_v_world(0) * dt;
            car_pos_world(1) += car_v_world(1) * dt;
            car_yaw_world += car_twist_body.twist.angular.z * dt;

            /* 2. find lavacons with certain range to mimic lidar */
            lavacons_body_near.resize(0);            
            for(auto&& lavacon : lavacons_world){
                if (lavacon.dist(car_pos_world(0), car_pos_world(1)) < max_detectable_range){
                    Eigen::Vector3d lavacon_world;
                    lavacon_world << lavacon.x, lavacon.y, 1;
                    Eigen::Vector3d lavacon_body = world_body_transform * lavacon_world;
                    lavacons_body_near.emplace_back(lavacon_body(0), lavacon_body(1)); 
                }
            }

            /* 3. publish found lavacons and updated car position */
            geometry_msgs::PoseArray posearray;
            posearray.header.frame_id = "body";
            posearray.header.stamp = ros::Time::now();
            for(auto&& l : lavacons_body_near){
                geometry_msgs::Pose p;
                p.position.x = l.x;
                p.position.y = l.y;
                posearray.poses.push_back(p);
            }
            lavacon_pos_pub.publish(posearray);

            visualize();
            rate.sleep();
        }
    }

    void visualize(){
        /* 
            1. update tf 
            2. publish all markers in world frame
            3. publish near markers in body frame
            4. publish car pos in world frame
            5. publish car pos(poseStamped)
        */
        static tf::TransformBroadcaster br;

        // 1. update tf
        tf::Transform body_world_transform;
        body_world_transform.setOrigin(tf::Vector3(car_pos_world(0), car_pos_world(1), 0.0));
        tf::Quaternion q;
        q.setRPY(0, 0, car_yaw_world);
        body_world_transform.setRotation(q);
        br.sendTransform(tf::StampedTransform(body_world_transform, ros::Time::now(), "world", "body"));

        // 2. publish all markers in world frame
        int id = 0;
        visualization_msgs::Marker markers_all_world;
        markers_all_world.header.frame_id = "world";
        markers_all_world.header.stamp = ros::Time::now();
        markers_all_world.ns = "fake_track_generator";
        markers_all_world.id = id++;
        markers_all_world.type = visualization_msgs::Marker::SPHERE_LIST;
        markers_all_world.action = visualization_msgs::Marker::ADD;
        markers_all_world.scale.x = 0.5;
        markers_all_world.scale.y = 0.5;
        for(auto&& lavacon_world: lavacons_world){
            geometry_msgs::Point p;
            std_msgs::ColorRGBA c;

            p.x = lavacon_world.x;
            p.y = lavacon_world.y;
            c.g = 1;
            c.a = 1;

            markers_all_world.points.push_back(p);
            markers_all_world.colors.push_back(c);
        }
        markers_all_world.lifetime = ros::Duration(0);
        lavacons_all_vis_pub.publish(markers_all_world);

        // 3. publish near markers in body frame
        visualization_msgs::Marker markers_near_body;
        markers_near_body.header.frame_id = "body";
        markers_near_body.header.stamp = ros::Time::now();
        markers_near_body.ns = "fake_track_generator";
        markers_near_body.id = id++;
        markers_near_body.type = visualization_msgs::Marker::SPHERE_LIST;
        markers_near_body.action = visualization_msgs::Marker::ADD;
        markers_near_body.scale.x = 0.3;
        markers_near_body.scale.y = 0.3;
        for(auto&& lavacon_body: lavacons_body_near){
            geometry_msgs::Point p;
            std_msgs::ColorRGBA c;

            p.x = lavacon_body.x;
            p.y = lavacon_body.y;
            c.r = 1;
            c.a = 1;

            markers_near_body.points.push_back(p);
            markers_near_body.colors.push_back(c);
        }
        markers_near_body.lifetime = ros::Duration(0);
        lavacons_near_vis_pub.publish(markers_near_body);

        // 4. publish car pose in world frame
        visualization_msgs::Marker marker_car_world;
        marker_car_world.header.frame_id = "world";
        marker_car_world.header.stamp = ros::Time::now();
        marker_car_world.ns = "fake_track_generator";
        marker_car_world.id = id++;
        marker_car_world.type = visualization_msgs::Marker::LINE_STRIP;
        marker_car_world.action = visualization_msgs::Marker::ADD;
        marker_car_world.scale.x = 0.1;
        marker_car_world.scale.y = 0.1;

        std_msgs::ColorRGBA c;
        c.b = 1;
        c.a = 1;

        constexpr double CAR_WIDTH = 0.8; // meter
        constexpr double CAR_HEIGHT = 1.3; // meter
        geometry_msgs::Point p;
        p.x = car_pos_world(0) + CAR_HEIGHT/2; //
        p.y = car_pos_world(1) + CAR_WIDTH/2;
        marker_car_world.points.push_back(p);
        marker_car_world.colors.push_back(c);
        p.x = car_pos_world(0) + CAR_HEIGHT/2; //
        p.y = car_pos_world(1) - CAR_WIDTH/2;
        marker_car_world.points.push_back(p);
        marker_car_world.colors.push_back(c);
        p.x = car_pos_world(0) - CAR_HEIGHT/2; //
        p.y = car_pos_world(1) - CAR_WIDTH/2;
        marker_car_world.points.push_back(p);
        marker_car_world.colors.push_back(c);
        p.x = car_pos_world(0) - CAR_HEIGHT/2; //
        p.y = car_pos_world(1) + CAR_WIDTH/2;
        marker_car_world.points.push_back(p);
        marker_car_world.colors.push_back(c);
        p.x = car_pos_world(0) + CAR_HEIGHT/2; //
        p.y = car_pos_world(1) + CAR_WIDTH/2;
        marker_car_world.points.push_back(p);
        marker_car_world.colors.push_back(c);
        
        marker_car_world.lifetime = ros::Duration(0);
        car_vis_pub.publish(marker_car_world);

        // 5. publish car pos(poseStamped)
        geometry_msgs::PoseStamped car_pos;
        car_pos.header.frame_id = "world";
        car_pos.header.stamp = ros::Time::now();
        car_pos.pose.position.x = car_pos_world(0);
        car_pos.pose.position.y = car_pos_world(1);
        car_pos_pub.publish(car_pos);
    }

    void velCB(const geometry_msgs::TwistStampedConstPtr& ptr){ 
        car_twist_body = *ptr;
    }
private:
    ros::NodeHandle nh;
    std::vector<Lavacon> lavacons_world, lavacons_body_near; // world frame

    ros::Publisher lavacons_all_vis_pub, lavacons_near_vis_pub, car_vis_pub;
    ros::Publisher lavacon_pos_pub, car_pos_pub;
    ros::Subscriber car_vel_sub;

    Eigen::Vector2d car_pos_world;  //world frame. 
    double car_yaw_world; //world frame
    geometry_msgs::TwistStamped car_twist_body; //body frame

    double max_detectable_range;
};

int main(int argc, char *argv[]){
    ros::init(argc, argv, "fake_track_generator");

    FakeTrackGenerator f;
    f.run();
}