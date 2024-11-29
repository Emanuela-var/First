#include <stdio.h>
#include <iostream>
#include <chrono>
#include <cstdlib>
#include <memory>
 
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
 
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
 
#include "kdl_robot.h"
#include "kdl_control.h"
#include "kdl_planner.h"
#include "kdl_parser/kdl_parser.hpp"
#include "kdl/frames.hpp"
 
using namespace KDL;
using FloatArray = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;
 
class VisionControlNode : public rclcpp::Node
{
public:
    VisionControlNode()
    :Node("ros2_kdl_vision_control"),
    node_handle_(std::shared_ptr<VisionControlNode>(this))
    {
 
        // Dichiarazione del parametro del task
        declare_parameter<std::string>("task", "positioning");
        get_parameter("task", task_);
        RCLCPP_INFO(get_logger(),"Current task is: '%s'", task_.c_str());
 
            if (!(task_ == "positioning" || task_ == "look-at-point"))
            {
                RCLCPP_INFO(get_logger(),"Selected task is not valid!"); return;
            }
 
        // Publisher per comandi di velocità
       // vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
 
        iteration_ = 0;
        t_ = 0;
        joint_state_available_ = false;
        aruco_available_=false;
 
 
                // Subscriber to jnt states
        arucoSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/aruco_single/pose", 20, std::bind(&VisionControlNode::aruco_subscriber, this, std::placeholders::_1));
 
        //     // Wait for the joint_state topic
        // while(!aruco_available_){
        //         RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
        //         rclcpp::spin_some(node_handle_);
        //     }
 
 
            // retrieve robot_description param
            auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
            auto parameter = parameters_client->get_parameters({"robot_description"});
 
            // create KDLrobot structure
            KDL::Tree robot_tree;
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
            }
            robot_ = std::make_shared<KDLRobot>(robot_tree);  
            
            // Create joint array
            unsigned int nj = robot_->getNrJnts();
            KDL::JntArray q_min(nj), q_max(nj);
            q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from urdf file
            q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from urdf file          
            robot_->setJntLimits(q_min,q_max);            
            joint_positions_.resize(nj);
            joint_velocities_.resize(nj);
 
            // Subscriber to jnt states
            jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&VisionControlNode::joint_state_subscriber, this, std::placeholders::_1));
 
            // Wait for the joint_state topic
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
                rclcpp::spin_some(node_handle_);
            }
 
            // Update KDLrobot object
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
            KDL::Frame f_T_ee = KDL::Frame::Identity();
            robot_->addEE(f_T_ee);
            robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
 
            init_cart_pose_ = robot_->getEEFrame();
 
            // Initialize controller
            //KDLController controller_(*robot_);
 
            Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0,0,0.1));
 
 
            Eigen::Vector3d end_position(Eigen::Vector3d(marker.p.data[0]+1,marker.p.data[1],marker.p.data[2]));
 
            double traj_duration = 1.5, acc_duration = 0.5, t = 0.0;
            planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position); // currently using trapezoidal velocity profile
 
            trajectory_point p = planner_.compute_trajectory(t);
 
            // compute errors
            Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(init_cart_pose_.p.data));
 
            KDL::JntArray q(nj);
            robot_->getInverseKinematics(init_cart_pose_, q);
            
 
            if(task_ == "positioning"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                            std::bind(&VisionControlNode::cmd_publisher, this));
            
                // Send joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_(i);
                }
            }
 
            std_msgs::msg::Float64MultiArray cmd_msg;
            cmd_msg.data = desired_commands_;
            cmdPublisher_->publish(cmd_msg);
 
            RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
 
 
    }

private:
    void cmd_publisher() {
        
         iteration_ = iteration_ + 1;
 
            // define trajectory
            double total_time = 1.5; //
            int trajectory_len = 150; //
            int loop_rate = trajectory_len / total_time;
            double dt = 1.0 / loop_rate;
            t_+=dt;
 
            if (t_ < total_time){
 
                // Set endpoint twist
                // double t = iteration_;
                // joint_velocities_.data[2] = 2 * 0.3 * cos(2 * M_PI * t / trajectory_len);
                // joint_velocities_.data[3] = -0.3 * sin(2 * M_PI * t / trajectory_len);
 
                // Integrate joint velocities
                // joint_positions_.data += joint_velocities_.data * dt;
 
                // Retrieve the trajectory point
                                    std::cout<<"Riga 184\n";
 
                trajectory_point p = planner_.compute_trajectory(t_);
 
                // Compute EE frame
                KDL::Frame cartpos = robot_->getEEFrame();           
                    std::cout<<"Riga 190\n";
 
                // Compute desired Frame
                KDL::Frame desFrame; desFrame.M = marker.M; desFrame.p = toKDL(p.pos);
 
                    std::cout<<"Riga 192\n";
 
 
                // compute errors
                Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));
                Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(marker.M));
                std::cout << "The error norm is : " << error.norm() << std::endl;
 
                if(task_ == "positioning"){
                    // Next Frame
                    KDL::Frame nextFrame; nextFrame.M = marker.M; nextFrame.p = cartpos.p + (toKDL(p.vel) + toKDL(1*error))*dt + toKDL(1*o_error);
 
                    // Compute IK
                    robot_->getInverseKinematics(nextFrame, joint_positions_);
                }
     
 
else {
            RCLCPP_WARN(this->get_logger(), "Unknown task: %s", task_.c_str());
            return;
        }
                std::cout<<"Riga 229\n";
 
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
    }
    
    else{
                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
                // Send joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = 0.0;
                }
                
               
            }
    
     // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
    
    
    }
 
 
 
    void aruco_subscriber(const geometry_msgs::msg::PoseStamped& pose_msg){
 
 
    std::cout<<"Riga 259\n";
 
     aruco_available_ = true;
     double x,y,z,q1,q2,q3,q4;
     x=pose_msg.pose.position.x;
     y=pose_msg.pose.position.y;
     z=pose_msg.pose.position.z;
     q1=pose_msg.pose.orientation.x;
     q2=pose_msg.pose.orientation.y;
     q3=pose_msg.pose.orientation.z;
     q4=pose_msg.pose.orientation.w;
     KDL::Rotation rotation= KDL::Rotation::Quaternion(q1,q2,q3,q4);
     KDL::Vector trans(x,y,z);
 
     marker.p=trans;
     marker.M=rotation;
 
 
 
    
 
 
 
 
    }
 
     void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){
 
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Positions %zu: %f", i, sensor_msg.position[i]);                
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Velocities %zu: %f", i, sensor_msg.velocity[i]);
            // }
            // std::cout<<"\n";
            // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
            //     RCLCPP_INFO(this->get_logger(), "Efforts %zu: %f", i, sensor_msg.effort[i]);
            // }
 
            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
        }
 
 
 
 
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arucoSubscriber_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
    rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr subTimer_;
    rclcpp::Node::SharedPtr node_handle_;
 
    std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    KDL::JntArray joint_positions_;
    KDL::JntArray joint_velocities_;
    std::shared_ptr<KDLRobot> robot_;
    KDLPlanner planner_;
    std::string task_;
    KDL::Frame marker;
    KDL::Frame init_cart_pose_;
    double t_;
    int iteration_;
    bool joint_state_available_;
    bool aruco_available_;
 
};
 
int main(int argc, char **argv) {
 
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionControlNode>());
    rclcpp::shutdown();
    return 1;
}