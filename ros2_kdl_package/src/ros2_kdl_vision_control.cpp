// #include <stdio.h>
// #include <iostream>
// #include <chrono>
// #include <cstdlib>
// #include <memory>
 
// #include "std_msgs/msg/float64_multi_array.hpp"
// #include "sensor_msgs/msg/joint_state.hpp"
// #include "geometry_msgs/msg/pose_stamped.hpp"
 
// #include "rclcpp/rclcpp.hpp"
// #include "rclcpp/wait_for_message.hpp"
 
// #include "kdl_robot.h"
// #include "kdl_control.h"
// #include "kdl_planner.h"
// #include "kdl_parser/kdl_parser.hpp"
// #include "kdl/frames.hpp"
 
// using namespace KDL;
// using FloatArray = std_msgs::msg::Float64MultiArray;
// using namespace std::chrono_literals;
 
// class VisionControlNode : public rclcpp::Node
// {
// public:
//     VisionControlNode()
//     :Node("ros2_kdl_vision_control"),
//     node_handle_(std::shared_ptr<VisionControlNode>(this))
//     {
 
//         // Dichiarazione del parametro del task
//         declare_parameter<std::string>("task", "positioning");
//         get_parameter("task", task_);
        
//         RCLCPP_INFO(get_logger(),"Current task is: '%s'", task_.c_str());
 
//             if (!(task_ == "positioning" || task_ == "look-at-point"))
//             {
//                 RCLCPP_INFO(get_logger(),"Selected task is not valid!"); return;
//             }
 
//         // Publisher per comandi di velocità
//        // vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
 
//         iteration_ = 0;
//         t_ = 0;
//         joint_state_available_ = false;
//         aruco_available_=false;
 
 
//         // Subscriber to /aruco_single/pose
//         arucoSubscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
//                 "/aruco_single/pose", 20, std::bind(&VisionControlNode::aruco_subscriber, this, std::placeholders::_1));
 
//         //     // Wait for the joint_state topic
//         // while(!aruco_available_){
//         //         RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
//         //         rclcpp::spin_some(node_handle_);
//         //     }
 
 
//             // retrieve robot_description param
//             auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
//             while (!parameters_client->wait_for_service(1s)) {
//                 if (!rclcpp::ok()) {
//                     RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
//                     rclcpp::shutdown();
//                 }
//                 RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
//             }
//             auto parameter = parameters_client->get_parameters({"robot_description"});
 
//             // create KDLrobot structure
//             KDL::Tree robot_tree;
//             if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
//                 std::cout << "Failed to retrieve robot_description param!";
//             }
//             robot_ = std::make_shared<KDLRobot>(robot_tree);  
            
//             // Create joint array
//             unsigned int nj = robot_->getNrJnts();
//             KDL::JntArray q_min(nj), q_max(nj);
//             q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; //-2*M_PI,-2*M_PI; // TODO: read from urdf file
//             q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96; //2*M_PI, 2*M_PI; // TODO: read from urdf file          
//             robot_->setJntLimits(q_min,q_max);            
//             joint_positions_.resize(nj);
//             joint_velocities_.resize(nj);
 
//             // Subscriber to jnt states
//             jointSubscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
//                 "/joint_states", 10, std::bind(&VisionControlNode::joint_state_subscriber, this, std::placeholders::_1));
 
//             // Wait for the joint_state topic
//             while(!joint_state_available_){
//                 RCLCPP_INFO(this->get_logger(), "No data received yet! ...");
//                 rclcpp::spin_some(node_handle_);
//             }
 
//             // Update KDLrobot object
//             robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
//             KDL::Frame f_T_ee = KDL::Frame::Identity();
//             robot_->addEE(f_T_ee);
//             robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
 
//             init_cart_pose_ = robot_->getEEFrame();
 
//             // Initialize controller
//             //KDLController controller_(*robot_);
 
//             Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) - Eigen::Vector3d(0,0,0.1));
 
 
//             Eigen::Vector3d end_position(Eigen::Vector3d(marker.p.data[0]+1,marker.p.data[1],marker.p.data[2]));
 
//             double traj_duration = 1.5, acc_duration = 0.5, t = 0.0;
//             planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position); // currently using trapezoidal velocity profile
 
//             trajectory_point p = planner_.compute_trajectory(t);
 
//             // compute errors
//             Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(init_cart_pose_.p.data));
 
//             KDL::JntArray q(nj);
//             robot_->getInverseKinematics(init_cart_pose_, q);
            
 
//             if(task_ == "positioning"){
//                 // Create cmd publisher
//                 cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
//                 timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
//                                             std::bind(&VisionControlNode::cmd_publisher, this));
            
//                 // Send joint position commands
//                 for (long int i = 0; i < joint_positions_.data.size(); ++i) {
//                     desired_commands_[i] = joint_positions_(i);
//                 }
//             }
 
//             std_msgs::msg::Float64MultiArray cmd_msg;
//             cmd_msg.data = desired_commands_;
//             cmdPublisher_->publish(cmd_msg);
 
//             RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
 
 
//     }

// private:
//     void cmd_publisher() {
        
//          iteration_ = iteration_ + 1;
 
//             // define trajectory
//             double total_time = 1.5; //
//             int trajectory_len = 150; //
//             int loop_rate = trajectory_len / total_time;
//             double dt = 1.0 / loop_rate;
//             t_+=dt;
 
//             if (t_ < total_time){
 
//                 // Set endpoint twist
//                 // double t = iteration_;
//                 // joint_velocities_.data[2] = 2 * 0.3 * cos(2 * M_PI * t / trajectory_len);
//                 // joint_velocities_.data[3] = -0.3 * sin(2 * M_PI * t / trajectory_len);
 
//                 // Integrate joint velocities
//                 // joint_positions_.data += joint_velocities_.data * dt;
 
//                 // Retrieve the trajectory point
                                    
 
//                 trajectory_point p = planner_.compute_trajectory(t_);
 
//                 // Compute EE frame
//                 KDL::Frame cartpos = robot_->getEEFrame();           

//                 // Compute desired Frame
//                 KDL::Frame desFrame; desFrame.M = marker.M; desFrame.p = toKDL(p.pos);
 
//                 // compute errors
//                 Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));
//                 Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(marker.M));
//                 std::cout << "The error norm is : " << error.norm() << std::endl;
 
//                 if(task_ == "positioning"){
//                     // Next Frame
//                     KDL::Frame nextFrame; nextFrame.M = marker.M; nextFrame.p = cartpos.p + (toKDL(p.vel) + toKDL(1*error))*dt + toKDL(1*o_error);
 
//                     // Compute IK
//                     robot_->getInverseKinematics(nextFrame, joint_positions_);
//                 }
     
//                 else {
//                     RCLCPP_WARN(this->get_logger(), "Unknown task: %s", task_.c_str());
//                     return;
//                 }
               
 
//                 std_msgs::msg::Float64MultiArray cmd_msg;
//                 cmd_msg.data = desired_commands_;
//                 cmdPublisher_->publish(cmd_msg);
//     }
    
//     else{
//                 RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
//                 // Send joint velocity commands
//                 for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
//                     desired_commands_[i] = 0.0;
//                 }
                
               
//             }
    
//      // Create msg and publish
//                 std_msgs::msg::Float64MultiArray cmd_msg;
//                 cmd_msg.data = desired_commands_;
//                 cmdPublisher_->publish(cmd_msg);
    
    
//     }
 
 
 
//     void aruco_subscriber(const geometry_msgs::msg::PoseStamped& pose_msg){
 
 
//     std::cout<<"Riga 259\n";
 
//      aruco_available_ = true;
//      double x,y,z,q1,q2,q3,q4;
//      x=pose_msg.pose.position.x;
//      y=pose_msg.pose.position.y;
//      z=pose_msg.pose.position.z;
//      q1=pose_msg.pose.orientation.x;
//      q2=pose_msg.pose.orientation.y;
//      q3=pose_msg.pose.orientation.z;
//      q4=pose_msg.pose.orientation.w;
//      KDL::Rotation rotation= KDL::Rotation::Quaternion(q1,q2,q3,q4);
//      KDL::Vector trans(x,y,z);
 
//      marker.p=trans;
//      marker.M=rotation;
 
 
 
    
 
 
 
 
//     }
 
//      void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){
 
//             // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
//             //     RCLCPP_INFO(this->get_logger(), "Positions %zu: %f", i, sensor_msg.position[i]);                
//             // }
//             // std::cout<<"\n";
//             // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
//             //     RCLCPP_INFO(this->get_logger(), "Velocities %zu: %f", i, sensor_msg.velocity[i]);
//             // }
//             // std::cout<<"\n";
//             // for (size_t i = 0; i < sensor_msg.effort.size(); ++i) {
//             //     RCLCPP_INFO(this->get_logger(), "Efforts %zu: %f", i, sensor_msg.effort[i]);
//             // }
 
//             joint_state_available_ = true;
//             for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
//                 joint_positions_.data[i] = sensor_msg.position[i];
//                 joint_velocities_.data[i] = sensor_msg.velocity[i];
//             }
//         }
 
 
 
 
//     rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr arucoSubscriber_;
//     rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSubscriber_;
//     rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::TimerBase::SharedPtr subTimer_;
//     rclcpp::Node::SharedPtr node_handle_;
 
//     std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//     KDL::JntArray joint_positions_;
//     KDL::JntArray joint_velocities_;
//     std::shared_ptr<KDLRobot> robot_;
//     KDLPlanner planner_;
//     std::string task_;
//     KDL::Frame marker;
//     KDL::Frame init_cart_pose_;
//     double t_;
//     int iteration_;
//     bool joint_state_available_;
//     bool aruco_available_;
 
// };
 
// int main(int argc, char **argv) {
 
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<VisionControlNode>());
//     rclcpp::shutdown();
//     return 1;
// }

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

class VisionControlNode : public rclcpp::Node {

public:
    VisionControlNode() 
    : Node("ros2_kdl_vision_control"),
    node_handle_(std::shared_ptr<VisionControlNode>(this))
     {

        // declare cmd_interface parameter (position, velocity or effort)
        declare_parameter("cmd_interface", "position"); // defaults to "position"
        get_parameter("cmd_interface", cmd_interface_);
            
            RCLCPP_INFO(get_logger(),"Current cmd interface is: '%s'", cmd_interface_.c_str());
 
            if (!(cmd_interface_ == "position" || "velocity" || "effort"))
            {
                RCLCPP_INFO(get_logger(),"Selected cmd interface is not valid!"); return;
            }

        // Parameters added in the constructor
        declare_parameter("time_law", "trapezoidal");  // Values: "trapezoidal" or "cubic"
        declare_parameter("path_type", "linear");      // Values: "linear" or "circular"
 
        get_parameter("time_law", time_law_);   
        get_parameter("path_type", path_type_);
 
            RCLCPP_INFO(get_logger(),"Current time_law is: '%s'", time_law_.c_str());
 
            if (!(time_law_ == "trapezoidal" || time_law_ == "cubic"))
            {
                RCLCPP_INFO(get_logger(),"Selected time_law is not valid!"); return;
            }
 
            RCLCPP_INFO(get_logger(),"Current path_type is: '%s'", path_type_.c_str());
 
            if (!(path_type_ == "linear" || path_type_ == "circular"))
            {
                RCLCPP_INFO(get_logger(),"Selected path_type_ is not valid!"); return;
            }
        
        declare_parameter("task", "positioning");
        declare_parameter<std::vector<double>>("position_offset", std::vector<double>{0.0, 0.0, 0.5});
        declare_parameter<std::vector<double>>("orientation_offset", std::vector<double>{0.0, 0.0, 0.0});

        
        get_parameter("task", task_);
        get_parameter("position_offset", position_offset_);
        get_parameter("orientation_offset", orientation_offset_);
        
        iteration_ = 0;
        t_ = 0;
        joint_state_available_ = false, aruco_pose_available = false;

        // Subscriber per la pose dell'ArUco marker
        aruco_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_single/pose", 10,
            std::bind(&VisionControlNode::arucoPoseCallback, this, std::placeholders::_1));

        // Wait for the /aruco_single/pose topic
        while(!aruco_pose_available){
                RCLCPP_INFO(this->get_logger(), "No data received yet from the /aruco_single/pose topic! ...");
                rclcpp::spin_some(node_handle_);
        }
        
        while (!aruco_pose_available) {
                RCLCPP_WARN(this->get_logger(), "Waiting for /aruco_single/pose data...");
                rclcpp::spin_some(node_handle_);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        // Verifica che i dati siano validi prima di usarli
        if (aruco_pose_available) {
            
            marker_position = Eigen::Vector3d(
                current_aruco_pose_.pose.position.x,
                current_aruco_pose_.pose.position.y,
                current_aruco_pose_.pose.position.z
            );
        } 
        else 
        {
            RCLCPP_ERROR(this->get_logger(), "No valid data received. Exiting.");
            return;
        }

        // etrieve robot_description param
        auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(node_handle_, "robot_state_publisher");
            
            while (!parameters_client->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                    rclcpp::shutdown();
                }
                RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
            }
        
        auto parameter = parameters_client->get_parameters({"robot_description"});

        //create KDLrobot structure
        KDL::Tree robot_tree;
            
            if (!kdl_parser::treeFromString(parameter[0].value_to_string(), robot_tree)){
                std::cout << "Failed to retrieve robot_description param!";
            }
        
        robot_ = std::make_shared<KDLRobot>(robot_tree);

        //Create joint array
        unsigned int nj = robot_->getNrJnts();
        KDL::JntArray q_min(nj), q_max(nj);
        q_min.data << -2.96,-2.09,-2.96,-2.09,-2.96,-2.09,-2.96; 
        q_max.data <<  2.96,2.09,2.96,2.09,2.96,2.09,2.96;        
        robot_->setJntLimits(q_min,q_max);            
        joint_positions_.resize(nj); 
        joint_velocities_.resize(nj);  

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&VisionControlNode::joint_state_subscriber, this, std::placeholders::_1));

        // Wait for the joint_state topic
            while(!joint_state_available_){
                RCLCPP_INFO(this->get_logger(), "No data received yet from the joint_state topic! ...");
                rclcpp::spin_some(node_handle_);
            }

        // Update KDLrobot object
        robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
        KDL::Frame f_T_ee = KDL::Frame::Identity();
        robot_->addEE(f_T_ee);
        robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));

        // Compute EE frame
        init_cart_pose_ = robot_->getEEFrame();

        // Initialize controller
        KDLController controller_(*robot_);
 
        // EE's trajectory initial position (just an offset)
        Eigen::Vector3d init_position(Eigen::Vector3d(init_cart_pose_.p.data) );

        //EE's trajectory end position
        Eigen::Vector3d end_position; 
        end_position << marker_position[0] + position_offset_[0],marker_position[1] + position_offset_[1],marker_position[2] + position_offset_[2];
        
        // Plan trajectory
        double traj_duration = 1.5, acc_duration = 0.5, t = 0.0, trajRadius = 0.2;

        //Example of planner configuration with a trajectory type
            if (path_type_ == "linear") {
 
                if(time_law_ == "cubic"){
                    planner_ = KDLPlanner(traj_duration, init_position, end_position);
                }
                else{
                    planner_ = KDLPlanner(traj_duration, acc_duration, init_position, end_position);
                }
            
            } else if (path_type_ == "circular") {
 
                if(time_law_ == "cubic"){
                    planner_ = KDLPlanner(traj_duration, init_position, trajRadius);
                }
                else{
                    planner_ = KDLPlanner(traj_duration, acc_duration, init_position, trajRadius);
                }
            }
        // Retrieve the first trajectory point
        trajectory_point p = planner_.compute_trajectory(t);

        // compute errors
            Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(init_cart_pose_.p.data));
            std::cout << "The initial error is : " << error << std::endl;
            
            if(cmd_interface_ == "position"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/iiwa_arm_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                            std::bind(&VisionControlNode::cmd_publisher, this));
            
                // Send joint position commands
                for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                    desired_commands_[i] = joint_positions_(i);
                }
            }
            else if(cmd_interface_ == "velocity"){
                // Create cmd publisher
                cmdPublisher_ = this->create_publisher<FloatArray>("/velocity_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                            std::bind(&VisionControlNode::cmd_publisher, this));
            
                // Send joint velocity commands
                for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                    desired_commands_[i] = joint_velocities_(i);
                }
            }else if(cmd_interface_ == "effort" || "effort_cartesian"){
                // Create cmd publisher
                
                cmdPublisher_ = this->create_publisher<FloatArray>("/effort_controller/commands", 10);
                timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                            std::bind(&VisionControlNode::cmd_publisher, this));
                
                for (long int i = 0; i < nj; ++i) {
                    desired_commands_[i] = 0;
                    
                }
 
            }else{
 
                std::cout<<"Error!";
 
            }
          
 
        // Create msg and publish
        std_msgs::msg::Float64MultiArray cmd_msg;
        cmd_msg.data = desired_commands_;
        cmdPublisher_->publish(cmd_msg);
           
        RCLCPP_INFO(this->get_logger(), "Starting trajectory execution ...");
    }

    private:

        // Callback per la pose dell'ArUco marker
        void arucoPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            
             if (!msg) {
                RCLCPP_ERROR(this->get_logger(), "Received null message!");
                return;
            }

            // Memorizza i dati ricevuti
            current_aruco_pose_ = *msg;
            aruco_pose_available = true;

            // Logga i dati ricevuti per debug
            RCLCPP_INFO(this->get_logger(), "Aruco Pose received: position (%f, %f, %f)", 
                        msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        }
 
        void cmd_publisher(){
            
            iteration_ = iteration_ + 1;
            
            // Define trajectory
            double total_time = 1.5; //
            int trajectory_len = 150; //
            int loop_rate = trajectory_len / total_time;
            double dt = 1.0 / loop_rate;
            t_+=dt;

            if (t_ < total_time){
                
                // Retrieve the trajectory point
                trajectory_point p = planner_.compute_trajectory(t_);
 
                //Compute EE frame
                KDL::Frame cartpos = robot_->getEEFrame();          

                // Ottieni la pose corrente del marker
                KDL::Frame marker_pose(
                
                KDL::Rotation::Quaternion(
                current_aruco_pose_.pose.orientation.x,
                current_aruco_pose_.pose.orientation.y,
                current_aruco_pose_.pose.orientation.z,
                current_aruco_pose_.pose.orientation.w),
                
                 KDL::Vector(
                current_aruco_pose_.pose.position.x,
                current_aruco_pose_.pose.position.y,
                current_aruco_pose_.pose.position.z));

                // Compute desired Frame
                KDL::Frame desFrame;
                desFrame = marker_pose;
                desFrame.p += KDL::Vector(position_offset_[0], position_offset_[1], position_offset_[2]);
                desFrame.M = desFrame.M * KDL::Rotation::RPY(
                orientation_offset_[0], orientation_offset_[1], orientation_offset_[2]);

                // Compute errors
                Eigen::Vector3d error = computeLinearError(p.pos, Eigen::Vector3d(cartpos.p.data));
                Eigen::Vector3d o_error = computeOrientationError(toEigen(init_cart_pose_.M), toEigen(cartpos.M));
                std::cout << "The error norm is : " << error.norm() << std::endl;

                KDLController controller_(*robot_);
 
 
                if(cmd_interface_ == "position"){
                    // Next Frame
                    KDL::Frame nextFrame; nextFrame.M = cartpos.M; nextFrame.p = cartpos.p + (toKDL(p.vel) + toKDL(1*error))*dt;
 
                    // Compute IK
                    robot_->getInverseKinematics(nextFrame, joint_positions_);
                }
                else if(cmd_interface_ == "velocity"){
                    // Compute differential IK
                    Vector6d cartvel; cartvel << p.vel + 5*error, o_error;
                    joint_velocities_.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;
                    joint_positions_.data = joint_positions_.data + joint_velocities_.data*dt;
                }
                else if(cmd_interface_ == "effort"){
                    
                    //Store the current joint velocity as a reference to calculate the next acceleration:
                    joint_velocity_old.data=joint_velocities_.data;

                    /*Combine a desired velocity p.vel with an error term for correction:
                    NOTE: The three zeros represent rotation components not considered here!*/
                    Vector6d cartvel; cartvel << p.vel + error, 0,0,0;
                    
                    //Update joint velocities, using the pseudoinverse of the end-effector Jacobian to map the desired Cartesian velocity (cartvel) in joint space:
                    dq_des.data = pseudoinverse(robot_->getEEJacobian().data)*cartvel;

                    //Calculate the new joint positions by integrating the velocities (joint_velocities_) with the time step dt:
                    q_des.data = joint_positions_.data + joint_velocities_.data*dt;

                    //Calculate joint acceleration by discrete numerical derivative:
                    joint_acceleration_d_.data=(joint_velocities_.data-joint_velocity_old.data)/dt;
                    
                    //Use the first method (idCntr) to calculate the required joint torques:
                   torque_values = controller_.idCntr(q_des,dq_des,joint_acceleration_d_, _Kp, _Kd);
                }
                else{
 
                    std::cout<<"Error!";
                }

                // Update KDLrobot structure
                robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));
 
                if(cmd_interface_ == "position"){
                    // Send joint position commands
                    for (long int i = 0; i < joint_positions_.data.size(); ++i) {
                        desired_commands_[i] = joint_positions_(i);
                    }
                }
                else if(cmd_interface_ == "velocity"){
                    // Send joint velocity commands
                    for (long int i = 0; i < joint_velocities_.data.size(); ++i) {
                        desired_commands_[i] = joint_velocities_(i);
                    }
                }
                else if(cmd_interface_ == "effort"){
                     // Send joint velocity commands
                    for (long int i = 0; i < torque_values.size(); ++i) {
                        desired_commands_[i] = torque_values(i);
                    }
 
                }else{
 
                    std::cout<<"Error!";
                }
                // Create msg and publish
                std_msgs::msg::Float64MultiArray cmd_msg;
                cmd_msg.data = desired_commands_;
                cmdPublisher_->publish(cmd_msg);
            }
            else{
                RCLCPP_INFO_ONCE(this->get_logger(), "Trajectory executed successfully ...");
                // Send joint effort commands
                if(cmd_interface_ == "effort"){
                
                    KDLController controller_(*robot_);
                    q_des.data=joint_positions_.data;
                    // Azzerare i valori di qd (velocità dei giunti)
                    dq_des.data = Eigen::VectorXd::Zero(7,1);
                    // // Azzerare i valori di qdd (accelerazioni dei giunti)
                    joint_acceleration_d_.data = Eigen::VectorXd::Zero(7,1);
 
                    torque_values = controller_.idCntr(q_des,dq_des,joint_acceleration_d_, _Kp, _Kd);
                    
                    // // Update KDLrobot structure
                    robot_->update(toStdVector(joint_positions_.data),toStdVector(joint_velocities_.data));  
                    
                    for (long int i = 0; i < torque_values.size(); ++i) {
    
                        desired_commands_[i] = torque_values(i);

                    }
                }else{
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



            }
        

        void joint_state_subscriber(const sensor_msgs::msg::JointState& sensor_msg){
 
            joint_state_available_ = true;
            for (unsigned int i  = 0; i < sensor_msg.position.size(); i++){
                joint_positions_.data[i] = sensor_msg.position[i];
                joint_velocities_.data[i] = sensor_msg.velocity[i];
            }
        }

        

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr aruco_pose_sub_;
        rclcpp::Publisher<FloatArray>::SharedPtr cmdPublisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::TimerBase::SharedPtr subTimer_;
        rclcpp::Node::SharedPtr node_handle_;

        std::vector<double> desired_commands_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        std::vector<double> position_offset_, orientation_offset_;
        geometry_msgs::msg::PoseStamped current_aruco_pose_; 

       Eigen::Vector3d marker_position;

        KDL::JntArray joint_positions_;
        KDL::JntArray joint_velocities_;
        KDL::JntArray joint_acceleration_d_;
        KDL::JntArray joint_velocity_old;
        Eigen::VectorXd torque_values;
        KDL::JntArray q_des;
        KDL::JntArray dq_des;
        KDL::Twist desVel;
        KDL::Twist desAcc;
        KDL::Frame desPos;

        std::shared_ptr<KDLRobot> robot_;
        KDLPlanner planner_;
 
        int iteration_;
        bool joint_state_available_, aruco_pose_available;
        double t_;
        std::string cmd_interface_;
        std::string time_law_;
        std::string path_type_;
        std::string task_;
        KDL::Frame init_cart_pose_;
         
        //Gains
        double _Kp = 170 ;  // Example value for proportional gain
        double _Kd =  30;   // Example value for derivative gain
        double _Kpp = 90;
        double _Kpo = 90;
        double _Kdp = 2*sqrt(_Kpp);
        double _Kdo = 2*sqrt(_Kpo);
    
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionControlNode>());
    rclcpp::shutdown();
    return 1;
}