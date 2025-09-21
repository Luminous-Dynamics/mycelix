/**
 * Robot Simulator Node for Mycelix
 * Simulates robot movement and state for demonstration purposes
 */

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <cmath>
#include <random>
#include <string>
#include <memory>

using namespace std::chrono_literals;

namespace mycelix {

enum class MovementPattern {
    CIRCULAR,
    LINEAR,
    RANDOM,
    FIGURE_EIGHT,
    SPIRAL
};

class RobotSimulatorNode : public rclcpp::Node {
public:
    RobotSimulatorNode() : Node("robot_simulator") {
        // Declare parameters
        this->declare_parameter<std::string>("robot_id", "robot_001");
        this->declare_parameter<double>("initial_x", 0.0);
        this->declare_parameter<double>("initial_y", 0.0);
        this->declare_parameter<double>("initial_theta", 0.0);
        this->declare_parameter<std::string>("movement_pattern", "circular");
        this->declare_parameter<double>("update_rate", 10.0);
        this->declare_parameter<double>("max_velocity", 1.0);
        this->declare_parameter<double>("max_angular_velocity", 1.0);
        this->declare_parameter<bool>("publish_sensors", true);
        this->declare_parameter<bool>("publish_tf", true);
        
        // Get parameters
        robot_id_ = this->get_parameter("robot_id").as_string();
        pose_.position.x = this->get_parameter("initial_x").as_double();
        pose_.position.y = this->get_parameter("initial_y").as_double();
        theta_ = this->get_parameter("initial_theta").as_double();
        
        std::string pattern_str = this->get_parameter("movement_pattern").as_string();
        movement_pattern_ = stringToPattern(pattern_str);
        
        update_rate_ = this->get_parameter("update_rate").as_double();
        max_velocity_ = this->get_parameter("max_velocity").as_double();
        max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
        publish_sensors_ = this->get_parameter("publish_sensors").as_bool();
        publish_tf_ = this->get_parameter("publish_tf").as_bool();
        
        // Set node namespace if not already set
        if (this->get_namespace() == "/") {
            RCLCPP_INFO(this->get_logger(), 
                "🤖 Robot Simulator '%s' initialized at (%.2f, %.2f)",
                robot_id_.c_str(), pose_.position.x, pose_.position.y);
        } else {
            RCLCPP_INFO(this->get_logger(), 
                "🤖 Robot Simulator initialized in namespace '%s'",
                this->get_namespace());
        }
        
        // Initialize random generator
        random_gen_ = std::mt19937(std::random_device{}());
        random_dist_ = std::uniform_real_distribution<>(-1.0, 1.0);
        
        // Initialize consciousness state
        consciousness_coherence_ = 0.5 + random_dist_(random_gen_) * 0.2;
        consciousness_resonance_ = 0.5 + random_dist_(random_gen_) * 0.2;
        
        // Setup publishers
        setupPublishers();
        
        // Setup subscriptions
        setupSubscriptions();
        
        // Setup TF broadcaster
        if (publish_tf_) {
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        }
        
        // Start update timer
        update_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_)),
            [this]() { updateRobotState(); });
        
        // Start sensor timer (5Hz)
        if (publish_sensors_) {
            sensor_timer_ = this->create_wall_timer(
                200ms, [this]() { publishSensorData(); });
        }
        
        // Start consciousness timer (2Hz)
        consciousness_timer_ = this->create_wall_timer(
            500ms, [this]() { updateConsciousness(); });
        
        // Initialize start time for pattern generation
        start_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), 
            "✅ Robot simulator ready - Pattern: %s", pattern_str.c_str());
    }

private:
    void setupPublishers() {
        // Odometry publisher
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        
        // Sensor publishers
        if (publish_sensors_) {
            laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
            imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        }
        
        // Consciousness state publisher
        consciousness_pub_ = this->create_publisher<std_msgs::msg::String>(
            "mycelix/consciousness_state", 10);
        
        // Learning metrics publisher
        learning_metrics_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "mycelix/learning_metrics", 10);
    }
    
    void setupSubscriptions() {
        // Subscribe to velocity commands
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                handleVelocityCommand(msg);
            });
        
        // Subscribe to task assignments
        task_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/mycelix/swarm_task", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                handleTaskAssignment(msg);
            });
        
        // Subscribe to formation commands
        formation_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/mycelix/formation_command", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                handleFormationCommand(msg);
            });
    }
    
    void updateRobotState() {
        // Update velocity based on movement pattern or commands
        geometry_msgs::msg::Twist velocity;
        
        if (manual_control_) {
            // Use commanded velocity
            velocity = commanded_velocity_;
        } else {
            // Generate velocity based on pattern
            velocity = generatePatternVelocity();
        }
        
        // Update position using simple integration
        double dt = 1.0 / update_rate_;
        
        // Update orientation
        theta_ += velocity.angular.z * dt;
        theta_ = normalizeAngle(theta_);
        
        // Update position
        pose_.position.x += velocity.linear.x * std::cos(theta_) * dt;
        pose_.position.y += velocity.linear.x * std::sin(theta_) * dt;
        pose_.position.z = 0.0;
        
        // Update pose orientation
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        pose_.orientation.x = q.x();
        pose_.orientation.y = q.y();
        pose_.orientation.z = q.z();
        pose_.orientation.w = q.w();
        
        // Store current velocity
        current_velocity_ = velocity;
        
        // Publish odometry
        publishOdometry();
        
        // Publish TF
        if (publish_tf_) {
            publishTransform();
        }
    }
    
    geometry_msgs::msg::Twist generatePatternVelocity() {
        geometry_msgs::msg::Twist vel;
        double elapsed = (this->now() - start_time_).seconds();
        
        switch (movement_pattern_) {
            case MovementPattern::CIRCULAR: {
                // Circular motion
                vel.linear.x = max_velocity_ * 0.5;
                vel.angular.z = max_angular_velocity_ * 0.3;
                break;
            }
            
            case MovementPattern::LINEAR: {
                // Back and forth linear motion
                double period = 10.0;  // 10 second period
                vel.linear.x = max_velocity_ * std::sin(2.0 * M_PI * elapsed / period);
                vel.angular.z = 0.0;
                
                // Add small rotation at endpoints
                if (std::abs(std::sin(2.0 * M_PI * elapsed / period)) < 0.1) {
                    vel.angular.z = max_angular_velocity_ * 0.5;
                }
                break;
            }
            
            case MovementPattern::RANDOM: {
                // Random walk with smooth transitions
                if (static_cast<int>(elapsed * 2) % 2 == 0) {
                    // Update random targets every 0.5 seconds
                    random_linear_target_ = random_dist_(random_gen_) * max_velocity_;
                    random_angular_target_ = random_dist_(random_gen_) * max_angular_velocity_;
                }
                
                // Smooth transition to targets
                double alpha = 0.1;
                vel.linear.x = (1 - alpha) * current_velocity_.linear.x + 
                              alpha * random_linear_target_;
                vel.angular.z = (1 - alpha) * current_velocity_.angular.z + 
                               alpha * random_angular_target_;
                break;
            }
            
            case MovementPattern::FIGURE_EIGHT: {
                // Figure-8 pattern
                double omega = 0.2;  // Angular frequency
                double a = 2.0;      // Size parameter
                
                double x_dot = a * omega * std::cos(omega * elapsed);
                double y_dot = a * omega * std::cos(2 * omega * elapsed);
                
                vel.linear.x = std::sqrt(x_dot * x_dot + y_dot * y_dot);
                vel.angular.z = std::atan2(y_dot, x_dot) - theta_;
                
                // Limit velocities
                vel.linear.x = std::min(vel.linear.x, max_velocity_);
                vel.angular.z = std::clamp(vel.angular.z, 
                    -max_angular_velocity_, max_angular_velocity_);
                break;
            }
            
            case MovementPattern::SPIRAL: {
                // Expanding spiral
                double radius = 0.5 + elapsed * 0.1;
                vel.linear.x = max_velocity_ * std::min(radius / 5.0, 1.0);
                vel.angular.z = max_angular_velocity_ * 0.3 / std::max(radius / 5.0, 1.0);
                
                // Reset after 20 seconds
                if (elapsed > 20.0) {
                    start_time_ = this->now();
                }
                break;
            }
        }
        
        return vel;
    }
    
    void publishOdometry() {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = robot_id_ + "_base_link";
        
        // Set position
        odom.pose.pose = pose_;
        
        // Set velocity
        odom.twist.twist = current_velocity_;
        
        // Set covariance (simplified)
        odom.pose.covariance[0] = 0.01;   // x
        odom.pose.covariance[7] = 0.01;   // y
        odom.pose.covariance[35] = 0.01;  // theta
        
        odom_pub_->publish(odom);
    }
    
    void publishTransform() {
        geometry_msgs::msg::TransformStamped transform;
        
        transform.header.stamp = this->now();
        transform.header.frame_id = "odom";
        transform.child_frame_id = robot_id_ + "_base_link";
        
        transform.transform.translation.x = pose_.position.x;
        transform.transform.translation.y = pose_.position.y;
        transform.transform.translation.z = pose_.position.z;
        
        transform.transform.rotation = pose_.orientation;
        
        tf_broadcaster_->sendTransform(transform);
    }
    
    void publishSensorData() {
        // Publish simulated laser scan
        sensor_msgs::msg::LaserScan scan;
        scan.header.stamp = this->now();
        scan.header.frame_id = robot_id_ + "_laser";
        
        scan.angle_min = -M_PI;
        scan.angle_max = M_PI;
        scan.angle_increment = M_PI / 180.0;  // 1 degree resolution
        scan.time_increment = 0.0;
        scan.scan_time = 0.1;
        scan.range_min = 0.1;
        scan.range_max = 10.0;
        
        // Generate simulated ranges (simple walls at boundaries)
        int num_readings = 360;
        scan.ranges.resize(num_readings);
        for (int i = 0; i < num_readings; i++) {
            double angle = scan.angle_min + i * scan.angle_increment;
            double world_angle = theta_ + angle;
            
            // Simple boundary walls at ±10m
            double wall_dist_x = (10.0 - std::abs(pose_.position.x)) / 
                std::abs(std::cos(world_angle));
            double wall_dist_y = (10.0 - std::abs(pose_.position.y)) / 
                std::abs(std::sin(world_angle));
            
            scan.ranges[i] = std::min({wall_dist_x, wall_dist_y, scan.range_max});
            
            // Add some noise
            scan.ranges[i] += random_dist_(random_gen_) * 0.05;
        }
        
        laser_pub_->publish(scan);
        
        // Publish simulated IMU data
        sensor_msgs::msg::Imu imu;
        imu.header.stamp = this->now();
        imu.header.frame_id = robot_id_ + "_imu";
        
        // Simulate accelerations (with noise)
        imu.linear_acceleration.x = current_velocity_.linear.x * update_rate_ + 
            random_dist_(random_gen_) * 0.1;
        imu.linear_acceleration.y = 0.0 + random_dist_(random_gen_) * 0.1;
        imu.linear_acceleration.z = 9.81 + random_dist_(random_gen_) * 0.05;
        
        // Simulate angular velocities
        imu.angular_velocity.x = random_dist_(random_gen_) * 0.01;
        imu.angular_velocity.y = random_dist_(random_gen_) * 0.01;
        imu.angular_velocity.z = current_velocity_.angular.z + 
            random_dist_(random_gen_) * 0.02;
        
        // Set orientation
        imu.orientation = pose_.orientation;
        
        imu_pub_->publish(imu);
    }
    
    void updateConsciousness() {
        // Simulate consciousness dynamics
        double dt = 0.5;  // 2Hz update
        
        // Random walk with mean reversion
        double coherence_target = 0.6 + 0.2 * std::sin(this->now().seconds() * 0.1);
        double resonance_target = 0.6 + 0.2 * std::cos(this->now().seconds() * 0.15);
        
        double alpha = 0.1;  // Mean reversion strength
        consciousness_coherence_ += alpha * (coherence_target - consciousness_coherence_) + 
            random_dist_(random_gen_) * 0.02;
        consciousness_resonance_ += alpha * (resonance_target - consciousness_resonance_) + 
            random_dist_(random_gen_) * 0.02;
        
        // Clamp values
        consciousness_coherence_ = std::clamp(consciousness_coherence_, 0.0, 1.0);
        consciousness_resonance_ = std::clamp(consciousness_resonance_, 0.0, 1.0);
        
        // Publish consciousness state
        std_msgs::msg::String consciousness_msg;
        consciousness_msg.data = "Coherence: " + 
            std::to_string(consciousness_coherence_).substr(0, 4) +
            " Resonance: " + 
            std::to_string(consciousness_resonance_).substr(0, 4);
        consciousness_pub_->publish(consciousness_msg);
        
        // Publish learning metrics (simulated)
        std_msgs::msg::Float64 metrics_msg;
        metrics_msg.data = consciousness_coherence_ * consciousness_resonance_;
        learning_metrics_pub_->publish(metrics_msg);
    }
    
    void handleVelocityCommand(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Store commanded velocity
        commanded_velocity_ = *msg;
        manual_control_ = true;
        
        // Reset manual control after 2 seconds of no commands
        manual_control_timer_ = this->create_wall_timer(
            2s, [this]() { 
                manual_control_ = false; 
                manual_control_timer_.reset();
            });
    }
    
    void handleTaskAssignment(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), 
            "📋 Task received: %s", msg->data.c_str());
        
        // Parse task and adjust behavior
        if (msg->data.find("EXPLORE") != std::string::npos) {
            movement_pattern_ = MovementPattern::RANDOM;
        } else if (msg->data.find("TRANSPORT") != std::string::npos) {
            movement_pattern_ = MovementPattern::LINEAR;
        } else if (msg->data.find("EMERGENT") != std::string::npos) {
            movement_pattern_ = MovementPattern::FIGURE_EIGHT;
        }
    }
    
    void handleFormationCommand(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), 
            "📐 Formation command: %s", msg->data.c_str());
        
        // Formation commands are handled by swarm coordinator
        // Here we just acknowledge and prepare for coordinated movement
        manual_control_ = true;
    }
    
    MovementPattern stringToPattern(const std::string& pattern) {
        if (pattern == "circular") return MovementPattern::CIRCULAR;
        if (pattern == "linear") return MovementPattern::LINEAR;
        if (pattern == "random") return MovementPattern::RANDOM;
        if (pattern == "figure_eight") return MovementPattern::FIGURE_EIGHT;
        if (pattern == "spiral") return MovementPattern::SPIRAL;
        return MovementPattern::CIRCULAR;
    }
    
    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    
    // Member variables
    std::string robot_id_;
    geometry_msgs::msg::Pose pose_;
    double theta_ = 0.0;
    MovementPattern movement_pattern_;
    double update_rate_;
    double max_velocity_;
    double max_angular_velocity_;
    bool publish_sensors_;
    bool publish_tf_;
    
    // Current state
    geometry_msgs::msg::Twist current_velocity_;
    geometry_msgs::msg::Twist commanded_velocity_;
    bool manual_control_ = false;
    
    // Consciousness state
    double consciousness_coherence_;
    double consciousness_resonance_;
    
    // Random motion targets
    double random_linear_target_ = 0.0;
    double random_angular_target_ = 0.0;
    
    // Timing
    rclcpp::Time start_time_;
    
    // Random number generation
    std::mt19937 random_gen_;
    std::uniform_real_distribution<> random_dist_;
    
    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr consciousness_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr learning_metrics_pub_;
    
    // Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr task_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr formation_sub_;
    
    // TF broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr update_timer_;
    rclcpp::TimerBase::SharedPtr sensor_timer_;
    rclcpp::TimerBase::SharedPtr consciousness_timer_;
    rclcpp::TimerBase::SharedPtr manual_control_timer_;
};

} // namespace mycelix

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<mycelix::RobotSimulatorNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}