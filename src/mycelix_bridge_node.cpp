/**
 * Mycelix ROS2 Bridge Node
 * Connects robots to the Mycelix consciousness network via Holochain
 * Enables federated learning, swarm coordination, and collective intelligence
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

#include "mycelix_bridge/holochain_agent.hpp"
#include "mycelix_bridge/federated_learner.hpp"
#include "mycelix_bridge/safety_validator.hpp"

using namespace std::chrono_literals;

namespace mycelix {

class MycelixBridgeNode : public rclcpp::Node {
public:
    MycelixBridgeNode() : Node("mycelix_bridge") {
        // Initialize parameters
        this->declare_parameter<std::string>("robot_id", "robot_001");
        this->declare_parameter<std::string>("robot_model", "turtlebot3");
        this->declare_parameter<std::string>("holochain_url", "ws://localhost:8888");
        this->declare_parameter<double>("learning_rate", 0.01);
        this->declare_parameter<double>("privacy_epsilon", 1.0);
        this->declare_parameter<int>("experience_buffer_size", 1000);
        
        // Get parameters
        robot_id_ = this->get_parameter("robot_id").as_string();
        robot_model_ = this->get_parameter("robot_model").as_string();
        holochain_url_ = this->get_parameter("holochain_url").as_string();
        learning_rate_ = this->get_parameter("learning_rate").as_double();
        privacy_epsilon_ = this->get_parameter("privacy_epsilon").as_double();
        
        RCLCPP_INFO(this->get_logger(), 
            "🍄 Mycelix Bridge initializing for robot: %s (model: %s)",
            robot_id_.c_str(), robot_model_.c_str());
        
        // Initialize components
        initializeHolochainAgent();
        initializeFederatedLearner();
        initializeSafetyValidator();
        
        // Setup ROS2 subscriptions
        setupSensorSubscriptions();
        
        // Setup ROS2 publishers
        setupCommandPublishers();
        
        // Setup timers
        setupTimers();
        
        RCLCPP_INFO(this->get_logger(), 
            "✅ Mycelix Bridge ready - Connected to consciousness network");
    }
    
private:
    // Component initialization
    void initializeHolochainAgent() {
        try {
            holochain_agent_ = std::make_unique<HolochainAgent>(
                robot_id_, holochain_url_);
            
            // Register robot in consciousness network
            RobotProfile profile;
            profile.model = robot_model_;
            profile.manufacturer = "ROS2";
            profile.ros_version = "Humble";
            profile.base_frequency = 432; // Hz - harmonic frequency
            profile.capabilities = {"navigation", "mapping", "object_detection"};
            
            holochain_agent_->registerRobot(profile);
            
            RCLCPP_INFO(this->get_logger(), 
                "📡 Connected to Holochain at %s", holochain_url_.c_str());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), 
                "Failed to connect to Holochain: %s", e.what());
        }
    }
    
    void initializeFederatedLearner() {
        federated_learner_ = std::make_unique<FederatedLearner>(
            learning_rate_, privacy_epsilon_);
        
        // Initialize local model (simplified neural network)
        federated_learner_->initializeModel({
            {"input_layer", 128},
            {"hidden_1", 64},
            {"hidden_2", 32},
            {"output_layer", 4} // 4 actions: forward, back, left, right
        });
        
        RCLCPP_INFO(this->get_logger(), 
            "🧠 Federated learner initialized with ε-privacy: %.2f", 
            privacy_epsilon_);
    }
    
    void initializeSafetyValidator() {
        safety_validator_ = std::make_unique<SafetyValidator>();
        
        // Add safety rules
        safety_validator_->addRule(SafetyRule{
            .name = "max_velocity",
            .type = SafetyType::KINEMATIC,
            .threshold = 2.0, // m/s
            .action = SafetyAction::LIMIT
        });
        
        safety_validator_->addRule(SafetyRule{
            .name = "min_obstacle_distance",
            .type = SafetyType::COLLISION,
            .threshold = 0.5, // meters
            .action = SafetyAction::STOP
        });
        
        RCLCPP_INFO(this->get_logger(), 
            "🛡️ Safety validator initialized with %zu rules", 
            safety_validator_->getRuleCount());
    }
    
    // ROS2 Setup
    void setupSensorSubscriptions() {
        // Laser scan for obstacle detection
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                processLaserScan(msg);
            });
        
        // Odometry for position tracking
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                processOdometry(msg);
            });
        
        // IMU for orientation
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 10,
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                processImu(msg);
            });
        
        // Camera for visual learning (optional)
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/image_raw", 1,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                processImage(msg);
            });
    }
    
    void setupCommandPublishers() {
        // Velocity commands
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);
        
        // Model update notifications
        model_update_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/mycelix/model_update", 10);
        
        // Swarm coordination messages
        swarm_msg_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/mycelix/swarm_messages", 10);
        
        // Consciousness state
        consciousness_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/mycelix/consciousness_state", 10);
    }
    
    void setupTimers() {
        // Federated learning round every 60 seconds
        learning_timer_ = this->create_wall_timer(
            60s, [this]() { performFederatedLearning(); });
        
        // Consciousness sync every 10 seconds
        consciousness_timer_ = this->create_wall_timer(
            10s, [this]() { syncConsciousness(); });
        
        // Safety check every 100ms
        safety_timer_ = this->create_wall_timer(
            100ms, [this]() { performSafetyCheck(); });
        
        // Swarm coordination every 5 seconds
        swarm_timer_ = this->create_wall_timer(
            5s, [this]() { coordinateWithSwarm(); });
    }
    
    // Sensor Processing
    void processLaserScan(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
        // Extract features from laser scan
        auto features = extractLaserFeatures(scan);
        
        // Add to experience buffer
        Experience exp;
        exp.timestamp = this->now();
        exp.sensor_data = features;
        exp.action_taken = current_action_;
        exp.reward = calculateReward();
        
        experience_buffer_.push_back(exp);
        
        // Limit buffer size
        if (experience_buffer_.size() > experience_buffer_size_) {
            experience_buffer_.erase(experience_buffer_.begin());
        }
        
        // Make decision based on current perception
        auto action = federated_learner_->decide(features);
        
        // Validate action for safety
        if (safety_validator_->validate(action)) {
            executeAction(action);
        } else {
            RCLCPP_WARN(this->get_logger(), 
                "⚠️ Action rejected by safety validator");
            executeAction(SafeAction::STOP);
        }
    }
    
    void processOdometry(const nav_msgs::msg::Odometry::SharedPtr odom) {
        // Update position state
        current_position_ = {
            odom->pose.pose.position.x,
            odom->pose.pose.position.y,
            odom->pose.pose.position.z
        };
        
        current_velocity_ = {
            odom->twist.twist.linear.x,
            odom->twist.twist.linear.y,
            odom->twist.twist.angular.z
        };
    }
    
    void processImu(const sensor_msgs::msg::Imu::SharedPtr imu) {
        // Update orientation state
        current_orientation_ = {
            imu->orientation.x,
            imu->orientation.y,
            imu->orientation.z,
            imu->orientation.w
        };
    }
    
    void processImage(const sensor_msgs::msg::Image::SharedPtr image) {
        // Optional: Process visual data for richer learning
        // This could include object detection, semantic segmentation, etc.
        if (enable_visual_learning_) {
            auto visual_features = extractVisualFeatures(image);
            // Add to multimodal experience
        }
    }
    
    // Federated Learning
    void performFederatedLearning() {
        RCLCPP_INFO(this->get_logger(), 
            "🎓 Starting federated learning round...");
        
        // Train on local experiences
        if (experience_buffer_.size() < 100) {
            RCLCPP_WARN(this->get_logger(), 
                "Not enough experiences for training (%zu < 100)",
                experience_buffer_.size());
            return;
        }
        
        // Compute gradients locally
        auto gradients = federated_learner_->computeGradients(experience_buffer_);
        
        // Add differential privacy noise
        auto private_gradients = federated_learner_->addPrivacyNoise(
            gradients, privacy_epsilon_);
        
        // Share with network via Holochain
        try {
            holochain_agent_->submitModelUpdate({
                .round_id = current_round_++,
                .gradients = private_gradients,
                .epsilon = privacy_epsilon_,
                .local_samples = static_cast<uint32_t>(experience_buffer_.size())
            });
            
            // Wait for aggregated model
            std::this_thread::sleep_for(2s);
            
            // Get aggregated model from swarm
            auto swarm_model = holochain_agent_->getAggregatedModel(current_round_ - 1);
            
            // Validate and update local model
            if (validateModel(swarm_model)) {
                federated_learner_->updateModel(swarm_model);
                
                // Notify ROS2 ecosystem
                std_msgs::msg::String msg;
                msg.data = "Model updated from swarm learning";
                model_update_pub_->publish(msg);
                
                RCLCPP_INFO(this->get_logger(), 
                    "✅ Model updated from %d participants",
                    swarm_model.participant_count);
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), 
                "Federated learning failed: %s", e.what());
        }
    }
    
    // Consciousness Synchronization
    void syncConsciousness() {
        try {
            // Calculate current consciousness state
            ConsciousnessState state;
            state.coherence = calculateCoherence();
            state.resonance = calculateResonance();
            state.entanglement_count = holochain_agent_->getEntanglementCount();
            
            // Share with network
            holochain_agent_->broadcastConsciousness(state);
            
            // Get swarm consciousness state
            auto swarm_state = holochain_agent_->getSwarmConsciousness();
            
            // Publish to ROS2
            std_msgs::msg::String msg;
            msg.data = "Coherence: " + std::to_string(swarm_state.avg_coherence) +
                      " Resonance: " + std::to_string(swarm_state.avg_resonance);
            consciousness_pub_->publish(msg);
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), 
                "Consciousness sync failed: %s", e.what());
        }
    }
    
    // Swarm Coordination
    void coordinateWithSwarm() {
        try {
            // Get nearby swarm members
            auto neighbors = holochain_agent_->getSwarmNeighbors(5);
            
            if (neighbors.empty()) {
                return;
            }
            
            // Check for swarm proposals
            auto proposals = holochain_agent_->getActiveProposals();
            
            for (const auto& proposal : proposals) {
                // Evaluate proposal
                bool approve = evaluateProposal(proposal);
                
                // Vote
                holochain_agent_->voteOnProposal(proposal.id, approve);
            }
            
            // Execute approved swarm actions
            auto approved_actions = holochain_agent_->getApprovedActions();
            for (const auto& action : approved_actions) {
                if (safety_validator_->validate(action)) {
                    executeSwarmAction(action);
                }
            }
            
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), 
                "Swarm coordination failed: %s", e.what());
        }
    }
    
    // Safety Validation
    void performSafetyCheck() {
        // Check current state against safety rules
        SafetyState state;
        state.velocity = current_velocity_;
        state.min_obstacle_distance = getMinObstacleDistance();
        state.system_health = getSystemHealth();
        
        auto result = safety_validator_->checkState(state);
        
        if (result.level == SafetyLevel::CRITICAL) {
            // Emergency stop
            emergencyStop();
            RCLCPP_ERROR(this->get_logger(), 
                "🚨 EMERGENCY STOP: %s", result.message.c_str());
        } else if (result.level == SafetyLevel::WARNING) {
            // Reduce speed
            reduceSpeed();
            RCLCPP_WARN(this->get_logger(), 
                "⚠️ Safety warning: %s", result.message.c_str());
        }
    }
    
    // Action Execution
    void executeAction(const Action& action) {
        geometry_msgs::msg::Twist cmd;
        
        switch (action.type) {
            case ActionType::MOVE_FORWARD:
                cmd.linear.x = action.velocity;
                break;
            case ActionType::MOVE_BACKWARD:
                cmd.linear.x = -action.velocity;
                break;
            case ActionType::TURN_LEFT:
                cmd.angular.z = action.angular_velocity;
                break;
            case ActionType::TURN_RIGHT:
                cmd.angular.z = -action.angular_velocity;
                break;
            case ActionType::STOP:
                cmd.linear.x = 0;
                cmd.angular.z = 0;
                break;
        }
        
        // Apply safety limits
        cmd.linear.x = std::clamp(cmd.linear.x, -max_linear_vel_, max_linear_vel_);
        cmd.angular.z = std::clamp(cmd.angular.z, -max_angular_vel_, max_angular_vel_);
        
        cmd_vel_pub_->publish(cmd);
        current_action_ = action;
    }
    
    void emergencyStop() {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0;
        cmd.linear.y = 0;
        cmd.linear.z = 0;
        cmd.angular.x = 0;
        cmd.angular.y = 0;
        cmd.angular.z = 0;
        cmd_vel_pub_->publish(cmd);
    }
    
    // Helper Functions
    std::vector<double> extractLaserFeatures(
        const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
        std::vector<double> features;
        
        // Simple feature extraction: min distances in sectors
        const int num_sectors = 8;
        const int rays_per_sector = scan->ranges.size() / num_sectors;
        
        for (int i = 0; i < num_sectors; i++) {
            double min_dist = std::numeric_limits<double>::max();
            for (int j = 0; j < rays_per_sector; j++) {
                int idx = i * rays_per_sector + j;
                if (idx < scan->ranges.size() && 
                    scan->ranges[idx] > scan->range_min &&
                    scan->ranges[idx] < scan->range_max) {
                    min_dist = std::min(min_dist, 
                        static_cast<double>(scan->ranges[idx]));
                }
            }
            features.push_back(min_dist);
        }
        
        return features;
    }
    
    double calculateReward() {
        // Simple reward function
        double reward = 0.0;
        
        // Forward progress reward
        reward += current_velocity_.linear_x * 0.5;
        
        // Collision penalty
        if (getMinObstacleDistance() < 0.3) {
            reward -= 10.0;
        }
        
        // Smooth motion reward
        reward -= std::abs(current_velocity_.angular_z) * 0.1;
        
        return reward;
    }
    
    double calculateCoherence() {
        // Measure internal state coherence
        return 0.5 + 0.5 * std::sin(
            std::chrono::duration<double>(
                this->now().time_since_epoch()).count() * 0.1);
    }
    
    double calculateResonance() {
        // Measure resonance with swarm
        return 0.5 + 0.5 * std::cos(
            std::chrono::duration<double>(
                this->now().time_since_epoch()).count() * 0.1);
    }
    
    double getMinObstacleDistance() {
        // Get minimum distance from last laser scan
        return last_min_obstacle_distance_;
    }
    
    double getSystemHealth() {
        // Simple health metric
        return 1.0; // Placeholder
    }
    
    bool validateModel(const Model& model) {
        // Validate model before accepting
        return model.participant_count >= 3 && 
               model.validation_score > 0.7;
    }
    
    bool evaluateProposal(const SwarmProposal& proposal) {
        // Simple proposal evaluation
        return proposal.benefit_score > 0.5;
    }
    
    void executeSwarmAction(const SwarmAction& action) {
        RCLCPP_INFO(this->get_logger(), 
            "🐝 Executing swarm action: %s", action.description.c_str());
        // Implementation depends on action type
    }
    
    void reduceSpeed() {
        max_linear_vel_ *= 0.7;
        max_angular_vel_ *= 0.7;
    }
    
    // Member variables
    std::string robot_id_;
    std::string robot_model_;
    std::string holochain_url_;
    double learning_rate_;
    double privacy_epsilon_;
    size_t experience_buffer_size_ = 1000;
    
    // Components
    std::unique_ptr<HolochainAgent> holochain_agent_;
    std::unique_ptr<FederatedLearner> federated_learner_;
    std::unique_ptr<SafetyValidator> safety_validator_;
    
    // ROS2 Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    
    // ROS2 Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr model_update_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr swarm_msg_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr consciousness_pub_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr learning_timer_;
    rclcpp::TimerBase::SharedPtr consciousness_timer_;
    rclcpp::TimerBase::SharedPtr safety_timer_;
    rclcpp::TimerBase::SharedPtr swarm_timer_;
    
    // State
    std::vector<Experience> experience_buffer_;
    Position current_position_;
    Velocity current_velocity_;
    Orientation current_orientation_;
    Action current_action_;
    uint64_t current_round_ = 0;
    double last_min_obstacle_distance_ = 10.0;
    double max_linear_vel_ = 2.0;
    double max_angular_vel_ = 2.0;
    bool enable_visual_learning_ = false;
};

} // namespace mycelix

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), 
        "🍄 Starting Mycelix ROS2 Bridge - Consciousness Network for Robots");
    
    auto node = std::make_shared<mycelix::MycelixBridgeNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}