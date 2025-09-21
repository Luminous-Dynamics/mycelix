/**
 * Swarm Coordinator Node for Mycelix
 * Coordinates multi-robot actions and consensus decisions
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <vector>
#include <map>
#include <cmath>
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

namespace mycelix {

// Formation types for swarm
enum class FormationType {
    TRIANGLE,
    LINE,
    CIRCLE,
    SQUARE,
    V_FORMATION,
    DYNAMIC
};

// Swarm task types
enum class SwarmTask {
    EXPLORATION,
    MAPPING,
    SEARCH_AND_RESCUE,
    PERIMETER_PATROL,
    COLLECTIVE_TRANSPORT,
    AREA_COVERAGE
};

// Robot state in swarm
struct RobotState {
    std::string id;
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Twist velocity;
    double coherence;
    double resonance;
    bool active;
    std::chrono::steady_clock::time_point last_update;
};

// Consensus decision
struct ConsensusDecision {
    std::string proposal_id;
    std::string description;
    std::map<std::string, bool> votes;
    double consensus_level;
    bool approved;
    std::chrono::steady_clock::time_point deadline;
};

class SwarmCoordinatorNode : public rclcpp::Node {
public:
    SwarmCoordinatorNode() : Node("swarm_coordinator") {
        // Declare parameters
        this->declare_parameter<int>("swarm_size", 3);
        this->declare_parameter<double>("coordination_frequency", 5.0);
        this->declare_parameter<double>("consensus_threshold", 0.66);
        this->declare_parameter<std::string>("formation_type", "triangle");
        this->declare_parameter<std::string>("holochain_url", "ws://localhost:8888");
        this->declare_parameter<double>("formation_spacing", 2.0);
        this->declare_parameter<double>("max_swarm_velocity", 1.0);
        
        // Get parameters
        swarm_size_ = this->get_parameter("swarm_size").as_int();
        coordination_frequency_ = this->get_parameter("coordination_frequency").as_double();
        consensus_threshold_ = this->get_parameter("consensus_threshold").as_double();
        formation_spacing_ = this->get_parameter("formation_spacing").as_double();
        max_swarm_velocity_ = this->get_parameter("max_swarm_velocity").as_double();
        
        std::string formation_str = this->get_parameter("formation_type").as_string();
        formation_type_ = stringToFormationType(formation_str);
        
        RCLCPP_INFO(this->get_logger(), 
            "🐝 Swarm Coordinator initialized for %d robots", swarm_size_);
        RCLCPP_INFO(this->get_logger(), 
            "📊 Consensus threshold: %.2f, Formation: %s", 
            consensus_threshold_, formation_str.c_str());
        
        // Initialize robot states
        initializeRobotStates();
        
        // Setup subscriptions for each robot
        setupRobotSubscriptions();
        
        // Setup publishers
        setupPublishers();
        
        // Setup coordination timer
        coordination_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / coordination_frequency_)),
            [this]() { coordinateSwarm(); });
        
        // Setup consensus timer
        consensus_timer_ = this->create_wall_timer(
            1s, [this]() { processConsensusDecisions(); });
        
        // Setup visualization timer
        visualization_timer_ = this->create_wall_timer(
            100ms, [this]() { publishVisualization(); });
        
        RCLCPP_INFO(this->get_logger(), 
            "✅ Swarm Coordinator ready - Collective intelligence active!");
    }

private:
    void initializeRobotStates() {
        for (int i = 0; i < swarm_size_; i++) {
            RobotState state;
            state.id = "robot_" + std::to_string(i + 1).insert(0, 3 - std::to_string(i + 1).length(), '0');
            state.active = false;
            state.coherence = 0.5;
            state.resonance = 0.5;
            state.last_update = std::chrono::steady_clock::now();
            robot_states_[state.id] = state;
        }
    }
    
    void setupRobotSubscriptions() {
        // Subscribe to odometry for each robot
        for (const auto& [robot_id, _] : robot_states_) {
            std::string odom_topic = "/" + robot_id + "/odom";
            auto odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                odom_topic, 10,
                [this, robot_id](const nav_msgs::msg::Odometry::SharedPtr msg) {
                    updateRobotState(robot_id, msg);
                });
            odom_subscriptions_.push_back(odom_sub);
            
            // Subscribe to consciousness state
            std::string consciousness_topic = "/" + robot_id + "/mycelix/consciousness_state";
            auto consciousness_sub = this->create_subscription<std_msgs::msg::String>(
                consciousness_topic, 10,
                [this, robot_id](const std_msgs::msg::String::SharedPtr msg) {
                    updateConsciousnessState(robot_id, msg);
                });
            consciousness_subscriptions_.push_back(consciousness_sub);
        }
        
        // Subscribe to swarm proposals
        proposal_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/mycelix/swarm_proposals", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                handleSwarmProposal(msg);
            });
    }
    
    void setupPublishers() {
        // Formation commands for each robot
        for (const auto& [robot_id, _] : robot_states_) {
            std::string cmd_topic = "/" + robot_id + "/cmd_vel";
            auto cmd_pub = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);
            cmd_publishers_[robot_id] = cmd_pub;
        }
        
        // Swarm state publisher
        swarm_state_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/mycelix/swarm_state", 10);
        
        // Formation visualization
        formation_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/mycelix/formation_markers", 10);
        
        // Consensus decisions
        consensus_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/mycelix/consensus_decisions", 10);
        
        // Swarm metrics
        metrics_pub_ = this->create_publisher<std_msgs::msg::Float64>(
            "/mycelix/swarm_metrics", 10);
    }
    
    void updateRobotState(const std::string& robot_id, 
                         const nav_msgs::msg::Odometry::SharedPtr odom) {
        if (robot_states_.find(robot_id) != robot_states_.end()) {
            robot_states_[robot_id].pose = odom->pose.pose;
            robot_states_[robot_id].velocity = odom->twist.twist;
            robot_states_[robot_id].active = true;
            robot_states_[robot_id].last_update = std::chrono::steady_clock::now();
        }
    }
    
    void updateConsciousnessState(const std::string& robot_id,
                                  const std_msgs::msg::String::SharedPtr msg) {
        // Parse consciousness state from message
        // Format: "Coherence: X.XX Resonance: Y.YY"
        std::string data = msg->data;
        
        // Simple parsing (in production, use JSON)
        size_t coherence_pos = data.find("Coherence: ");
        size_t resonance_pos = data.find("Resonance: ");
        
        if (coherence_pos != std::string::npos && resonance_pos != std::string::npos) {
            double coherence = std::stod(data.substr(coherence_pos + 11, 4));
            double resonance = std::stod(data.substr(resonance_pos + 11, 4));
            
            if (robot_states_.find(robot_id) != robot_states_.end()) {
                robot_states_[robot_id].coherence = coherence;
                robot_states_[robot_id].resonance = resonance;
            }
        }
    }
    
    void handleSwarmProposal(const std_msgs::msg::String::SharedPtr msg) {
        // Create new consensus decision
        ConsensusDecision decision;
        decision.proposal_id = generateProposalId();
        decision.description = msg->data;
        decision.deadline = std::chrono::steady_clock::now() + 5s;
        decision.consensus_level = 0.0;
        decision.approved = false;
        
        // Initialize votes
        for (const auto& [robot_id, _] : robot_states_) {
            decision.votes[robot_id] = false;  // Will be updated by robot votes
        }
        
        pending_decisions_[decision.proposal_id] = decision;
        
        RCLCPP_INFO(this->get_logger(), 
            "📋 New swarm proposal: %s", decision.description.c_str());
    }
    
    void coordinateSwarm() {
        // Check active robots
        int active_count = 0;
        geometry_msgs::msg::Point swarm_center;
        
        for (const auto& [robot_id, state] : robot_states_) {
            if (state.active) {
                active_count++;
                swarm_center.x += state.pose.position.x;
                swarm_center.y += state.pose.position.y;
            }
        }
        
        if (active_count < 2) {
            return;  // Need at least 2 robots for swarm behavior
        }
        
        // Calculate swarm center
        swarm_center.x /= active_count;
        swarm_center.y /= active_count;
        
        // Calculate target formation positions
        auto target_positions = calculateFormation(swarm_center, active_count);
        
        // Send movement commands to achieve formation
        int robot_index = 0;
        for (const auto& [robot_id, state] : robot_states_) {
            if (!state.active) continue;
            
            if (robot_index < target_positions.size()) {
                geometry_msgs::msg::Twist cmd = calculateMovementCommand(
                    state.pose.position, target_positions[robot_index]);
                
                // Apply swarm coordination adjustments
                applySwarmBehavior(cmd, robot_id);
                
                // Publish command
                if (cmd_publishers_.find(robot_id) != cmd_publishers_.end()) {
                    cmd_publishers_[robot_id]->publish(cmd);
                }
                
                robot_index++;
            }
        }
        
        // Publish swarm state
        publishSwarmState();
        
        // Calculate and publish swarm metrics
        publishSwarmMetrics();
    }
    
    std::vector<geometry_msgs::msg::Point> calculateFormation(
        const geometry_msgs::msg::Point& center, int robot_count) {
        
        std::vector<geometry_msgs::msg::Point> positions;
        
        switch (formation_type_) {
            case FormationType::TRIANGLE:
                positions = calculateTriangleFormation(center, robot_count);
                break;
            case FormationType::LINE:
                positions = calculateLineFormation(center, robot_count);
                break;
            case FormationType::CIRCLE:
                positions = calculateCircleFormation(center, robot_count);
                break;
            case FormationType::SQUARE:
                positions = calculateSquareFormation(center, robot_count);
                break;
            case FormationType::V_FORMATION:
                positions = calculateVFormation(center, robot_count);
                break;
            case FormationType::DYNAMIC:
                positions = calculateDynamicFormation(center, robot_count);
                break;
        }
        
        return positions;
    }
    
    std::vector<geometry_msgs::msg::Point> calculateTriangleFormation(
        const geometry_msgs::msg::Point& center, int robot_count) {
        
        std::vector<geometry_msgs::msg::Point> positions;
        
        if (robot_count <= 3) {
            // Simple triangle
            for (int i = 0; i < robot_count; i++) {
                geometry_msgs::msg::Point pos;
                double angle = 2.0 * M_PI * i / 3.0;
                pos.x = center.x + formation_spacing_ * std::cos(angle);
                pos.y = center.y + formation_spacing_ * std::sin(angle);
                pos.z = 0.0;
                positions.push_back(pos);
            }
        } else {
            // Multi-layer triangle
            int layer = 0;
            int robots_placed = 0;
            
            while (robots_placed < robot_count) {
                int robots_in_layer = 3 * (layer + 1);
                double radius = formation_spacing_ * (layer + 1);
                
                for (int i = 0; i < robots_in_layer && robots_placed < robot_count; i++) {
                    geometry_msgs::msg::Point pos;
                    double angle = 2.0 * M_PI * i / robots_in_layer;
                    pos.x = center.x + radius * std::cos(angle);
                    pos.y = center.y + radius * std::sin(angle);
                    pos.z = 0.0;
                    positions.push_back(pos);
                    robots_placed++;
                }
                layer++;
            }
        }
        
        return positions;
    }
    
    std::vector<geometry_msgs::msg::Point> calculateLineFormation(
        const geometry_msgs::msg::Point& center, int robot_count) {
        
        std::vector<geometry_msgs::msg::Point> positions;
        
        for (int i = 0; i < robot_count; i++) {
            geometry_msgs::msg::Point pos;
            pos.x = center.x + formation_spacing_ * (i - robot_count / 2.0);
            pos.y = center.y;
            pos.z = 0.0;
            positions.push_back(pos);
        }
        
        return positions;
    }
    
    std::vector<geometry_msgs::msg::Point> calculateCircleFormation(
        const geometry_msgs::msg::Point& center, int robot_count) {
        
        std::vector<geometry_msgs::msg::Point> positions;
        double radius = formation_spacing_ * robot_count / (2.0 * M_PI);
        
        for (int i = 0; i < robot_count; i++) {
            geometry_msgs::msg::Point pos;
            double angle = 2.0 * M_PI * i / robot_count;
            pos.x = center.x + radius * std::cos(angle);
            pos.y = center.y + radius * std::sin(angle);
            pos.z = 0.0;
            positions.push_back(pos);
        }
        
        return positions;
    }
    
    std::vector<geometry_msgs::msg::Point> calculateSquareFormation(
        const geometry_msgs::msg::Point& center, int robot_count) {
        
        std::vector<geometry_msgs::msg::Point> positions;
        int side_length = std::ceil(std::sqrt(robot_count));
        
        for (int i = 0; i < robot_count; i++) {
            geometry_msgs::msg::Point pos;
            int row = i / side_length;
            int col = i % side_length;
            
            pos.x = center.x + formation_spacing_ * (col - side_length / 2.0);
            pos.y = center.y + formation_spacing_ * (row - side_length / 2.0);
            pos.z = 0.0;
            positions.push_back(pos);
        }
        
        return positions;
    }
    
    std::vector<geometry_msgs::msg::Point> calculateVFormation(
        const geometry_msgs::msg::Point& center, int robot_count) {
        
        std::vector<geometry_msgs::msg::Point> positions;
        
        // Lead robot
        geometry_msgs::msg::Point lead_pos;
        lead_pos.x = center.x;
        lead_pos.y = center.y;
        lead_pos.z = 0.0;
        positions.push_back(lead_pos);
        
        // Wing robots
        for (int i = 1; i < robot_count; i++) {
            geometry_msgs::msg::Point pos;
            int side = (i % 2 == 0) ? 1 : -1;
            int distance_back = (i + 1) / 2;
            
            pos.x = center.x - formation_spacing_ * distance_back * 0.7;
            pos.y = center.y + side * formation_spacing_ * distance_back * 0.5;
            pos.z = 0.0;
            positions.push_back(pos);
        }
        
        return positions;
    }
    
    std::vector<geometry_msgs::msg::Point> calculateDynamicFormation(
        const geometry_msgs::msg::Point& center, int robot_count) {
        
        // Dynamic formation based on swarm consciousness level
        double avg_coherence = calculateAverageCoherence();
        
        if (avg_coherence > 0.8) {
            // High coherence: tight formation
            return calculateTriangleFormation(center, robot_count);
        } else if (avg_coherence > 0.5) {
            // Medium coherence: circle formation
            return calculateCircleFormation(center, robot_count);
        } else {
            // Low coherence: spread out line
            return calculateLineFormation(center, robot_count);
        }
    }
    
    geometry_msgs::msg::Twist calculateMovementCommand(
        const geometry_msgs::msg::Point& current_pos,
        const geometry_msgs::msg::Point& target_pos) {
        
        geometry_msgs::msg::Twist cmd;
        
        // Calculate error
        double dx = target_pos.x - current_pos.x;
        double dy = target_pos.y - current_pos.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        // Simple proportional controller
        const double kp_linear = 0.5;
        const double kp_angular = 2.0;
        
        if (distance > 0.1) {  // Position tolerance
            // Linear velocity
            cmd.linear.x = kp_linear * distance;
            cmd.linear.x = std::min(cmd.linear.x, max_swarm_velocity_);
            
            // Angular velocity to face target
            double target_angle = std::atan2(dy, dx);
            cmd.angular.z = kp_angular * target_angle;
        }
        
        return cmd;
    }
    
    void applySwarmBehavior(geometry_msgs::msg::Twist& cmd, 
                            const std::string& robot_id) {
        // Apply Reynolds' rules: cohesion, separation, alignment
        
        const auto& current_state = robot_states_[robot_id];
        
        // Separation: avoid collision
        for (const auto& [other_id, other_state] : robot_states_) {
            if (other_id == robot_id || !other_state.active) continue;
            
            double dx = current_state.pose.position.x - other_state.pose.position.x;
            double dy = current_state.pose.position.y - other_state.pose.position.y;
            double dist = std::sqrt(dx * dx + dy * dy);
            
            if (dist < 1.0) {  // Minimum separation distance
                // Repulsion force
                double repulsion = (1.0 - dist) / dist;
                cmd.linear.x += repulsion * dx * 0.3;
                cmd.linear.y += repulsion * dy * 0.3;
            }
        }
        
        // Limit velocities
        cmd.linear.x = std::clamp(cmd.linear.x, -max_swarm_velocity_, max_swarm_velocity_);
        cmd.linear.y = std::clamp(cmd.linear.y, -max_swarm_velocity_, max_swarm_velocity_);
        cmd.angular.z = std::clamp(cmd.angular.z, -2.0, 2.0);
    }
    
    void processConsensusDecisions() {
        auto now = std::chrono::steady_clock::now();
        std::vector<std::string> completed;
        
        for (auto& [proposal_id, decision] : pending_decisions_) {
            if (now > decision.deadline) {
                // Calculate consensus
                int votes_for = 0;
                int total_votes = 0;
                
                for (const auto& [robot_id, vote] : decision.votes) {
                    if (robot_states_[robot_id].active) {
                        total_votes++;
                        if (vote) votes_for++;
                    }
                }
                
                if (total_votes > 0) {
                    decision.consensus_level = 
                        static_cast<double>(votes_for) / total_votes;
                    decision.approved = 
                        decision.consensus_level >= consensus_threshold_;
                }
                
                // Publish decision
                publishConsensusDecision(decision);
                completed.push_back(proposal_id);
                
                RCLCPP_INFO(this->get_logger(), 
                    "🗳️ Consensus decision: %s - %s (%.2f%%)",
                    decision.description.c_str(),
                    decision.approved ? "APPROVED" : "REJECTED",
                    decision.consensus_level * 100);
            }
        }
        
        // Remove completed decisions
        for (const auto& id : completed) {
            pending_decisions_.erase(id);
        }
    }
    
    void publishSwarmState() {
        std_msgs::msg::String msg;
        
        // Calculate swarm metrics
        double avg_coherence = calculateAverageCoherence();
        double avg_resonance = calculateAverageResonance();
        double swarm_spread = calculateSwarmSpread();
        int active_robots = countActiveRobots();
        
        // Format message (in production, use JSON)
        msg.data = "Active: " + std::to_string(active_robots) + "/" + 
                  std::to_string(swarm_size_) +
                  " | Coherence: " + std::to_string(avg_coherence) +
                  " | Resonance: " + std::to_string(avg_resonance) +
                  " | Spread: " + std::to_string(swarm_spread) + "m";
        
        swarm_state_pub_->publish(msg);
    }
    
    void publishSwarmMetrics() {
        std_msgs::msg::Float64 coherence_msg;
        coherence_msg.data = calculateAverageCoherence();
        metrics_pub_->publish(coherence_msg);
    }
    
    void publishConsensusDecision(const ConsensusDecision& decision) {
        std_msgs::msg::String msg;
        msg.data = "Decision: " + decision.description +
                  " | Approved: " + (decision.approved ? "YES" : "NO") +
                  " | Consensus: " + std::to_string(decision.consensus_level * 100) + "%";
        consensus_pub_->publish(msg);
    }
    
    void publishVisualization() {
        visualization_msgs::msg::MarkerArray markers;
        int marker_id = 0;
        
        // Robot positions
        for (const auto& [robot_id, state] : robot_states_) {
            if (!state.active) continue;
            
            visualization_msgs::msg::Marker robot_marker;
            robot_marker.header.frame_id = "map";
            robot_marker.header.stamp = this->now();
            robot_marker.ns = "robots";
            robot_marker.id = marker_id++;
            robot_marker.type = visualization_msgs::msg::Marker::SPHERE;
            robot_marker.action = visualization_msgs::msg::Marker::ADD;
            
            robot_marker.pose = state.pose;
            robot_marker.scale.x = 0.3;
            robot_marker.scale.y = 0.3;
            robot_marker.scale.z = 0.3;
            
            // Color based on coherence
            robot_marker.color.r = 1.0 - state.coherence;
            robot_marker.color.g = state.coherence;
            robot_marker.color.b = state.resonance;
            robot_marker.color.a = 1.0;
            
            robot_marker.lifetime = rclcpp::Duration(0, 200000000);  // 0.2s
            
            markers.markers.push_back(robot_marker);
            
            // Add text label
            visualization_msgs::msg::Marker text_marker = robot_marker;
            text_marker.id = marker_id++;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.pose.position.z += 0.5;
            text_marker.text = robot_id;
            text_marker.scale.z = 0.2;
            
            markers.markers.push_back(text_marker);
        }
        
        // Formation connections
        auto active_robots = getActiveRobotIds();
        if (active_robots.size() > 1) {
            visualization_msgs::msg::Marker line_marker;
            line_marker.header.frame_id = "map";
            line_marker.header.stamp = this->now();
            line_marker.ns = "formation";
            line_marker.id = marker_id++;
            line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
            line_marker.action = visualization_msgs::msg::Marker::ADD;
            
            line_marker.scale.x = 0.02;
            line_marker.color.r = 0.0;
            line_marker.color.g = 0.8;
            line_marker.color.b = 0.8;
            line_marker.color.a = 0.5;
            
            // Create connections between neighboring robots
            for (size_t i = 0; i < active_robots.size(); i++) {
                for (size_t j = i + 1; j < active_robots.size(); j++) {
                    const auto& state1 = robot_states_[active_robots[i]];
                    const auto& state2 = robot_states_[active_robots[j]];
                    
                    double dx = state1.pose.position.x - state2.pose.position.x;
                    double dy = state1.pose.position.y - state2.pose.position.y;
                    double dist = std::sqrt(dx * dx + dy * dy);
                    
                    if (dist < formation_spacing_ * 2.0) {
                        line_marker.points.push_back(state1.pose.position);
                        line_marker.points.push_back(state2.pose.position);
                    }
                }
            }
            
            if (!line_marker.points.empty()) {
                line_marker.lifetime = rclcpp::Duration(0, 200000000);  // 0.2s
                markers.markers.push_back(line_marker);
            }
        }
        
        formation_marker_pub_->publish(markers);
    }
    
    // Helper functions
    FormationType stringToFormationType(const std::string& str) {
        if (str == "triangle") return FormationType::TRIANGLE;
        if (str == "line") return FormationType::LINE;
        if (str == "circle") return FormationType::CIRCLE;
        if (str == "square") return FormationType::SQUARE;
        if (str == "v_formation") return FormationType::V_FORMATION;
        if (str == "dynamic") return FormationType::DYNAMIC;
        return FormationType::TRIANGLE;
    }
    
    std::string generateProposalId() {
        static int proposal_counter = 0;
        return "proposal_" + std::to_string(++proposal_counter);
    }
    
    double calculateAverageCoherence() {
        double sum = 0.0;
        int count = 0;
        
        for (const auto& [_, state] : robot_states_) {
            if (state.active) {
                sum += state.coherence;
                count++;
            }
        }
        
        return count > 0 ? sum / count : 0.0;
    }
    
    double calculateAverageResonance() {
        double sum = 0.0;
        int count = 0;
        
        for (const auto& [_, state] : robot_states_) {
            if (state.active) {
                sum += state.resonance;
                count++;
            }
        }
        
        return count > 0 ? sum / count : 0.0;
    }
    
    double calculateSwarmSpread() {
        auto active_robots = getActiveRobotIds();
        if (active_robots.size() < 2) return 0.0;
        
        // Calculate center
        double cx = 0.0, cy = 0.0;
        for (const auto& id : active_robots) {
            cx += robot_states_[id].pose.position.x;
            cy += robot_states_[id].pose.position.y;
        }
        cx /= active_robots.size();
        cy /= active_robots.size();
        
        // Calculate average distance from center
        double sum_dist = 0.0;
        for (const auto& id : active_robots) {
            double dx = robot_states_[id].pose.position.x - cx;
            double dy = robot_states_[id].pose.position.y - cy;
            sum_dist += std::sqrt(dx * dx + dy * dy);
        }
        
        return sum_dist / active_robots.size();
    }
    
    int countActiveRobots() {
        int count = 0;
        for (const auto& [_, state] : robot_states_) {
            if (state.active) count++;
        }
        return count;
    }
    
    std::vector<std::string> getActiveRobotIds() {
        std::vector<std::string> active;
        for (const auto& [id, state] : robot_states_) {
            if (state.active) {
                active.push_back(id);
            }
        }
        return active;
    }
    
    // Member variables
    int swarm_size_;
    double coordination_frequency_;
    double consensus_threshold_;
    double formation_spacing_;
    double max_swarm_velocity_;
    FormationType formation_type_;
    SwarmTask current_task_ = SwarmTask::EXPLORATION;
    
    // Robot states
    std::map<std::string, RobotState> robot_states_;
    
    // Consensus decisions
    std::map<std::string, ConsensusDecision> pending_decisions_;
    
    // ROS2 interfaces
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subscriptions_;
    std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> consciousness_subscriptions_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr proposal_sub_;
    
    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr> cmd_publishers_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr swarm_state_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr formation_marker_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr consensus_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr metrics_pub_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr coordination_timer_;
    rclcpp::TimerBase::SharedPtr consensus_timer_;
    rclcpp::TimerBase::SharedPtr visualization_timer_;
};

} // namespace mycelix

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), 
        "🐝 Starting Mycelix Swarm Coordinator - Collective Intelligence Active");
    
    auto node = std::make_shared<mycelix::SwarmCoordinatorNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}