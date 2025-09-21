/**
 * Demo Controller for Mycelix Three-Agent Demonstration
 * Orchestrates different phases of collective intelligence demonstration
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <vector>
#include <chrono>
#include <string>
#include <map>

using namespace std::chrono_literals;

namespace mycelix {

class DemoControllerNode : public rclcpp::Node {
public:
    DemoControllerNode() : Node("demo_controller"), current_phase_(0) {
        // Declare parameters
        this->declare_parameter<int>("demo_duration", 300);  // 5 minutes
        this->declare_parameter<std::vector<std::string>>(
            "scenario_sequence", 
            {"exploration", "formation", "collective_task", "consensus", "emergence"});
        
        // Get parameters
        demo_duration_ = this->get_parameter("demo_duration").as_int();
        scenario_sequence_ = this->get_parameter("scenario_sequence")
            .as_string_array();
        
        // Calculate phase duration
        if (!scenario_sequence_.empty()) {
            phase_duration_ = demo_duration_ / scenario_sequence_.size();
        } else {
            phase_duration_ = demo_duration_;
        }
        
        // Initialize demo start time
        demo_start_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), 
            "🎬 Demo Controller initialized");
        RCLCPP_INFO(this->get_logger(), 
            "⏱️ Total duration: %d seconds, %zu phases",
            demo_duration_, scenario_sequence_.size());
        
        // Setup publishers
        setupPublishers();
        
        // Setup subscriptions
        setupSubscriptions();
        
        // Start demo timer
        demo_timer_ = this->create_wall_timer(
            1s, [this]() { updateDemoPhase(); });
        
        // Start command timer
        command_timer_ = this->create_wall_timer(
            200ms, [this]() { sendPhaseCommands(); });
        
        // Start announcement
        announcePhase(0);
        
        RCLCPP_INFO(this->get_logger(), "🚀 Demo started!");
    }

private:
    void setupPublishers() {
        // Scenario commands
        scenario_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/mycelix/demo_scenario", 10);
        
        // Task assignments
        task_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/mycelix/swarm_task", 10);
        
        // Formation commands
        formation_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/mycelix/formation_command", 10);
        
        // Consensus proposals
        proposal_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/mycelix/swarm_proposals", 10);
        
        // Demo status
        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/mycelix/demo_status", 10);
        
        // Visualization markers for demo phase
        phase_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/mycelix/demo_phase_marker", 10);
    }
    
    void setupSubscriptions() {
        // Subscribe to swarm metrics
        metrics_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "/mycelix/swarm_metrics", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) {
                current_coherence_ = msg->data;
            });
        
        // Subscribe to consensus decisions
        consensus_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/mycelix/consensus_decisions", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                handleConsensusDecision(msg);
            });
    }
    
    void updateDemoPhase() {
        auto elapsed = (this->now() - demo_start_time_).seconds();
        
        // Check if demo is complete
        if (elapsed >= demo_duration_) {
            RCLCPP_INFO(this->get_logger(), 
                "✅ Demo complete! Total time: %.1f seconds", elapsed);
            publishDemoSummary();
            rclcpp::shutdown();
            return;
        }
        
        // Calculate current phase
        int new_phase = static_cast<int>(elapsed / phase_duration_);
        
        // Check for phase transition
        if (new_phase != current_phase_ && new_phase < scenario_sequence_.size()) {
            current_phase_ = new_phase;
            announcePhase(current_phase_);
        }
        
        // Publish status
        publishStatus(elapsed);
    }
    
    void announcePhase(int phase_index) {
        if (phase_index >= scenario_sequence_.size()) return;
        
        std::string phase_name = scenario_sequence_[phase_index];
        
        RCLCPP_INFO(this->get_logger(), 
            "\n" 
            "=================================================\n"
            "📍 PHASE %d: %s\n"
            "=================================================\n",
            phase_index + 1, phase_name.c_str());
        
        // Send phase announcement
        std_msgs::msg::String scenario_msg;
        scenario_msg.data = "PHASE_START: " + phase_name;
        scenario_pub_->publish(scenario_msg);
        
        // Initialize phase-specific behaviors
        initializePhase(phase_name);
        
        // Update visualization
        publishPhaseMarker(phase_name);
    }
    
    void initializePhase(const std::string& phase_name) {
        if (phase_name == "exploration") {
            // Random exploration phase
            std_msgs::msg::String task_msg;
            task_msg.data = "EXPLORE: random_walk radius=10.0";
            task_pub_->publish(task_msg);
            
            RCLCPP_INFO(this->get_logger(), 
                "🔍 Robots exploring environment randomly");
            
        } else if (phase_name == "formation") {
            // Formation control phase
            std_msgs::msg::String formation_msg;
            formation_msg.data = "FORMATION: triangle spacing=2.0";
            formation_pub_->publish(formation_msg);
            
            RCLCPP_INFO(this->get_logger(), 
                "△ Forming triangle formation");
            
        } else if (phase_name == "collective_task") {
            // Collective transport simulation
            std_msgs::msg::String task_msg;
            task_msg.data = "TRANSPORT: virtual_object mass=5.0 target_x=5.0 target_y=5.0";
            task_pub_->publish(task_msg);
            
            RCLCPP_INFO(this->get_logger(), 
                "📦 Simulating collective transport task");
            
            // Create consensus proposal for transport strategy
            std::string proposal = "Adopt coordinated push strategy for transport";
            proposeToSwarm(proposal);
            
        } else if (phase_name == "consensus") {
            // Consensus decision phase
            std::string proposal = "Change formation to circle for better coverage";
            proposeToSwarm(proposal);
            
            RCLCPP_INFO(this->get_logger(), 
                "🗳️ Testing consensus decision making");
            
        } else if (phase_name == "emergence") {
            // Emergent behavior phase
            std_msgs::msg::String task_msg;
            task_msg.data = "EMERGENT: enable_self_organization=true";
            task_pub_->publish(task_msg);
            
            RCLCPP_INFO(this->get_logger(), 
                "✨ Observing emergent collective behavior");
        }
    }
    
    void sendPhaseCommands() {
        // Send continuous commands based on current phase
        if (current_phase_ >= scenario_sequence_.size()) return;
        
        std::string phase_name = scenario_sequence_[current_phase_];
        
        // Phase-specific continuous behaviors
        if (phase_name == "exploration") {
            // Encourage exploration by varying parameters
            static double exploration_radius = 5.0;
            exploration_radius += 0.1 * std::sin(this->now().seconds() * 0.1);
            
            if (static_cast<int>(this->now().seconds()) % 10 == 0) {
                std_msgs::msg::String task_msg;
                task_msg.data = "EXPLORE: radius=" + 
                    std::to_string(exploration_radius);
                task_pub_->publish(task_msg);
            }
            
        } else if (phase_name == "formation") {
            // Rotate through formations
            static int formation_counter = 0;
            if (static_cast<int>(this->now().seconds()) % 15 == 0) {
                std::vector<std::string> formations = 
                    {"triangle", "line", "circle", "square"};
                
                std_msgs::msg::String formation_msg;
                formation_msg.data = "FORMATION: " + 
                    formations[formation_counter % formations.size()];
                formation_pub_->publish(formation_msg);
                
                formation_counter++;
            }
            
        } else if (phase_name == "collective_task") {
            // Update virtual object position
            static double object_angle = 0.0;
            object_angle += 0.01;
            
            double target_x = 5.0 * std::cos(object_angle);
            double target_y = 5.0 * std::sin(object_angle);
            
            if (static_cast<int>(this->now().seconds()) % 5 == 0) {
                std_msgs::msg::String task_msg;
                task_msg.data = "TRANSPORT: target_x=" + 
                    std::to_string(target_x) + " target_y=" + 
                    std::to_string(target_y);
                task_pub_->publish(task_msg);
            }
        }
    }
    
    void proposeToSwarm(const std::string& proposal) {
        std_msgs::msg::String proposal_msg;
        proposal_msg.data = proposal;
        proposal_pub_->publish(proposal_msg);
        
        RCLCPP_INFO(this->get_logger(), 
            "💡 Proposed to swarm: %s", proposal.c_str());
    }
    
    void handleConsensusDecision(const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), 
            "📊 Consensus reached: %s", msg->data.c_str());
        
        // Track consensus decisions for summary
        consensus_decisions_.push_back(msg->data);
    }
    
    void publishStatus(double elapsed_time) {
        std_msgs::msg::String status_msg;
        
        int phase_num = current_phase_ + 1;
        std::string phase_name = current_phase_ < scenario_sequence_.size() ? 
            scenario_sequence_[current_phase_] : "complete";
        
        double phase_progress = (elapsed_time - current_phase_ * phase_duration_) / 
            phase_duration_ * 100.0;
        
        status_msg.data = 
            "Phase " + std::to_string(phase_num) + "/" + 
            std::to_string(scenario_sequence_.size()) + ": " + phase_name +
            " | Progress: " + std::to_string(static_cast<int>(phase_progress)) + "%" +
            " | Coherence: " + std::to_string(current_coherence_) +
            " | Time: " + std::to_string(static_cast<int>(elapsed_time)) + "/" +
            std::to_string(demo_duration_) + "s";
        
        status_pub_->publish(status_msg);
    }
    
    void publishPhaseMarker(const std::string& phase_name) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "demo_phase";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 5.0;
        
        marker.scale.z = 1.0;
        
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        
        marker.text = "PHASE: " + phase_name;
        marker.lifetime = rclcpp::Duration(phase_duration_, 0);
        
        phase_marker_pub_->publish(marker);
    }
    
    void publishDemoSummary() {
        RCLCPP_INFO(this->get_logger(), 
            "\n"
            "========================================\n"
            "          DEMO SUMMARY                  \n"
            "========================================\n"
            "Total Duration: %d seconds\n"
            "Phases Completed: %zu\n"
            "Average Coherence: %.2f\n"
            "Consensus Decisions: %zu\n"
            "========================================\n",
            demo_duration_,
            scenario_sequence_.size(),
            current_coherence_,
            consensus_decisions_.size());
        
        // Publish final summary
        std_msgs::msg::String summary_msg;
        summary_msg.data = "DEMO_COMPLETE: " + 
            std::to_string(scenario_sequence_.size()) + " phases, " +
            std::to_string(consensus_decisions_.size()) + " decisions";
        status_pub_->publish(summary_msg);
    }
    
    // Member variables
    int demo_duration_;
    int phase_duration_;
    int current_phase_;
    double current_coherence_ = 0.5;
    
    std::vector<std::string> scenario_sequence_;
    std::vector<std::string> consensus_decisions_;
    
    rclcpp::Time demo_start_time_;
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr scenario_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr formation_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr proposal_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr phase_marker_pub_;
    
    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr metrics_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr consensus_sub_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr demo_timer_;
    rclcpp::TimerBase::SharedPtr command_timer_;
};

} // namespace mycelix

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), 
        "\n"
        "╔═══════════════════════════════════════════════════════╗\n"
        "║    🎬 MYCELIX THREE-AGENT DEMO CONTROLLER            ║\n"
        "║    Orchestrating Collective Intelligence Demo         ║\n"
        "╚═══════════════════════════════════════════════════════╝\n");
    
    auto node = std::make_shared<mycelix::DemoControllerNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}
