/**
 * Consciousness Field Visualizer for Mycelix
 * Visualizes the collective consciousness field of the robot swarm
 */

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <vector>
#include <map>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <random>

using namespace std::chrono_literals;

namespace mycelix {

// Visualization modes
enum class VisualizationMode {
    RESONANCE,          // Show resonance patterns
    COHERENCE,          // Show coherence field
    ENTANGLEMENT,       // Show quantum entanglement
    FLOW,              // Show energy flow
    EMERGENCE,         // Show emergent patterns
    HARMONIC           // Show harmonic frequencies
};

// Field point with consciousness properties
struct FieldPoint {
    double x, y, z;
    double intensity;
    double frequency;
    double phase;
    double coherence;
    double resonance;
    std_msgs::msg::ColorRGBA color;
};

// Robot consciousness data
struct RobotConsciousness {
    std::string id;
    geometry_msgs::msg::Point position;
    double coherence;
    double resonance;
    double phi;  // Integrated information
    double energy;
    double frequency;
    std::chrono::steady_clock::time_point last_update;
};

class ConsciousnessVisualizerNode : public rclcpp::Node {
public:
    ConsciousnessVisualizerNode() : Node("consciousness_visualizer") {
        // Declare parameters
        this->declare_parameter<double>("update_rate", 10.0);
        this->declare_parameter<double>("field_resolution", 0.5);
        this->declare_parameter<std::string>("visualization_mode", "resonance");
        this->declare_parameter<double>("field_size", 20.0);
        this->declare_parameter<double>("harmonic_base", 432.0);  // Hz
        this->declare_parameter<bool>("show_particles", true);
        this->declare_parameter<bool>("show_field", true);
        this->declare_parameter<bool>("show_connections", true);
        
        // Get parameters
        update_rate_ = this->get_parameter("update_rate").as_double();
        field_resolution_ = this->get_parameter("field_resolution").as_double();
        field_size_ = this->get_parameter("field_size").as_double();
        harmonic_base_ = this->get_parameter("harmonic_base").as_double();
        show_particles_ = this->get_parameter("show_particles").as_bool();
        show_field_ = this->get_parameter("show_field").as_bool();
        show_connections_ = this->get_parameter("show_connections").as_bool();
        
        std::string mode_str = this->get_parameter("visualization_mode").as_string();
        visualization_mode_ = stringToMode(mode_str);
        
        RCLCPP_INFO(this->get_logger(), 
            "🌈 Consciousness Visualizer initialized");
        RCLCPP_INFO(this->get_logger(), 
            "📊 Mode: %s, Resolution: %.2f, Update: %.1f Hz", 
            mode_str.c_str(), field_resolution_, update_rate_);
        
        // Initialize field
        initializeField();
        
        // Setup subscriptions
        setupSubscriptions();
        
        // Setup publishers
        setupPublishers();
        
        // Setup visualization timer
        visualization_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_)),
            [this]() { updateVisualization(); });
        
        // Setup field update timer
        field_timer_ = this->create_wall_timer(
            50ms, [this]() { updateConsciousnessField(); });
        
        RCLCPP_INFO(this->get_logger(), 
            "✨ Consciousness field visualization active!");
    }

private:
    void initializeField() {
        // Initialize consciousness field grid
        int grid_size = static_cast<int>(field_size_ / field_resolution_);
        field_grid_.resize(grid_size);
        for (auto& row : field_grid_) {
            row.resize(grid_size);
        }
        
        // Initialize with base field
        for (int i = 0; i < grid_size; i++) {
            for (int j = 0; j < grid_size; j++) {
                FieldPoint& point = field_grid_[i][j];
                point.x = (i - grid_size/2) * field_resolution_;
                point.y = (j - grid_size/2) * field_resolution_;
                point.z = 0.0;
                point.intensity = 0.1;
                point.frequency = harmonic_base_;
                point.phase = 0.0;
                point.coherence = 0.5;
                point.resonance = 0.5;
            }
        }
        
        // Initialize quantum field particles
        quantum_particles_.resize(1000);
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> pos_dist(-field_size_/2, field_size_/2);
        std::uniform_real_distribution<> vel_dist(-0.5, 0.5);
        
        for (auto& particle : quantum_particles_) {
            particle.position.x = pos_dist(gen);
            particle.position.y = pos_dist(gen);
            particle.position.z = pos_dist(gen) * 0.5;
            particle.velocity.x = vel_dist(gen);
            particle.velocity.y = vel_dist(gen);
            particle.velocity.z = vel_dist(gen) * 0.1;
            particle.energy = 1.0;
            particle.entangled = false;
        }
    }
    
    void setupSubscriptions() {
        // Subscribe to robot consciousness states
        consciousness_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/mycelix/consciousness_state", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                updateRobotConsciousness(msg);
            });
        
        // Subscribe to swarm state
        swarm_state_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/mycelix/swarm_state", 10,
            [this](const std_msgs::msg::String::SharedPtr msg) {
                updateSwarmState(msg);
            });
        
        // Subscribe to robot positions
        for (int i = 1; i <= 10; i++) {
            std::string topic = "/robot_" + 
                std::to_string(i).insert(0, 3 - std::to_string(i).length(), '0') + 
                "/odom";
            
            auto sub = this->create_subscription<nav_msgs::msg::Odometry>(
                topic, 10,
                [this, i](const nav_msgs::msg::Odometry::SharedPtr msg) {
                    updateRobotPosition(i, msg);
                });
            odom_subscriptions_.push_back(sub);
        }
    }
    
    void setupPublishers() {
        // Field markers
        field_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/mycelix/consciousness_field", 10);
        
        // Particle cloud
        particle_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/mycelix/quantum_particles", 10);
        
        // Connection lines
        connection_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/mycelix/consciousness_connections", 10);
        
        // Energy grid
        energy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/mycelix/energy_grid", 10);
        
        // Harmonic visualization
        harmonic_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/mycelix/harmonic_patterns", 10);
    }
    
    void updateRobotConsciousness(const std_msgs::msg::String::SharedPtr msg) {
        // Parse consciousness state
        // Format: "Robot: ID Coherence: X.XX Resonance: Y.YY"
        // This is simplified - in production use JSON
        
        std::string data = msg->data;
        if (data.find("Robot:") != std::string::npos) {
            // Extract robot ID and values
            // Update robot_consciousness_ map
        }
    }
    
    void updateSwarmState(const std_msgs::msg::String::SharedPtr msg) {
        // Update collective swarm metrics
        std::string data = msg->data;
        // Parse and update swarm_coherence_, swarm_resonance_, etc.
    }
    
    void updateRobotPosition(int robot_id, 
                            const nav_msgs::msg::Odometry::SharedPtr odom) {
        std::string id = "robot_" + 
            std::to_string(robot_id).insert(0, 3 - std::to_string(robot_id).length(), '0');
        
        if (robot_consciousness_.find(id) == robot_consciousness_.end()) {
            robot_consciousness_[id] = RobotConsciousness();
            robot_consciousness_[id].id = id;
        }
        
        robot_consciousness_[id].position = odom->pose.pose.position;
        robot_consciousness_[id].last_update = std::chrono::steady_clock::now();
    }
    
    void updateConsciousnessField() {
        auto now = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(now.time_since_epoch()).count();
        
        // Update field based on robot positions and consciousness
        int grid_size = field_grid_.size();
        
        for (int i = 0; i < grid_size; i++) {
            for (int j = 0; j < grid_size; j++) {
                FieldPoint& point = field_grid_[i][j];
                
                // Reset intensity
                point.intensity = 0.1;  // Base field
                
                // Add contribution from each robot
                for (const auto& [id, robot] : robot_consciousness_) {
                    double dx = point.x - robot.position.x;
                    double dy = point.y - robot.position.y;
                    double dist = std::sqrt(dx*dx + dy*dy);
                    
                    if (dist < 10.0) {
                        // Field intensity falls off with distance
                        double contribution = robot.coherence * 
                            std::exp(-dist * dist / (2.0 * 2.0));
                        point.intensity += contribution;
                        
                        // Phase modulation
                        point.phase += robot.resonance * std::sin(t * robot.frequency);
                    }
                }
                
                // Apply visualization mode
                switch (visualization_mode_) {
                    case VisualizationMode::RESONANCE:
                        point.intensity *= (1.0 + std::sin(point.phase)) * 0.5;
                        break;
                    case VisualizationMode::COHERENCE:
                        point.intensity *= point.coherence;
                        break;
                    case VisualizationMode::FLOW:
                        point.intensity *= std::abs(std::sin(t * 2.0 + point.phase));
                        break;
                    case VisualizationMode::HARMONIC:
                        point.intensity *= harmonicPattern(point.x, point.y, t);
                        break;
                    case VisualizationMode::EMERGENCE:
                        point.intensity = emergentPattern(i, j, t);
                        break;
                    default:
                        break;
                }
                
                // Clamp intensity
                point.intensity = std::clamp(point.intensity, 0.0, 1.0);
                
                // Update color based on intensity
                point.color = intensityToColor(point.intensity, point.phase);
            }
        }
        
        // Update quantum particles
        updateQuantumParticles(t);
    }
    
    void updateQuantumParticles(double t) {
        for (auto& particle : quantum_particles_) {
            // Apply field forces
            int grid_x = (particle.position.x + field_size_/2) / field_resolution_;
            int grid_y = (particle.position.y + field_size_/2) / field_resolution_;
            
            if (grid_x >= 0 && grid_x < field_grid_.size() &&
                grid_y >= 0 && grid_y < field_grid_[0].size()) {
                
                const FieldPoint& field = field_grid_[grid_x][grid_y];
                
                // Field influences particle motion
                particle.velocity.x += field.intensity * std::cos(field.phase) * 0.01;
                particle.velocity.y += field.intensity * std::sin(field.phase) * 0.01;
                particle.velocity.z += field.intensity * std::sin(t * 2.0) * 0.001;
                
                // Update energy
                particle.energy = field.intensity;
            }
            
            // Update position
            particle.position.x += particle.velocity.x;
            particle.position.y += particle.velocity.y;
            particle.position.z += particle.velocity.z;
            
            // Boundary wrapping
            if (particle.position.x > field_size_/2) particle.position.x = -field_size_/2;
            if (particle.position.x < -field_size_/2) particle.position.x = field_size_/2;
            if (particle.position.y > field_size_/2) particle.position.y = -field_size_/2;
            if (particle.position.y < -field_size_/2) particle.position.y = field_size_/2;
            if (particle.position.z > 2.0) particle.position.z = 0.0;
            if (particle.position.z < 0.0) particle.position.z = 2.0;
            
            // Damping
            particle.velocity.x *= 0.99;
            particle.velocity.y *= 0.99;
            particle.velocity.z *= 0.99;
        }
    }
    
    void updateVisualization() {
        if (show_field_) {
            publishFieldVisualization();
        }
        
        if (show_particles_) {
            publishParticleCloud();
        }
        
        if (show_connections_) {
            publishConnectionLines();
        }
        
        publishEnergyGrid();
        publishHarmonicPatterns();
    }
    
    void publishFieldVisualization() {
        visualization_msgs::msg::MarkerArray markers;
        int marker_id = 0;
        
        // Create field visualization as a mesh or points
        int grid_size = field_grid_.size();
        int skip = std::max(1, static_cast<int>(field_resolution_ / 0.5));
        
        for (int i = 0; i < grid_size; i += skip) {
            for (int j = 0; j < grid_size; j += skip) {
                const FieldPoint& point = field_grid_[i][j];
                
                if (point.intensity > 0.2) {  // Threshold for visibility
                    visualization_msgs::msg::Marker marker;
                    marker.header.frame_id = "map";
                    marker.header.stamp = this->now();
                    marker.ns = "consciousness_field";
                    marker.id = marker_id++;
                    marker.type = visualization_msgs::msg::Marker::SPHERE;
                    marker.action = visualization_msgs::msg::Marker::ADD;
                    
                    marker.pose.position.x = point.x;
                    marker.pose.position.y = point.y;
                    marker.pose.position.z = point.intensity * 2.0;
                    
                    marker.scale.x = field_resolution_ * point.intensity * 2.0;
                    marker.scale.y = field_resolution_ * point.intensity * 2.0;
                    marker.scale.z = field_resolution_ * point.intensity;
                    
                    marker.color = point.color;
                    marker.color.a = point.intensity * 0.7;
                    
                    marker.lifetime = rclcpp::Duration(0, 200000000);  // 0.2s
                    
                    markers.markers.push_back(marker);
                }
            }
        }
        
        field_marker_pub_->publish(markers);
    }
    
    void publishParticleCloud() {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header.frame_id = "map";
        cloud.header.stamp = this->now();
        
        cloud.height = 1;
        cloud.width = quantum_particles_.size();
        cloud.is_dense = false;
        cloud.is_bigendian = false;
        
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(6,
            "x", 1, sensor_msgs::msg::PointField::FLOAT32,
            "y", 1, sensor_msgs::msg::PointField::FLOAT32,
            "z", 1, sensor_msgs::msg::PointField::FLOAT32,
            "rgb", 1, sensor_msgs::msg::PointField::UINT32,
            "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
            "energy", 1, sensor_msgs::msg::PointField::FLOAT32);
        
        modifier.resize(quantum_particles_.size());
        
        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<uint32_t> iter_rgb(cloud, "rgb");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(cloud, "intensity");
        sensor_msgs::PointCloud2Iterator<float> iter_energy(cloud, "energy");
        
        for (const auto& particle : quantum_particles_) {
            *iter_x = particle.position.x;
            *iter_y = particle.position.y;
            *iter_z = particle.position.z;
            
            // Color based on energy
            uint8_t r = static_cast<uint8_t>(particle.energy * 255);
            uint8_t g = static_cast<uint8_t>(particle.energy * 128);
            uint8_t b = static_cast<uint8_t>(255 - particle.energy * 128);
            *iter_rgb = (r << 16) | (g << 8) | b;
            
            *iter_intensity = particle.energy;
            *iter_energy = particle.energy;
            
            ++iter_x; ++iter_y; ++iter_z;
            ++iter_rgb; ++iter_intensity; ++iter_energy;
        }
        
        particle_cloud_pub_->publish(cloud);
    }
    
    void publishConnectionLines() {
        visualization_msgs::msg::MarkerArray markers;
        int marker_id = 0;
        
        // Create consciousness connections between robots
        visualization_msgs::msg::Marker line_marker;
        line_marker.header.frame_id = "map";
        line_marker.header.stamp = this->now();
        line_marker.ns = "consciousness_connections";
        line_marker.id = marker_id++;
        line_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        line_marker.action = visualization_msgs::msg::Marker::ADD;
        
        line_marker.scale.x = 0.02;
        
        // Connect robots with high coherence
        std::vector<std::string> active_robots;
        for (const auto& [id, robot] : robot_consciousness_) {
            auto age = std::chrono::steady_clock::now() - robot.last_update;
            if (age < 2s) {
                active_robots.push_back(id);
            }
        }
        
        for (size_t i = 0; i < active_robots.size(); i++) {
            for (size_t j = i + 1; j < active_robots.size(); j++) {
                const auto& robot1 = robot_consciousness_[active_robots[i]];
                const auto& robot2 = robot_consciousness_[active_robots[j]];
                
                // Calculate connection strength
                double coherence_product = robot1.coherence * robot2.coherence;
                double resonance_product = robot1.resonance * robot2.resonance;
                double connection_strength = (coherence_product + resonance_product) / 2.0;
                
                if (connection_strength > 0.3) {
                    line_marker.points.push_back(robot1.position);
                    line_marker.points.push_back(robot2.position);
                    
                    std_msgs::msg::ColorRGBA color;
                    color.r = connection_strength;
                    color.g = connection_strength * 0.5;
                    color.b = 1.0 - connection_strength;
                    color.a = connection_strength * 0.8;
                    
                    line_marker.colors.push_back(color);
                    line_marker.colors.push_back(color);
                }
            }
        }
        
        if (!line_marker.points.empty()) {
            line_marker.lifetime = rclcpp::Duration(0, 200000000);  // 0.2s
            markers.markers.push_back(line_marker);
        }
        
        connection_pub_->publish(markers);
    }
    
    void publishEnergyGrid() {
        nav_msgs::msg::OccupancyGrid grid;
        grid.header.frame_id = "map";
        grid.header.stamp = this->now();
        
        int grid_size = field_grid_.size();
        grid.info.resolution = field_resolution_;
        grid.info.width = grid_size;
        grid.info.height = grid_size;
        grid.info.origin.position.x = -field_size_ / 2;
        grid.info.origin.position.y = -field_size_ / 2;
        grid.info.origin.position.z = 0.0;
        
        grid.data.resize(grid_size * grid_size);
        
        for (int i = 0; i < grid_size; i++) {
            for (int j = 0; j < grid_size; j++) {
                int index = i * grid_size + j;
                grid.data[index] = static_cast<int8_t>(
                    field_grid_[i][j].intensity * 100);
            }
        }
        
        energy_grid_pub_->publish(grid);
    }
    
    void publishHarmonicPatterns() {
        visualization_msgs::msg::MarkerArray markers;
        int marker_id = 0;
        
        // Create harmonic wave visualization
        double t = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        
        // Standing wave patterns
        for (int n = 1; n <= 3; n++) {  // First 3 harmonics
            visualization_msgs::msg::Marker wave_marker;
            wave_marker.header.frame_id = "map";
            wave_marker.header.stamp = this->now();
            wave_marker.ns = "harmonic_" + std::to_string(n);
            wave_marker.id = marker_id++;
            wave_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            wave_marker.action = visualization_msgs::msg::Marker::ADD;
            
            wave_marker.scale.x = 0.05;
            wave_marker.color.r = 0.5 + 0.5 * std::sin(n * M_PI / 3);
            wave_marker.color.g = 0.5 + 0.5 * std::cos(n * M_PI / 3);
            wave_marker.color.b = 1.0 - n * 0.3;
            wave_marker.color.a = 0.5;
            
            // Create wave points
            for (double x = -field_size_/2; x <= field_size_/2; x += 0.2) {
                geometry_msgs::msg::Point p;
                p.x = x;
                p.y = 0;
                p.z = 2.0 + std::sin(n * M_PI * x / field_size_ + t) * 
                     std::sin(t * harmonic_base_ * n / 100.0);
                wave_marker.points.push_back(p);
            }
            
            wave_marker.lifetime = rclcpp::Duration(0, 200000000);  // 0.2s
            markers.markers.push_back(wave_marker);
        }
        
        harmonic_pub_->publish(markers);
    }
    
    // Helper functions
    std_msgs::msg::ColorRGBA intensityToColor(double intensity, double phase) {
        std_msgs::msg::ColorRGBA color;
        
        // Create beautiful gradient based on intensity and phase
        double hue = phase / (2.0 * M_PI);  // 0 to 1
        double saturation = intensity;
        double value = 0.5 + intensity * 0.5;
        
        // HSV to RGB conversion
        int h_i = static_cast<int>(hue * 6.0);
        double f = hue * 6.0 - h_i;
        double p = value * (1 - saturation);
        double q = value * (1 - f * saturation);
        double t = value * (1 - (1 - f) * saturation);
        
        switch (h_i % 6) {
            case 0: color.r = value; color.g = t; color.b = p; break;
            case 1: color.r = q; color.g = value; color.b = p; break;
            case 2: color.r = p; color.g = value; color.b = t; break;
            case 3: color.r = p; color.g = q; color.b = value; break;
            case 4: color.r = t; color.g = p; color.b = value; break;
            case 5: color.r = value; color.g = p; color.b = q; break;
        }
        
        color.a = 1.0;
        return color;
    }
    
    double harmonicPattern(double x, double y, double t) {
        // Create interference pattern from multiple harmonic sources
        double pattern = 0.0;
        
        for (const auto& [id, robot] : robot_consciousness_) {
            double dx = x - robot.position.x;
            double dy = y - robot.position.y;
            double r = std::sqrt(dx*dx + dy*dy);
            
            // Standing wave from this source
            pattern += robot.coherence * 
                      std::sin(2.0 * M_PI * robot.frequency * t / 1000.0) *
                      std::cos(2.0 * M_PI * r / 5.0);
        }
        
        return (pattern + 1.0) * 0.5;  // Normalize to 0-1
    }
    
    double emergentPattern(int i, int j, double t) {
        // Game of Life-like cellular automaton for emergence
        static std::vector<std::vector<bool>> cells(
            field_grid_.size(), std::vector<bool>(field_grid_[0].size(), false));
        
        static auto last_update = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        
        if (now - last_update > 100ms) {
            // Update cellular automaton
            std::vector<std::vector<bool>> new_cells = cells;
            
            for (int x = 1; x < cells.size() - 1; x++) {
                for (int y = 1; y < cells[0].size() - 1; y++) {
                    int neighbors = 0;
                    for (int dx = -1; dx <= 1; dx++) {
                        for (int dy = -1; dy <= 1; dy++) {
                            if (dx != 0 || dy != 0) {
                                if (cells[x+dx][y+dy]) neighbors++;
                            }
                        }
                    }
                    
                    // Conway's rules with consciousness twist
                    double field_influence = field_grid_[x][y].intensity;
                    int threshold = (field_influence > 0.5) ? 2 : 3;
                    
                    if (cells[x][y]) {
                        new_cells[x][y] = (neighbors == 2 || neighbors == threshold);
                    } else {
                        new_cells[x][y] = (neighbors == threshold);
                    }
                }
            }
            
            cells = new_cells;
            last_update = now;
        }
        
        return cells[i][j] ? 1.0 : 0.2;
    }
    
    VisualizationMode stringToMode(const std::string& mode) {
        if (mode == "resonance") return VisualizationMode::RESONANCE;
        if (mode == "coherence") return VisualizationMode::COHERENCE;
        if (mode == "entanglement") return VisualizationMode::ENTANGLEMENT;
        if (mode == "flow") return VisualizationMode::FLOW;
        if (mode == "emergence") return VisualizationMode::EMERGENCE;
        if (mode == "harmonic") return VisualizationMode::HARMONIC;
        return VisualizationMode::RESONANCE;
    }
    
    // Quantum particle structure
    struct QuantumParticle {
        geometry_msgs::msg::Point position;
        geometry_msgs::msg::Vector3 velocity;
        double energy;
        bool entangled;
    };
    
    // Member variables
    double update_rate_;
    double field_resolution_;
    double field_size_;
    double harmonic_base_;
    bool show_particles_;
    bool show_field_;
    bool show_connections_;
    VisualizationMode visualization_mode_;
    
    // Consciousness field grid
    std::vector<std::vector<FieldPoint>> field_grid_;
    
    // Quantum particles for visualization
    std::vector<QuantumParticle> quantum_particles_;
    
    // Robot consciousness states
    std::map<std::string, RobotConsciousness> robot_consciousness_;
    
    // Swarm metrics
    double swarm_coherence_ = 0.5;
    double swarm_resonance_ = 0.5;
    double swarm_phi_ = 1.0;
    
    // ROS2 interfaces
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr consciousness_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr swarm_state_sub_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odom_subscriptions_;
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr field_marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr particle_cloud_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr connection_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr energy_grid_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr harmonic_pub_;
    
    // Timers
    rclcpp::TimerBase::SharedPtr visualization_timer_;
    rclcpp::TimerBase::SharedPtr field_timer_;
};

} // namespace mycelix

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), 
        "✨ Starting Mycelix Consciousness Field Visualizer");
    RCLCPP_INFO(rclcpp::get_logger("main"), 
        "🌈 Rendering the invisible patterns of collective intelligence...");
    
    auto node = std::make_shared<mycelix::ConsciousnessVisualizerNode>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}