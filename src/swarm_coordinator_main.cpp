/**
 * Swarm Coordinator Main Executable
 * Standalone program for running the swarm coordination node
 */

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <csignal>
#include <atomic>

// Include the swarm coordinator node implementation
#include "swarm_coordinator.cpp"

std::atomic<bool> running(true);

void signalHandler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        RCLCPP_INFO(rclcpp::get_logger("main"), 
            "🛑 Shutdown signal received. Gracefully stopping swarm coordinator...");
        running = false;
    }
}

void printBanner() {
    std::cout << R"(
╔════════════════════════════════════════════════════════════════╗
║                                                                ║
║     🐝 MYCELIX SWARM COORDINATOR v1.0.0                       ║
║     Collective Intelligence for Multi-Robot Systems           ║
║                                                                ║
║     Features:                                                  ║
║     • Dynamic Formation Control (6 types)                     ║
║     • Byzantine-Resistant Consensus                           ║
║     • Real-time Consciousness Field Synchronization           ║
║     • Reynolds' Swarm Behavior Rules                          ║
║     • Distributed Decision Making                             ║
║                                                                ║
╚════════════════════════════════════════════════════════════════╝
)" << std::endl;
}

void printUsage(const char* program_name) {
    std::cout << "\nUsage: " << program_name << " [options]\n\n"
              << "Options:\n"
              << "  --ros-args               Pass ROS2 arguments\n"
              << "  --swarm_size <N>         Number of robots in swarm (default: 3)\n"
              << "  --formation <type>       Formation type: triangle, line, circle, square, v_formation, dynamic\n"
              << "  --consensus <threshold>  Consensus threshold (0.0-1.0, default: 0.66)\n"
              << "  --spacing <meters>       Formation spacing in meters (default: 2.0)\n"
              << "  --frequency <Hz>         Coordination frequency (default: 5.0)\n"
              << "  --holochain <url>        Holochain conductor URL (default: ws://localhost:8888)\n"
              << "  --help                   Show this help message\n\n"
              << "Examples:\n"
              << "  " << program_name << " --swarm_size 5 --formation triangle\n"
              << "  " << program_name << " --consensus 0.75 --spacing 3.0\n"
              << "  " << program_name << " --ros-args -p swarm_size:=10 -p formation_type:=dynamic\n\n";
}

int main(int argc, char* argv[]) {
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            printBanner();
            printUsage(argv[0]);
            return 0;
        }
    }
    
    // Print startup banner
    printBanner();
    
    // Register signal handlers
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), 
        "🚀 Initializing Mycelix Swarm Coordinator...");
    
    // Set executor options for better real-time performance
    rclcpp::ExecutorOptions executor_options;
    rclcpp::executors::MultiThreadedExecutor executor(executor_options, 4);
    
    try {
        // Create the swarm coordinator node
        auto node = std::make_shared<mycelix::SwarmCoordinatorNode>();
        
        RCLCPP_INFO(rclcpp::get_logger("main"), 
            "✅ Swarm Coordinator successfully initialized!");
        
        // Add node to executor
        executor.add_node(node);
        
        // Main loop with clean shutdown
        while (running && rclcpp::ok()) {
            executor.spin_some(std::chrono::milliseconds(100));
        }
        
        RCLCPP_INFO(rclcpp::get_logger("main"), 
            "🌙 Swarm Coordinator shutting down gracefully...");
        
        // Clean shutdown
        executor.remove_node(node);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), 
            "❌ Fatal error in Swarm Coordinator: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    // Shutdown ROS2
    rclcpp::shutdown();
    
    RCLCPP_INFO(rclcpp::get_logger("main"), 
        "👋 Swarm Coordinator has been stopped. Goodbye!");
    
    return 0;
}