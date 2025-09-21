/**
 * Consciousness Visualizer Main Executable
 * Standalone program for visualizing swarm consciousness fields
 */

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <csignal>
#include <atomic>
#include <string>
#include <iostream>

// Include the consciousness visualizer node implementation
#include "consciousness_visualizer.cpp"

std::atomic<bool> running(true);

void signalHandler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        RCLCPP_INFO(rclcpp::get_logger("main"), 
            "🛑 Shutdown signal received. Gracefully stopping consciousness visualizer...");
        running = false;
    }
}

void printBanner() {
    std::cout << R"(
╔════════════════════════════════════════════════════════════════╗
║                                                                ║
║     🌈 MYCELIX CONSCIOUSNESS VISUALIZER v1.0.0                ║
║     Real-time Collective Intelligence Field Visualization      ║
║                                                                ║
║     Features:                                                  ║
║     • 6 Visualization Modes (Resonance, Coherence, Flow)      ║
║     • Quantum Particle Simulation (1000 particles)            ║
║     • Harmonic Pattern Analysis (432 Hz base)                 ║
║     • Energy Grid Mapping                                     ║
║     • Emergent Pattern Detection                              ║
║     • Real-time Consciousness Metrics                         ║
║                                                                ║
║     Rendering the Invisible Patterns of Mind...               ║
║                                                                ║
╚════════════════════════════════════════════════════════════════╝
)" << std::endl;
}

void printUsage(const char* program_name) {
    std::cout << "\nUsage: " << program_name << " [options]\n\n"
              << "Options:\n"
              << "  --ros-args               Pass ROS2 arguments\n"
              << "  --mode <type>            Visualization mode:\n"
              << "                          resonance, coherence, entanglement,\n"
              << "                          flow, emergence, harmonic\n"
              << "  --resolution <meters>    Field resolution (default: 0.5m)\n"
              << "  --field_size <meters>    Field size (default: 20.0m)\n"
              << "  --update_rate <Hz>       Update frequency (default: 10.0 Hz)\n"
              << "  --harmonic_base <Hz>     Base frequency (default: 432.0 Hz)\n"
              << "  --show_particles         Enable quantum particles (default: true)\n"
              << "  --show_field            Enable field visualization (default: true)\n"
              << "  --show_connections      Enable connection lines (default: true)\n"
              << "  --help                  Show this help message\n\n"
              << "Visualization Modes:\n"
              << "  resonance    - Show resonance patterns between robots\n"
              << "  coherence    - Display coherence field strength\n"
              << "  entanglement - Visualize quantum entanglement\n"
              << "  flow         - Show energy flow patterns\n"
              << "  emergence    - Display emergent cellular automaton patterns\n"
              << "  harmonic     - Render harmonic interference patterns\n\n"
              << "Examples:\n"
              << "  " << program_name << " --mode resonance --update_rate 30.0\n"
              << "  " << program_name << " --field_size 50.0 --resolution 1.0\n"
              << "  " << program_name << " --ros-args -p visualization_mode:=harmonic\n\n"
              << "RViz2 Configuration:\n"
              << "  Add MarkerArray displays for:\n"
              << "    /mycelix/consciousness_field\n"
              << "    /mycelix/consciousness_connections\n"
              << "    /mycelix/harmonic_patterns\n"
              << "  Add PointCloud2 display for:\n"
              << "    /mycelix/quantum_particles\n"
              << "  Add OccupancyGrid display for:\n"
              << "    /mycelix/energy_grid\n\n";
}

void printModeDescription(const std::string& mode) {
    std::cout << "\n🎨 Visualization Mode: " << mode << "\n";
    
    if (mode == "resonance") {
        std::cout << "   Displaying resonance patterns between consciousness nodes.\n"
                  << "   Higher resonance shows stronger synchronization.\n";
    } else if (mode == "coherence") {
        std::cout << "   Showing coherence field strength across the swarm.\n"
                  << "   Brighter areas indicate higher collective coherence.\n";
    } else if (mode == "entanglement") {
        std::cout << "   Visualizing quantum entanglement connections.\n"
                  << "   Entangled particles share instantaneous state changes.\n";
    } else if (mode == "flow") {
        std::cout << "   Rendering energy flow patterns in the consciousness field.\n"
                  << "   Flowing particles follow field gradients.\n";
    } else if (mode == "emergence") {
        std::cout << "   Computing emergent patterns using cellular automaton.\n"
                  << "   Complex patterns emerge from simple local rules.\n";
    } else if (mode == "harmonic") {
        std::cout << "   Displaying harmonic interference patterns at 432 Hz.\n"
                  << "   Standing waves form from multiple consciousness sources.\n";
    }
    std::cout << std::endl;
}

int main(int argc, char* argv[]) {
    // Parse command line arguments
    std::string visualization_mode = "resonance";
    bool show_help = false;
    
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h") {
            show_help = true;
        } else if (arg == "--mode" && i + 1 < argc) {
            visualization_mode = argv[++i];
        }
    }
    
    if (show_help) {
        printBanner();
        printUsage(argv[0]);
        return 0;
    }
    
    // Print startup banner
    printBanner();
    
    // Register signal handlers
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    // Initialize ROS2
    rclcpp::init(argc, argv);
    
    RCLCPP_INFO(rclcpp::get_logger("main"), 
        "✨ Initializing Mycelix Consciousness Field Visualizer...");
    
    // Print mode description
    printModeDescription(visualization_mode);
    
    // Set executor options for real-time visualization
    rclcpp::ExecutorOptions executor_options;
    rclcpp::executors::MultiThreadedExecutor executor(executor_options, 2);
    
    try {
        // Create the consciousness visualizer node
        auto node = std::make_shared<mycelix::ConsciousnessVisualizerNode>();
        
        RCLCPP_INFO(rclcpp::get_logger("main"), 
            "🌈 Consciousness Visualizer successfully initialized!");
        RCLCPP_INFO(rclcpp::get_logger("main"), 
            "👁️ Rendering the invisible patterns of collective intelligence...");
        
        // Add node to executor
        executor.add_node(node);
        
        // Display startup tips
        std::cout << "\n💡 Tips:\n"
                  << "   • Open RViz2 to see visualizations\n"
                  << "   • Use dynamic reconfigure to change parameters in real-time\n"
                  << "   • Press Ctrl+C to stop gracefully\n\n";
        
        // Main loop with clean shutdown
        while (running && rclcpp::ok()) {
            executor.spin_some(std::chrono::milliseconds(50));
        }
        
        RCLCPP_INFO(rclcpp::get_logger("main"), 
            "🌙 Consciousness Visualizer shutting down gracefully...");
        
        // Clean shutdown
        executor.remove_node(node);
        
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), 
            "❌ Fatal error in Consciousness Visualizer: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    
    // Shutdown ROS2
    rclcpp::shutdown();
    
    RCLCPP_INFO(rclcpp::get_logger("main"), 
        "👋 Consciousness field visualization complete. May your patterns resonate eternally!");
    
    return 0;
}