/**
 * Holochain Agent Interface for ROS2 Bridge
 * Manages connection and communication with Holochain network
 */

#ifndef MYCELIX_BRIDGE_HOLOCHAIN_AGENT_HPP
#define MYCELIX_BRIDGE_HOLOCHAIN_AGENT_HPP

#include <string>
#include <vector>
#include <memory>
#include <map>
#include <chrono>
#include <functional>

namespace mycelix {

// Forward declarations
struct Model;
struct ModelUpdate;
struct ConsciousnessState;
struct SwarmState;
struct SwarmProposal;
struct SwarmAction;
struct RobotProfile;
struct RobotAction;
struct ValidationResult;

/**
 * Main Holochain agent interface
 * Handles all communication with the Mycelix consciousness network
 */
class HolochainAgent {
public:
    /**
     * Constructor
     * @param agent_id Unique identifier for this robot agent
     * @param conductor_url WebSocket URL for Holochain conductor
     */
    HolochainAgent(const std::string& agent_id, 
                   const std::string& conductor_url = "ws://localhost:8888");
    
    ~HolochainAgent();
    
    // === Registration & Identity ===
    
    /**
     * Register this robot in the consciousness network
     * @param profile Robot profile with capabilities
     * @return Success status
     */
    bool registerRobot(const RobotProfile& profile);
    
    /**
     * Update robot profile
     * @param profile Updated profile information
     */
    void updateProfile(const RobotProfile& profile);
    
    /**
     * Get agent's public key
     * @return Agent public key as string
     */
    std::string getAgentPubKey() const;
    
    // === Federated Learning ===
    
    /**
     * Submit model update for federated learning round
     * @param update Model gradients and metadata
     * @return Submission hash
     */
    std::string submitModelUpdate(const ModelUpdate& update);
    
    /**
     * Get aggregated model from swarm
     * @param round_id Learning round identifier
     * @return Aggregated model from swarm
     */
    Model getAggregatedModel(uint64_t round_id);
    
    /**
     * Get current swarm model
     * @return Latest aggregated model
     */
    Model getSwarmModel();
    
    /**
     * Publish gradients for federated learning
     * @param gradients Model gradients
     */
    void publishGradients(const std::vector<double>& gradients);
    
    // === Action Validation ===
    
    /**
     * Validate robot action through network consensus
     * @param action Proposed robot action
     * @return Validation result with approval/rejection
     */
    bool validateAction(const RobotAction& action);
    
    /**
     * Submit action for swarm validation
     * @param action Proposed action
     * @return Validation hash
     */
    std::string submitForValidation(const RobotAction& action);
    
    // === Consciousness Synchronization ===
    
    /**
     * Broadcast consciousness state to network
     * @param state Current consciousness metrics
     */
    void broadcastConsciousness(const ConsciousnessState& state);
    
    /**
     * Get swarm consciousness state
     * @return Aggregated consciousness metrics
     */
    SwarmState getSwarmConsciousness();
    
    /**
     * Get entanglement count
     * @return Number of consciousness connections
     */
    uint32_t getEntanglementCount() const;
    
    // === Swarm Coordination ===
    
    /**
     * Get nearby swarm members
     * @param max_count Maximum number of neighbors to return
     * @return List of nearby agent IDs
     */
    std::vector<std::string> getSwarmNeighbors(size_t max_count = 10);
    
    /**
     * Get active proposals from swarm
     * @return List of pending proposals
     */
    std::vector<SwarmProposal> getActiveProposals();
    
    /**
     * Vote on swarm proposal
     * @param proposal_id Proposal identifier
     * @param approve Vote (true = approve, false = reject)
     */
    void voteOnProposal(const std::string& proposal_id, bool approve);
    
    /**
     * Get approved swarm actions
     * @return List of consensus-approved actions
     */
    std::vector<SwarmAction> getApprovedActions();
    
    /**
     * Propose action to swarm
     * @param action Proposed swarm action
     * @return Proposal hash
     */
    std::string proposeToSwarm(const SwarmAction& action);
    
    // === Connection Management ===
    
    /**
     * Connect to Holochain conductor
     * @return Connection success
     */
    bool connect();
    
    /**
     * Disconnect from conductor
     */
    void disconnect();
    
    /**
     * Check connection status
     * @return True if connected
     */
    bool isConnected() const;
    
    /**
     * Set connection callback
     * @param callback Function called on connection state change
     */
    void setConnectionCallback(std::function<void(bool)> callback);
    
    // === Metrics & Monitoring ===
    
    /**
     * Get network metrics
     * @return Map of metric name to value
     */
    std::map<std::string, double> getNetworkMetrics() const;
    
    /**
     * Get agent reputation score
     * @param agent_id Agent to query (empty for self)
     * @return Reputation score (0.0 to 1.0)
     */
    double getReputationScore(const std::string& agent_id = "") const;
    
private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

// === Data Structures ===

/**
 * Robot profile for registration
 */
struct RobotProfile {
    std::string model;
    std::string manufacturer;
    std::string ros_version;
    double base_frequency;  // Resonance frequency in Hz
    std::vector<std::string> capabilities;
    std::map<std::string, std::string> metadata;
};

/**
 * Robot action for validation
 */
struct RobotAction {
    std::string action_type;
    std::vector<double> parameters;
    double confidence;
    std::chrono::milliseconds timeout;
    std::string safety_context;
};

/**
 * Model update for federated learning
 */
struct ModelUpdate {
    uint64_t round_id;
    std::vector<double> gradients;
    double epsilon;  // Privacy budget used
    uint32_t local_samples;
    double loss;
    std::map<std::string, double> metrics;
};

/**
 * Aggregated model from swarm
 */
struct Model {
    std::map<std::string, std::vector<double>> parameters;
    uint64_t version;
    uint32_t participant_count;
    double validation_score;
    std::chrono::system_clock::time_point timestamp;
};

/**
 * Consciousness state metrics
 */
struct ConsciousnessState {
    double coherence;       // Internal coherence (0.0 to 1.0)
    double resonance;       // Swarm resonance (0.0 to 1.0)
    uint32_t entanglement_count;
    double awareness_level;
    double integration_phi; // Integrated information
    std::map<std::string, double> custom_metrics;
};

/**
 * Swarm consciousness state
 */
struct SwarmState {
    size_t swarm_size;
    double avg_coherence;
    double avg_resonance;
    double collective_phi;
    double emergence_indicator;
    std::map<std::string, double> swarm_metrics;
};

/**
 * Swarm proposal for voting
 */
struct SwarmProposal {
    std::string id;
    std::string proposer;
    std::string action_type;
    std::vector<double> parameters;
    double benefit_score;
    double risk_score;
    std::chrono::system_clock::time_point deadline;
    uint32_t votes_for;
    uint32_t votes_against;
};

/**
 * Approved swarm action
 */
struct SwarmAction {
    std::string id;
    std::string action_type;
    std::string description;
    std::vector<double> parameters;
    double consensus_level;
    std::chrono::system_clock::time_point execution_time;
};

/**
 * Validation result
 */
struct ValidationResult {
    bool approved;
    std::string reason;
    double confidence;
    std::vector<std::string> validators;
    std::chrono::milliseconds validation_time;
};

} // namespace mycelix

#endif // MYCELIX_BRIDGE_HOLOCHAIN_AGENT_HPP