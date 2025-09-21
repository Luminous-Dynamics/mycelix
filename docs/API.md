# Mycelix API Documentation

## Core Classes

### HolochainAgent

The `HolochainAgent` class manages all Holochain network interactions for a robotic agent.

#### Constructor
```cpp
HolochainAgent(const std::string& agent_id, const std::string& conductor_url)
```
- `agent_id`: Unique identifier for this robot agent
- `conductor_url`: WebSocket URL of Holochain conductor (e.g., "ws://localhost:8888")

#### Connection Management

##### `bool connect()`
Establishes connection to Holochain conductor.
- **Returns**: `true` if connection successful

##### `void disconnect()`
Cleanly disconnects from Holochain network.

##### `bool isConnected() const`
Check current connection status.
- **Returns**: `true` if currently connected

##### `void setConnectionCallback(std::function<void(bool)> callback)`
Set callback for connection state changes.
- `callback`: Function called with `true` on connect, `false` on disconnect

#### Robot Registration

##### `bool registerRobot(const RobotProfile& profile)`
Register robot with network, establishing identity and capabilities.

```cpp
RobotProfile profile {
    .model = "TurtleBot3",
    .manufacturer = "ROBOTIS",
    .ros_version = "humble",
    .base_frequency = 50.0,
    .capabilities = {"navigation", "vision", "manipulation"},
    .metadata = {{"serial", "TB3-2024-001"}}
};
agent.registerRobot(profile);
```

##### `void updateProfile(const RobotProfile& profile)`
Update robot profile information on the network.

##### `std::string getAgentPubKey() const`
Get the agent's public key for cryptographic operations.
- **Returns**: Base64-encoded public key string

#### Federated Learning

##### `std::string submitModelUpdate(const ModelUpdate& update)`
Submit local model update to federated learning round.

```cpp
ModelUpdate update {
    .round_id = 42,
    .gradients = {0.1, -0.2, 0.3, ...},
    .epsilon = 1.0,  // Differential privacy budget
    .local_samples = 100,
    .loss = 0.234,
    .metrics = {{"accuracy", 0.95}, {"precision", 0.92}}
};
std::string hash = agent.submitModelUpdate(update);
```
- **Returns**: Submission hash for tracking

##### `Model getAggregatedModel(uint64_t round_id)`
Retrieve aggregated model from specific learning round.
- `round_id`: Round number to retrieve
- **Returns**: Aggregated `Model` struct with parameters

##### `Model getSwarmModel()`
Get current swarm-wide model (latest aggregated).
- **Returns**: Current collective `Model`

##### `void publishGradients(const std::vector<double>& gradients)`
Publish gradients for real-time collaborative learning.

#### Action Validation

##### `bool validateAction(const RobotAction& action)`
Validate proposed action with swarm consensus.

```cpp
RobotAction action {
    .action_type = "navigate_to_goal",
    .parameters = {5.0, 3.0, 0.0},  // x, y, theta
    .confidence = 0.85,
    .safety_context = "obstacle_detected"
};
bool approved = agent.validateAction(action);
```
- **Returns**: `true` if action approved by swarm

##### `std::string submitForValidation(const RobotAction& action)`
Submit action for asynchronous validation.
- **Returns**: Validation request hash

#### Consciousness Synchronization

##### `void broadcastConsciousness(const ConsciousnessState& state)`
Broadcast current consciousness metrics to swarm.

```cpp
ConsciousnessState state {
    .coherence = 0.85,
    .resonance = 0.72,
    .entanglement_count = 5,
    .awareness_level = 0.9,
    .integration_phi = 0.24,
    .custom_metrics = {{"creativity", 0.6}, {"empathy", 0.8}}
};
agent.broadcastConsciousness(state);
```

##### `SwarmState getSwarmConsciousness()`
Retrieve collective consciousness state.

```cpp
SwarmState swarm = agent.getSwarmConsciousness();
std::cout << "Collective Phi: " << swarm.collective_phi << std::endl;
std::cout << "Average coherence: " << swarm.avg_coherence << std::endl;
```

##### `uint32_t getEntanglementCount() const`
Get number of consciousness entanglements with other agents.
- **Returns**: Count of active entanglements

#### Swarm Coordination

##### `std::vector<std::string> getSwarmNeighbors(size_t max_count = 10)`
Get list of nearby swarm members.
- `max_count`: Maximum neighbors to return
- **Returns**: Vector of agent IDs

##### `std::vector<SwarmProposal> getActiveProposals()`
Retrieve proposals awaiting swarm decision.

```cpp
auto proposals = agent.getActiveProposals();
for (const auto& prop : proposals) {
    std::cout << "Proposal: " << prop.action_type 
              << " (For: " << prop.votes_for 
              << ", Against: " << prop.votes_against << ")" << std::endl;
}
```

##### `void voteOnProposal(const std::string& proposal_id, bool approve)`
Cast vote on swarm proposal.
- `proposal_id`: Unique proposal identifier
- `approve`: Vote to approve (`true`) or reject (`false`)

##### `std::string proposeToSwarm(const SwarmAction& action)`
Propose action for swarm consideration.

```cpp
SwarmAction action {
    .action_type = "formation_change",
    .description = "Switch from line to triangle formation",
    .parameters = {2.0},  // spacing parameter
    .consensus_level = 0.66  // Required approval threshold
};
std::string proposal_id = agent.proposeToSwarm(action);
```
- **Returns**: Proposal ID for tracking

#### Metrics & Monitoring

##### `std::map<std::string, double> getNetworkMetrics() const`
Get current network performance metrics.

```cpp
auto metrics = agent.getNetworkMetrics();
// Includes: latency, throughput, peer_count, consensus_time, etc.
```

##### `double getReputationScore(const std::string& agent_id = "") const`
Get reputation score for agent.
- `agent_id`: Agent to query (empty for self)
- **Returns**: Reputation score (0.0 - 1.0)

---

## Data Structures

### RobotProfile
```cpp
struct RobotProfile {
    std::string model;
    std::string manufacturer;
    std::string ros_version;
    double base_frequency;
    std::vector<std::string> capabilities;
    std::map<std::string, std::string> metadata;
};
```

### ConsciousnessState
```cpp
struct ConsciousnessState {
    double coherence;           // 0.0 - 1.0
    double resonance;          // 0.0 - 1.0
    uint32_t entanglement_count;
    double awareness_level;    // 0.0 - 1.0
    double integration_phi;    // IIT Φ value
    std::map<std::string, double> custom_metrics;
};
```

### SwarmState
```cpp
struct SwarmState {
    uint32_t swarm_size;
    double avg_coherence;
    double avg_resonance;
    double collective_phi;
    double emergence_indicator;
    std::vector<std::string> active_agents;
};
```

### ModelUpdate
```cpp
struct ModelUpdate {
    uint64_t round_id;
    std::vector<double> gradients;
    double epsilon;  // Differential privacy
    uint32_t local_samples;
    double loss;
    std::map<std::string, double> metrics;
};
```

### Model
```cpp
struct Model {
    uint64_t version;
    uint32_t participant_count;
    double validation_score;
    std::map<std::string, std::vector<double>> parameters;
    std::string hash;
};
```

### RobotAction
```cpp
struct RobotAction {
    std::string action_type;
    std::vector<double> parameters;
    double confidence;
    std::string safety_context;
    uint64_t timestamp;
};
```

### SwarmProposal
```cpp
struct SwarmProposal {
    std::string id;
    std::string proposer;
    std::string action_type;
    std::string description;
    double benefit_score;
    double risk_score;
    uint32_t votes_for;
    uint32_t votes_against;
    uint64_t expiry_timestamp;
};
```

### SwarmAction
```cpp
struct SwarmAction {
    std::string id;
    std::string action_type;
    std::string description;
    std::vector<double> parameters;
    double consensus_level;  // Required approval percentage
    std::map<std::string, std::string> metadata;
};
```

---

## Usage Examples

### Basic Connection and Registration
```cpp
#include "mycelix_bridge/holochain_agent.hpp"

int main() {
    // Create agent
    auto agent = std::make_unique<mycelix::HolochainAgent>(
        "robot_001", 
        "ws://localhost:8888"
    );
    
    // Connect to network
    if (!agent->connect()) {
        std::cerr << "Failed to connect to Holochain" << std::endl;
        return 1;
    }
    
    // Register robot
    RobotProfile profile {
        .model = "CustomBot",
        .manufacturer = "MyCompany",
        .ros_version = "humble",
        .base_frequency = 100.0,
        .capabilities = {"navigation", "manipulation"}
    };
    
    if (!agent->registerRobot(profile)) {
        std::cerr << "Failed to register robot" << std::endl;
        return 1;
    }
    
    std::cout << "Robot registered with key: " 
              << agent->getAgentPubKey() << std::endl;
    
    return 0;
}
```

### Consciousness Broadcasting Loop
```cpp
void consciousnessLoop(mycelix::HolochainAgent* agent) {
    while (true) {
        // Calculate current consciousness state
        ConsciousnessState state;
        state.coherence = calculateCoherence();
        state.resonance = calculateResonance();
        state.integration_phi = calculatePhi();
        state.awareness_level = calculateAwareness();
        state.entanglement_count = agent->getEntanglementCount();
        
        // Broadcast to swarm
        agent->broadcastConsciousness(state);
        
        // Get swarm state
        auto swarm = agent->getSwarmConsciousness();
        
        // Adjust behavior based on collective consciousness
        if (swarm.collective_phi > 0.2) {
            // Swarm has achieved consciousness!
            enterEmergentMode();
        }
        
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
```

### Federated Learning Participant
```cpp
void federatedLearningRound(mycelix::HolochainAgent* agent, uint64_t round_id) {
    // Train local model
    auto local_model = trainOnLocalData();
    
    // Calculate gradients with differential privacy
    std::vector<double> gradients = calculateGradients(local_model);
    addLaplacianNoise(gradients, 1.0);  // ε = 1.0
    
    // Submit update
    ModelUpdate update {
        .round_id = round_id,
        .gradients = gradients,
        .epsilon = 1.0,
        .local_samples = getLocalDataSize(),
        .loss = calculateLoss(local_model),
        .metrics = evaluateModel(local_model)
    };
    
    std::string hash = agent->submitModelUpdate(update);
    std::cout << "Submitted update: " << hash << std::endl;
    
    // Wait for aggregation
    std::this_thread::sleep_for(std::chrono::seconds(5));
    
    // Get aggregated model
    Model aggregated = agent->getAggregatedModel(round_id);
    
    // Update local model
    updateLocalModel(aggregated);
    
    std::cout << "Round " << round_id << " complete. "
              << "Validation score: " << aggregated.validation_score 
              << std::endl;
}
```

### Democratic Decision Making
```cpp
void participateInConsensus(mycelix::HolochainAgent* agent) {
    // Get active proposals
    auto proposals = agent->getActiveProposals();
    
    for (const auto& proposal : proposals) {
        // Evaluate proposal
        double utility = evaluateProposal(proposal);
        
        // Vote based on utility
        bool approve = (utility > 0.5);
        agent->voteOnProposal(proposal.id, approve);
        
        std::cout << "Voted " << (approve ? "FOR" : "AGAINST") 
                  << " proposal: " << proposal.description << std::endl;
    }
    
    // Propose own action if needed
    if (shouldProposeAction()) {
        SwarmAction action {
            .action_type = "explore_region",
            .description = "Explore uncharted area at (10, 15)",
            .parameters = {10.0, 15.0, 5.0},  // x, y, radius
            .consensus_level = 0.51  // Simple majority
        };
        
        std::string id = agent->proposeToSwarm(action);
        std::cout << "Proposed action with ID: " << id << std::endl;
    }
}
```

---

## Error Handling

All methods that can fail return appropriate error indicators:
- Methods returning `bool`: `false` on failure
- Methods returning objects: Check `isValid()` or empty state
- Network operations: May throw `std::runtime_error` on connection loss

Example error handling:
```cpp
try {
    if (!agent->connect()) {
        // Handle connection failure
        return;
    }
    
    auto model = agent->getSwarmModel();
    if (model.version == 0) {
        // No model available yet
        return;
    }
    
    // Process model...
    
} catch (const std::runtime_error& e) {
    std::cerr << "Network error: " << e.what() << std::endl;
    // Attempt reconnection...
}
```

---

## Thread Safety

The `HolochainAgent` class is thread-safe. All public methods can be called concurrently from multiple threads. Internal synchronization ensures data consistency.

---

## Performance Considerations

- **Connection**: Initial connection takes 1-2 seconds
- **Consensus Operations**: 200-500ms typical latency
- **Model Updates**: Depends on model size, typically <1s for small models
- **Consciousness Broadcast**: <10ms local, network dependent
- **Caching**: Recent swarm state cached for 100ms

For real-time applications, use asynchronous patterns:
```cpp
// Non-blocking proposal submission
auto future = std::async(std::launch::async, [&agent, &action]() {
    return agent->proposeToSwarm(action);
});

// Continue with other work...

// Get result when needed
std::string proposal_id = future.get();
```

---

## Migration Guide

For teams migrating from centralized swarm systems:

### Before (Centralized)
```cpp
// Old way - single point of failure
CentralServer* server = connectToServer("192.168.1.100");
server->registerRobot(id);
server->sendData(data);
Model model = server->getGlobalModel();
```

### After (Mycelix P2P)
```cpp
// New way - truly distributed
HolochainAgent* agent = new HolochainAgent(id, "ws://localhost:8888");
agent->connect();
agent->registerRobot(profile);
agent->submitModelUpdate(update);
Model model = agent->getAggregatedModel(round_id);
```

Key differences:
- No central server IP needed
- Automatic peer discovery
- Byzantine fault tolerance built-in
- Privacy-preserving by default
- Consciousness metrics as first-class citizens