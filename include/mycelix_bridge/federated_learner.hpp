/**
 * Federated Learning Interface for Mycelix
 * Privacy-preserving collective intelligence for robots
 */

#ifndef MYCELIX_BRIDGE_FEDERATED_LEARNER_HPP
#define MYCELIX_BRIDGE_FEDERATED_LEARNER_HPP

#include <vector>
#include <map>
#include <string>
#include <memory>
#include <chrono>

namespace mycelix {

// Forward declarations
struct Model;
struct Experience;
struct Action;

/**
 * Action types for robot control
 */
enum class ActionType {
    STOP,
    MOVE_FORWARD,
    MOVE_BACKWARD,
    TURN_LEFT,
    TURN_RIGHT,
    CUSTOM
};

/**
 * Robot action with metadata
 */
struct Action {
    ActionType type;
    double velocity = 0.0;
    double angular_velocity = 0.0;
    double confidence = 0.0;
    std::vector<double> custom_params;
};

/**
 * Experience for learning
 */
struct Experience {
    std::chrono::system_clock::time_point timestamp;
    std::vector<double> sensor_data;
    Action action_taken;
    double reward;
    std::vector<double> next_state;
    bool terminal;
    std::map<std::string, double> metadata;
};

/**
 * Position in 3D space
 */
struct Position {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

/**
 * Velocity in 3D space
 */
struct Velocity {
    double linear_x = 0.0;
    double linear_y = 0.0;
    double angular_z = 0.0;
};

/**
 * Orientation quaternion
 */
struct Orientation {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
};

/**
 * Safe action for emergency situations
 */
struct SafeAction {
    static Action STOP;
    static Action SLOW_FORWARD;
    static Action EMERGENCY_STOP;
};

/**
 * Main federated learning interface
 * Handles local training and privacy-preserving gradient sharing
 */
class FederatedLearner {
public:
    /**
     * Constructor
     * @param learning_rate Learning rate for gradient descent
     * @param privacy_epsilon Differential privacy budget
     */
    FederatedLearner(double learning_rate = 0.01, 
                     double privacy_epsilon = 1.0);
    
    virtual ~FederatedLearner();
    
    // === Model Management ===
    
    /**
     * Initialize neural network model
     * @param layer_sizes Map of layer names to sizes
     */
    void initializeModel(const std::map<std::string, int>& layer_sizes);
    
    /**
     * Load pre-trained model
     * @param model_path Path to model file
     * @return Success status
     */
    bool loadModel(const std::string& model_path);
    
    /**
     * Save current model
     * @param model_path Path to save model
     * @return Success status
     */
    bool saveModel(const std::string& model_path);
    
    /**
     * Get current model
     * @return Current model parameters
     */
    Model getModel() const;
    
    /**
     * Update model with swarm knowledge
     * @param swarm_model Aggregated model from swarm
     */
    void updateModel(const Model& swarm_model);
    
    // === Decision Making ===
    
    /**
     * Make decision based on sensor input
     * @param features Sensor features
     * @return Recommended action
     */
    Action decide(const std::vector<double>& features);
    
    /**
     * Make decision with uncertainty
     * @param features Sensor features
     * @param temperature Exploration temperature
     * @return Action with confidence
     */
    Action decideWithUncertainty(const std::vector<double>& features, 
                                 double temperature = 1.0);
    
    /**
     * Get action probabilities
     * @param features Sensor features
     * @return Probability distribution over actions
     */
    std::vector<double> getActionProbabilities(const std::vector<double>& features);
    
    // === Training ===
    
    /**
     * Compute gradients from experiences
     * @param experiences Buffer of experiences
     * @return Gradient vector
     */
    std::vector<double> computeGradients(const std::vector<Experience>& experiences);
    
    /**
     * Train on single experience
     * @param experience Single experience
     * @return Loss value
     */
    double trainStep(const Experience& experience);
    
    /**
     * Train on batch of experiences
     * @param experiences Batch of experiences
     * @param epochs Number of training epochs
     * @return Average loss
     */
    double trainBatch(const std::vector<Experience>& experiences, 
                      int epochs = 1);
    
    // === Privacy Mechanisms ===
    
    /**
     * Add differential privacy noise to gradients
     * @param gradients Raw gradients
     * @param epsilon Privacy budget
     * @return Private gradients
     */
    std::vector<double> addPrivacyNoise(const std::vector<double>& gradients,
                                        double epsilon);
    
    /**
     * Clip gradients for privacy
     * @param gradients Raw gradients
     * @param max_norm Maximum L2 norm
     * @return Clipped gradients
     */
    std::vector<double> clipGradients(const std::vector<double>& gradients,
                                      double max_norm = 1.0);
    
    /**
     * Compute privacy budget used
     * @return Current privacy expenditure
     */
    double getPrivacyBudgetUsed() const;
    
    // === Federated Learning ===
    
    /**
     * Prepare model update for sharing
     * @return Model update with privacy guarantees
     */
    ModelUpdate prepareModelUpdate();
    
    /**
     * Apply federated averaging
     * @param local_model Local model
     * @param global_model Global model
     * @param alpha Interpolation factor
     * @return Averaged model
     */
    Model federatedAveraging(const Model& local_model,
                            const Model& global_model,
                            double alpha = 0.5);
    
    /**
     * Personalize global model
     * @param global_model Global model
     * @param personalization_weight Local weight (0-1)
     * @return Personalized model
     */
    Model personalizeModel(const Model& global_model,
                          double personalization_weight = 0.3);
    
    // === Metrics & Evaluation ===
    
    /**
     * Evaluate model performance
     * @param test_experiences Test set
     * @return Performance metrics
     */
    std::map<std::string, double> evaluate(const std::vector<Experience>& test_experiences);
    
    /**
     * Get training statistics
     * @return Map of statistic names to values
     */
    std::map<std::string, double> getTrainingStats() const;
    
    /**
     * Reset training statistics
     */
    void resetStats();
    
    // === Configuration ===
    
    /**
     * Set learning rate
     * @param rate New learning rate
     */
    void setLearningRate(double rate);
    
    /**
     * Set privacy epsilon
     * @param epsilon New privacy budget
     */
    void setPrivacyEpsilon(double epsilon);
    
    /**
     * Get learning rate
     * @return Current learning rate
     */
    double getLearningRate() const;
    
    /**
     * Get privacy epsilon
     * @return Current privacy budget
     */
    double getPrivacyEpsilon() const;
    
protected:
    // Protected members for derived classes
    double learning_rate_;
    double privacy_epsilon_;
    std::map<std::string, std::vector<double>> model_;
    int model_version_ = 0;
    
private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

/**
 * Advanced federated learner with additional algorithms
 */
class AdvancedFederatedLearner : public FederatedLearner {
public:
    AdvancedFederatedLearner(double learning_rate = 0.01,
                             double privacy_epsilon = 1.0);
    
    /**
     * FedProx update for heterogeneous data
     * @param swarm_model Global model
     * @param proximal_term Proximal regularization strength
     */
    void updateModelFedProx(const Model& swarm_model, 
                           double proximal_term = 0.1);
    
    /**
     * Byzantine-resistant aggregation
     * @param all_gradients Gradients from all agents
     * @return Robust aggregated gradient
     */
    std::vector<double> byzantineRobustAggregation(
        const std::vector<std::vector<double>>& all_gradients);
    
    /**
     * Asynchronous federated learning update
     * @param swarm_model Global model
     * @param staleness Age of global model
     */
    void asyncUpdate(const Model& swarm_model, int staleness);
    
    /**
     * Differential privacy with adaptive clipping
     * @param gradients Raw gradients
     * @return Adaptively clipped private gradients
     */
    std::vector<double> adaptiveClipping(const std::vector<double>& gradients);
};

/**
 * Hierarchical federated learner for large-scale deployments
 */
class HierarchicalFederatedLearner {
public:
    HierarchicalFederatedLearner();
    
    /**
     * Setup tier structure
     * @param num_tiers Number of hierarchical tiers
     */
    void setupTiers(int num_tiers = 4);
    
    /**
     * Hierarchical aggregation across tiers
     * @param tier_models Models from each tier
     * @return Hierarchically aggregated model
     */
    Model hierarchicalAggregation(
        const std::map<int, std::vector<Model>>& tier_models);
    
    /**
     * Get tier for this agent
     * @param agent_id Agent identifier
     * @return Tier level (1-4)
     */
    int getAgentTier(const std::string& agent_id) const;
    
    /**
     * Promote agent to higher tier
     * @param agent_id Agent to promote
     */
    void promoteAgent(const std::string& agent_id);
    
private:
    struct Tier;
    std::vector<Tier> tiers_;
    std::map<std::string, int> agent_tiers_;
};

} // namespace mycelix

#endif // MYCELIX_BRIDGE_FEDERATED_LEARNER_HPP