/**
 * Federated Learning Implementation for Mycelix
 * Privacy-preserving collective intelligence for robots
 */

#include "mycelix_bridge/federated_learner.hpp"
#include <random>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <sstream>
#include <iomanip>

namespace mycelix {

// Implementation class for Pimpl idiom
class FederatedLearner::Impl {
public:
    Impl(double learning_rate, double privacy_epsilon)
        : learning_rate_(learning_rate), 
          privacy_epsilon_(privacy_epsilon),
          random_gen_(std::random_device{}()),
          model_version_(0),
          uncertainty_threshold_(0.3) {
    }
    
    // Initialize a simple neural network model
    void initializeModel(const std::map<std::string, int>& layer_sizes) {
        model_.clear();
        layer_sizes_ = layer_sizes;
        
        // Initialize weights randomly using Xavier initialization
        auto it = layer_sizes.begin();
        std::string prev_layer = it->first;
        int prev_size = it->second;
        ++it;
        
        while (it != layer_sizes.end()) {
            std::string curr_layer = it->first;
            int curr_size = it->second;
            
            // Initialize weight matrix between layers
            std::string weight_key = prev_layer + "_to_" + curr_layer;
            model_[weight_key] = initializeWeights(prev_size, curr_size);
            
            // Initialize bias for current layer
            std::string bias_key = curr_layer + "_bias";
            model_[bias_key] = std::vector<double>(curr_size, 0.0);
            
            prev_layer = curr_layer;
            prev_size = curr_size;
            ++it;
        }
        
        // Initialize momentum and adaptive learning rate parameters
        for (const auto& [key, params] : model_) {
            momentum_[key] = std::vector<double>(params.size(), 0.0);
            adaptive_lr_[key] = std::vector<double>(params.size(), learning_rate_);
        }
        
        std::cout << "Model initialized with " << countParameters() << " parameters" << std::endl;
    }
    
    // Make a decision based on current sensor input
    Action decide(const std::vector<double>& features) {
        // Forward pass through the network
        std::vector<double> activation = features;
        activations_.clear();
        activations_["input"] = features;
        
        // Process through each layer
        for (const auto& [layer_name, layer_size] : layer_sizes_) {
            if (layer_name == "input_layer") continue;
            
            activation = forwardLayer(activation, layer_name);
            activations_[layer_name] = activation;
        }
        
        // Convert output to action
        return outputToAction(activation);
    }
    
    // Make decision with uncertainty quantification
    ActionWithUncertainty decideWithUncertainty(const std::vector<double>& features) {
        // Use dropout for uncertainty estimation
        const int num_samples = 10;
        std::vector<std::vector<double>> predictions;
        
        for (int i = 0; i < num_samples; i++) {
            auto activation = forwardWithDropout(features, 0.1);
            predictions.push_back(activation);
        }
        
        // Calculate mean and variance
        std::vector<double> mean(predictions[0].size(), 0.0);
        std::vector<double> variance(predictions[0].size(), 0.0);
        
        for (const auto& pred : predictions) {
            for (size_t i = 0; i < pred.size(); i++) {
                mean[i] += pred[i] / num_samples;
            }
        }
        
        for (const auto& pred : predictions) {
            for (size_t i = 0; i < pred.size(); i++) {
                double diff = pred[i] - mean[i];
                variance[i] += diff * diff / num_samples;
            }
        }
        
        ActionWithUncertainty result;
        result.action = outputToAction(mean);
        result.confidence = 1.0 - std::sqrt(*std::max_element(variance.begin(), variance.end()));
        result.uncertainty = *std::max_element(variance.begin(), variance.end());
        result.action_probabilities = mean;
        
        return result;
    }
    
    // Get action probabilities
    std::vector<double> getActionProbabilities(const std::vector<double>& features) {
        auto activation = features;
        for (const auto& [layer_name, layer_size] : layer_sizes_) {
            if (layer_name == "input_layer") continue;
            activation = forwardLayer(activation, layer_name);
        }
        return softmax(activation);
    }
    
    // Train on single experience
    void trainStep(const Experience& experience) {
        // Forward pass
        auto output = getActionProbabilities(experience.sensor_data);
        
        // Calculate loss (cross-entropy)
        double loss = 0.0;
        std::vector<double> gradient(output.size());
        
        // Create target from reward
        std::vector<double> target(output.size(), 0.0);
        int action_idx = actionToIndex(experience.action_taken);
        target[action_idx] = experience.reward > 0 ? 1.0 : 0.0;
        
        // Compute gradient
        for (size_t i = 0; i < output.size(); i++) {
            gradient[i] = output[i] - target[i];
            loss -= target[i] * std::log(output[i] + 1e-10);
        }
        
        // Backpropagation
        backpropagate(gradient);
        
        // Update weights
        updateWeights(learning_rate_);
    }
    
    // Train on batch
    void trainBatch(const std::vector<Experience>& batch) {
        // Accumulate gradients
        std::map<std::string, std::vector<double>> accumulated_gradients;
        
        for (const auto& exp : batch) {
            trainStep(exp);
        }
        
        // Average and apply
        for (auto& [key, grads] : accumulated_gradients) {
            for (auto& g : grads) {
                g /= batch.size();
            }
        }
    }
    
    // Compute gradients from local experiences
    std::vector<double> computeGradients(const std::vector<Experience>& experiences) {
        std::vector<double> total_gradients;
        
        // Simplified gradient computation
        // In production, use automatic differentiation
        for (const auto& exp : experiences) {
            auto gradients = computeExperienceGradient(exp);
            
            if (total_gradients.empty()) {
                total_gradients = gradients;
            } else {
                // Accumulate gradients
                for (size_t i = 0; i < gradients.size(); i++) {
                    total_gradients[i] += gradients[i];
                }
            }
        }
        
        // Average gradients
        double scale = 1.0 / experiences.size();
        for (auto& grad : total_gradients) {
            grad *= scale;
        }
        
        // Apply gradient clipping for stability
        clipGradients(total_gradients, 1.0);
        
        return total_gradients;
    }
    
    // Add differential privacy noise to gradients
    std::vector<double> addPrivacyNoise(
        const std::vector<double>& gradients, 
        double epsilon) {
        
        std::vector<double> noisy_gradients = gradients;
        
        // Laplacian noise for differential privacy
        double sensitivity = 1.0; // L1 sensitivity
        double scale = sensitivity / epsilon;
        
        std::exponential_distribution<> exp_dist(1.0 / scale);
        std::uniform_real_distribution<> uniform_dist(0, 1);
        
        for (auto& grad : noisy_gradients) {
            // Generate Laplacian noise
            double u = uniform_dist(random_gen_) - 0.5;
            double sign = (u < 0) ? -1 : 1;
            double noise = sign * exp_dist(random_gen_);
            
            grad += noise;
        }
        
        return noisy_gradients;
    }
    
    // Update model with aggregated gradients from swarm
    void updateModel(const Model& swarm_model) {
        // Apply federated averaging
        for (const auto& [param_name, param_value] : swarm_model.parameters) {
            if (model_.find(param_name) != model_.end()) {
                // Weighted average between local and swarm model
                double local_weight = 0.3; // Keep 30% local
                double swarm_weight = 0.7; // Take 70% from swarm
                
                for (size_t i = 0; i < model_[param_name].size(); i++) {
                    model_[param_name][i] = 
                        local_weight * model_[param_name][i] + 
                        swarm_weight * param_value[i];
                }
            }
        }
        
        model_version_++;
        std::cout << "Model updated to version " << model_version_ << std::endl;
    }
    
    // Get current model parameters
    Model getModel() const {
        Model m;
        m.parameters = model_;
        m.version = model_version_;
        m.validation_score = calculateValidationScore();
        m.participant_count = 1;
        m.metadata["learning_rate"] = learning_rate_;
        m.metadata["privacy_epsilon"] = privacy_epsilon_;
        return m;
    }
    
    // Serialize model to string
    std::string serializeModel() const {
        std::stringstream ss;
        ss << "MODEL_V1\n";
        ss << model_version_ << "\n";
        
        // Write layer sizes
        ss << layer_sizes_.size() << "\n";
        for (const auto& [name, size] : layer_sizes_) {
            ss << name << " " << size << "\n";
        }
        
        // Write parameters
        ss << model_.size() << "\n";
        for (const auto& [key, params] : model_) {
            ss << key << " " << params.size() << "\n";
            for (double val : params) {
                ss << std::fixed << std::setprecision(8) << val << " ";
            }
            ss << "\n";
        }
        
        return ss.str();
    }
    
    // Deserialize model from string
    void deserializeModel(const std::string& serialized) {
        std::istringstream ss(serialized);
        std::string header;
        ss >> header;
        
        if (header != "MODEL_V1") {
            throw std::runtime_error("Invalid model format");
        }
        
        ss >> model_version_;
        
        // Read layer sizes
        size_t num_layers;
        ss >> num_layers;
        layer_sizes_.clear();
        for (size_t i = 0; i < num_layers; i++) {
            std::string name;
            int size;
            ss >> name >> size;
            layer_sizes_[name] = size;
        }
        
        // Read parameters
        size_t num_params;
        ss >> num_params;
        model_.clear();
        for (size_t i = 0; i < num_params; i++) {
            std::string key;
            size_t param_size;
            ss >> key >> param_size;
            
            std::vector<double> params(param_size);
            for (size_t j = 0; j < param_size; j++) {
                ss >> params[j];
            }
            model_[key] = params;
        }
    }
    
    // Evaluate model performance
    double evaluateModel(const std::vector<Experience>& test_data) const {
        if (test_data.empty()) return 0.0;
        
        double correct = 0.0;
        for (const auto& exp : test_data) {
            auto probs = const_cast<Impl*>(this)->getActionProbabilities(exp.sensor_data);
            int predicted = std::distance(probs.begin(), 
                std::max_element(probs.begin(), probs.end()));
            int actual = actionToIndex(exp.action_taken);
            
            if (predicted == actual) {
                correct += 1.0;
            }
        }
        
        return correct / test_data.size();
    }
    
    // Set learning parameters
    void setLearningRate(double rate) {
        learning_rate_ = rate;
    }
    
    double getLearningRate() const {
        return learning_rate_;
    }
    
    void setPrivacyEpsilon(double epsilon) {
        privacy_epsilon_ = epsilon;
    }
    
    double getPrivacyEpsilon() const {
        return privacy_epsilon_;
    }
    
    void setUncertaintyThreshold(double threshold) {
        uncertainty_threshold_ = threshold;
    }
    
    size_t countParameters() const {
        size_t count = 0;
        for (const auto& [_, params] : model_) {
            count += params.size();
        }
        return count;
    }
    
private:
    // Initialize weight matrix with Xavier initialization
    std::vector<double> initializeWeights(int input_size, int output_size) {
        std::vector<double> weights;
        weights.reserve(input_size * output_size);
        
        double xavier_std = std::sqrt(2.0 / (input_size + output_size));
        std::normal_distribution<> dist(0, xavier_std);
        
        for (int i = 0; i < input_size * output_size; i++) {
            weights.push_back(dist(random_gen_));
        }
        
        return weights;
    }
    
    // Forward pass through one layer
    std::vector<double> forwardLayer(
        const std::vector<double>& input, 
        const std::string& layer_name) {
        
        // Find weight matrix and bias
        std::string weight_key = findWeightKey(layer_name);
        std::string bias_key = layer_name + "_bias";
        
        const auto& weights = model_[weight_key];
        const auto& bias = model_[bias_key];
        
        std::vector<double> output(bias.size(), 0.0);
        
        // Matrix multiplication
        int input_size = input.size();
        int output_size = bias.size();
        
        for (int i = 0; i < output_size; i++) {
            for (int j = 0; j < input_size; j++) {
                output[i] += input[j] * weights[j * output_size + i];
            }
            output[i] += bias[i];
            
            // Apply ReLU activation (except last layer)
            if (layer_name != "output_layer") {
                output[i] = std::max(0.0, output[i]);
            }
        }
        
        // Softmax for output layer
        if (layer_name == "output_layer") {
            output = softmax(output);
        }
        
        return output;
    }
    
    // Find weight matrix key for a layer
    std::string findWeightKey(const std::string& layer_name) {
        for (const auto& [key, _] : model_) {
            if (key.find("_to_" + layer_name) != std::string::npos) {
                return key;
            }
        }
        return "";
    }
    
    // Softmax activation
    std::vector<double> softmax(const std::vector<double>& input) {
        std::vector<double> output = input;
        
        // Subtract max for numerical stability
        double max_val = *std::max_element(input.begin(), input.end());
        for (auto& val : output) {
            val = std::exp(val - max_val);
        }
        
        // Normalize
        double sum = std::accumulate(output.begin(), output.end(), 0.0);
        for (auto& val : output) {
            val /= sum;
        }
        
        return output;
    }
    
    // Convert network output to action
    Action outputToAction(const std::vector<double>& output) {
        // Find action with highest probability
        auto max_it = std::max_element(output.begin(), output.end());
        int action_idx = std::distance(output.begin(), max_it);
        
        Action action;
        switch (action_idx) {
            case 0:
                action.type = ActionType::MOVE_FORWARD;
                action.velocity = 0.5;
                break;
            case 1:
                action.type = ActionType::MOVE_BACKWARD;
                action.velocity = 0.3;
                break;
            case 2:
                action.type = ActionType::TURN_LEFT;
                action.angular_velocity = 0.5;
                break;
            case 3:
                action.type = ActionType::TURN_RIGHT;
                action.angular_velocity = 0.5;
                break;
            default:
                action.type = ActionType::STOP;
                break;
        }
        
        action.confidence = *max_it;
        return action;
    }
    
    // Forward pass with dropout
    std::vector<double> forwardWithDropout(const std::vector<double>& features, double dropout_rate) {
        std::vector<double> activation = features;
        std::uniform_real_distribution<> dropout_dist(0.0, 1.0);
        
        for (const auto& [layer_name, layer_size] : layer_sizes_) {
            if (layer_name == "input_layer") continue;
            
            // Apply dropout mask
            std::vector<double> masked_activation = activation;
            for (auto& val : masked_activation) {
                if (dropout_dist(random_gen_) < dropout_rate) {
                    val = 0.0;
                }
            }
            
            activation = forwardLayer(masked_activation, layer_name);
        }
        
        return activation;
    }
    
    // Backpropagation algorithm
    void backpropagate(const std::vector<double>& output_gradient) {
        gradients_.clear();
        std::vector<double> delta = output_gradient;
        
        // Backpropagate through layers in reverse
        auto layers = std::vector<std::pair<std::string, int>>(
            layer_sizes_.begin(), layer_sizes_.end());
        
        for (auto it = layers.rbegin(); it != layers.rend() - 1; ++it) {
            if (it->first == "input_layer") continue;
            
            std::string weight_key = findWeightKey(it->first);
            std::string bias_key = it->first + "_bias";
            
            // Store gradients
            gradients_[bias_key] = delta;
            
            // Compute weight gradients
            auto prev_activation = getPreviousActivation(it->first);
            std::vector<double> weight_grad;
            for (const auto& prev_val : prev_activation) {
                for (const auto& delta_val : delta) {
                    weight_grad.push_back(prev_val * delta_val);
                }
            }
            gradients_[weight_key] = weight_grad;
            
            // Propagate error back
            delta = computePreviousDelta(delta, weight_key);
        }
    }
    
    // Update weights using gradients
    void updateWeights(double learning_rate) {
        for (const auto& [key, grad] : gradients_) {
            if (model_.find(key) != model_.end()) {
                auto& params = model_[key];
                auto& momentum = momentum_[key];
                auto& adaptive = adaptive_lr_[key];
                
                // Adam optimizer
                const double beta1 = 0.9;
                const double beta2 = 0.999;
                const double epsilon = 1e-8;
                
                for (size_t i = 0; i < params.size(); i++) {
                    // Update momentum
                    momentum[i] = beta1 * momentum[i] + (1 - beta1) * grad[i];
                    
                    // Update adaptive learning rate
                    adaptive[i] = beta2 * adaptive[i] + (1 - beta2) * grad[i] * grad[i];
                    
                    // Apply update
                    double m_hat = momentum[i] / (1 - std::pow(beta1, model_version_ + 1));
                    double v_hat = adaptive[i] / (1 - std::pow(beta2, model_version_ + 1));
                    
                    params[i] -= learning_rate * m_hat / (std::sqrt(v_hat) + epsilon);
                }
            }
        }
    }
    
    // Helper to get previous layer activation
    std::vector<double> getPreviousActivation(const std::string& layer_name) {
        // Find the previous layer
        auto it = std::find_if(layer_sizes_.begin(), layer_sizes_.end(),
            [&](const auto& pair) { return pair.first == layer_name; });
        
        if (it != layer_sizes_.begin()) {
            --it;
            if (activations_.find(it->first) != activations_.end()) {
                return activations_[it->first];
            }
        }
        
        return std::vector<double>();
    }
    
    // Compute delta for previous layer
    std::vector<double> computePreviousDelta(const std::vector<double>& delta, 
                                            const std::string& weight_key) {
        const auto& weights = model_[weight_key];
        int prev_size = weights.size() / delta.size();
        std::vector<double> prev_delta(prev_size, 0.0);
        
        for (int i = 0; i < prev_size; i++) {
            for (size_t j = 0; j < delta.size(); j++) {
                prev_delta[i] += delta[j] * weights[i * delta.size() + j];
            }
            // Apply ReLU derivative
            if (prev_delta[i] < 0) {
                prev_delta[i] = 0;
            }
        }
        
        return prev_delta;
    }
    
    // Convert action to index
    int actionToIndex(const Action& action) const {
        switch (action.type) {
            case ActionType::MOVE_FORWARD: return 0;
            case ActionType::MOVE_BACKWARD: return 1;
            case ActionType::TURN_LEFT: return 2;
            case ActionType::TURN_RIGHT: return 3;
            default: return 4;
        }
    }
    
    // Compute gradient for single experience
    std::vector<double> computeExperienceGradient(const Experience& exp) {
        // Perform forward pass
        auto output = getActionProbabilities(exp.sensor_data);
        
        // Create target
        std::vector<double> target(output.size(), 0.0);
        int action_idx = actionToIndex(exp.action_taken);
        target[action_idx] = exp.reward > 0 ? 1.0 : 0.0;
        
        // Compute output gradient
        std::vector<double> output_grad(output.size());
        for (size_t i = 0; i < output.size(); i++) {
            output_grad[i] = output[i] - target[i];
        }
        
        // Backpropagate
        backpropagate(output_grad);
        
        // Flatten gradients
        std::vector<double> flat_gradients;
        for (const auto& [key, grad] : gradients_) {
            flat_gradients.insert(flat_gradients.end(), grad.begin(), grad.end());
        }
        
        return flat_gradients;
    }
    
    // Clip gradients to prevent explosion
    void clipGradients(std::vector<double>& gradients, double max_norm) {
        // Calculate L2 norm
        double norm = 0.0;
        for (const auto& grad : gradients) {
            norm += grad * grad;
        }
        norm = std::sqrt(norm);
        
        // Clip if necessary
        if (norm > max_norm) {
            double scale = max_norm / norm;
            for (auto& grad : gradients) {
                grad *= scale;
            }
        }
    }
    
    // Calculate validation score for model
    double calculateValidationScore() const {
        // Placeholder - in production, use validation set
        return 0.85 + 0.1 * std::sin(model_version_ * 0.1);
    }
    
private:
    double learning_rate_;
    double privacy_epsilon_;
    std::mt19937 random_gen_;
    
    std::map<std::string, int> layer_sizes_;
    std::map<std::string, std::vector<double>> model_;
    std::map<std::string, std::vector<double>> momentum_;
    std::map<std::string, std::vector<double>> adaptive_lr_;
    std::map<std::string, std::vector<double>> gradients_;
    std::map<std::string, std::vector<double>> activations_;
    int model_version_ = 0;
    double uncertainty_threshold_;
};

// Additional helper implementations

class AdvancedFederatedLearner : public FederatedLearner {
public:
    AdvancedFederatedLearner(double learning_rate, double privacy_epsilon)
        : FederatedLearner(learning_rate, privacy_epsilon) {
    }
    
    // Federated Proximal optimization (FedProx)
    void updateModelFedProx(
        const Model& swarm_model, 
        double proximal_term) {
        
        // FedProx adds a proximal term to handle heterogeneous data
        for (const auto& [param_name, swarm_params] : swarm_model.parameters) {
            if (model_.find(param_name) != model_.end()) {
                auto& local_params = model_[param_name];
                
                for (size_t i = 0; i < local_params.size(); i++) {
                    // Gradient with proximal term
                    double gradient = swarm_params[i] - local_params[i];
                    local_params[i] += learning_rate_ * 
                        (gradient - proximal_term * (local_params[i] - swarm_params[i]));
                }
            }
        }
    }
    
    // Personalized federated learning
    Model personalizeModel(const Model& global_model, double personalization_weight) {
        Model personalized = global_model;
        
        // Blend global and local models
        for (auto& [param_name, params] : personalized.parameters) {
            if (model_.find(param_name) != model_.end()) {
                for (size_t i = 0; i < params.size(); i++) {
                    params[i] = personalization_weight * model_[param_name][i] +
                               (1 - personalization_weight) * params[i];
                }
            }
        }
        
        return personalized;
    }
    
    // Byzantine-resistant aggregation
    std::vector<double> byzantineRobustAggregation(
        const std::vector<std::vector<double>>& all_gradients) {
        
        // Krum algorithm for Byzantine resistance
        int n = all_gradients.size();
        int f = n / 4; // Assume up to 25% malicious
        int k = n - f - 2;
        
        std::vector<double> scores(n, 0.0);
        
        // Calculate scores for each gradient
        for (int i = 0; i < n; i++) {
            std::vector<double> distances;
            for (int j = 0; j < n; j++) {
                if (i != j) {
                    double dist = l2Distance(all_gradients[i], all_gradients[j]);
                    distances.push_back(dist);
                }
            }
            
            // Sort and take k closest
            std::sort(distances.begin(), distances.end());
            scores[i] = std::accumulate(
                distances.begin(), 
                distances.begin() + k, 
                0.0);
        }
        
        // Select gradient with minimum score
        auto min_it = std::min_element(scores.begin(), scores.end());
        int best_idx = std::distance(scores.begin(), min_it);
        
        return all_gradients[best_idx];
    }
    
private:
    double l2Distance(const std::vector<double>& a, const std::vector<double>& b) {
        double dist = 0.0;
        for (size_t i = 0; i < a.size(); i++) {
            double diff = a[i] - b[i];
            dist += diff * diff;
        }
        return std::sqrt(dist);
    }
};

// Hierarchical federated learning for large-scale deployments
class HierarchicalFederatedLearner {
public:
    HierarchicalFederatedLearner() {
        // Setup tier structure
        setupTiers();
    }
    
    void setupTiers() {
        // Tier 1: Edge devices (individual robots)
        // Tier 2: Local aggregators (per area)
        // Tier 3: Regional aggregators
        // Tier 4: Global model
        
        tiers_ = {
            {1, "edge", 10},      // 10ms aggregation
            {2, "local", 100},    // 100ms aggregation
            {3, "regional", 1000}, // 1s aggregation
            {4, "global", 10000}   // 10s aggregation
        };
    }
    
    Model hierarchicalAggregation(
        const std::map<int, std::vector<Model>>& tier_models) {
        
        Model aggregated;
        
        // Start from edge tier and aggregate upwards
        for (const auto& [tier_id, tier_name, _] : tiers_) {
            if (tier_models.find(tier_id) != tier_models.end()) {
                const auto& models = tier_models.at(tier_id);
                
                // Aggregate models at this tier
                aggregated = aggregateModels(models);
                
                // Pass to next tier
                if (tier_id < 4) {
                    // Continue aggregation
                }
            }
        }
        
        return aggregated;
    }
    
private:
    Model aggregateModels(const std::vector<Model>& models) {
        Model aggregated;
        
        if (models.empty()) return aggregated;
        
        // Initialize with first model structure
        aggregated = models[0];
        
        // Average all parameters
        for (auto& [param_name, params] : aggregated.parameters) {
            // Zero out for averaging
            std::fill(params.begin(), params.end(), 0.0);
            
            // Sum all models
            for (const auto& model : models) {
                if (model.parameters.find(param_name) != model.parameters.end()) {
                    const auto& model_params = model.parameters.at(param_name);
                    for (size_t i = 0; i < params.size(); i++) {
                        params[i] += model_params[i];
                    }
                }
            }
            
            // Average
            double scale = 1.0 / models.size();
            for (auto& param : params) {
                param *= scale;
            }
        }
        
        return aggregated;
    }
    
    struct Tier {
        int id;
        std::string name;
        int aggregation_interval_ms;
    };
    
    std::vector<Tier> tiers_;
};

// FederatedLearner main class implementation (Pimpl wrapper)
FederatedLearner::FederatedLearner(double learning_rate, double privacy_epsilon)
    : impl_(std::make_unique<Impl>(learning_rate, privacy_epsilon)) {}

FederatedLearner::~FederatedLearner() = default;

void FederatedLearner::initializeModel(const std::map<std::string, int>& layer_sizes) {
    impl_->initializeModel(layer_sizes);
}

Action FederatedLearner::decide(const std::vector<double>& features) {
    return impl_->decide(features);
}

ActionWithUncertainty FederatedLearner::decideWithUncertainty(const std::vector<double>& features) {
    return impl_->decideWithUncertainty(features);
}

std::vector<double> FederatedLearner::getActionProbabilities(const std::vector<double>& features) {
    return impl_->getActionProbabilities(features);
}

void FederatedLearner::trainStep(const Experience& experience) {
    impl_->trainStep(experience);
}

void FederatedLearner::trainBatch(const std::vector<Experience>& batch) {
    impl_->trainBatch(batch);
}

std::vector<double> FederatedLearner::computeGradients(const std::vector<Experience>& experiences) {
    return impl_->computeGradients(experiences);
}

std::vector<double> FederatedLearner::addPrivacyNoise(
    const std::vector<double>& gradients, 
    double epsilon) {
    return impl_->addPrivacyNoise(gradients, epsilon);
}

void FederatedLearner::updateModel(const Model& swarm_model) {
    impl_->updateModel(swarm_model);
}

Model FederatedLearner::getModel() const {
    return impl_->getModel();
}

std::string FederatedLearner::serializeModel() const {
    return impl_->serializeModel();
}

void FederatedLearner::deserializeModel(const std::string& serialized) {
    impl_->deserializeModel(serialized);
}

double FederatedLearner::evaluateModel(const std::vector<Experience>& test_data) const {
    return impl_->evaluateModel(test_data);
}

void FederatedLearner::setLearningRate(double rate) {
    impl_->setLearningRate(rate);
}

double FederatedLearner::getLearningRate() const {
    return impl_->getLearningRate();
}

void FederatedLearner::setPrivacyEpsilon(double epsilon) {
    impl_->setPrivacyEpsilon(epsilon);
}

double FederatedLearner::getPrivacyEpsilon() const {
    return impl_->getPrivacyEpsilon();
}

void FederatedLearner::setUncertaintyThreshold(double threshold) {
    impl_->setUncertaintyThreshold(threshold);
}

size_t FederatedLearner::countParameters() const {
    return impl_->countParameters();
}

} // namespace mycelix