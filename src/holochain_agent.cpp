/**
 * Holochain Agent Implementation
 * Manages connection and communication with Holochain network
 */

#include "mycelix_bridge/holochain_agent.hpp"
#include <websocketpp/config/asio_client.hpp>
#include <websocketpp/client.hpp>
#include <json/json.h>
#include <thread>
#include <mutex>
#include <queue>
#include <atomic>

namespace mycelix {

typedef websocketpp::client<websocketpp::config::asio_tls_client> client;
typedef websocketpp::config::asio_client::message_type::ptr message_ptr;

class HolochainAgent::Impl {
public:
    Impl(const std::string& agent_id, const std::string& conductor_url)
        : agent_id_(agent_id), conductor_url_(conductor_url), connected_(false) {
        
        // Initialize WebSocket client
        ws_client_.init_asio();
        ws_client_.clear_access_channels(websocketpp::log::alevel::all);
        ws_client_.set_access_channels(websocketpp::log::alevel::connect |
                                       websocketpp::log::alevel::disconnect |
                                       websocketpp::log::alevel::app);
        
        // Set up message handler
        ws_client_.set_message_handler(
            [this](websocketpp::connection_hdl hdl, message_ptr msg) {
                handleMessage(msg->get_payload());
            });
        
        // Set up connection handler
        ws_client_.set_open_handler(
            [this](websocketpp::connection_hdl hdl) {
                connected_ = true;
                if (connection_callback_) {
                    connection_callback_(true);
                }
            });
        
        // Set up disconnection handler
        ws_client_.set_close_handler(
            [this](websocketpp::connection_hdl hdl) {
                connected_ = false;
                if (connection_callback_) {
                    connection_callback_(false);
                }
            });
        
        // Start WebSocket thread
        ws_thread_ = std::thread([this]() {
            ws_client_.run();
        });
    }
    
    ~Impl() {
        disconnect();
        if (ws_thread_.joinable()) {
            ws_thread_.join();
        }
    }
    
    // Connection Management
    bool connect() {
        if (connected_) {
            return true;
        }
        
        try {
            websocketpp::lib::error_code ec;
            auto con = ws_client_.get_connection(conductor_url_, ec);
            
            if (ec) {
                return false;
            }
            
            ws_client_.connect(con);
            connection_ = con->get_handle();
            
            // Wait for connection
            std::this_thread::sleep_for(std::chrono::seconds(2));
            return connected_;
            
        } catch (const std::exception& e) {
            return false;
        }
    }
    
    void disconnect() {
        if (connected_ && connection_.lock()) {
            websocketpp::lib::error_code ec;
            ws_client_.close(connection_, websocketpp::close::status::normal, "", ec);
            connected_ = false;
        }
    }
    
    bool isConnected() const {
        return connected_;
    }
    
    void setConnectionCallback(std::function<void(bool)> callback) {
        connection_callback_ = callback;
    }
    
    // Registration & Identity
    bool registerRobot(const RobotProfile& profile) {
        if (!connected_) {
            return false;
        }
        
        Json::Value request;
        request["type"] = "register_robot";
        request["agent_id"] = agent_id_;
        request["profile"]["model"] = profile.model;
        request["profile"]["manufacturer"] = profile.manufacturer;
        request["profile"]["ros_version"] = profile.ros_version;
        request["profile"]["base_frequency"] = profile.base_frequency;
        
        for (const auto& cap : profile.capabilities) {
            request["profile"]["capabilities"].append(cap);
        }
        
        for (const auto& [key, value] : profile.metadata) {
            request["profile"]["metadata"][key] = value;
        }
        
        return sendRequest(request);
    }
    
    void updateProfile(const RobotProfile& profile) {
        Json::Value request;
        request["type"] = "update_profile";
        request["agent_id"] = agent_id_;
        request["profile"]["model"] = profile.model;
        request["profile"]["manufacturer"] = profile.manufacturer;
        
        sendRequest(request);
    }
    
    std::string getAgentPubKey() const {
        return agent_pubkey_;
    }
    
    // Federated Learning
    std::string submitModelUpdate(const ModelUpdate& update) {
        Json::Value request;
        request["type"] = "submit_model_update";
        request["agent_id"] = agent_id_;
        request["round_id"] = static_cast<Json::UInt64>(update.round_id);
        request["epsilon"] = update.epsilon;
        request["local_samples"] = update.local_samples;
        request["loss"] = update.loss;
        
        for (const auto& grad : update.gradients) {
            request["gradients"].append(grad);
        }
        
        for (const auto& [key, value] : update.metrics) {
            request["metrics"][key] = value;
        }
        
        sendRequest(request);
        
        // Generate submission hash
        return generateHash(request.toStyledString());
    }
    
    Model getAggregatedModel(uint64_t round_id) {
        Json::Value request;
        request["type"] = "get_aggregated_model";
        request["round_id"] = static_cast<Json::UInt64>(round_id);
        
        sendRequest(request);
        
        // Wait for response
        std::unique_lock<std::mutex> lock(model_mutex_);
        model_cv_.wait_for(lock, std::chrono::seconds(5), 
            [this]() { return latest_model_.version > 0; });
        
        return latest_model_;
    }
    
    Model getSwarmModel() {
        Json::Value request;
        request["type"] = "get_swarm_model";
        
        sendRequest(request);
        
        // Wait for response
        std::unique_lock<std::mutex> lock(model_mutex_);
        model_cv_.wait_for(lock, std::chrono::seconds(2));
        
        return latest_model_;
    }
    
    void publishGradients(const std::vector<double>& gradients) {
        Json::Value request;
        request["type"] = "publish_gradients";
        request["agent_id"] = agent_id_;
        
        for (const auto& grad : gradients) {
            request["gradients"].append(grad);
        }
        
        sendRequest(request);
    }
    
    // Action Validation
    bool validateAction(const RobotAction& action) {
        Json::Value request;
        request["type"] = "validate_action";
        request["agent_id"] = agent_id_;
        request["action"]["type"] = action.action_type;
        request["action"]["confidence"] = action.confidence;
        request["action"]["safety_context"] = action.safety_context;
        
        for (const auto& param : action.parameters) {
            request["action"]["parameters"].append(param);
        }
        
        sendRequest(request);
        
        // Wait for validation response
        std::unique_lock<std::mutex> lock(validation_mutex_);
        validation_cv_.wait_for(lock, std::chrono::milliseconds(500));
        
        return last_validation_result_.approved;
    }
    
    std::string submitForValidation(const RobotAction& action) {
        Json::Value request;
        request["type"] = "submit_for_validation";
        request["agent_id"] = agent_id_;
        request["action"]["type"] = action.action_type;
        request["action"]["confidence"] = action.confidence;
        
        sendRequest(request);
        
        return generateHash(request.toStyledString());
    }
    
    // Consciousness Synchronization
    void broadcastConsciousness(const ConsciousnessState& state) {
        Json::Value request;
        request["type"] = "broadcast_consciousness";
        request["agent_id"] = agent_id_;
        request["state"]["coherence"] = state.coherence;
        request["state"]["resonance"] = state.resonance;
        request["state"]["entanglement_count"] = state.entanglement_count;
        request["state"]["awareness_level"] = state.awareness_level;
        request["state"]["integration_phi"] = state.integration_phi;
        
        for (const auto& [key, value] : state.custom_metrics) {
            request["state"]["custom_metrics"][key] = value;
        }
        
        sendRequest(request);
    }
    
    SwarmState getSwarmConsciousness() {
        Json::Value request;
        request["type"] = "get_swarm_consciousness";
        
        sendRequest(request);
        
        // Wait for response
        std::unique_lock<std::mutex> lock(swarm_mutex_);
        swarm_cv_.wait_for(lock, std::chrono::seconds(1));
        
        return latest_swarm_state_;
    }
    
    uint32_t getEntanglementCount() const {
        return entanglement_count_;
    }
    
    // Swarm Coordination
    std::vector<std::string> getSwarmNeighbors(size_t max_count) {
        Json::Value request;
        request["type"] = "get_swarm_neighbors";
        request["max_count"] = static_cast<Json::UInt>(max_count);
        
        sendRequest(request);
        
        // Wait for response
        std::unique_lock<std::mutex> lock(neighbors_mutex_);
        neighbors_cv_.wait_for(lock, std::chrono::milliseconds(500));
        
        return swarm_neighbors_;
    }
    
    std::vector<SwarmProposal> getActiveProposals() {
        Json::Value request;
        request["type"] = "get_active_proposals";
        
        sendRequest(request);
        
        // Wait for response
        std::unique_lock<std::mutex> lock(proposals_mutex_);
        proposals_cv_.wait_for(lock, std::chrono::milliseconds(500));
        
        return active_proposals_;
    }
    
    void voteOnProposal(const std::string& proposal_id, bool approve) {
        Json::Value request;
        request["type"] = "vote_on_proposal";
        request["proposal_id"] = proposal_id;
        request["approve"] = approve;
        request["agent_id"] = agent_id_;
        
        sendRequest(request);
    }
    
    std::vector<SwarmAction> getApprovedActions() {
        Json::Value request;
        request["type"] = "get_approved_actions";
        
        sendRequest(request);
        
        // Wait for response
        std::unique_lock<std::mutex> lock(actions_mutex_);
        actions_cv_.wait_for(lock, std::chrono::milliseconds(500));
        
        return approved_actions_;
    }
    
    std::string proposeToSwarm(const SwarmAction& action) {
        Json::Value request;
        request["type"] = "propose_to_swarm";
        request["agent_id"] = agent_id_;
        request["action"]["type"] = action.action_type;
        request["action"]["description"] = action.description;
        request["action"]["consensus_level"] = action.consensus_level;
        
        for (const auto& param : action.parameters) {
            request["action"]["parameters"].append(param);
        }
        
        sendRequest(request);
        
        return generateHash(request.toStyledString());
    }
    
    // Metrics & Monitoring
    std::map<std::string, double> getNetworkMetrics() const {
        return network_metrics_;
    }
    
    double getReputationScore(const std::string& agent_id) const {
        if (agent_id.empty() || agent_id == agent_id_) {
            return reputation_score_;
        }
        
        // Query network for other agent's reputation
        auto it = peer_reputations_.find(agent_id);
        if (it != peer_reputations_.end()) {
            return it->second;
        }
        
        return 0.5; // Default neutral reputation
    }
    
private:
    void handleMessage(const std::string& payload) {
        try {
            Json::CharReaderBuilder builder;
            Json::Value response;
            std::istringstream stream(payload);
            std::string errors;
            
            if (!Json::parseFromStream(builder, stream, &response, &errors)) {
                return;
            }
            
            std::string type = response["type"].asString();
            
            if (type == "model_update") {
                handleModelUpdate(response);
            } else if (type == "swarm_state") {
                handleSwarmState(response);
            } else if (type == "validation_result") {
                handleValidationResult(response);
            } else if (type == "neighbors_list") {
                handleNeighborsList(response);
            } else if (type == "proposals_list") {
                handleProposalsList(response);
            } else if (type == "approved_actions") {
                handleApprovedActions(response);
            } else if (type == "metrics_update") {
                handleMetricsUpdate(response);
            }
        } catch (const std::exception& e) {
            // Log error
        }
    }
    
    void handleModelUpdate(const Json::Value& data) {
        std::lock_guard<std::mutex> lock(model_mutex_);
        
        latest_model_.version = data["version"].asUInt64();
        latest_model_.participant_count = data["participant_count"].asUInt();
        latest_model_.validation_score = data["validation_score"].asDouble();
        
        for (const auto& member : data["parameters"].getMemberNames()) {
            std::vector<double> params;
            for (const auto& val : data["parameters"][member]) {
                params.push_back(val.asDouble());
            }
            latest_model_.parameters[member] = params;
        }
        
        model_cv_.notify_all();
    }
    
    void handleSwarmState(const Json::Value& data) {
        std::lock_guard<std::mutex> lock(swarm_mutex_);
        
        latest_swarm_state_.swarm_size = data["swarm_size"].asUInt();
        latest_swarm_state_.avg_coherence = data["avg_coherence"].asDouble();
        latest_swarm_state_.avg_resonance = data["avg_resonance"].asDouble();
        latest_swarm_state_.collective_phi = data["collective_phi"].asDouble();
        latest_swarm_state_.emergence_indicator = data["emergence_indicator"].asDouble();
        
        swarm_cv_.notify_all();
    }
    
    void handleValidationResult(const Json::Value& data) {
        std::lock_guard<std::mutex> lock(validation_mutex_);
        
        last_validation_result_.approved = data["approved"].asBool();
        last_validation_result_.reason = data["reason"].asString();
        last_validation_result_.confidence = data["confidence"].asDouble();
        
        validation_cv_.notify_all();
    }
    
    void handleNeighborsList(const Json::Value& data) {
        std::lock_guard<std::mutex> lock(neighbors_mutex_);
        
        swarm_neighbors_.clear();
        for (const auto& neighbor : data["neighbors"]) {
            swarm_neighbors_.push_back(neighbor.asString());
        }
        
        neighbors_cv_.notify_all();
    }
    
    void handleProposalsList(const Json::Value& data) {
        std::lock_guard<std::mutex> lock(proposals_mutex_);
        
        active_proposals_.clear();
        for (const auto& prop : data["proposals"]) {
            SwarmProposal proposal;
            proposal.id = prop["id"].asString();
            proposal.proposer = prop["proposer"].asString();
            proposal.action_type = prop["action_type"].asString();
            proposal.benefit_score = prop["benefit_score"].asDouble();
            proposal.risk_score = prop["risk_score"].asDouble();
            proposal.votes_for = prop["votes_for"].asUInt();
            proposal.votes_against = prop["votes_against"].asUInt();
            
            active_proposals_.push_back(proposal);
        }
        
        proposals_cv_.notify_all();
    }
    
    void handleApprovedActions(const Json::Value& data) {
        std::lock_guard<std::mutex> lock(actions_mutex_);
        
        approved_actions_.clear();
        for (const auto& act : data["actions"]) {
            SwarmAction action;
            action.id = act["id"].asString();
            action.action_type = act["action_type"].asString();
            action.description = act["description"].asString();
            action.consensus_level = act["consensus_level"].asDouble();
            
            for (const auto& param : act["parameters"]) {
                action.parameters.push_back(param.asDouble());
            }
            
            approved_actions_.push_back(action);
        }
        
        actions_cv_.notify_all();
    }
    
    void handleMetricsUpdate(const Json::Value& data) {
        std::lock_guard<std::mutex> lock(metrics_mutex_);
        
        for (const auto& member : data["metrics"].getMemberNames()) {
            network_metrics_[member] = data["metrics"][member].asDouble();
        }
        
        if (data.isMember("reputation_score")) {
            reputation_score_ = data["reputation_score"].asDouble();
        }
        
        if (data.isMember("entanglement_count")) {
            entanglement_count_ = data["entanglement_count"].asUInt();
        }
    }
    
    bool sendRequest(const Json::Value& request) {
        if (!connected_ || !connection_.lock()) {
            return false;
        }
        
        Json::StreamWriterBuilder builder;
        std::string payload = Json::writeString(builder, request);
        
        websocketpp::lib::error_code ec;
        ws_client_.send(connection_, payload, websocketpp::frame::opcode::text, ec);
        
        return !ec;
    }
    
    std::string generateHash(const std::string& data) {
        // Simple hash generation (in production, use proper crypto)
        std::hash<std::string> hasher;
        return std::to_string(hasher(data));
    }
    
    // Member variables
    std::string agent_id_;
    std::string conductor_url_;
    std::string agent_pubkey_;
    std::atomic<bool> connected_;
    
    // WebSocket
    client ws_client_;
    websocketpp::connection_hdl connection_;
    std::thread ws_thread_;
    
    // Callbacks
    std::function<void(bool)> connection_callback_;
    
    // State
    Model latest_model_;
    SwarmState latest_swarm_state_;
    ValidationResult last_validation_result_;
    std::vector<std::string> swarm_neighbors_;
    std::vector<SwarmProposal> active_proposals_;
    std::vector<SwarmAction> approved_actions_;
    std::map<std::string, double> network_metrics_;
    std::map<std::string, double> peer_reputations_;
    double reputation_score_ = 0.5;
    uint32_t entanglement_count_ = 0;
    
    // Synchronization
    mutable std::mutex model_mutex_;
    mutable std::condition_variable model_cv_;
    mutable std::mutex swarm_mutex_;
    mutable std::condition_variable swarm_cv_;
    mutable std::mutex validation_mutex_;
    mutable std::condition_variable validation_cv_;
    mutable std::mutex neighbors_mutex_;
    mutable std::condition_variable neighbors_cv_;
    mutable std::mutex proposals_mutex_;
    mutable std::condition_variable proposals_cv_;
    mutable std::mutex actions_mutex_;
    mutable std::condition_variable actions_cv_;
    mutable std::mutex metrics_mutex_;
};

// HolochainAgent implementation
HolochainAgent::HolochainAgent(const std::string& agent_id, 
                               const std::string& conductor_url)
    : impl_(std::make_unique<Impl>(agent_id, conductor_url)) {}

HolochainAgent::~HolochainAgent() = default;

bool HolochainAgent::registerRobot(const RobotProfile& profile) {
    return impl_->registerRobot(profile);
}

void HolochainAgent::updateProfile(const RobotProfile& profile) {
    impl_->updateProfile(profile);
}

std::string HolochainAgent::getAgentPubKey() const {
    return impl_->getAgentPubKey();
}

std::string HolochainAgent::submitModelUpdate(const ModelUpdate& update) {
    return impl_->submitModelUpdate(update);
}

Model HolochainAgent::getAggregatedModel(uint64_t round_id) {
    return impl_->getAggregatedModel(round_id);
}

Model HolochainAgent::getSwarmModel() {
    return impl_->getSwarmModel();
}

void HolochainAgent::publishGradients(const std::vector<double>& gradients) {
    impl_->publishGradients(gradients);
}

bool HolochainAgent::validateAction(const RobotAction& action) {
    return impl_->validateAction(action);
}

std::string HolochainAgent::submitForValidation(const RobotAction& action) {
    return impl_->submitForValidation(action);
}

void HolochainAgent::broadcastConsciousness(const ConsciousnessState& state) {
    impl_->broadcastConsciousness(state);
}

SwarmState HolochainAgent::getSwarmConsciousness() {
    return impl_->getSwarmConsciousness();
}

uint32_t HolochainAgent::getEntanglementCount() const {
    return impl_->getEntanglementCount();
}

std::vector<std::string> HolochainAgent::getSwarmNeighbors(size_t max_count) {
    return impl_->getSwarmNeighbors(max_count);
}

std::vector<SwarmProposal> HolochainAgent::getActiveProposals() {
    return impl_->getActiveProposals();
}

void HolochainAgent::voteOnProposal(const std::string& proposal_id, bool approve) {
    impl_->voteOnProposal(proposal_id, approve);
}

std::vector<SwarmAction> HolochainAgent::getApprovedActions() {
    return impl_->getApprovedActions();
}

std::string HolochainAgent::proposeToSwarm(const SwarmAction& action) {
    return impl_->proposeToSwarm(action);
}

bool HolochainAgent::connect() {
    return impl_->connect();
}

void HolochainAgent::disconnect() {
    impl_->disconnect();
}

bool HolochainAgent::isConnected() const {
    return impl_->isConnected();
}

void HolochainAgent::setConnectionCallback(std::function<void(bool)> callback) {
    impl_->setConnectionCallback(callback);
}

std::map<std::string, double> HolochainAgent::getNetworkMetrics() const {
    return impl_->getNetworkMetrics();
}

double HolochainAgent::getReputationScore(const std::string& agent_id) const {
    return impl_->getReputationScore(agent_id);
}

} // namespace mycelix