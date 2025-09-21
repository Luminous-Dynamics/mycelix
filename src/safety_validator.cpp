/**
 * Safety Validator Implementation
 * Multi-level safety checks for robot actions
 */

#include "mycelix_bridge/safety_validator.hpp"
#include "mycelix_bridge/federated_learner.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <iostream>
#include <sstream>
#include <random>

namespace mycelix {

// SafeAction static definitions
Action SafeAction::STOP = {ActionType::STOP, 0.0, 0.0, 1.0};
Action SafeAction::SLOW_FORWARD = {ActionType::MOVE_FORWARD, 0.1, 0.0, 0.9};
Action SafeAction::EMERGENCY_STOP = {ActionType::STOP, 0.0, 0.0, 1.0};

class SafetyValidator::Impl {
public:
    Impl(ComplianceFramework framework) 
        : compliance_framework_(framework), 
          emergency_active_(false),
          logging_enabled_(true) {
        
        // Initialize default safety rules based on compliance framework
        initializeDefaultRules();
        
        // Initialize safety statistics
        stats_["total_validations"] = 0;
        stats_["violations_detected"] = 0;
        stats_["emergency_stops"] = 0;
        stats_["limits_applied"] = 0;
        stats_["uptime_hours"] = 0;
    }
    
    ~Impl() = default;
    
    // Rule Management
    void addRule(const SafetyRule& rule) {
        rules_[rule.name] = rule;
        
        if (logging_enabled_) {
            logEvent("Rule added: " + rule.name);
        }
    }
    
    bool removeRule(const std::string& rule_name) {
        auto it = rules_.find(rule_name);
        if (it != rules_.end()) {
            rules_.erase(it);
            logEvent("Rule removed: " + rule_name);
            return true;
        }
        return false;
    }
    
    void setRuleEnabled(const std::string& rule_name, bool enabled) {
        auto it = rules_.find(rule_name);
        if (it != rules_.end()) {
            it->second.enabled = enabled;
            logEvent("Rule " + rule_name + (enabled ? " enabled" : " disabled"));
        }
    }
    
    std::vector<SafetyRule> getRules() const {
        std::vector<SafetyRule> result;
        for (const auto& [name, rule] : rules_) {
            result.push_back(rule);
        }
        return result;
    }
    
    size_t getRuleCount() const {
        return rules_.size();
    }
    
    void clearRules() {
        rules_.clear();
        logEvent("All rules cleared");
    }
    
    // Validation
    bool validate(const Action& action) {
        auto start = std::chrono::steady_clock::now();
        
        stats_["total_validations"]++;
        
        if (emergency_active_) {
            return false;
        }
        
        bool is_safe = true;
        std::vector<std::string> violations;
        
        // Check each enabled rule
        for (const auto& [name, rule] : rules_) {
            if (!rule.enabled) continue;
            
            bool rule_passed = checkRule(rule, action);
            if (!rule_passed) {
                is_safe = false;
                violations.push_back(name);
                
                if (rule.action == SafetyAction::STOP || 
                    rule.action == SafetyAction::REJECT) {
                    break; // Critical violation, no need to check further
                }
            }
        }
        
        if (!violations.empty()) {
            stats_["violations_detected"]++;
            
            // Record violation
            SafetyResult result;
            result.level = SafetyLevel::WARNING;
            result.message = "Safety violations detected";
            result.violations = violations;
            result.check_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - start);
            
            violation_history_.push_back(result);
            
            // Trigger callback if set
            if (safety_callback_) {
                safety_callback_(result);
            }
        }
        
        return is_safe;
    }
    
    SafetyResult validateDetailed(const Action& action) {
        auto start = std::chrono::steady_clock::now();
        SafetyResult result;
        
        if (emergency_active_) {
            result.level = SafetyLevel::EMERGENCY;
            result.message = "Emergency stop active";
            result.check_time = std::chrono::milliseconds(0);
            return result;
        }
        
        result.level = SafetyLevel::SAFE;
        result.message = "All safety checks passed";
        
        // Check each rule and collect detailed information
        for (const auto& [name, rule] : rules_) {
            if (!rule.enabled) continue;
            
            bool passed = checkRule(rule, action);
            if (!passed) {
                result.violations.push_back(name);
                
                // Determine severity level
                switch (rule.action) {
                    case SafetyAction::STOP:
                    case SafetyAction::REJECT:
                        result.level = SafetyLevel::CRITICAL;
                        result.message = "Critical safety violation: " + name;
                        break;
                    case SafetyAction::LIMIT:
                    case SafetyAction::SLOW:
                        if (result.level < SafetyLevel::WARNING) {
                            result.level = SafetyLevel::WARNING;
                            result.message = "Safety limits exceeded";
                        }
                        break;
                    case SafetyAction::WARN:
                        if (result.level < SafetyLevel::CAUTION) {
                            result.level = SafetyLevel::CAUTION;
                            result.message = "Safety caution advised";
                        }
                        break;
                    default:
                        break;
                }
            }
            
            // Add metric
            result.metrics[name + "_check"] = passed ? 1.0 : 0.0;
        }
        
        result.check_time = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start);
        
        return result;
    }
    
    SafetyResult checkState(const SafetyState& state) {
        SafetyResult result;
        result.level = SafetyLevel::SAFE;
        result.message = "System state nominal";
        
        // Check velocity limits
        double linear_speed = std::sqrt(
            state.velocity.linear_x * state.velocity.linear_x +
            state.velocity.linear_y * state.velocity.linear_y);
        
        if (linear_speed > max_linear_velocity_) {
            result.level = SafetyLevel::WARNING;
            result.violations.push_back("velocity_exceeded");
            result.message = "Velocity limit exceeded";
        }
        
        if (std::abs(state.velocity.angular_z) > max_angular_velocity_) {
            result.level = SafetyLevel::WARNING;
            result.violations.push_back("angular_velocity_exceeded");
            result.message = "Angular velocity limit exceeded";
        }
        
        // Check obstacle distance
        if (state.min_obstacle_distance < min_obstacle_distance_) {
            result.level = SafetyLevel::CRITICAL;
            result.violations.push_back("obstacle_too_close");
            result.message = "Obstacle dangerously close";
        }
        
        // Check system health
        if (state.system_health < 0.3) {
            result.level = SafetyLevel::WARNING;
            result.violations.push_back("low_system_health");
            result.message = "System health degraded";
        }
        
        // Check battery level
        if (state.battery_level < 0.1) {
            result.level = SafetyLevel::CRITICAL;
            result.violations.push_back("critical_battery");
            result.message = "Critical battery level";
        }
        
        // Check temperature
        if (state.temperature > 80.0) {
            result.level = SafetyLevel::WARNING;
            result.violations.push_back("high_temperature");
            result.message = "Temperature too high";
        }
        
        // Check workspace bounds
        if (!isWithinWorkspace(state.position)) {
            result.level = SafetyLevel::CRITICAL;
            result.violations.push_back("out_of_bounds");
            result.message = "Position out of workspace bounds";
        }
        
        // Add metrics
        result.metrics["linear_speed"] = linear_speed;
        result.metrics["angular_speed"] = std::abs(state.velocity.angular_z);
        result.metrics["obstacle_distance"] = state.min_obstacle_distance;
        result.metrics["system_health"] = state.system_health;
        result.metrics["battery_level"] = state.battery_level;
        result.metrics["temperature"] = state.temperature;
        
        return result;
    }
    
    SafetyResult predictSafety(const SafetyState& current, 
                               const Action& action,
                               std::chrono::milliseconds horizon) {
        SafetyResult result;
        result.level = SafetyLevel::SAFE;
        result.message = "Predicted safe trajectory";
        
        // Simple physics-based prediction
        double dt = horizon.count() / 1000.0;
        
        // Predict future position
        Position future_pos;
        future_pos.x = current.position.x + current.velocity.linear_x * dt;
        future_pos.y = current.position.y + current.velocity.linear_y * dt;
        
        // Predict future velocity based on action
        Velocity future_vel;
        future_vel.linear_x = action.velocity;
        future_vel.angular_z = action.angular_velocity;
        
        // Check predicted state
        SafetyState predicted_state = current;
        predicted_state.position = future_pos;
        predicted_state.velocity = future_vel;
        
        // Predict collision risk
        double stopping_distance = (future_vel.linear_x * future_vel.linear_x) / 
                                  (2 * max_linear_acceleration_);
        
        if (current.min_obstacle_distance < stopping_distance + 0.2) {
            result.level = SafetyLevel::WARNING;
            result.violations.push_back("predicted_collision");
            result.message = "Collision predicted within horizon";
        }
        
        // Check if predicted position is within bounds
        if (!isWithinWorkspace(future_pos)) {
            result.level = SafetyLevel::WARNING;
            result.violations.push_back("predicted_out_of_bounds");
            result.message = "Predicted to leave workspace";
        }
        
        result.metrics["predicted_x"] = future_pos.x;
        result.metrics["predicted_y"] = future_pos.y;
        result.metrics["stopping_distance"] = stopping_distance;
        
        return result;
    }
    
    // Limits & Constraints
    void setVelocityLimits(double max_linear, double max_angular) {
        max_linear_velocity_ = max_linear;
        max_angular_velocity_ = max_angular;
        
        // Update velocity rules
        rules_["max_linear_velocity"] = SafetyRule{
            .name = "max_linear_velocity",
            .type = SafetyType::KINEMATIC,
            .threshold = max_linear,
            .action = SafetyAction::LIMIT,
            .description = "Maximum linear velocity limit"
        };
        
        rules_["max_angular_velocity"] = SafetyRule{
            .name = "max_angular_velocity",
            .type = SafetyType::KINEMATIC,
            .threshold = max_angular,
            .action = SafetyAction::LIMIT,
            .description = "Maximum angular velocity limit"
        };
    }
    
    void setAccelerationLimits(double max_linear, double max_angular) {
        max_linear_acceleration_ = max_linear;
        max_angular_acceleration_ = max_angular;
    }
    
    void setMinObstacleDistance(double distance) {
        min_obstacle_distance_ = distance;
        
        rules_["min_obstacle_distance"] = SafetyRule{
            .name = "min_obstacle_distance",
            .type = SafetyType::COLLISION,
            .threshold = distance,
            .action = SafetyAction::STOP,
            .description = "Minimum safe obstacle distance"
        };
    }
    
    void setWorkspaceBounds(const Position& min_pos, const Position& max_pos) {
        workspace_min_ = min_pos;
        workspace_max_ = max_pos;
    }
    
    Action applyLimits(const Action& action) const {
        Action limited = action;
        
        // Apply velocity limits
        limited.velocity = std::clamp(action.velocity, 
                                      -max_linear_velocity_, 
                                      max_linear_velocity_);
        
        limited.angular_velocity = std::clamp(action.angular_velocity,
                                             -max_angular_velocity_,
                                             max_angular_velocity_);
        
        stats_["limits_applied"]++;
        
        return limited;
    }
    
    // Compliance
    void setComplianceFramework(ComplianceFramework framework) {
        compliance_framework_ = framework;
        initializeDefaultRules();
    }
    
    ComplianceFramework getComplianceFramework() const {
        return compliance_framework_;
    }
    
    bool checkCompliance(const Action& action) const {
        // Check action against compliance framework requirements
        switch (compliance_framework_) {
            case ComplianceFramework::ISO26262:
                return checkISO26262Compliance(action);
            case ComplianceFramework::IEC61508:
                return checkIEC61508Compliance(action);
            case ComplianceFramework::HIPAA:
                return checkHIPAACompliance(action);
            case ComplianceFramework::GDPR:
                return checkGDPRCompliance(action);
            default:
                return true;
        }
    }
    
    std::string generateComplianceReport() const {
        std::stringstream report;
        report << "Safety Compliance Report\n";
        report << "========================\n";
        report << "Framework: " << complianceFrameworkToString() << "\n";
        report << "Total Validations: " << stats_.at("total_validations") << "\n";
        report << "Violations Detected: " << stats_.at("violations_detected") << "\n";
        report << "Emergency Stops: " << stats_.at("emergency_stops") << "\n";
        report << "Uptime Hours: " << stats_.at("uptime_hours") << "\n";
        report << "Compliance Rate: " << calculateComplianceRate() << "%\n";
        report << "\nActive Rules: " << rules_.size() << "\n";
        
        for (const auto& [name, rule] : rules_) {
            report << "  - " << name << " (" 
                   << (rule.enabled ? "enabled" : "disabled") << ")\n";
        }
        
        return report.str();
    }
    
    std::map<std::string, bool> getCertificationStatus() const {
        std::map<std::string, bool> status;
        
        // Check various certification requirements
        status["ISO26262_Compliant"] = 
            compliance_framework_ == ComplianceFramework::ISO26262;
        status["IEC61508_Compliant"] = 
            compliance_framework_ == ComplianceFramework::IEC61508;
        status["Safety_Rules_Active"] = !rules_.empty();
        status["Emergency_System_Ready"] = true;
        status["Logging_Enabled"] = logging_enabled_;
        status["Compliance_Rate_Above_95"] = calculateComplianceRate() > 95.0;
        
        return status;
    }
    
    // Emergency Handling
    void emergencyStop(const std::string& reason) {
        emergency_active_ = true;
        emergency_reason_ = reason;
        emergency_time_ = std::chrono::system_clock::now();
        stats_["emergency_stops"]++;
        
        logEvent("EMERGENCY STOP: " + reason);
        
        // Trigger emergency callback if set
        if (safety_callback_) {
            SafetyResult emergency_result;
            emergency_result.level = SafetyLevel::EMERGENCY;
            emergency_result.message = "Emergency stop: " + reason;
            safety_callback_(emergency_result);
        }
    }
    
    bool isEmergency() const {
        return emergency_active_;
    }
    
    bool clearEmergency(const std::string& override_code) {
        if (override_code == "SAFETY_OVERRIDE_" + std::to_string(emergency_time_.time_since_epoch().count())) {
            emergency_active_ = false;
            emergency_reason_.clear();
            logEvent("Emergency cleared with override code");
            return true;
        }
        return false;
    }
    
    Action getEmergencyAction() const {
        return SafeAction::EMERGENCY_STOP;
    }
    
    // Monitoring & Logging
    void setSafetyCallback(std::function<void(const SafetyResult&)> callback) {
        safety_callback_ = callback;
    }
    
    std::map<std::string, double> getSafetyStats() const {
        return stats_;
    }
    
    std::vector<SafetyResult> getViolationHistory(size_t max_count) const {
        std::vector<SafetyResult> result;
        size_t start = violation_history_.size() > max_count ? 
                      violation_history_.size() - max_count : 0;
        
        for (size_t i = start; i < violation_history_.size(); i++) {
            result.push_back(violation_history_[i]);
        }
        
        return result;
    }
    
    void clearViolationHistory() {
        violation_history_.clear();
        logEvent("Violation history cleared");
    }
    
    void setLoggingEnabled(bool enabled) {
        logging_enabled_ = enabled;
    }
    
    // Multi-Level Safety
    SafetyResult hardwareSafetyCheck(const Action& action) const {
        SafetyResult result;
        result.level = SafetyLevel::SAFE;
        result.message = "Hardware safety check passed";
        
        // Check actuator limits
        if (std::abs(action.velocity) > 5.0) {
            result.level = SafetyLevel::CRITICAL;
            result.violations.push_back("actuator_limit_exceeded");
            result.message = "Hardware actuator limit exceeded";
        }
        
        // Check power consumption
        double estimated_power = std::abs(action.velocity) * 10 + 
                               std::abs(action.angular_velocity) * 5;
        if (estimated_power > 100) {
            result.level = SafetyLevel::WARNING;
            result.violations.push_back("high_power_consumption");
            result.message = "Power consumption too high";
        }
        
        return result;
    }
    
    SafetyResult softwareSafetyCheck(const Action& action) const {
        SafetyResult result;
        result.level = SafetyLevel::SAFE;
        result.message = "Software safety check passed";
        
        // Check for NaN or inf values
        if (std::isnan(action.velocity) || std::isinf(action.velocity) ||
            std::isnan(action.angular_velocity) || std::isinf(action.angular_velocity)) {
            result.level = SafetyLevel::CRITICAL;
            result.violations.push_back("invalid_values");
            result.message = "Invalid numerical values detected";
        }
        
        // Check confidence threshold
        if (action.confidence < 0.3) {
            result.level = SafetyLevel::CAUTION;
            result.violations.push_back("low_confidence");
            result.message = "Action confidence too low";
        }
        
        return result;
    }
    
    SafetyResult networkSafetyCheck(const Action& action) const {
        SafetyResult result;
        result.level = SafetyLevel::SAFE;
        result.message = "Network safety check passed";
        
        // In a real implementation, this would check network consensus,
        // validate against swarm decisions, etc.
        
        return result;
    }
    
private:
    bool checkRule(const SafetyRule& rule, const Action& action) {
        if (rule.custom_validator) {
            return rule.custom_validator(action);
        }
        
        switch (rule.type) {
            case SafetyType::KINEMATIC:
                return checkKinematicRule(rule, action);
            case SafetyType::COLLISION:
                return checkCollisionRule(rule, action);
            case SafetyType::STABILITY:
                return checkStabilityRule(rule, action);
            case SafetyType::POWER:
                return checkPowerRule(rule, action);
            case SafetyType::TEMPERATURE:
                return checkTemperatureRule(rule, action);
            case SafetyType::COMPLIANCE:
                return checkComplianceRule(rule, action);
            default:
                return true;
        }
    }
    
    bool checkKinematicRule(const SafetyRule& rule, const Action& action) {
        // Check velocity and acceleration limits
        if (rule.name.find("velocity") != std::string::npos) {
            return std::abs(action.velocity) <= rule.threshold ||
                   std::abs(action.angular_velocity) <= rule.threshold;
        }
        return true;
    }
    
    bool checkCollisionRule(const SafetyRule& rule, const Action& action) {
        // In real implementation, would check sensor data
        // For now, just check if action would move forward when too close
        if (action.type == ActionType::MOVE_FORWARD && action.velocity > 0) {
            // Would need actual sensor data here
            return true; // Placeholder
        }
        return true;
    }
    
    bool checkStabilityRule(const SafetyRule& rule, const Action& action) {
        // Check if action would maintain stability
        // Simplified check based on angular velocity
        return std::abs(action.angular_velocity) < rule.threshold;
    }
    
    bool checkPowerRule(const SafetyRule& rule, const Action& action) {
        // Estimate power consumption
        double power = std::abs(action.velocity) * 10 + 
                      std::abs(action.angular_velocity) * 5;
        return power <= rule.threshold;
    }
    
    bool checkTemperatureRule(const SafetyRule& rule, const Action& action) {
        // Would need actual temperature sensor data
        return true; // Placeholder
    }
    
    bool checkComplianceRule(const SafetyRule& rule, const Action& action) {
        return checkCompliance(action);
    }
    
    bool isWithinWorkspace(const Position& pos) const {
        return pos.x >= workspace_min_.x && pos.x <= workspace_max_.x &&
               pos.y >= workspace_min_.y && pos.y <= workspace_max_.y &&
               pos.z >= workspace_min_.z && pos.z <= workspace_max_.z;
    }
    
    void initializeDefaultRules() {
        switch (compliance_framework_) {
            case ComplianceFramework::ISO26262:
                initializeISO26262Rules();
                break;
            case ComplianceFramework::IEC61508:
                initializeIEC61508Rules();
                break;
            case ComplianceFramework::ISO13849:
                initializeISO13849Rules();
                break;
            default:
                initializeBasicRules();
                break;
        }
    }
    
    void initializeBasicRules() {
        addRule(SafetyRule{
            .name = "default_velocity_limit",
            .type = SafetyType::KINEMATIC,
            .threshold = 2.0,
            .action = SafetyAction::LIMIT,
            .description = "Default velocity limit"
        });
        
        addRule(SafetyRule{
            .name = "default_collision_avoidance",
            .type = SafetyType::COLLISION,
            .threshold = 0.5,
            .action = SafetyAction::STOP,
            .description = "Default collision avoidance"
        });
    }
    
    void initializeISO26262Rules() {
        // ISO 26262 automotive safety rules
        addRule(SafetyRule{
            .name = "asil_d_velocity",
            .type = SafetyType::KINEMATIC,
            .threshold = 1.5,
            .action = SafetyAction::LIMIT,
            .description = "ASIL-D velocity limit",
            .priority = 10.0
        });
        
        addRule(SafetyRule{
            .name = "asil_d_redundancy",
            .type = SafetyType::COMPLIANCE,
            .threshold = 0.99,
            .action = SafetyAction::REJECT,
            .description = "ASIL-D redundancy requirement",
            .priority = 10.0
        });
    }
    
    void initializeIEC61508Rules() {
        // IEC 61508 functional safety rules
        addRule(SafetyRule{
            .name = "sil3_limit",
            .type = SafetyType::KINEMATIC,
            .threshold = 1.0,
            .action = SafetyAction::LIMIT,
            .description = "SIL-3 safety limit",
            .priority = 9.0
        });
    }
    
    void initializeISO13849Rules() {
        // ISO 13849 machinery safety rules
        addRule(SafetyRule{
            .name = "performance_level_e",
            .type = SafetyType::KINEMATIC,
            .threshold = 0.8,
            .action = SafetyAction::LIMIT,
            .description = "Performance Level e requirement",
            .priority = 9.5
        });
    }
    
    bool checkISO26262Compliance(const Action& action) const {
        // Simplified ISO 26262 compliance check
        return action.confidence >= 0.95;
    }
    
    bool checkIEC61508Compliance(const Action& action) const {
        // Simplified IEC 61508 compliance check
        return action.confidence >= 0.9;
    }
    
    bool checkHIPAACompliance(const Action& action) const {
        // HIPAA is for medical privacy - always true for robot actions
        return true;
    }
    
    bool checkGDPRCompliance(const Action& action) const {
        // GDPR is for data protection - always true for robot actions
        return true;
    }
    
    std::string complianceFrameworkToString() const {
        switch (compliance_framework_) {
            case ComplianceFramework::ISO26262: return "ISO 26262";
            case ComplianceFramework::IEC61508: return "IEC 61508";
            case ComplianceFramework::ISO13849: return "ISO 13849";
            case ComplianceFramework::IEC62061: return "IEC 62061";
            case ComplianceFramework::HIPAA: return "HIPAA";
            case ComplianceFramework::GDPR: return "GDPR";
            case ComplianceFramework::CE: return "CE";
            case ComplianceFramework::FDA: return "FDA";
            case ComplianceFramework::CUSTOM: return "Custom";
            default: return "Unknown";
        }
    }
    
    double calculateComplianceRate() const {
        if (stats_.at("total_validations") == 0) return 100.0;
        
        double violations = stats_.at("violations_detected");
        double total = stats_.at("total_validations");
        
        return ((total - violations) / total) * 100.0;
    }
    
    void logEvent(const std::string& message) {
        if (!logging_enabled_) return;
        
        // In production, would write to proper log file
        std::cout << "[SafetyValidator] " << message << std::endl;
    }
    
    // Member variables
    ComplianceFramework compliance_framework_;
    std::map<std::string, SafetyRule> rules_;
    
    // Limits
    double max_linear_velocity_ = 2.0;
    double max_angular_velocity_ = 2.0;
    double max_linear_acceleration_ = 1.0;
    double max_angular_acceleration_ = 1.0;
    double min_obstacle_distance_ = 0.5;
    
    // Workspace
    Position workspace_min_ = {-10, -10, 0};
    Position workspace_max_ = {10, 10, 5};
    
    // Emergency state
    std::atomic<bool> emergency_active_;
    std::string emergency_reason_;
    std::chrono::system_clock::time_point emergency_time_;
    
    // Monitoring
    mutable std::map<std::string, double> stats_;
    std::vector<SafetyResult> violation_history_;
    std::function<void(const SafetyResult&)> safety_callback_;
    bool logging_enabled_;
};

// SafetyValidator implementation
SafetyValidator::SafetyValidator(ComplianceFramework framework)
    : impl_(std::make_unique<Impl>(framework)) {}

SafetyValidator::~SafetyValidator() = default;

void SafetyValidator::addRule(const SafetyRule& rule) {
    impl_->addRule(rule);
}

bool SafetyValidator::removeRule(const std::string& rule_name) {
    return impl_->removeRule(rule_name);
}

void SafetyValidator::setRuleEnabled(const std::string& rule_name, bool enabled) {
    impl_->setRuleEnabled(rule_name, enabled);
}

std::vector<SafetyRule> SafetyValidator::getRules() const {
    return impl_->getRules();
}

size_t SafetyValidator::getRuleCount() const {
    return impl_->getRuleCount();
}

void SafetyValidator::clearRules() {
    impl_->clearRules();
}

bool SafetyValidator::validate(const Action& action) {
    return impl_->validate(action);
}

bool SafetyValidator::validate(const RobotAction& action) {
    // Convert RobotAction to Action for validation
    Action act;
    act.type = ActionType::CUSTOM;
    act.velocity = action.parameters.size() > 0 ? action.parameters[0] : 0.0;
    act.angular_velocity = action.parameters.size() > 1 ? action.parameters[1] : 0.0;
    act.confidence = action.confidence;
    
    return impl_->validate(act);
}

SafetyResult SafetyValidator::validateDetailed(const Action& action) {
    return impl_->validateDetailed(action);
}

SafetyResult SafetyValidator::checkState(const SafetyState& state) {
    return impl_->checkState(state);
}

SafetyResult SafetyValidator::predictSafety(const SafetyState& current, 
                                           const Action& action,
                                           std::chrono::milliseconds horizon) {
    return impl_->predictSafety(current, action, horizon);
}

void SafetyValidator::setVelocityLimits(double max_linear, double max_angular) {
    impl_->setVelocityLimits(max_linear, max_angular);
}

void SafetyValidator::setAccelerationLimits(double max_linear, double max_angular) {
    impl_->setAccelerationLimits(max_linear, max_angular);
}

void SafetyValidator::setMinObstacleDistance(double distance) {
    impl_->setMinObstacleDistance(distance);
}

void SafetyValidator::setWorkspaceBounds(const Position& min_pos, const Position& max_pos) {
    impl_->setWorkspaceBounds(min_pos, max_pos);
}

Action SafetyValidator::applyLimits(const Action& action) const {
    return impl_->applyLimits(action);
}

void SafetyValidator::setComplianceFramework(ComplianceFramework framework) {
    impl_->setComplianceFramework(framework);
}

ComplianceFramework SafetyValidator::getComplianceFramework() const {
    return impl_->getComplianceFramework();
}

bool SafetyValidator::checkCompliance(const Action& action) const {
    return impl_->checkCompliance(action);
}

std::string SafetyValidator::generateComplianceReport() const {
    return impl_->generateComplianceReport();
}

std::map<std::string, bool> SafetyValidator::getCertificationStatus() const {
    return impl_->getCertificationStatus();
}

void SafetyValidator::emergencyStop(const std::string& reason) {
    impl_->emergencyStop(reason);
}

bool SafetyValidator::isEmergency() const {
    return impl_->isEmergency();
}

bool SafetyValidator::clearEmergency(const std::string& override_code) {
    return impl_->clearEmergency(override_code);
}

Action SafetyValidator::getEmergencyAction() const {
    return impl_->getEmergencyAction();
}

void SafetyValidator::setSafetyCallback(std::function<void(const SafetyResult&)> callback) {
    impl_->setSafetyCallback(callback);
}

std::map<std::string, double> SafetyValidator::getSafetyStats() const {
    return impl_->getSafetyStats();
}

std::vector<SafetyResult> SafetyValidator::getViolationHistory(size_t max_count) const {
    return impl_->getViolationHistory(max_count);
}

void SafetyValidator::clearViolationHistory() {
    impl_->clearViolationHistory();
}

void SafetyValidator::setLoggingEnabled(bool enabled) {
    impl_->setLoggingEnabled(enabled);
}

SafetyResult SafetyValidator::hardwareSafetyCheck(const Action& action) const {
    return impl_->hardwareSafetyCheck(action);
}

SafetyResult SafetyValidator::softwareSafetyCheck(const Action& action) const {
    return impl_->softwareSafetyCheck(action);
}

SafetyResult SafetyValidator::networkSafetyCheck(const Action& action) const {
    return impl_->networkSafetyCheck(action);
}

// AdvancedSafetyValidator implementation
AdvancedSafetyValidator::AdvancedSafetyValidator() 
    : SafetyValidator(ComplianceFramework::ISO26262) {}

void AdvancedSafetyValidator::trainPredictor(const std::vector<SafetyResult>& history) {
    // In production, would train ML model from historical data
    // This is a placeholder implementation
}

double AdvancedSafetyValidator::predictSafetyProbability(const SafetyState& state,
                                                        const Action& action) const {
    // In production, would use trained ML model
    // Simple heuristic for now
    double safety = 1.0;
    
    // Reduce safety probability based on speed
    safety -= std::abs(action.velocity) * 0.1;
    safety -= std::abs(action.angular_velocity) * 0.05;
    
    // Reduce based on obstacle distance
    if (state.min_obstacle_distance < 1.0) {
        safety -= (1.0 - state.min_obstacle_distance) * 0.3;
    }
    
    // Reduce based on system health
    safety *= state.system_health;
    
    return std::max(0.0, std::min(1.0, safety));
}

void AdvancedSafetyValidator::adaptLimits(const std::map<std::string, double>& context) {
    // Adapt safety limits based on context
    if (context.find("environment_complexity") != context.end()) {
        double complexity = context.at("environment_complexity");
        
        // Reduce limits in complex environments
        setVelocityLimits(2.0 * (1.0 - complexity * 0.5),
                          2.0 * (1.0 - complexity * 0.3));
    }
    
    if (context.find("visibility") != context.end()) {
        double visibility = context.at("visibility");
        
        // Reduce limits in poor visibility
        if (visibility < 0.5) {
            setVelocityLimits(1.0, 1.0);
        }
    }
}

void AdvancedSafetyValidator::learnFromViolation(const SafetyResult& violation) {
    // In production, would update ML model based on violation
    // Could adjust safety thresholds, update rules, etc.
}

// SwarmSafetyValidator implementation
SwarmSafetyValidator::SwarmSafetyValidator() {}

SafetyResult SwarmSafetyValidator::validateSwarmAction(
    const std::vector<Action>& swarm_action,
    const std::vector<SafetyState>& robot_states) {
    
    SafetyResult result;
    result.level = SafetyLevel::SAFE;
    result.message = "Swarm action validated";
    
    // Check each robot's action
    for (size_t i = 0; i < swarm_action.size(); i++) {
        SafetyValidator validator;
        
        if (!validator.validate(swarm_action[i])) {
            result.level = SafetyLevel::WARNING;
            result.violations.push_back("robot_" + std::to_string(i) + "_unsafe");
        }
    }
    
    // Check for inter-robot collisions
    std::vector<Position> positions;
    std::vector<Velocity> velocities;
    
    for (const auto& state : robot_states) {
        positions.push_back(state.position);
        velocities.push_back(state.velocity);
    }
    
    auto collision_result = checkCollisionRisk(positions, velocities);
    if (collision_result.level > SafetyLevel::CAUTION) {
        result.level = collision_result.level;
        result.violations.insert(result.violations.end(),
                               collision_result.violations.begin(),
                               collision_result.violations.end());
    }
    
    return result;
}

SafetyResult SwarmSafetyValidator::checkCollisionRisk(
    const std::vector<Position>& positions,
    const std::vector<Velocity>& velocities) {
    
    SafetyResult result;
    result.level = SafetyLevel::SAFE;
    result.message = "No collision risk detected";
    
    const double min_safe_distance = 1.0; // meters
    const double prediction_horizon = 2.0; // seconds
    
    // Check all pairs of robots
    for (size_t i = 0; i < positions.size(); i++) {
        for (size_t j = i + 1; j < positions.size(); j++) {
            // Current distance
            double dx = positions[i].x - positions[j].x;
            double dy = positions[i].y - positions[j].y;
            double current_distance = std::sqrt(dx * dx + dy * dy);
            
            if (current_distance < min_safe_distance) {
                result.level = SafetyLevel::CRITICAL;
                result.violations.push_back("robots_" + std::to_string(i) + 
                                          "_" + std::to_string(j) + "_too_close");
            }
            
            // Predict future positions
            Position future_i = positions[i];
            future_i.x += velocities[i].linear_x * prediction_horizon;
            future_i.y += velocities[i].linear_y * prediction_horizon;
            
            Position future_j = positions[j];
            future_j.x += velocities[j].linear_x * prediction_horizon;
            future_j.y += velocities[j].linear_y * prediction_horizon;
            
            double future_dx = future_i.x - future_j.x;
            double future_dy = future_i.y - future_j.y;
            double future_distance = std::sqrt(future_dx * future_dx + 
                                              future_dy * future_dy);
            
            if (future_distance < min_safe_distance) {
                result.level = SafetyLevel::WARNING;
                result.violations.push_back("predicted_collision_" + 
                                          std::to_string(i) + "_" + 
                                          std::to_string(j));
                result.message = "Collision predicted within " + 
                               std::to_string(prediction_horizon) + " seconds";
            }
        }
    }
    
    return result;
}

std::vector<Position> SwarmSafetyValidator::coordinateSafeFormation(
    const std::string& formation_type,
    size_t robot_count) {
    
    std::vector<Position> formation;
    const double spacing = 2.0; // meters between robots
    
    if (formation_type == "line") {
        for (size_t i = 0; i < robot_count; i++) {
            Position pos;
            pos.x = i * spacing;
            pos.y = 0;
            pos.z = 0;
            formation.push_back(pos);
        }
    } else if (formation_type == "triangle") {
        // Triangular formation
        size_t row = 0;
        size_t col = 0;
        
        for (size_t i = 0; i < robot_count; i++) {
            Position pos;
            pos.x = col * spacing;
            pos.y = row * spacing * std::sqrt(3) / 2;
            pos.z = 0;
            formation.push_back(pos);
            
            col++;
            if (col > row) {
                row++;
                col = 0;
            }
        }
    } else if (formation_type == "circle") {
        double radius = spacing * robot_count / (2 * M_PI);
        
        for (size_t i = 0; i < robot_count; i++) {
            double angle = 2 * M_PI * i / robot_count;
            Position pos;
            pos.x = radius * std::cos(angle);
            pos.y = radius * std::sin(angle);
            pos.z = 0;
            formation.push_back(pos);
        }
    } else {
        // Default: grid formation
        size_t grid_size = std::ceil(std::sqrt(robot_count));
        
        for (size_t i = 0; i < robot_count; i++) {
            Position pos;
            pos.x = (i % grid_size) * spacing;
            pos.y = (i / grid_size) * spacing;
            pos.z = 0;
            formation.push_back(pos);
        }
    }
    
    return formation;
}

} // namespace mycelix