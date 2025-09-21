/**
 * Safety Validation System for Mycelix
 * Multi-level safety checks for robot actions
 */

#ifndef MYCELIX_BRIDGE_SAFETY_VALIDATOR_HPP
#define MYCELIX_BRIDGE_SAFETY_VALIDATOR_HPP

#include <vector>
#include <string>
#include <memory>
#include <chrono>
#include <functional>
#include <map>

namespace mycelix {

// Forward declarations
struct Action;
struct RobotAction;
struct Position;
struct Velocity;

/**
 * Safety rule types
 */
enum class SafetyType {
    KINEMATIC,      // Velocity, acceleration limits
    COLLISION,      // Obstacle avoidance
    STABILITY,      // Balance and stability
    POWER,          // Power and energy limits
    TEMPERATURE,    // Thermal limits
    COMPLIANCE,     // Regulatory compliance
    CUSTOM          // User-defined safety rules
};

/**
 * Safety actions to take
 */
enum class SafetyAction {
    ALLOW,          // Allow action as-is
    LIMIT,          // Apply limits but allow
    STOP,           // Stop immediately
    SLOW,           // Reduce speed
    WARN,           // Issue warning but allow
    REJECT          // Reject action completely
};

/**
 * Safety levels for alerts
 */
enum class SafetyLevel {
    SAFE,           // All systems nominal
    CAUTION,        // Approaching limits
    WARNING,        // Limits exceeded, action needed
    CRITICAL,       // Critical safety violation
    EMERGENCY       // Emergency stop required
};

/**
 * Safety rule definition
 */
struct SafetyRule {
    std::string name;
    SafetyType type;
    double threshold;
    SafetyAction action;
    std::string description;
    bool enabled = true;
    double priority = 1.0;
    std::function<bool(const Action&)> custom_validator;
};

/**
 * Safety state for checking
 */
struct SafetyState {
    Velocity velocity;
    double min_obstacle_distance;
    double system_health;
    double battery_level;
    double temperature;
    Position position;
    std::map<std::string, double> custom_metrics;
};

/**
 * Safety check result
 */
struct SafetyResult {
    SafetyLevel level;
    std::string message;
    std::vector<std::string> violations;
    std::map<std::string, double> metrics;
    std::chrono::milliseconds check_time;
};

/**
 * Compliance framework standards
 */
enum class ComplianceFramework {
    ISO26262,       // Automotive safety
    IEC61508,       // Functional safety
    ISO13849,       // Machinery safety
    IEC62061,       // Safety systems
    HIPAA,          // Medical privacy
    GDPR,           // Data protection
    CE,             // European conformity
    FDA,            // Food and drug administration
    CUSTOM          // Custom compliance
};

/**
 * Main safety validator class
 * Validates robot actions against safety rules
 */
class SafetyValidator {
public:
    /**
     * Constructor
     * @param framework Optional compliance framework
     */
    SafetyValidator(ComplianceFramework framework = ComplianceFramework::ISO26262);
    
    virtual ~SafetyValidator();
    
    // === Rule Management ===
    
    /**
     * Add safety rule
     * @param rule Safety rule to add
     */
    void addRule(const SafetyRule& rule);
    
    /**
     * Remove safety rule
     * @param rule_name Name of rule to remove
     * @return True if removed
     */
    bool removeRule(const std::string& rule_name);
    
    /**
     * Enable/disable rule
     * @param rule_name Name of rule
     * @param enabled Enable state
     */
    void setRuleEnabled(const std::string& rule_name, bool enabled);
    
    /**
     * Get all rules
     * @return Vector of safety rules
     */
    std::vector<SafetyRule> getRules() const;
    
    /**
     * Get rule count
     * @return Number of active rules
     */
    size_t getRuleCount() const;
    
    /**
     * Clear all rules
     */
    void clearRules();
    
    // === Validation ===
    
    /**
     * Validate action against safety rules
     * @param action Action to validate
     * @return True if safe
     */
    bool validate(const Action& action);
    
    /**
     * Validate robot action
     * @param action Robot action to validate
     * @return True if safe
     */
    bool validate(const RobotAction& action);
    
    /**
     * Validate with detailed result
     * @param action Action to validate
     * @return Detailed validation result
     */
    SafetyResult validateDetailed(const Action& action);
    
    /**
     * Check current safety state
     * @param state Current safety state
     * @return Safety check result
     */
    SafetyResult checkState(const SafetyState& state);
    
    /**
     * Predict safety of future state
     * @param current Current state
     * @param action Proposed action
     * @param horizon Time horizon in ms
     * @return Predicted safety result
     */
    SafetyResult predictSafety(const SafetyState& current, 
                               const Action& action,
                               std::chrono::milliseconds horizon);
    
    // === Limits & Constraints ===
    
    /**
     * Set velocity limits
     * @param max_linear Maximum linear velocity (m/s)
     * @param max_angular Maximum angular velocity (rad/s)
     */
    void setVelocityLimits(double max_linear, double max_angular);
    
    /**
     * Set acceleration limits
     * @param max_linear Maximum linear acceleration (m/s²)
     * @param max_angular Maximum angular acceleration (rad/s²)
     */
    void setAccelerationLimits(double max_linear, double max_angular);
    
    /**
     * Set minimum obstacle distance
     * @param distance Minimum safe distance (meters)
     */
    void setMinObstacleDistance(double distance);
    
    /**
     * Set workspace boundaries
     * @param min_pos Minimum position
     * @param max_pos Maximum position
     */
    void setWorkspaceBounds(const Position& min_pos, const Position& max_pos);
    
    /**
     * Apply safety limits to action
     * @param action Action to limit
     * @return Limited action
     */
    Action applyLimits(const Action& action) const;
    
    // === Compliance ===
    
    /**
     * Set compliance framework
     * @param framework Compliance standard
     */
    void setComplianceFramework(ComplianceFramework framework);
    
    /**
     * Get compliance framework
     * @return Current framework
     */
    ComplianceFramework getComplianceFramework() const;
    
    /**
     * Check compliance
     * @param action Action to check
     * @return Compliance status
     */
    bool checkCompliance(const Action& action) const;
    
    /**
     * Generate compliance report
     * @return Compliance report
     */
    std::string generateComplianceReport() const;
    
    /**
     * Get certification status
     * @return Map of certification to status
     */
    std::map<std::string, bool> getCertificationStatus() const;
    
    // === Emergency Handling ===
    
    /**
     * Trigger emergency stop
     * @param reason Reason for emergency
     */
    void emergencyStop(const std::string& reason);
    
    /**
     * Check if in emergency state
     * @return True if emergency active
     */
    bool isEmergency() const;
    
    /**
     * Clear emergency state
     * @param override_code Safety override code
     * @return True if cleared
     */
    bool clearEmergency(const std::string& override_code);
    
    /**
     * Get emergency action
     * @return Safe emergency action
     */
    Action getEmergencyAction() const;
    
    // === Monitoring & Logging ===
    
    /**
     * Set safety callback
     * @param callback Function called on safety events
     */
    void setSafetyCallback(std::function<void(const SafetyResult&)> callback);
    
    /**
     * Get safety statistics
     * @return Map of statistic to value
     */
    std::map<std::string, double> getSafetyStats() const;
    
    /**
     * Get violation history
     * @param max_count Maximum number to return
     * @return Recent violations
     */
    std::vector<SafetyResult> getViolationHistory(size_t max_count = 100) const;
    
    /**
     * Clear violation history
     */
    void clearViolationHistory();
    
    /**
     * Enable safety logging
     * @param enabled Enable state
     */
    void setLoggingEnabled(bool enabled);
    
    // === Multi-Level Safety ===
    
    /**
     * Perform hardware-level safety check
     * @param action Action to check
     * @return Safety result
     */
    SafetyResult hardwareSafetyCheck(const Action& action) const;
    
    /**
     * Perform software-level safety check
     * @param action Action to check
     * @return Safety result
     */
    SafetyResult softwareSafetyCheck(const Action& action) const;
    
    /**
     * Perform network-level safety check
     * @param action Action to check
     * @return Safety result
     */
    SafetyResult networkSafetyCheck(const Action& action) const;
    
private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

/**
 * Advanced safety validator with ML-based prediction
 */
class AdvancedSafetyValidator : public SafetyValidator {
public:
    AdvancedSafetyValidator();
    
    /**
     * Train safety predictor from history
     * @param history Historical safety data
     */
    void trainPredictor(const std::vector<SafetyResult>& history);
    
    /**
     * Predict safety using ML model
     * @param state Current state
     * @param action Proposed action
     * @return Predicted safety probability
     */
    double predictSafetyProbability(const SafetyState& state,
                                    const Action& action) const;
    
    /**
     * Adaptive safety limits based on context
     * @param context Environmental context
     */
    void adaptLimits(const std::map<std::string, double>& context);
    
    /**
     * Learn from safety violations
     * @param violation Safety violation event
     */
    void learnFromViolation(const SafetyResult& violation);
};

/**
 * Swarm safety validator for multi-robot systems
 */
class SwarmSafetyValidator {
public:
    SwarmSafetyValidator();
    
    /**
     * Validate swarm action
     * @param swarm_action Collective action
     * @param robot_states States of all robots
     * @return Safety result for swarm
     */
    SafetyResult validateSwarmAction(
        const std::vector<Action>& swarm_action,
        const std::vector<SafetyState>& robot_states);
    
    /**
     * Check for inter-robot collisions
     * @param positions Robot positions
     * @param velocities Robot velocities
     * @return Collision risk assessment
     */
    SafetyResult checkCollisionRisk(
        const std::vector<Position>& positions,
        const std::vector<Velocity>& velocities);
    
    /**
     * Coordinate safe formation
     * @param formation_type Type of formation
     * @param robot_count Number of robots
     * @return Safe formation parameters
     */
    std::vector<Position> coordinateSafeFormation(
        const std::string& formation_type,
        size_t robot_count);
};

} // namespace mycelix

#endif // MYCELIX_BRIDGE_SAFETY_VALIDATOR_HPP