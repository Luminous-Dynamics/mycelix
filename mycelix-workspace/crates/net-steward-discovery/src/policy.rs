use net_steward_schema::{
    BlastRadiusRiskTier, DriftStatus, EventMatcher, ExecutionMode, HumanReadableIncidentSummary,
    RemediationPolicy, Severity,
};

/// Evaluates an observed incident summary against the active remediation policy rules.
/// Returns the matched rule and execution verdict if a matching rule is found.
pub fn evaluate_policy(
    policy: &RemediationPolicy,
    incident: &HumanReadableIncidentSummary,
) -> PolicyEvaluationVerdict {
    for rule in &policy.rules {
        if matches_event(&rule.event_matcher, incident) {
            // Check risk barriers
            if incident.blast_radius_score > rule.max_allowed_blast_radius {
                return PolicyEvaluationVerdict::Escalated {
                    rule_name: rule.rule_name.clone(),
                    reason: format!(
                        "Blast radius score {:.2} exceeds maximum allowed {:.2}",
                        incident.blast_radius_score, rule.max_allowed_blast_radius
                    ),
                };
            }

            // Check risk tier mapping
            if risk_tier_severity(incident.safety_verdict, incident.blast_radius_score)
                > rule.max_allowed_risk_tier
            {
                return PolicyEvaluationVerdict::Escalated {
                    rule_name: rule.rule_name.clone(),
                    reason: format!(
                        "Risk tier exceeds policy allowed limit of {:?}",
                        rule.max_allowed_risk_tier
                    ),
                };
            }

            // Match successfully
            return PolicyEvaluationVerdict::Matched {
                rule_name: rule.rule_name.clone(),
                execution_mode: rule.execution_mode,
                required_witnesses: rule.required_witnesses,
                target_rollback_generation: rule.target_rollback_generation.clone(),
            };
        }
    }

    // Default fallback mode if no matching policy is found.
    PolicyEvaluationVerdict::DefaultFallback
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PolicyEvaluationVerdict {
    /// Found a matching policy rule that can be processed.
    Matched {
        rule_name: String,
        execution_mode: ExecutionMode,
        required_witnesses: u32,
        target_rollback_generation: Option<String>,
    },
    /// The event matched a rule but was escalated to manual/operator approval due to risk limits.
    Escalated { rule_name: String, reason: String },
    /// No policy rule matched the incident; fallback to default gate rules.
    DefaultFallback,
}

fn matches_event(matcher: &EventMatcher, incident: &HumanReadableIncidentSummary) -> bool {
    // 1. Check node target matching
    if matcher.node_id != "any" && matcher.node_id != incident.incident_id {
        // Also check if incident_id contains the node_id
        if !incident.incident_id.contains(&matcher.node_id) {
            return false;
        }
    }

    // 2. Check severity mapping (we use recommended action or violations to infer severity, or use a heuristic map)
    let incident_severity = infer_incident_severity(incident);
    if incident_severity < matcher.min_severity {
        return false;
    }

    // 3. Check systemd unit matches if specified
    if let Some(ref target_unit) = matcher.systemd_unit {
        let mut unit_matched = false;
        if incident.root_cause.contains(target_unit) {
            unit_matched = true;
        }
        for violation in &incident.safety_violations {
            if violation.contains(target_unit) {
                unit_matched = true;
            }
        }
        if !unit_matched {
            return false;
        }
    }

    // 4. Check max allowed drift status matches if specified
    if let Some(target_drift) = matcher.max_drift_status {
        let incident_drift = infer_incident_drift(incident);
        if drift_status_rank(incident_drift) > drift_status_rank(target_drift) {
            return false;
        }
    }

    true
}

fn drift_status_rank(status: DriftStatus) -> u32 {
    match status {
        DriftStatus::InSync => 0,
        DriftStatus::DriftDetected => 1,
        DriftStatus::Unknown => 2,
        DriftStatus::Unmanaged => 3,
    }
}

fn infer_incident_severity(incident: &HumanReadableIncidentSummary) -> Severity {
    // If there are safety violations, it's at least Medium, potentially High or Critical
    if !incident.safety_violations.is_empty() {
        if incident
            .recommended_action
            .to_lowercase()
            .contains("critical")
        {
            Severity::Critical
        } else {
            Severity::High
        }
    } else if incident.root_cause.to_lowercase().contains("warning") {
        Severity::Medium
    } else if incident.root_cause.to_lowercase().contains("info") {
        Severity::Info
    } else {
        Severity::Low
    }
}

fn infer_incident_drift(incident: &HumanReadableIncidentSummary) -> DriftStatus {
    if incident.root_cause.to_lowercase().contains("drift") {
        DriftStatus::DriftDetected
    } else if incident.root_cause.to_lowercase().contains("unmanaged") {
        DriftStatus::Unmanaged
    } else if incident.root_cause.to_lowercase().contains("unknown") {
        DriftStatus::Unknown
    } else {
        DriftStatus::InSync
    }
}

fn risk_tier_severity(
    verdict: net_steward_schema::SafetyVerdict,
    blast_radius: f32,
) -> BlastRadiusRiskTier {
    match verdict {
        net_steward_schema::SafetyVerdict::Blocked => BlastRadiusRiskTier::Critical,
        net_steward_schema::SafetyVerdict::Warning => {
            if blast_radius > 0.6 {
                BlastRadiusRiskTier::High
            } else if blast_radius > 0.3 {
                BlastRadiusRiskTier::Moderate
            } else {
                BlastRadiusRiskTier::Low
            }
        }
        net_steward_schema::SafetyVerdict::Safe => {
            if blast_radius > 0.2 {
                BlastRadiusRiskTier::Low
            } else {
                BlastRadiusRiskTier::Negligible
            }
        }
    }
}
