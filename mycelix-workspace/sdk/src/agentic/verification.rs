// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Formal Verification Framework
//!
//! Provides tools for formally verifying trust system properties,
//! including invariant checking, property specification, and proof obligations.
//!
//! Features:
//! - Invariant types (trust bounds, Byzantine tolerance, slashing bounds)
//! - Property formulas with temporal logic
//! - Proof obligations with witnesses and counterexamples
//! - Verification engine with comprehensive checking

use serde::{Deserialize, Serialize};
use std::collections::HashMap;

/// System invariant to verify
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Invariant {
    /// Invariant ID
    pub invariant_id: String,
    /// Human-readable name
    pub name: String,
    /// Description
    pub description: String,
    /// Invariant type
    pub invariant_type: InvariantType,
    /// Severity if violated
    pub severity: ViolationSeverity,
}

/// Types of invariants
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum InvariantType {
    /// Trust scores must be within bounds.
    TrustBounds {
        /// Minimum allowed trust.
        min: f64,
        /// Maximum allowed trust.
        max: f64,
    },
    /// Byzantine tolerance threshold.
    ByzantineTolerance {
        /// Maximum fraction of Byzantine agents tolerated.
        max_byzantine_fraction: f64,
    },
    /// Slashing amounts must be bounded.
    SlashingBounds {
        /// Maximum fraction that can be slashed.
        max_slash_fraction: f64,
    },
    /// K-Vector dimension bounds.
    KVectorBounds {
        /// Dimension name.
        dimension: String,
        /// Minimum allowed value.
        min: f64,
        /// Maximum allowed value.
        max: f64,
    },
    /// Monotonicity (values only increase or decrease).
    Monotonicity {
        /// Property to check.
        property: String,
        /// Required direction.
        direction: MonotonicityDirection,
    },
    /// Conservation (sum of values is constant).
    Conservation {
        /// Property to check.
        property: String,
        /// Allowed deviation.
        tolerance: f64,
    },
    /// Custom predicate.
    Custom {
        /// Predicate identifier.
        predicate: String,
    },
}

/// Direction for monotonicity invariants
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum MonotonicityDirection {
    /// Non-decreasing (can increase or stay same)
    NonDecreasing,
    /// Non-increasing (can decrease or stay same)
    NonIncreasing,
    /// Strictly increasing
    StrictlyIncreasing,
    /// Strictly decreasing
    StrictlyDecreasing,
}

/// Severity of invariant violation
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
pub enum ViolationSeverity {
    /// Warning only
    Warning = 0,
    /// Error (should not happen)
    Error = 1,
    /// Critical (system integrity compromised)
    Critical = 2,
    /// Fatal (system must halt)
    Fatal = 3,
}

/// Result of checking an invariant
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InvariantCheckResult {
    /// Invariant that was checked
    pub invariant_id: String,
    /// Whether invariant holds
    pub holds: bool,
    /// Violation details if any
    pub violation: Option<InvariantViolation>,
    /// Check timestamp
    pub timestamp: u64,
}

/// Details of an invariant violation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct InvariantViolation {
    /// Actual value that violated invariant
    pub actual_value: String,
    /// Expected constraint
    pub expected: String,
    /// Context (e.g., which agent)
    pub context: HashMap<String, String>,
    /// Severity
    pub severity: ViolationSeverity,
}

/// Property specification for verification
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PropertySpec {
    /// Property ID
    pub property_id: String,
    /// Human-readable name
    pub name: String,
    /// Description
    pub description: String,
    /// Property formula (temporal logic)
    pub formula: PropertyFormula,
    /// Whether property is safety or liveness
    pub property_kind: PropertyKind,
}

/// Kind of property
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum PropertyKind {
    /// Safety property (nothing bad happens)
    Safety,
    /// Liveness property (something good eventually happens)
    Liveness,
    /// Fairness property
    Fairness,
}

/// Temporal logic formula
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum PropertyFormula {
    /// Atomic predicate
    Atomic(AtomicPredicate),
    /// Conjunction (AND)
    And(Box<PropertyFormula>, Box<PropertyFormula>),
    /// Disjunction (OR)
    Or(Box<PropertyFormula>, Box<PropertyFormula>),
    /// Negation (NOT)
    Not(Box<PropertyFormula>),
    /// Implication (IF-THEN)
    Implies(Box<PropertyFormula>, Box<PropertyFormula>),
    /// Always (globally true)
    Always(Box<PropertyFormula>),
    /// Eventually (true at some point)
    Eventually(Box<PropertyFormula>),
    /// Until (true until another becomes true)
    Until(Box<PropertyFormula>, Box<PropertyFormula>),
}

/// Atomic predicate in property formula
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum AtomicPredicate {
    /// Trust score comparison.
    TrustCompare {
        /// Agent to check.
        agent_id: String,
        /// Comparison operator.
        operator: CompareOp,
        /// Threshold value.
        value: f64,
    },
    /// Byzantine count comparison.
    ByzantineCount {
        /// Comparison operator.
        operator: CompareOp,
        /// Threshold count.
        value: usize,
    },
    /// Network health comparison.
    HealthCompare {
        /// Comparison operator.
        operator: CompareOp,
        /// Threshold value.
        value: f64,
    },
    /// Agent status check.
    AgentStatus {
        /// Agent to check.
        agent_id: String,
        /// Expected status.
        status: String,
    },
    /// Consensus reached.
    ConsensusReached {
        /// Proposal identifier.
        proposal_id: String,
    },
    /// Custom predicate.
    Custom {
        /// Predicate name.
        name: String,
        /// Predicate parameters.
        params: HashMap<String, String>,
    },
}

/// Comparison operator
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum CompareOp {
    /// Less than
    Lt,
    /// Less than or equal
    Le,
    /// Equal
    Eq,
    /// Greater than or equal
    Ge,
    /// Greater than
    Gt,
    /// Not equal
    Ne,
}

impl CompareOp {
    /// Evaluate comparison
    pub fn eval(&self, left: f64, right: f64) -> bool {
        // Non-finite values are invalid evidence, not ordinary comparison values.
        if !left.is_finite() || !right.is_finite() {
            return false;
        }
        match self {
            CompareOp::Lt => left < right,
            CompareOp::Le => left <= right,
            CompareOp::Eq => (left - right).abs() < f64::EPSILON,
            CompareOp::Ge => left >= right,
            CompareOp::Gt => left > right,
            CompareOp::Ne => (left - right).abs() >= f64::EPSILON,
        }
    }

    /// Evaluate comparison for integers
    pub fn eval_usize(&self, left: usize, right: usize) -> bool {
        match self {
            CompareOp::Lt => left < right,
            CompareOp::Le => left <= right,
            CompareOp::Eq => left == right,
            CompareOp::Ge => left >= right,
            CompareOp::Gt => left > right,
            CompareOp::Ne => left != right,
        }
    }
}

/// Proof obligation
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProofObligation {
    /// Obligation ID
    pub obligation_id: String,
    /// Property to prove
    pub property: PropertySpec,
    /// Proof status
    pub status: ProofStatus,
    /// Proof technique used
    pub technique: Option<ProofTechnique>,
    /// Witness if proven
    pub witness: Option<ProofWitness>,
    /// Counterexample if disproven
    pub counterexample: Option<Counterexample>,
}

/// Status of a proof obligation
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProofStatus {
    /// Not yet attempted
    Pending,
    /// Proof in progress
    InProgress,
    /// Successfully proven
    Proven,
    /// Disproven (counterexample found)
    Disproven,
    /// Unknown (could not determine)
    Unknown,
    /// Timeout
    Timeout,
}

/// Proof technique
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProofTechnique {
    /// Invariant induction
    Induction,
    /// Model checking
    ModelChecking,
    /// Theorem proving
    TheoremProving,
    /// Runtime verification
    RuntimeVerification,
    /// Simulation-based testing
    Simulation,
}

/// Witness for a proven property
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProofWitness {
    /// Technique used
    pub technique: ProofTechnique,
    /// Witness data (technique-specific)
    pub data: HashMap<String, String>,
    /// Verification timestamp
    pub verified_at: u64,
}

/// Counterexample for a disproven property
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Counterexample {
    /// Trace of states leading to violation
    pub trace: Vec<SystemState>,
    /// The action that caused violation
    pub violating_action: Option<Action>,
    /// Description of violation
    pub description: String,
}

/// System state snapshot
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SystemState {
    /// State index in trace
    pub index: usize,
    /// Timestamp
    pub timestamp: u64,
    /// Agent trust scores
    pub trust_scores: HashMap<String, f64>,
    /// Byzantine agent count
    pub byzantine_count: usize,
    /// Network health
    pub network_health: f64,
    /// Additional state variables
    pub variables: HashMap<String, String>,
}

/// Action that can change state
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Action {
    /// Action type
    pub action_type: String,
    /// Actor (agent ID)
    pub actor: String,
    /// Action parameters
    pub params: HashMap<String, String>,
    /// Timestamp
    pub timestamp: u64,
}

/// Construct a failed invariant result for violated or insufficient evidence.
fn failed_evidence(
    invariant: &Invariant,
    timestamp: u64,
    actual_value: impl Into<String>,
    expected: impl Into<String>,
) -> InvariantCheckResult {
    InvariantCheckResult {
        invariant_id: invariant.invariant_id.clone(),
        holds: false,
        violation: Some(InvariantViolation {
            actual_value: actual_value.into(),
            expected: expected.into(),
            context: HashMap::new(),
            severity: invariant.severity,
        }),
        timestamp,
    }
}

/// Verification engine
pub struct VerificationEngine {
    /// Registered invariants
    invariants: Vec<Invariant>,
    /// Registered properties
    properties: Vec<PropertySpec>,
    /// Proof obligations
    obligations: Vec<ProofObligation>,
    /// Check history
    history: Vec<InvariantCheckResult>,
    /// Event log
    events: Vec<VerificationEvent>,
}

impl Default for VerificationEngine {
    fn default() -> Self {
        Self::new()
    }
}

impl VerificationEngine {
    /// Create new verification engine
    pub fn new() -> Self {
        Self {
            invariants: vec![],
            properties: vec![],
            obligations: vec![],
            history: vec![],
            events: vec![],
        }
    }

    /// Create with default invariants
    pub fn with_defaults() -> Self {
        let mut engine = Self::new();
        engine.add_default_invariants();
        engine.add_default_properties();
        engine
    }

    /// Add default system invariants
    fn add_default_invariants(&mut self) {
        // Trust bounds invariant
        self.register_invariant(Invariant {
            invariant_id: "trust_bounds".to_string(),
            name: "Trust Score Bounds".to_string(),
            description: "All trust scores must be between 0 and 1".to_string(),
            invariant_type: InvariantType::TrustBounds { min: 0.0, max: 1.0 },
            severity: ViolationSeverity::Critical,
        });

        // Byzantine tolerance invariant
        self.register_invariant(Invariant {
            invariant_id: "byzantine_tolerance".to_string(),
            name: "Byzantine Tolerance".to_string(),
            description: "Byzantine agents must not exceed 34% of network (validated threshold)"
                .to_string(),
            invariant_type: InvariantType::ByzantineTolerance {
                max_byzantine_fraction: 0.34,
            },
            severity: ViolationSeverity::Fatal,
        });

        // Slashing bounds invariant
        self.register_invariant(Invariant {
            invariant_id: "slashing_bounds".to_string(),
            name: "Slashing Bounds".to_string(),
            description: "Slashing cannot exceed 50% of stake".to_string(),
            invariant_type: InvariantType::SlashingBounds {
                max_slash_fraction: 0.5,
            },
            severity: ViolationSeverity::Error,
        });

        // K-Vector bounds
        for dim in [
            "k_r",
            "k_a",
            "k_i",
            "k_p",
            "k_m",
            "k_s",
            "k_h",
            "k_topo",
            "k_v",
            "k_coherence",
        ] {
            self.register_invariant(Invariant {
                invariant_id: format!("{}_bounds", dim),
                name: format!("{} Bounds", dim.to_uppercase()),
                description: format!("{} must be between 0 and 1", dim),
                invariant_type: InvariantType::KVectorBounds {
                    dimension: dim.to_string(),
                    min: 0.0,
                    max: 1.0,
                },
                severity: ViolationSeverity::Error,
            });
        }
    }

    /// Add default properties
    fn add_default_properties(&mut self) {
        // Safety: Trust never exceeds bounds
        self.register_property(PropertySpec {
            property_id: "safety_trust_bounds".to_string(),
            name: "Trust Bounds Safety".to_string(),
            description: "Trust scores always remain within valid bounds".to_string(),
            formula: PropertyFormula::Always(Box::new(PropertyFormula::Atomic(
                AtomicPredicate::Custom {
                    name: "trust_in_bounds".to_string(),
                    params: HashMap::new(),
                },
            ))),
            property_kind: PropertyKind::Safety,
        });

        // Liveness: Malicious agents eventually detected
        self.register_property(PropertySpec {
            property_id: "liveness_detection".to_string(),
            name: "Malicious Agent Detection".to_string(),
            description: "Malicious behavior is eventually detected".to_string(),
            formula: PropertyFormula::Implies(
                Box::new(PropertyFormula::Atomic(AtomicPredicate::Custom {
                    name: "malicious_behavior".to_string(),
                    params: HashMap::new(),
                })),
                Box::new(PropertyFormula::Eventually(Box::new(
                    PropertyFormula::Atomic(AtomicPredicate::Custom {
                        name: "detected".to_string(),
                        params: HashMap::new(),
                    }),
                ))),
            ),
            property_kind: PropertyKind::Liveness,
        });
    }

    /// Register an invariant
    pub fn register_invariant(&mut self, invariant: Invariant) {
        self.invariants.push(invariant);
    }

    /// Register a property
    pub fn register_property(&mut self, property: PropertySpec) {
        self.properties.push(property);
    }

    /// Check all invariants against current state
    pub fn check_invariants(&mut self, state: &SystemState) -> Vec<InvariantCheckResult> {
        let timestamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs();

        let mut results = vec![];

        for invariant in &self.invariants {
            let result = self.check_invariant(invariant, state, timestamp);
            results.push(result.clone());
            self.history.push(result.clone());

            if !result.holds {
                self.events.push(VerificationEvent {
                    event_type: VerificationEventType::InvariantViolation,
                    invariant_id: Some(invariant.invariant_id.clone()),
                    property_id: None,
                    timestamp,
                    details: result.violation.as_ref().map(|v| v.actual_value.clone()),
                });
            }
        }

        results
    }

    /// Check a single invariant
    fn check_invariant(
        &self,
        invariant: &Invariant,
        state: &SystemState,
        timestamp: u64,
    ) -> InvariantCheckResult {
        let (holds, violation) = match &invariant.invariant_type {
            InvariantType::TrustBounds { min, max } => {
                if !min.is_finite() || !max.is_finite() || min > max {
                    return failed_evidence(
                        invariant,
                        timestamp,
                        format!("invalid trust bounds: [{}, {}]", min, max),
                        "finite ordered trust bounds",
                    );
                }
                let mut violation = None;
                let mut holds = true;
                for (agent_id, score) in &state.trust_scores {
                    if !score.is_finite() || *score < *min || *score > *max {
                        holds = false;
                        violation = Some(InvariantViolation {
                            actual_value: format!("{}: {}", agent_id, score),
                            expected: format!("finite trust in [{}, {}]", min, max),
                            context: [("agent_id".to_string(), agent_id.clone())]
                                .into_iter()
                                .collect(),
                            severity: invariant.severity,
                        });
                        break;
                    }
                }
                (holds, violation)
            }

            InvariantType::ByzantineTolerance {
                max_byzantine_fraction,
            } => {
                let total = state.trust_scores.len();
                if total == 0 || state.byzantine_count > total {
                    return failed_evidence(
                        invariant,
                        timestamp,
                        format!(
                            "byzantine_count={} known_agents={}",
                            state.byzantine_count, total
                        ),
                        "a non-empty known-agent set and byzantine_count <= known_agents",
                    );
                }
                if !max_byzantine_fraction.is_finite()
                    || !(0.0..=1.0).contains(max_byzantine_fraction)
                {
                    return failed_evidence(
                        invariant,
                        timestamp,
                        format!(
                            "invalid maximum Byzantine fraction: {}",
                            max_byzantine_fraction
                        ),
                        "a finite fraction in [0, 1]",
                    );
                }
                let fraction = state.byzantine_count as f64 / total as f64;
                if fraction > *max_byzantine_fraction {
                    (
                        false,
                        Some(InvariantViolation {
                            actual_value: format!(
                                "{}/{} = {:.2}%",
                                state.byzantine_count,
                                total,
                                fraction * 100.0
                            ),
                            expected: format!("≤{:.2}%", max_byzantine_fraction * 100.0),
                            context: HashMap::new(),
                            severity: invariant.severity,
                        }),
                    )
                } else {
                    (true, None)
                }
            }

            InvariantType::SlashingBounds { max_slash_fraction } => {
                if !max_slash_fraction.is_finite() || !(0.0..=1.0).contains(max_slash_fraction) {
                    return failed_evidence(
                        invariant,
                        timestamp,
                        format!("invalid maximum slash fraction: {}", max_slash_fraction),
                        "a finite fraction in [0, 1]",
                    );
                }
                let slash_str = match state.variables.get("last_slash_fraction") {
                    Some(value) => value,
                    None => {
                        return failed_evidence(
                            invariant,
                            timestamp,
                            "missing last_slash_fraction",
                            "a present, finite slash fraction in [0, 1]",
                        );
                    }
                };
                let slash = match slash_str.parse::<f64>() {
                    Ok(value) if value.is_finite() && (0.0..=1.0).contains(&value) => value,
                    _ => {
                        return failed_evidence(
                            invariant,
                            timestamp,
                            format!("invalid last_slash_fraction: {}", slash_str),
                            "a finite slash fraction in [0, 1]",
                        );
                    }
                };
                if slash > *max_slash_fraction {
                    return failed_evidence(
                        invariant,
                        timestamp,
                        format!("last_slash_fraction={}", slash),
                        format!("slash fraction ≤ {}", max_slash_fraction),
                    );
                }
                (true, None)
            }

            InvariantType::KVectorBounds {
                dimension,
                min,
                max,
            } => {
                if !min.is_finite() || !max.is_finite() || min > max {
                    return failed_evidence(
                        invariant,
                        timestamp,
                        format!("invalid bounds for {}: [{}, {}]", dimension, min, max),
                        "finite ordered bounds",
                    );
                }
                let key = format!("kvector_{}", dimension);
                let value_str = match state.variables.get(&key) {
                    Some(value) => value,
                    None => {
                        return failed_evidence(
                            invariant,
                            timestamp,
                            format!("missing {}", key),
                            format!("{} present, finite, and in [{}, {}]", key, min, max),
                        );
                    }
                };
                let value = match value_str.parse::<f64>() {
                    Ok(value) if value.is_finite() => value,
                    _ => {
                        return failed_evidence(
                            invariant,
                            timestamp,
                            format!("invalid {}: {}", key, value_str),
                            format!("{} present, finite, and in [{}, {}]", key, min, max),
                        );
                    }
                };
                if value < *min || value > *max {
                    return failed_evidence(
                        invariant,
                        timestamp,
                        format!("{}={}", dimension, value),
                        format!("{} in [{}, {}]", dimension, min, max),
                    );
                }
                (true, None)
            }

            InvariantType::Monotonicity {
                property,
                direction,
            } => {
                let key = format!("prev_{}", property);
                let prev_str = match state.variables.get(&key) {
                    Some(value) => value,
                    None => {
                        return failed_evidence(
                            invariant,
                            timestamp,
                            format!("missing {}", key),
                            format!("finite previous and current values for {}", property),
                        );
                    }
                };
                let curr_str = match state.variables.get(property) {
                    Some(value) => value,
                    None => {
                        return failed_evidence(
                            invariant,
                            timestamp,
                            format!("missing {}", property),
                            format!("finite previous and current values for {}", property),
                        );
                    }
                };
                let prev = match prev_str.parse::<f64>() {
                    Ok(value) if value.is_finite() => value,
                    _ => {
                        return failed_evidence(
                            invariant,
                            timestamp,
                            format!("invalid {}: {}", key, prev_str),
                            "finite, parseable previous and current values",
                        );
                    }
                };
                let curr = match curr_str.parse::<f64>() {
                    Ok(value) if value.is_finite() => value,
                    _ => {
                        return failed_evidence(
                            invariant,
                            timestamp,
                            format!("invalid {}: {}", property, curr_str),
                            "finite, parseable previous and current values",
                        );
                    }
                };
                let violated = match direction {
                    MonotonicityDirection::NonDecreasing => curr < prev,
                    MonotonicityDirection::NonIncreasing => curr > prev,
                    MonotonicityDirection::StrictlyIncreasing => curr <= prev,
                    MonotonicityDirection::StrictlyDecreasing => curr >= prev,
                };
                if violated {
                    return failed_evidence(
                        invariant,
                        timestamp,
                        format!("{}: {} -> {}", property, prev, curr),
                        format!("{:?}", direction),
                    );
                }
                (true, None)
            }

            InvariantType::Conservation {
                property,
                tolerance,
            } => {
                if !tolerance.is_finite() || *tolerance < 0.0 {
                    return failed_evidence(
                        invariant,
                        timestamp,
                        format!("invalid conservation tolerance: {}", tolerance),
                        "a finite non-negative tolerance",
                    );
                }
                let key = format!("{}_total", property);
                let prev_key = format!("prev_{}_total", property);
                let curr_str = match state.variables.get(&key) {
                    Some(value) => value,
                    None => {
                        return failed_evidence(
                            invariant,
                            timestamp,
                            format!("missing {}", key),
                            "finite current and previous totals",
                        );
                    }
                };
                let prev_str = match state.variables.get(&prev_key) {
                    Some(value) => value,
                    None => {
                        return failed_evidence(
                            invariant,
                            timestamp,
                            format!("missing {}", prev_key),
                            "finite current and previous totals",
                        );
                    }
                };
                let curr = match curr_str.parse::<f64>() {
                    Ok(value) if value.is_finite() => value,
                    _ => {
                        return failed_evidence(
                            invariant,
                            timestamp,
                            format!("invalid {}: {}", key, curr_str),
                            "finite, parseable current and previous totals",
                        );
                    }
                };
                let prev = match prev_str.parse::<f64>() {
                    Ok(value) if value.is_finite() => value,
                    _ => {
                        return failed_evidence(
                            invariant,
                            timestamp,
                            format!("invalid {}: {}", prev_key, prev_str),
                            "finite, parseable current and previous totals",
                        );
                    }
                };
                let diff = (curr - prev).abs();
                if !diff.is_finite() || diff > *tolerance {
                    return failed_evidence(
                        invariant,
                        timestamp,
                        format!(
                            "{} changed by {} (tolerance: {})",
                            property, diff, tolerance
                        ),
                        format!("change ≤ {}", tolerance),
                    );
                }
                (true, None)
            }

            InvariantType::Custom { predicate } => {
                match state.variables.get(predicate).map(String::as_str) {
                    Some("true") => (true, None),
                    Some(value) => {
                        return failed_evidence(
                            invariant,
                            timestamp,
                            format!("custom predicate {} returned {:?}", predicate, value),
                            "explicit literal true",
                        );
                    }
                    None => {
                        return failed_evidence(
                            invariant,
                            timestamp,
                            format!("missing custom predicate {}", predicate),
                            "explicit literal true",
                        );
                    }
                }
            }
        };

        InvariantCheckResult {
            invariant_id: invariant.invariant_id.clone(),
            holds,
            violation,
            timestamp,
        }
    }

    /// Create a proof obligation for a property
    pub fn create_obligation(&mut self, property_id: &str) -> Option<String> {
        let property = self
            .properties
            .iter()
            .find(|p| p.property_id == property_id)?;

        let obligation_id = format!("obl_{}_{}", property_id, self.obligations.len());

        let obligation = ProofObligation {
            obligation_id: obligation_id.clone(),
            property: property.clone(),
            status: ProofStatus::Pending,
            technique: None,
            witness: None,
            counterexample: None,
        };

        self.obligations.push(obligation);
        Some(obligation_id)
    }

    /// Get verification summary
    pub fn summary(&self) -> VerificationSummary {
        let total_checks = self.history.len();
        let violations = self.history.iter().filter(|r| !r.holds).count();

        let proven = self
            .obligations
            .iter()
            .filter(|o| o.status == ProofStatus::Proven)
            .count();

        let disproven = self
            .obligations
            .iter()
            .filter(|o| o.status == ProofStatus::Disproven)
            .count();

        VerificationSummary {
            total_invariants: self.invariants.len(),
            total_properties: self.properties.len(),
            total_obligations: self.obligations.len(),
            total_checks,
            violations,
            proven,
            disproven,
        }
    }

    /// Get violation history
    pub fn violations(&self) -> Vec<&InvariantCheckResult> {
        self.history.iter().filter(|r| !r.holds).collect()
    }

    /// Get events
    pub fn events(&self) -> &[VerificationEvent] {
        &self.events
    }
}

/// Verification event
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VerificationEvent {
    /// Event type
    pub event_type: VerificationEventType,
    /// Related invariant ID
    pub invariant_id: Option<String>,
    /// Related property ID
    pub property_id: Option<String>,
    /// Timestamp
    pub timestamp: u64,
    /// Additional details
    pub details: Option<String>,
}

/// Types of verification events
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum VerificationEventType {
    /// Invariant was violated
    InvariantViolation,
    /// Property was proven
    PropertyProven,
    /// Property was disproven
    PropertyDisproven,
    /// Check completed
    CheckCompleted,
}

/// Summary of verification status
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VerificationSummary {
    /// Total registered invariants
    pub total_invariants: usize,
    /// Total registered properties
    pub total_properties: usize,
    /// Total proof obligations
    pub total_obligations: usize,
    /// Total invariant checks performed
    pub total_checks: usize,
    /// Number of violations detected
    pub violations: usize,
    /// Number of properties proven
    pub proven: usize,
    /// Number of properties disproven
    pub disproven: usize,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn create_test_state() -> SystemState {
        let mut trust_scores = HashMap::new();
        trust_scores.insert("agent_1".to_string(), 0.8);
        trust_scores.insert("agent_2".to_string(), 0.6);
        trust_scores.insert("agent_3".to_string(), 0.7);

        let mut variables = HashMap::new();
        variables.insert("last_slash_fraction".to_string(), "0.0".to_string());
        for dimension in [
            "k_r",
            "k_a",
            "k_i",
            "k_p",
            "k_m",
            "k_s",
            "k_h",
            "k_topo",
            "k_v",
            "k_coherence",
        ] {
            variables.insert(format!("kvector_{}", dimension), "0.5".to_string());
        }

        SystemState {
            index: 0,
            timestamp: 1000,
            trust_scores,
            byzantine_count: 0,
            network_health: 0.9,
            variables,
        }
    }

    #[test]
    fn test_trust_bounds_invariant() {
        let mut engine = VerificationEngine::with_defaults();
        let state = create_test_state();

        let results = engine.check_invariants(&state);
        let trust_bounds_result = results
            .iter()
            .find(|r| r.invariant_id == "trust_bounds")
            .unwrap();

        assert!(trust_bounds_result.holds);
    }

    #[test]
    fn test_trust_bounds_violation() {
        let mut engine = VerificationEngine::with_defaults();
        let mut state = create_test_state();
        state.trust_scores.insert("bad_agent".to_string(), 1.5); // Invalid

        let results = engine.check_invariants(&state);
        let trust_bounds_result = results
            .iter()
            .find(|r| r.invariant_id == "trust_bounds")
            .unwrap();

        assert!(!trust_bounds_result.holds);
        assert!(trust_bounds_result.violation.is_some());
    }

    #[test]
    fn test_byzantine_tolerance() {
        let mut engine = VerificationEngine::with_defaults();
        let mut state = create_test_state();

        // Under threshold (0/3 = 0%)
        let results = engine.check_invariants(&state);
        let byz_result = results
            .iter()
            .find(|r| r.invariant_id == "byzantine_tolerance")
            .unwrap();
        assert!(byz_result.holds);

        // Over threshold (2/3 = 66%)
        state.byzantine_count = 2;
        let results = engine.check_invariants(&state);
        let byz_result = results
            .iter()
            .find(|r| r.invariant_id == "byzantine_tolerance")
            .unwrap();
        assert!(!byz_result.holds);
    }

    #[test]
    fn test_compare_operators() {
        assert!(CompareOp::Lt.eval(1.0, 2.0));
        assert!(!CompareOp::Lt.eval(2.0, 1.0));

        assert!(CompareOp::Le.eval(1.0, 1.0));
        assert!(CompareOp::Le.eval(1.0, 2.0));

        assert!(CompareOp::Eq.eval(1.0, 1.0));
        assert!(!CompareOp::Eq.eval(1.0, 1.1));

        assert!(CompareOp::Ge.eval(2.0, 1.0));
        assert!(CompareOp::Ge.eval(1.0, 1.0));

        assert!(CompareOp::Gt.eval(2.0, 1.0));
        assert!(!CompareOp::Gt.eval(1.0, 1.0));

        assert!(CompareOp::Ne.eval(1.0, 2.0));
        assert!(!CompareOp::Ne.eval(1.0, 1.0));
    }

    #[test]
    fn test_verification_summary() {
        let mut engine = VerificationEngine::with_defaults();
        let state = create_test_state();

        engine.check_invariants(&state);
        engine.check_invariants(&state);

        let summary = engine.summary();
        assert!(summary.total_invariants > 0);
        assert!(summary.total_checks > 0);
        assert_eq!(summary.violations, 0);
    }

    #[test]
    fn test_create_obligation() {
        let mut engine = VerificationEngine::with_defaults();

        let obligation_id = engine.create_obligation("safety_trust_bounds");
        assert!(obligation_id.is_some());

        let summary = engine.summary();
        assert_eq!(summary.total_obligations, 1);
    }

    #[test]
    fn test_property_formula_construction() {
        // Safety: Always(trust_valid)
        let safety =
            PropertyFormula::Always(Box::new(PropertyFormula::Atomic(AtomicPredicate::Custom {
                name: "trust_valid".to_string(),
                params: HashMap::new(),
            })));

        // Check structure
        if let PropertyFormula::Always(inner) = safety {
            if let PropertyFormula::Atomic(AtomicPredicate::Custom { name, .. }) = *inner {
                assert_eq!(name, "trust_valid");
            } else {
                panic!("Expected Custom predicate");
            }
        } else {
            panic!("Expected Always formula");
        }
    }

    #[test]
    fn test_non_finite_trust_score_fails_closed() {
        let mut engine = VerificationEngine::with_defaults();
        let mut state = create_test_state();
        state.trust_scores.insert("nan_agent".to_string(), f64::NAN);
        let result = engine
            .check_invariants(&state)
            .into_iter()
            .find(|r| r.invariant_id == "trust_bounds")
            .unwrap();
        assert!(!result.holds);
        assert!(result.violation.is_some());
    }

    #[test]
    fn test_missing_or_malformed_slashing_and_kvector_fail_closed() {
        let engine = VerificationEngine::new();
        let mut state = create_test_state();
        state.variables.remove("last_slash_fraction");
        let slashing = Invariant {
            invariant_id: "slashing".into(),
            name: "Slashing".into(),
            description: "Slash must be bounded".into(),
            invariant_type: InvariantType::SlashingBounds {
                max_slash_fraction: 0.5,
            },
            severity: ViolationSeverity::Critical,
        };
        assert!(!engine.check_invariant(&slashing, &state, 1000).holds);
        state
            .variables
            .insert("last_slash_fraction".into(), "NaN".into());
        assert!(!engine.check_invariant(&slashing, &state, 1000).holds);

        state.variables.remove("kvector_k_r");
        let kvector = Invariant {
            invariant_id: "kvector".into(),
            name: "K-vector".into(),
            description: "Dimension must be bounded".into(),
            invariant_type: InvariantType::KVectorBounds {
                dimension: "k_r".into(),
                min: 0.0,
                max: 1.0,
            },
            severity: ViolationSeverity::Critical,
        };
        assert!(!engine.check_invariant(&kvector, &state, 1000).holds);
        state.variables.insert("kvector_k_r".into(), "NaN".into());
        assert!(!engine.check_invariant(&kvector, &state, 1000).holds);
        state.variables.insert("kvector_k_r".into(), "0.5".into());
        assert!(engine.check_invariant(&kvector, &state, 1000).holds);
    }

    #[test]
    fn test_empty_network_cannot_hide_byzantine_count() {
        let engine = VerificationEngine::new();
        let mut state = create_test_state();
        state.trust_scores.clear();
        state.byzantine_count = 1;
        let invariant = Invariant {
            invariant_id: "byzantine".into(),
            name: "Byzantine threshold".into(),
            description: "Byzantine count must be consistent".into(),
            invariant_type: InvariantType::ByzantineTolerance {
                max_byzantine_fraction: 0.34,
            },
            severity: ViolationSeverity::Fatal,
        };
        assert!(!engine.check_invariant(&invariant, &state, 1000).holds);
    }

    #[test]
    fn test_custom_predicate_requires_explicit_true() {
        let engine = VerificationEngine::new();
        let invariant = Invariant {
            invariant_id: "custom".into(),
            name: "Custom predicate".into(),
            description: "Explicit true is required".into(),
            invariant_type: InvariantType::Custom {
                predicate: "predicate_result".into(),
            },
            severity: ViolationSeverity::Critical,
        };
        let mut state = create_test_state();
        assert!(!engine.check_invariant(&invariant, &state, 1000).holds);
        state
            .variables
            .insert("predicate_result".into(), "unknown".into());
        assert!(!engine.check_invariant(&invariant, &state, 1000).holds);
        state
            .variables
            .insert("predicate_result".into(), "true".into());
        assert!(engine.check_invariant(&invariant, &state, 1000).holds);
    }

    #[test]
    fn test_monotonicity_and_conservation_require_complete_evidence() {
        let engine = VerificationEngine::new();
        let state = create_test_state();
        let monotonicity = Invariant {
            invariant_id: "monotonicity".into(),
            name: "Monotonicity".into(),
            description: "Requires before and after values".into(),
            invariant_type: InvariantType::Monotonicity {
                property: "value".into(),
                direction: MonotonicityDirection::NonDecreasing,
            },
            severity: ViolationSeverity::Critical,
        };
        assert!(!engine.check_invariant(&monotonicity, &state, 1000).holds);

        let conservation = Invariant {
            invariant_id: "conservation".into(),
            name: "Conservation".into(),
            description: "Requires current and prior totals".into(),
            invariant_type: InvariantType::Conservation {
                property: "mass".into(),
                tolerance: 1e-9,
            },
            severity: ViolationSeverity::Critical,
        };
        assert!(!engine.check_invariant(&conservation, &state, 1000).holds);
    }

    #[test]
    fn test_comparisons_reject_non_finite_operands() {
        assert!(!CompareOp::Eq.eval(f64::NAN, f64::NAN));
        assert!(!CompareOp::Ne.eval(f64::NAN, 1.0));
        assert!(!CompareOp::Gt.eval(f64::INFINITY, 1.0));
    }

    #[test]
    fn test_violation_severity() {
        assert!(ViolationSeverity::Fatal > ViolationSeverity::Critical);
        assert!(ViolationSeverity::Critical > ViolationSeverity::Error);
        assert!(ViolationSeverity::Error > ViolationSeverity::Warning);
    }

    #[test]
    fn test_monotonicity_invariant() {
        let engine = VerificationEngine::new();

        let invariant = Invariant {
            invariant_id: "mono_test".to_string(),
            name: "Monotonicity Test".to_string(),
            description: "Value must not decrease".to_string(),
            invariant_type: InvariantType::Monotonicity {
                property: "value".to_string(),
                direction: MonotonicityDirection::NonDecreasing,
            },
            severity: ViolationSeverity::Warning,
        };

        let mut state = create_test_state();
        state
            .variables
            .insert("prev_value".to_string(), "0.5".to_string());
        state
            .variables
            .insert("value".to_string(), "0.3".to_string()); // Decreased!

        let result = engine.check_invariant(&invariant, &state, 1000);
        assert!(!result.holds);
    }
}
