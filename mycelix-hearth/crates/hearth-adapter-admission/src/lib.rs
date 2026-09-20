// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

//! Pure, deterministic admission profiles for Hearth edge adapters.
//!
//! HTH-AUTO-004D establishes that an adapter's own `supports(...)` claim is
//! never sufficient execution authority. Exact plan-step semantics must fit one
//! and only one reviewed adapter profile before runtime integration may dispatch.

use hearth_automation_types::{
    AutomationPlan, AutomationValue, ConsequenceClass, PlanStep, Reversibility, StepId,
    BASIS_POINTS_MAX,
};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

pub const ADAPTER_ADMISSION_SCHEMA_VERSION: u16 = 1;
const MAX_ID_LEN: usize = 256;
const MAX_REF_LEN: usize = 1024;
const MAX_TEXT_BOUND: usize = 65_536;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AdapterTransport {
    LocalProcess,
    LocalNetwork,
    RemoteService,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ValueConstraint {
    Bool,
    SignedRange { min: i64, max: i64 },
    UnsignedRange { min: u64, max: u64 },
    BasisPointsRange { min: u32, max: u32 },
    FixedRange {
        min_mantissa: i64,
        max_mantissa: i64,
        scale: u8,
    },
    TextMaxLen { max_len: usize },
    TextEnum { values: BTreeSet<String> },
    TextList {
        max_items: usize,
        max_item_len: usize,
    },
}

impl ValueConstraint {
    fn validate(&self) -> Result<(), AdmissionError> {
        match self {
            Self::SignedRange { min, max } if min > max => {
                Err(AdmissionError::InvalidProfile("signed range is inverted".into()))
            }
            Self::UnsignedRange { min, max } if min > max => {
                Err(AdmissionError::InvalidProfile("unsigned range is inverted".into()))
            }
            Self::BasisPointsRange { min, max }
                if min > max || *max > BASIS_POINTS_MAX =>
            {
                Err(AdmissionError::InvalidProfile(
                    "basis-point range is invalid".into(),
                ))
            }
            Self::FixedRange {
                min_mantissa,
                max_mantissa,
                scale,
            } if min_mantissa > max_mantissa || *scale > 18 => Err(
                AdmissionError::InvalidProfile("fixed-point range is invalid".into()),
            ),
            Self::TextMaxLen { max_len } if *max_len == 0 || *max_len > MAX_TEXT_BOUND => Err(
                AdmissionError::InvalidProfile("text length bound is invalid".into()),
            ),
            Self::TextEnum { values } if values.is_empty() => Err(
                AdmissionError::InvalidProfile("text enum must not be empty".into()),
            ),
            Self::TextEnum { values }
                if values.iter().any(|value| value.trim().is_empty()) =>
            {
                Err(AdmissionError::InvalidProfile(
                    "text enum contains an empty value".into(),
                ))
            }
            Self::TextList {
                max_items,
                max_item_len,
            } if *max_items == 0
                || *max_item_len == 0
                || *max_item_len > MAX_TEXT_BOUND =>
            {
                Err(AdmissionError::InvalidProfile(
                    "text-list bounds are invalid".into(),
                ))
            }
            _ => Ok(()),
        }
    }

    fn admits(&self, value: &AutomationValue) -> bool {
        match (self, value) {
            (Self::Bool, AutomationValue::Bool(_)) => true,
            (Self::SignedRange { min, max }, AutomationValue::Signed(value)) => {
                value >= min && value <= max
            }
            (Self::UnsignedRange { min, max }, AutomationValue::Unsigned(value)) => {
                value >= min && value <= max
            }
            (Self::BasisPointsRange { min, max }, AutomationValue::BasisPoints(value)) => {
                value >= min && value <= max
            }
            (
                Self::FixedRange {
                    min_mantissa,
                    max_mantissa,
                    scale,
                },
                AutomationValue::Fixed {
                    mantissa,
                    scale: value_scale,
                },
            ) => value_scale == scale && mantissa >= min_mantissa && mantissa <= max_mantissa,
            (Self::TextMaxLen { max_len }, AutomationValue::Text(value)) => {
                value.len() <= *max_len
            }
            (Self::TextEnum { values }, AutomationValue::Text(value)) => values.contains(value),
            (
                Self::TextList {
                    max_items,
                    max_item_len,
                },
                AutomationValue::TextList(values),
            ) => {
                values.len() <= *max_items
                    && values.iter().all(|value| value.len() <= *max_item_len)
            }
            _ => false,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ArgumentRule {
    pub key: String,
    pub required: bool,
    pub constraint: ValueConstraint,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ActionRule {
    pub rule_id: String,
    pub capability: String,
    pub operation: String,
    pub target_kinds: BTreeSet<String>,
    pub allow_as_primary: bool,
    pub allow_as_compensation: bool,
    pub minimum_consequence: ConsequenceClass,
    pub maximum_consequence: ConsequenceClass,
    pub allowed_reversibility: Vec<Reversibility>,
    pub max_timeout_ms: u64,
    pub max_attempts: u16,
    pub max_verify_within_ms: u64,
    pub min_confidence_bp: u32,
    pub min_independent_sources: u16,
    pub max_evidence_age_ms: Option<u64>,
    pub arguments: Vec<ArgumentRule>,
    pub allow_extra_arguments: bool,
}

impl ActionRule {
    fn validate(&self) -> Result<(), AdmissionError> {
        validate_id("rule_id", &self.rule_id)?;
        validate_id("capability", &self.capability)?;
        validate_id("operation", &self.operation)?;
        if self.target_kinds.is_empty()
            || self.target_kinds.iter().any(|value| value.trim().is_empty())
        {
            return Err(AdmissionError::InvalidProfile(
                "target_kinds must contain only non-empty values".into(),
            ));
        }
        if !self.allow_as_primary && !self.allow_as_compensation {
            return Err(AdmissionError::InvalidProfile(
                "rule is unusable in both primary and compensation roles".into(),
            ));
        }
        if self.minimum_consequence > self.maximum_consequence {
            return Err(AdmissionError::InvalidProfile(
                "consequence range is inverted".into(),
            ));
        }
        if self.allowed_reversibility.is_empty() {
            return Err(AdmissionError::InvalidProfile(
                "allowed_reversibility must not be empty".into(),
            ));
        }
        for (index, value) in self.allowed_reversibility.iter().enumerate() {
            if self.allowed_reversibility[..index].contains(value) {
                return Err(AdmissionError::InvalidProfile(
                    "allowed_reversibility contains duplicates".into(),
                ));
            }
        }
        if self.max_timeout_ms == 0
            || self.max_attempts == 0
            || self.max_verify_within_ms == 0
            || self.min_independent_sources == 0
            || self.min_confidence_bp > BASIS_POINTS_MAX
            || self.max_evidence_age_ms == Some(0)
        {
            return Err(AdmissionError::InvalidProfile(
                "execution/verification bounds are invalid".into(),
            ));
        }

        let mut keys = BTreeSet::new();
        for argument in &self.arguments {
            validate_id("argument key", &argument.key)?;
            if !keys.insert(argument.key.clone()) {
                return Err(AdmissionError::InvalidProfile(format!(
                    "duplicate argument rule: {}",
                    argument.key
                )));
            }
            argument.constraint.validate()?;
        }
        Ok(())
    }

    fn admits_step_action(
        &self,
        step: &PlanStep,
        action: &hearth_automation_types::ActionSpec,
        role: ActionRole,
    ) -> bool {
        if action.capability != self.capability || action.operation != self.operation {
            return false;
        }
        if !self.target_kinds.contains(&action.target.kind) {
            return false;
        }
        if (role == ActionRole::Primary && !self.allow_as_primary)
            || (role == ActionRole::Compensation && !self.allow_as_compensation)
        {
            return false;
        }
        if step.consequence < self.minimum_consequence
            || step.consequence > self.maximum_consequence
            || !self.allowed_reversibility.contains(&step.reversibility)
            || step.timeout_ms > self.max_timeout_ms
            || step.retry.max_attempts > self.max_attempts
            || step.outcome.verify_within_ms > self.max_verify_within_ms
            || step.outcome.evidence.min_confidence_bp < self.min_confidence_bp
            || step.outcome.evidence.min_independent_sources < self.min_independent_sources
        {
            return false;
        }
        if let Some(max_age) = self.max_evidence_age_ms {
            match step.outcome.evidence.max_age_ms {
                Some(actual) if actual <= max_age => {}
                _ => return false,
            }
        }

        for argument_rule in &self.arguments {
            match action.arguments.get(&argument_rule.key) {
                Some(value) if argument_rule.constraint.admits(value) => {}
                Some(_) => return false,
                None if argument_rule.required => return false,
                None => {}
            }
        }
        if !self.allow_extra_arguments {
            for key in action.arguments.keys() {
                if !self.arguments.iter().any(|rule| rule.key == *key) {
                    return false;
                }
            }
        }
        true
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AdapterProfile {
    pub schema_version: u16,
    pub adapter_id: String,
    pub implementation_ref: String,
    pub profile_ref: String,
    pub transport: AdapterTransport,
    pub rules: Vec<ActionRule>,
}

impl AdapterProfile {
    pub fn validate(&self) -> Result<(), AdmissionError> {
        if self.schema_version != ADAPTER_ADMISSION_SCHEMA_VERSION {
            return Err(AdmissionError::UnsupportedSchema(self.schema_version));
        }
        validate_id("adapter_id", &self.adapter_id)?;
        validate_ref("implementation_ref", &self.implementation_ref)?;
        validate_ref("profile_ref", &self.profile_ref)?;
        if self.rules.is_empty() {
            return Err(AdmissionError::InvalidProfile(
                "adapter profile must contain at least one rule".into(),
            ));
        }

        let mut rule_ids = BTreeSet::new();
        let mut semantic_keys = BTreeSet::new();
        for rule in &self.rules {
            rule.validate()?;
            if !rule_ids.insert(rule.rule_id.clone()) {
                return Err(AdmissionError::InvalidProfile(format!(
                    "duplicate rule_id: {}",
                    rule.rule_id
                )));
            }
            if !semantic_keys.insert((rule.capability.clone(), rule.operation.clone())) {
                return Err(AdmissionError::InvalidProfile(format!(
                    "duplicate capability/operation rule: {}/{}",
                    rule.capability, rule.operation
                )));
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ActionRole {
    Primary,
    Compensation,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AdapterAdmission {
    pub adapter_id: String,
    pub implementation_ref: String,
    pub profile_ref: String,
    pub rule_id: String,
    pub role: ActionRole,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct StepAdapterAdmission {
    pub step_id: StepId,
    pub primary: AdapterAdmission,
    pub compensation: Option<AdapterAdmission>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PlanAdapterAdmission {
    pub plan_id: String,
    pub steps: Vec<StepAdapterAdmission>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AdmissionError {
    UnsupportedSchema(u16),
    InvalidProfile(String),
    InvalidStep(String),
    InvalidPlan(String),
    NoQualifiedAdapter {
        step_id: String,
        role: ActionRole,
        capability: String,
        operation: String,
        target_kind: String,
    },
    AmbiguousQualifiedAdapters {
        step_id: String,
        role: ActionRole,
        adapter_ids: Vec<String>,
    },
}

impl std::fmt::Display for AdmissionError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{self:?}")
    }
}
impl std::error::Error for AdmissionError {}

pub fn admit_step(
    profiles: &[AdapterProfile],
    step: &PlanStep,
) -> Result<StepAdapterAdmission, AdmissionError> {
    step.validate("step")
        .map_err(|error| AdmissionError::InvalidStep(error.to_string()))?;
    for profile in profiles {
        profile.validate()?;
    }

    let primary = select_adapter(profiles, step, &step.action, ActionRole::Primary)?;
    let compensation = step
        .compensation
        .as_ref()
        .map(|action| select_adapter(profiles, step, action, ActionRole::Compensation))
        .transpose()?;

    Ok(StepAdapterAdmission {
        step_id: step.id.clone(),
        primary,
        compensation,
    })
}

pub fn admit_plan(
    profiles: &[AdapterProfile],
    plan: &AutomationPlan,
) -> Result<PlanAdapterAdmission, AdmissionError> {
    plan.validate()
        .map_err(|error| AdmissionError::InvalidPlan(error.to_string()))?;
    for profile in profiles {
        profile.validate()?;
    }

    let mut steps = Vec::with_capacity(plan.steps.len());
    for step in &plan.steps {
        steps.push(admit_step(profiles, step)?);
    }
    Ok(PlanAdapterAdmission {
        plan_id: plan.id.0.clone(),
        steps,
    })
}

fn select_adapter(
    profiles: &[AdapterProfile],
    step: &PlanStep,
    action: &hearth_automation_types::ActionSpec,
    role: ActionRole,
) -> Result<AdapterAdmission, AdmissionError> {
    let mut matches = Vec::new();
    for profile in profiles {
        if let Some(rule) = profile
            .rules
            .iter()
            .find(|rule| rule.capability == action.capability && rule.operation == action.operation)
            && rule.admits_step_action(step, action, role)
        {
            matches.push(AdapterAdmission {
                adapter_id: profile.adapter_id.clone(),
                implementation_ref: profile.implementation_ref.clone(),
                profile_ref: profile.profile_ref.clone(),
                rule_id: rule.rule_id.clone(),
                role,
            });
        }
    }

    matches.sort_by(|left, right| {
        (&left.adapter_id, &left.profile_ref, &left.rule_id).cmp(&(
            &right.adapter_id,
            &right.profile_ref,
            &right.rule_id,
        ))
    });

    match matches.len() {
        0 => Err(AdmissionError::NoQualifiedAdapter {
            step_id: step.id.0.clone(),
            role,
            capability: action.capability.clone(),
            operation: action.operation.clone(),
            target_kind: action.target.kind.clone(),
        }),
        1 => Ok(matches.remove(0)),
        _ => Err(AdmissionError::AmbiguousQualifiedAdapters {
            step_id: step.id.0.clone(),
            role,
            adapter_ids: matches
                .into_iter()
                .map(|value| value.adapter_id)
                .collect(),
        }),
    }
}

fn validate_id(field: &str, value: &str) -> Result<(), AdmissionError> {
    if value.trim().is_empty() || value.len() > MAX_ID_LEN {
        return Err(AdmissionError::InvalidProfile(format!(
            "{field} must be non-empty and <= {MAX_ID_LEN} bytes"
        )));
    }
    Ok(())
}

fn validate_ref(field: &str, value: &str) -> Result<(), AdmissionError> {
    if value.trim().is_empty() || value.len() > MAX_REF_LEN {
        return Err(AdmissionError::InvalidProfile(format!(
            "{field} must be non-empty and <= {MAX_REF_LEN} bytes"
        )));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use hearth_automation_types::{
        ActionSpec, AuthorityRequirement, ComparisonOp, ConditionExpr, EntityRef,
        EvidenceRequirement, OutcomeExpectation, OutcomePolicy, RetryPolicy, StateExpectation,
        UnverifiedDisposition,
    };
    use std::collections::BTreeMap;

    fn action() -> ActionSpec {
        ActionSpec {
            capability: "home.hvac.control".into(),
            target: EntityRef {
                kind: "thermostat".into(),
                id: "hall".into(),
            },
            operation: "set_temperature".into(),
            arguments: BTreeMap::from([(
                "temperature_tenths_c".into(),
                AutomationValue::Signed(215),
            )]),
        }
    }

    fn step() -> PlanStep {
        let target = EntityRef {
            kind: "thermostat".into(),
            id: "hall".into(),
        };
        PlanStep {
            id: StepId("step:hvac".into()),
            depends_on: Vec::new(),
            preconditions: ConditionExpr::Always,
            action: action(),
            consequence: ConsequenceClass::ReversibleAct,
            reversibility: Reversibility::Reversible,
            authority: AuthorityRequirement::Capability {
                capability: "home.hvac.control".into(),
            },
            outcome: OutcomePolicy {
                expectations: vec![OutcomeExpectation::State(StateExpectation {
                    subject: target,
                    attribute: "temperature_tenths_c".into(),
                    op: ComparisonOp::Eq,
                    expected: AutomationValue::Signed(215),
                })],
                verify_within_ms: 5_000,
                evidence: EvidenceRequirement {
                    max_age_ms: Some(2_000),
                    min_confidence_bp: 9_500,
                    min_independent_sources: 1,
                    required_namespaces: Vec::new(),
                },
                on_unverified: UnverifiedDisposition::StopAndNotify,
            },
            timeout_ms: 2_000,
            retry: RetryPolicy {
                max_attempts: 2,
                initial_backoff_ms: 100,
                max_backoff_ms: 500,
            },
            compensation: None,
        }
    }

    fn profile(adapter_id: &str) -> AdapterProfile {
        AdapterProfile {
            schema_version: ADAPTER_ADMISSION_SCHEMA_VERSION,
            adapter_id: adapter_id.into(),
            implementation_ref: format!("nix:adapter:{adapter_id}:sha256:test"),
            profile_ref: format!("profile:{adapter_id}:v1"),
            transport: AdapterTransport::LocalNetwork,
            rules: vec![ActionRule {
                rule_id: "hvac.set-temperature".into(),
                capability: "home.hvac.control".into(),
                operation: "set_temperature".into(),
                target_kinds: BTreeSet::from(["thermostat".into()]),
                allow_as_primary: true,
                allow_as_compensation: false,
                minimum_consequence: ConsequenceClass::ReversibleAct,
                maximum_consequence: ConsequenceClass::ReversibleAct,
                allowed_reversibility: vec![Reversibility::Reversible],
                max_timeout_ms: 5_000,
                max_attempts: 2,
                max_verify_within_ms: 10_000,
                min_confidence_bp: 9_000,
                min_independent_sources: 1,
                max_evidence_age_ms: Some(5_000),
                arguments: vec![ArgumentRule {
                    key: "temperature_tenths_c".into(),
                    required: true,
                    constraint: ValueConstraint::SignedRange { min: 150, max: 300 },
                }],
                allow_extra_arguments: false,
            }],
        }
    }

    #[test]
    fn exactly_one_matching_profile_is_admitted() {
        let admitted = admit_step(&[profile("home-assistant")], &step()).unwrap();
        assert_eq!(admitted.primary.adapter_id, "home-assistant");
        assert_eq!(admitted.primary.rule_id, "hvac.set-temperature");
    }

    #[test]
    fn registration_order_never_breaks_adapter_ambiguity() {
        let error = admit_step(&[profile("matter"), profile("home-assistant")], &step())
            .unwrap_err();
        assert!(matches!(
            error,
            AdmissionError::AmbiguousQualifiedAdapters { .. }
        ));
    }

    #[test]
    fn underclassified_dangerous_operation_is_rejected() {
        let mut strict = profile("home-assistant");
        strict.rules[0].minimum_consequence = ConsequenceClass::CriticalAct;
        strict.rules[0].maximum_consequence = ConsequenceClass::CriticalAct;
        assert!(matches!(
            admit_step(&[strict], &step()),
            Err(AdmissionError::NoQualifiedAdapter { .. })
        ));
    }

    #[test]
    fn out_of_range_argument_is_rejected() {
        let mut invalid = step();
        invalid.action.arguments.insert(
            "temperature_tenths_c".into(),
            AutomationValue::Signed(450),
        );
        assert!(matches!(
            admit_step(&[profile("home-assistant")], &invalid),
            Err(AdmissionError::NoQualifiedAdapter { .. })
        ));
    }

    #[test]
    fn unknown_argument_is_rejected_by_default() {
        let mut invalid = step();
        invalid
            .action
            .arguments
            .insert("vendor_magic".into(), AutomationValue::Bool(true));
        assert!(matches!(
            admit_step(&[profile("home-assistant")], &invalid),
            Err(AdmissionError::NoQualifiedAdapter { .. })
        ));
    }

    #[test]
    fn unbounded_freshness_cannot_satisfy_bounded_profile() {
        let mut stale = step();
        stale.outcome.evidence.max_age_ms = None;
        assert!(matches!(
            admit_step(&[profile("home-assistant")], &stale),
            Err(AdmissionError::NoQualifiedAdapter { .. })
        ));
    }

    #[test]
    fn implementation_and_profile_identity_are_preserved_in_admission() {
        let profile = profile("matter");
        let admitted = admit_step(&[profile.clone()], &step()).unwrap();
        assert_eq!(admitted.primary.implementation_ref, profile.implementation_ref);
        assert_eq!(admitted.primary.profile_ref, profile.profile_ref);
    }
}
