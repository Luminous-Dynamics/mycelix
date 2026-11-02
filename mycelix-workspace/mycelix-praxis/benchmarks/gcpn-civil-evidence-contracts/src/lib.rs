//! Authority-aware civil-protection evidence contracts.
//!
//! This companion contract layer consumes the existing compositional audit
//! but preserves separate claims for authority, channel delivery, accessible
//! presentation, comprehension, action execution and case-frame accounting.
//! It does not issue alerts, infer legal authority from signatures, identify
//! individual recipients, or equate message delivery with human action.

use std::collections::{BTreeMap, BTreeSet};

use gcpn_compositional_assurance::{
    AuditReport, ClaimDisposition, ClaimResult, Stage,
};

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BaseClaimBinding {
    pub stage: Stage,
    pub claim_id: String,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SignatureStatus {
    Verified,
    Unverified,
    Invalid,
}

impl SignatureStatus {
    pub const fn label(self) -> &'static str {
        match self {
            Self::Verified => "verified",
            Self::Unverified => "unverified",
            Self::Invalid => "invalid",
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum RegistryAuthorityStatus {
    Recognized,
    Unknown,
    Disputed,
    Revoked,
}

impl RegistryAuthorityStatus {
    pub const fn label(self) -> &'static str {
        match self {
            Self::Recognized => "recognized",
            Self::Unknown => "unknown",
            Self::Disputed => "disputed",
            Self::Revoked => "revoked",
        }
    }
}

/// Exact identity/scope that must be bound to the warning and decision.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AuthorityRequirement {
    pub authority_id: String,
    pub jurisdiction_id: String,
    pub hazard_class_id: String,
    pub cap_originator: String,
    pub cap_message_id: String,
    pub cap_version: String,
    pub decision_id: String,
    pub event_time: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AuthorityAssertion {
    pub assertion_id: String,
    pub authority_id: String,
    pub jurisdiction_id: String,
    pub hazard_class_id: String,
    pub cap_originator: String,
    pub cap_message_id: String,
    pub cap_version: String,
    pub decision_id: String,
    pub valid_from: u64,
    /// Exclusive upper bound.
    pub valid_until: u64,
    pub recorded_at: u64,
    pub signature_status: SignatureStatus,
    /// This is a declared authority-registry state, not a legal opinion.
    pub registry_status: RegistryAuthorityStatus,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DeliveryState {
    ProviderAccepted,
    DeviceDelivered,
    Failed,
    Delayed,
    Unknown,
    Duplicate,
}

impl DeliveryState {
    pub const fn label(self) -> &'static str {
        match self {
            Self::ProviderAccepted => "provider_accepted",
            Self::DeviceDelivered => "device_delivered",
            Self::Failed => "failed",
            Self::Delayed => "delayed",
            Self::Unknown => "unknown",
            Self::Duplicate => "duplicate",
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ChannelCohortTarget {
    pub cohort_id: String,
    pub channel_id: String,
    /// Intended unique members of this cohort/channel target.
    /// Cross-channel unique-person reach must be deduplicated by a separate
    /// privacy-preserving reach contract and is not inferred here.
    pub intended_receivers: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DeliveryReceipt {
    pub receipt_id: String,
    pub message_id: String,
    pub message_version: String,
    pub cohort_id: String,
    pub channel_id: String,
    pub state: DeliveryState,
    /// Aggregate unique recipients for this cohort/channel/state.
    pub count: u64,
    pub duplicate_of: Option<String>,
    pub recorded_at: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AccessibilityState {
    Accessible,
    NotAccessible,
    Unknown,
}

impl AccessibilityState {
    pub const fn label(self) -> &'static str {
        match self {
            Self::Accessible => "accessible",
            Self::NotAccessible => "not_accessible",
            Self::Unknown => "unknown",
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AccessibilityRequirement {
    pub cohort_id: String,
    pub language_id: String,
    pub format_id: String,
    pub assistive_technology_profile_id: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AccessibilityEvidence {
    pub cohort_id: String,
    pub language_id: String,
    pub format_id: String,
    pub assistive_technology_profile_id: String,
    pub test_protocol_id: String,
    pub state: AccessibilityState,
    pub tested_at: u64,
    pub recorded_at: u64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ComprehensionRequirement {
    pub cohort_id: String,
    pub language_id: String,
    pub test_protocol_id: String,
    pub minimum_comprehension_rate: f64,
    pub minimum_intent_rate: f64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ComprehensionEvidence {
    pub cohort_id: String,
    pub language_id: String,
    pub test_protocol_id: String,
    pub tested_count: u64,
    pub understood_count: u64,
    /// Count of tested participants who correctly identify the intended action.
    pub intended_action_count: u64,
    pub tested_at: u64,
    pub recorded_at: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ActionState {
    Approved,
    Dispatched,
    Started,
    Completed,
    Failed,
    Cancelled,
    Unknown,
}

impl ActionState {
    pub const fn label(self) -> &'static str {
        match self {
            Self::Approved => "approved",
            Self::Dispatched => "dispatched",
            Self::Started => "started",
            Self::Completed => "completed",
            Self::Failed => "failed",
            Self::Cancelled => "cancelled",
            Self::Unknown => "unknown",
        }
    }

    fn allows_transition_to(self, next: Self) -> bool {
        if matches!(self, Self::Unknown) || self == next {
            return true;
        }
        match self {
            Self::Approved => matches!(next, Self::Dispatched | Self::Failed | Self::Cancelled),
            Self::Dispatched => matches!(next, Self::Started | Self::Failed | Self::Cancelled),
            Self::Started => matches!(next, Self::Completed | Self::Failed | Self::Cancelled),
            Self::Completed | Self::Failed | Self::Cancelled => false,
            Self::Unknown => true,
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ActionRequirement {
    pub action_id: String,
    pub decision_id: String,
    pub authority_assertion_id: String,
    pub executor_agency_id: String,
    pub action_type_id: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ActionReceipt {
    pub receipt_id: String,
    pub action_id: String,
    pub decision_id: String,
    pub authority_assertion_id: String,
    pub executor_agency_id: String,
    pub action_type_id: String,
    pub state: ActionState,
    pub event_time: u64,
    pub recorded_at: u64,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CaseDisposition {
    Observed,
    Missing,
    Disputed,
    Excluded,
}

impl CaseDisposition {
    pub const fn label(self) -> &'static str {
        match self {
            Self::Observed => "observed",
            Self::Missing => "missing",
            Self::Disputed => "disputed",
            Self::Excluded => "excluded",
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FrameCase {
    pub case_key: String,
    pub disposition: CaseDisposition,
    pub recorded_at: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CaseFrame {
    pub frame_id: String,
    pub inclusion_criteria_id: String,
    pub declared_eligible_cases: u64,
    pub cases: Vec<FrameCase>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CivilEvidenceProfile {
    pub scenario_id: String,
    pub as_of_cutoff: u64,
    pub base_audit: AuditReport,
    pub required_base_claims: Vec<BaseClaimBinding>,
    pub authority_requirement: AuthorityRequirement,
    pub authority_assertion: Option<AuthorityAssertion>,
    pub delivery_message_id: String,
    pub delivery_message_version: String,
    pub delivery_targets: Vec<ChannelCohortTarget>,
    pub delivery_receipts: Vec<DeliveryReceipt>,
    pub accessibility_requirements: Vec<AccessibilityRequirement>,
    pub accessibility_evidence: Vec<AccessibilityEvidence>,
    pub comprehension_requirements: Vec<ComprehensionRequirement>,
    pub comprehension_evidence: Vec<ComprehensionEvidence>,
    pub action_requirements: Vec<ActionRequirement>,
    pub action_receipts: Vec<ActionReceipt>,
    pub case_frame: CaseFrame,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum CivilViolation {
    BaseAuditCutoffMismatch,
    BaseClaimMissing,
    BaseClaimNotSupporting,
    AuthorityMissing,
    AuthoritySignatureNotVerified,
    AuthorityRegistryUnknown,
    AuthorityRegistryDisputed,
    AuthorityRevoked,
    AuthorityScopeMismatch,
    AuthorityMessageBindingMismatch,
    AuthorityDecisionBindingMismatch,
    AuthorityNotYetValid,
    AuthorityExpired,
    AuthorityValidityIntervalInvalid,
    AuthorityRecordedAfterCutoff,
    DeliveryTargetMissing,
    DeliveryReceiptIdentityMismatch,
    DeliveryReceiptAfterCutoff,
    DeliveryCountExceedsTarget,
    DeviceDeliveryExceedsProviderAcceptance,
    DuplicateReceiptWithoutSource,
    DuplicateReceiptSourceMissing,
    AccessibilityEvidenceMissing,
    AccessibilityEvidenceAfterCutoff,
    AccessibilityNotSupported,
    ComprehensionEvidenceMissing,
    ComprehensionEvidenceAfterCutoff,
    InvalidComprehensionCounts,
    ComprehensionBelowThreshold,
    ActionReceiptMissing,
    ActionReceiptBindingMismatch,
    ActionReceiptAfterCutoff,
    ActionTransitionRegression,
    ActionNotCompleted,
    CaseFrameMissing,
    DuplicateCaseKey,
    CaseFrameAfterCutoff,
    CaseFrameDenominatorMismatch,
    CaseFrameMissingOutcomes,
    CaseFrameDisputedOutcomes,
}

impl CivilViolation {
    pub const fn label(self) -> &'static str {
        match self {
            Self::BaseAuditCutoffMismatch => "base_audit_cutoff_mismatch",
            Self::BaseClaimMissing => "base_claim_missing",
            Self::BaseClaimNotSupporting => "base_claim_not_supporting",
            Self::AuthorityMissing => "authority_missing",
            Self::AuthoritySignatureNotVerified => "authority_signature_not_verified",
            Self::AuthorityRegistryUnknown => "authority_registry_unknown",
            Self::AuthorityRegistryDisputed => "authority_registry_disputed",
            Self::AuthorityRevoked => "authority_revoked",
            Self::AuthorityScopeMismatch => "authority_scope_mismatch",
            Self::AuthorityMessageBindingMismatch => "authority_message_binding_mismatch",
            Self::AuthorityDecisionBindingMismatch => "authority_decision_binding_mismatch",
            Self::AuthorityNotYetValid => "authority_not_yet_valid",
            Self::AuthorityExpired => "authority_expired",
            Self::AuthorityValidityIntervalInvalid => "authority_validity_interval_invalid",
            Self::AuthorityRecordedAfterCutoff => "authority_recorded_after_cutoff",
            Self::DeliveryTargetMissing => "delivery_target_missing",
            Self::DeliveryReceiptIdentityMismatch => "delivery_receipt_identity_mismatch",
            Self::DeliveryReceiptAfterCutoff => "delivery_receipt_after_cutoff",
            Self::DeliveryCountExceedsTarget => "delivery_count_exceeds_target",
            Self::DeviceDeliveryExceedsProviderAcceptance => "device_delivery_exceeds_provider_acceptance",
            Self::DuplicateReceiptWithoutSource => "duplicate_receipt_without_source",
            Self::DuplicateReceiptSourceMissing => "duplicate_receipt_source_missing",
            Self::AccessibilityEvidenceMissing => "accessibility_evidence_missing",
            Self::AccessibilityEvidenceAfterCutoff => "accessibility_evidence_after_cutoff",
            Self::AccessibilityNotSupported => "accessibility_not_supported",
            Self::ComprehensionEvidenceMissing => "comprehension_evidence_missing",
            Self::ComprehensionEvidenceAfterCutoff => "comprehension_evidence_after_cutoff",
            Self::InvalidComprehensionCounts => "invalid_comprehension_counts",
            Self::ComprehensionBelowThreshold => "comprehension_below_threshold",
            Self::ActionReceiptMissing => "action_receipt_missing",
            Self::ActionReceiptBindingMismatch => "action_receipt_binding_mismatch",
            Self::ActionReceiptAfterCutoff => "action_receipt_after_cutoff",
            Self::ActionTransitionRegression => "action_transition_regression",
            Self::ActionNotCompleted => "action_not_completed",
            Self::CaseFrameMissing => "case_frame_missing",
            Self::DuplicateCaseKey => "duplicate_case_key",
            Self::CaseFrameAfterCutoff => "case_frame_after_cutoff",
            Self::CaseFrameDenominatorMismatch => "case_frame_denominator_mismatch",
            Self::CaseFrameMissingOutcomes => "case_frame_missing_outcomes",
            Self::CaseFrameDisputedOutcomes => "case_frame_disputed_outcomes",
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CivilEvidenceError {
    InvalidIdentifier,
    DuplicateReceiptId(String),
    DuplicateRequirement(String),
    DuplicateCohortChannel(String),
    DuplicateActionId(String),
    DuplicateAccessibilityKey(String),
    DuplicateComprehensionKey(String),
    DuplicateCaseKey(String),
    InvalidThreshold(String),
    InvalidDeliveryCount(String),
    InvalidComprehensionCount(String),
    InvalidActionSequence(String),
    MissingRequiredClaims,
}

impl std::fmt::Display for CivilEvidenceError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidIdentifier => write!(formatter, "identifier is not canonical"),
            Self::DuplicateReceiptId(value) => write!(formatter, "duplicate receipt ID: {value}"),
            Self::DuplicateRequirement(value) => write!(formatter, "duplicate requirement: {value}"),
            Self::DuplicateCohortChannel(value) => write!(formatter, "duplicate cohort/channel target: {value}"),
            Self::DuplicateActionId(value) => write!(formatter, "duplicate action requirement: {value}"),
            Self::DuplicateAccessibilityKey(value) => write!(formatter, "duplicate accessibility key: {value}"),
            Self::DuplicateComprehensionKey(value) => write!(formatter, "duplicate comprehension key: {value}"),
            Self::DuplicateCaseKey(value) => write!(formatter, "duplicate case key: {value}"),
            Self::InvalidThreshold(value) => write!(formatter, "invalid comprehension threshold: {value}"),
            Self::InvalidDeliveryCount(value) => write!(formatter, "invalid delivery count for {value}"),
            Self::InvalidComprehensionCount(value) => write!(formatter, "invalid comprehension count for {value}"),
            Self::InvalidActionSequence(value) => write!(formatter, "invalid action sequence for {value}"),
            Self::MissingRequiredClaims => write!(formatter, "base audit claim bindings must not be empty"),
        }
    }
}

impl std::error::Error for CivilEvidenceError {}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BaseAuditAssessment {
    pub disposition: ClaimDisposition,
    pub required_claims: usize,
    pub supporting_claims: usize,
    pub violations: Vec<CivilViolation>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AuthorityAssessment {
    pub disposition: ClaimDisposition,
    pub assertion_id: Option<String>,
    pub registry_status: Option<RegistryAuthorityStatus>,
    pub signature_status: Option<SignatureStatus>,
    pub violations: Vec<CivilViolation>,
}

#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct DeliveryCounts {
    pub intended_receivers: u64,
    pub provider_accepted: u64,
    pub device_delivered: u64,
    pub failed: u64,
    pub delayed: u64,
    pub unknown: u64,
    pub duplicates: u64,
    pub not_confirmed_device_delivered: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DeliveryAssessment {
    pub cohort_id: String,
    pub channel_id: String,
    pub disposition: ClaimDisposition,
    pub counts: DeliveryCounts,
    pub violations: Vec<CivilViolation>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AccessibilityAssessment {
    pub cohort_id: String,
    pub language_id: String,
    pub format_id: String,
    pub disposition: ClaimDisposition,
    pub state: Option<AccessibilityState>,
    pub violations: Vec<CivilViolation>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ComprehensionAssessment {
    pub cohort_id: String,
    pub language_id: String,
    pub disposition: ClaimDisposition,
    pub tested_count: u64,
    pub comprehension_rate: Option<f64>,
    pub intended_action_rate: Option<f64>,
    pub violations: Vec<CivilViolation>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ActionAssessment {
    pub action_id: String,
    pub disposition: ClaimDisposition,
    pub latest_state: Option<ActionState>,
    pub receipt_count: usize,
    pub violations: Vec<CivilViolation>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CaseFrameAssessment {
    pub frame_id: String,
    pub disposition: ClaimDisposition,
    pub eligible_cases: u64,
    pub observed_cases: u64,
    pub missing_cases: u64,
    pub disputed_cases: u64,
    pub excluded_cases: u64,
    pub violations: Vec<CivilViolation>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct CivilEvidenceReport {
    pub scenario_id: String,
    pub as_of_cutoff: u64,
    pub base_audit: BaseAuditAssessment,
    pub authority: AuthorityAssessment,
    pub delivery: Vec<DeliveryAssessment>,
    pub accessibility: Vec<AccessibilityAssessment>,
    pub comprehension: Vec<ComprehensionAssessment>,
    pub actions: Vec<ActionAssessment>,
    pub case_frame: CaseFrameAssessment,
}

impl CivilEvidenceReport {
    /// Stable, non-compensatory, per-claim CSV. There is deliberately no
    /// aggregate civil-protection score.
    pub fn to_csv(&self) -> String {
        let mut output = String::from(
            "scenario_id,claim_family,claim_key,qualifier,disposition,observed_value,secondary_value,violations\n",
        );

        output.push_str(&format!(
            "{},base_audit,required_claims,{}, {},{},{},{}\n",
            self.scenario_id,
            self.base_audit.required_claims,
            label(self.base_audit.disposition),
            self.base_audit.supporting_claims,
            self.base_audit.required_claims.saturating_sub(self.base_audit.supporting_claims),
            violation_labels(&self.base_audit.violations)
        ));

        let authority_registry = self.authority.registry_status
            .map(RegistryAuthorityStatus::label)
            .unwrap_or("missing");
        let authority_signature = self.authority.signature_status
            .map(SignatureStatus::label)
            .unwrap_or("missing");
        output.push_str(&format!(
            "{},authority,{},registry={},{},{},{},{}\n",
            self.scenario_id,
            self.authority.assertion_id.as_deref().unwrap_or("missing"),
            authority_registry,
            label(self.authority.disposition),
            authority_signature,
            "signature_status_only_not_legal_authority",
            violation_labels(&self.authority.violations)
        ));

        for delivery in &self.delivery {
            output.push_str(&format!(
                "{},delivery,{},channel={},{},{},accepted={};not_confirmed_delivered={},{}\n",
                self.scenario_id,
                delivery.cohort_id,
                delivery.channel_id,
                label(delivery.disposition),
                delivery.counts.device_delivered,
                delivery.counts.provider_accepted,
                delivery.counts.not_confirmed_device_delivered,
                violation_labels(&delivery.violations)
            ));
        }

        for accessibility in &self.accessibility {
            output.push_str(&format!(
                "{},accessibility,{},{}:{},{},{},{},{}\n",
                self.scenario_id,
                accessibility.cohort_id,
                accessibility.language_id,
                accessibility.format_id,
                label(accessibility.disposition),
                accessibility.state.map(AccessibilityState::label).unwrap_or("missing"),
                "",
                violation_labels(&accessibility.violations)
            ));
        }

        for comprehension in &self.comprehension {
            output.push_str(&format!(
                "{},comprehension,{},language={},{},{:.4},{:.4},{}\n",
                self.scenario_id,
                comprehension.cohort_id,
                comprehension.language_id,
                label(comprehension.disposition),
                comprehension.comprehension_rate.unwrap_or(-1.0),
                comprehension.intended_action_rate.unwrap_or(-1.0),
                violation_labels(&comprehension.violations)
            ));
        }

        for action in &self.actions {
            output.push_str(&format!(
                "{},action,{},latest_state={},{},{},{},{}\n",
                self.scenario_id,
                action.action_id,
                action.latest_state.map(ActionState::label).unwrap_or("missing"),
                label(action.disposition),
                action.receipt_count,
                "",
                violation_labels(&action.violations)
            ));
        }

        output.push_str(&format!(
            "{},case_frame,{},eligibility=observed+missing+disputed,{},{},{},{}\n",
            self.scenario_id,
            self.case_frame.frame_id,
            label(self.case_frame.disposition),
            self.case_frame.observed_cases,
            format!(
                "missing={};disputed={};excluded={}",
                self.case_frame.missing_cases,
                self.case_frame.disputed_cases,
                self.case_frame.excluded_cases
            ),
            violation_labels(&self.case_frame.violations)
        ));
        output
    }
}

fn label(disposition: ClaimDisposition) -> &'static str {
    match disposition {
        ClaimDisposition::Supports => "supports",
        ClaimDisposition::Refutes => "refutes",
        ClaimDisposition::Indeterminate => "indeterminate",
        ClaimDisposition::OutOfScope => "out_of_scope",
        ClaimDisposition::Unavailable => "unavailable",
    }
}

fn violation_labels(violations: &[CivilViolation]) -> String {
    violations.iter().map(|violation| violation.label()).collect::<Vec<_>>().join("|")
}

fn valid_identifier(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 256
        && value.bytes().all(|byte| byte.is_ascii_alphanumeric() || b"-_.:".contains(&byte))
}

fn check_identifier(value: &str) -> Result<(), CivilEvidenceError> {
    if valid_identifier(value) {
        Ok(())
    } else {
        Err(CivilEvidenceError::InvalidIdentifier)
    }
}

pub fn audit_civil_evidence(
    profile: &CivilEvidenceProfile,
) -> Result<CivilEvidenceReport, CivilEvidenceError> {
    validate_profile_identifiers(profile)?;
    let base_audit = assess_base_audit(profile);
    let authority = assess_authority(profile);
    let delivery = assess_delivery(profile)?;
    let accessibility = assess_accessibility(profile);
    let comprehension = assess_comprehension(profile)?;
    let actions = assess_actions(profile)?;
    let case_frame = assess_case_frame(profile)?;

    Ok(CivilEvidenceReport {
        scenario_id: profile.scenario_id.clone(),
        as_of_cutoff: profile.as_of_cutoff,
        base_audit,
        authority,
        delivery,
        accessibility,
        comprehension,
        actions,
        case_frame,
    })
}

fn validate_profile_identifiers(profile: &CivilEvidenceProfile) -> Result<(), CivilEvidenceError> {
    if profile.authority_assertion.is_some() {
        let assertion = profile.authority_assertion.as_ref().expect("checked above");
        for value in [
            assertion.assertion_id.as_str(),
            assertion.authority_id.as_str(),
            assertion.jurisdiction_id.as_str(),
            assertion.hazard_class_id.as_str(),
            assertion.cap_originator.as_str(),
            assertion.cap_message_id.as_str(),
            assertion.cap_version.as_str(),
            assertion.decision_id.as_str(),
        ] {
            check_identifier(value)?;
        }
    }
    for value in [
        profile.scenario_id.as_str(),
        profile.delivery_message_id.as_str(),
        profile.delivery_message_version.as_str(),
        profile.authority_requirement.authority_id.as_str(),
        profile.authority_requirement.jurisdiction_id.as_str(),
        profile.authority_requirement.hazard_class_id.as_str(),
        profile.authority_requirement.cap_originator.as_str(),
        profile.authority_requirement.cap_message_id.as_str(),
        profile.authority_requirement.cap_version.as_str(),
        profile.authority_requirement.decision_id.as_str(),
        profile.case_frame.frame_id.as_str(),
        profile.case_frame.inclusion_criteria_id.as_str(),
    ] {
        check_identifier(value)?;
    }
    if profile.required_base_claims.is_empty() {
        return Err(CivilEvidenceError::MissingRequiredClaims);
    }
    if profile.delivery_targets.is_empty() {
        return Err(CivilEvidenceError::InvalidDeliveryCount(
            "at least one intended cohort/channel target is required".into(),
        ));
    }
    let mut base_claim_keys = BTreeSet::new();
    for claim in &profile.required_base_claims {
        if !base_claim_keys.insert((claim.stage, claim.claim_id.as_str())) {
            return Err(CivilEvidenceError::DuplicateRequirement(claim.claim_id.clone()));
        }
    }
    for claim in &profile.required_base_claims {
        check_identifier(&claim.claim_id)?;
    }
    for target in &profile.delivery_targets {
        check_identifier(&target.cohort_id)?;
        check_identifier(&target.channel_id)?;
    }
    for receipt in &profile.delivery_receipts {
        for value in [
            receipt.receipt_id.as_str(),
            receipt.message_id.as_str(),
            receipt.message_version.as_str(),
            receipt.cohort_id.as_str(),
            receipt.channel_id.as_str(),
        ] {
            check_identifier(value)?;
        }
        if let Some(duplicate_of) = &receipt.duplicate_of {
            check_identifier(duplicate_of)?;
        }
    }
    let mut accessibility_keys = BTreeSet::new();
    for requirement in &profile.accessibility_requirements {
        let key = (
            requirement.cohort_id.as_str(),
            requirement.language_id.as_str(),
            requirement.format_id.as_str(),
            requirement.assistive_technology_profile_id.as_str(),
        );
        if !accessibility_keys.insert(key) {
            return Err(CivilEvidenceError::DuplicateAccessibilityKey(format!(
                "{}:{}:{}:{}",
                requirement.cohort_id,
                requirement.language_id,
                requirement.format_id,
                requirement.assistive_technology_profile_id
            )));
        }
        check_identifier(&requirement.cohort_id)?;
        check_identifier(&requirement.language_id)?;
        check_identifier(&requirement.format_id)?;
        check_identifier(&requirement.assistive_technology_profile_id)?;
    }
    let mut comprehension_keys = BTreeSet::new();
    for requirement in &profile.comprehension_requirements {
        let key = (
            requirement.cohort_id.as_str(),
            requirement.language_id.as_str(),
            requirement.test_protocol_id.as_str(),
        );
        if !comprehension_keys.insert(key) {
            return Err(CivilEvidenceError::DuplicateComprehensionKey(format!(
                "{}:{}:{}",
                requirement.cohort_id,
                requirement.language_id,
                requirement.test_protocol_id
            )));
        }
        check_identifier(&requirement.cohort_id)?;
        check_identifier(&requirement.language_id)?;
        check_identifier(&requirement.test_protocol_id)?;
        if !requirement.minimum_comprehension_rate.is_finite()
            || !requirement.minimum_intent_rate.is_finite()
            || !(0.0..=1.0).contains(&requirement.minimum_comprehension_rate)
            || !(0.0..=1.0).contains(&requirement.minimum_intent_rate)
        {
            return Err(CivilEvidenceError::InvalidThreshold(requirement.cohort_id.clone()));
        }
    }
    for requirement in &profile.action_requirements {
        for value in [
            requirement.action_id.as_str(),
            requirement.decision_id.as_str(),
            requirement.authority_assertion_id.as_str(),
            requirement.executor_agency_id.as_str(),
            requirement.action_type_id.as_str(),
        ] {
            check_identifier(value)?;
        }
    }
    let mut accessibility_evidence_keys = BTreeSet::new();
    for evidence in &profile.accessibility_evidence {
        let key = (
            evidence.cohort_id.as_str(),
            evidence.language_id.as_str(),
            evidence.format_id.as_str(),
            evidence.assistive_technology_profile_id.as_str(),
        );
        if !accessibility_evidence_keys.insert(key) {
            return Err(CivilEvidenceError::DuplicateAccessibilityKey(format!(
                "{}:{}:{}:{}",
                evidence.cohort_id,
                evidence.language_id,
                evidence.format_id,
                evidence.assistive_technology_profile_id
            )));
        }
        for value in [
            evidence.cohort_id.as_str(),
            evidence.language_id.as_str(),
            evidence.format_id.as_str(),
            evidence.assistive_technology_profile_id.as_str(),
            evidence.test_protocol_id.as_str(),
        ] {
            check_identifier(value)?;
        }
    }
    for evidence in &profile.comprehension_evidence {
        for value in [
            evidence.cohort_id.as_str(),
            evidence.language_id.as_str(),
            evidence.test_protocol_id.as_str(),
        ] {
            check_identifier(value)?;
        }
    }
    Ok(())
}

fn assess_base_audit(profile: &CivilEvidenceProfile) -> BaseAuditAssessment {
    let mut violations = BTreeSet::new();
    if profile.base_audit.as_of_cutoff != profile.as_of_cutoff {
        violations.insert(CivilViolation::BaseAuditCutoffMismatch);
    }

    let mut supporting = 0usize;
    for binding in &profile.required_base_claims {
        let claim: Option<&ClaimResult> = profile
            .base_audit
            .claims
            .iter()
            .find(|claim| claim.stage == binding.stage && claim.claim_id == binding.claim_id);
        match claim {
            None => {
                violations.insert(CivilViolation::BaseClaimMissing);
            }
            Some(claim) if claim.disposition == ClaimDisposition::Supports => supporting += 1,
            Some(_) => {
                violations.insert(CivilViolation::BaseClaimNotSupporting);
            }
        }
    }

    let disposition = if supporting == profile.required_base_claims.len()
        && violations.is_empty()
    {
        ClaimDisposition::Supports
    } else {
        ClaimDisposition::Indeterminate
    };

    BaseAuditAssessment {
        disposition,
        required_claims: profile.required_base_claims.len(),
        supporting_claims: supporting,
        violations: violations.into_iter().collect(),
    }
}

fn assess_authority(profile: &CivilEvidenceProfile) -> AuthorityAssessment {
    let mut violations = BTreeSet::new();
    let Some(assertion) = &profile.authority_assertion else {
        violations.insert(CivilViolation::AuthorityMissing);
        return AuthorityAssessment {
            disposition: ClaimDisposition::Unavailable,
            assertion_id: None,
            registry_status: None,
            signature_status: None,
            violations: violations.into_iter().collect(),
        };
    };

    if assertion.recorded_at > profile.as_of_cutoff {
        violations.insert(CivilViolation::AuthorityRecordedAfterCutoff);
    }
    if assertion.valid_until <= assertion.valid_from {
        violations.insert(CivilViolation::AuthorityValidityIntervalInvalid);
    }
    if assertion.authority_id != profile.authority_requirement.authority_id
        || assertion.jurisdiction_id != profile.authority_requirement.jurisdiction_id
        || assertion.hazard_class_id != profile.authority_requirement.hazard_class_id
    {
        violations.insert(CivilViolation::AuthorityScopeMismatch);
    }
    if assertion.cap_originator != profile.authority_requirement.cap_originator
        || assertion.cap_message_id != profile.authority_requirement.cap_message_id
        || assertion.cap_version != profile.authority_requirement.cap_version
    {
        violations.insert(CivilViolation::AuthorityMessageBindingMismatch);
    }
    if assertion.decision_id != profile.authority_requirement.decision_id {
        violations.insert(CivilViolation::AuthorityDecisionBindingMismatch);
    }
    if profile.authority_requirement.event_time < assertion.valid_from {
        violations.insert(CivilViolation::AuthorityNotYetValid);
    }
    if profile.authority_requirement.event_time >= assertion.valid_until {
        violations.insert(CivilViolation::AuthorityExpired);
    }

    match assertion.signature_status {
        SignatureStatus::Verified => {}
        SignatureStatus::Unverified => {
            violations.insert(CivilViolation::AuthoritySignatureNotVerified);
        }
        SignatureStatus::Invalid => {
            violations.insert(CivilViolation::AuthoritySignatureNotVerified);
        }
    }
    match assertion.registry_status {
        RegistryAuthorityStatus::Recognized => {}
        RegistryAuthorityStatus::Unknown => {
            violations.insert(CivilViolation::AuthorityRegistryUnknown);
        }
        RegistryAuthorityStatus::Disputed => {
            violations.insert(CivilViolation::AuthorityRegistryDisputed);
        }
        RegistryAuthorityStatus::Revoked => {
            violations.insert(CivilViolation::AuthorityRevoked);
        }
    }

    AuthorityAssessment {
        disposition: if violations.is_empty() {
            ClaimDisposition::Supports
        } else {
            ClaimDisposition::Indeterminate
        },
        assertion_id: Some(assertion.assertion_id.clone()),
        registry_status: Some(assertion.registry_status),
        signature_status: Some(assertion.signature_status),
        violations: violations.into_iter().collect(),
    }
}

fn assess_delivery(
    profile: &CivilEvidenceProfile,
) -> Result<Vec<DeliveryAssessment>, CivilEvidenceError> {
    let mut target_keys = BTreeSet::new();
    for target in &profile.delivery_targets {
        let key = (target.cohort_id.as_str(), target.channel_id.as_str());
        if !target_keys.insert(key) {
            return Err(CivilEvidenceError::DuplicateCohortChannel(format!(
                "{}:{}",
                target.cohort_id, target.channel_id
            )));
        }
    }

    let mut receipt_ids = BTreeSet::new();
    for receipt in &profile.delivery_receipts {
        check_identifier(&receipt.receipt_id)?;
        check_identifier(&receipt.message_id)?;
        check_identifier(&receipt.message_version)?;
        check_identifier(&receipt.cohort_id)?;
        check_identifier(&receipt.channel_id)?;
        if !receipt_ids.insert(receipt.receipt_id.as_str()) {
            return Err(CivilEvidenceError::DuplicateReceiptId(receipt.receipt_id.clone()));
        }
    }

    for receipt in &profile.delivery_receipts {
        if receipt.state == DeliveryState::Duplicate {
            let Some(source_id) = receipt.duplicate_of.as_deref() else {
                return Err(CivilEvidenceError::InvalidDeliveryCount(receipt.receipt_id.clone()));
            };
            if !profile.delivery_receipts.iter().any(|source| {
                source.receipt_id == source_id
                    && source.cohort_id == receipt.cohort_id
                    && source.channel_id == receipt.channel_id
                    && source.message_id == receipt.message_id
                    && source.message_version == receipt.message_version
                    && source.state != DeliveryState::Duplicate
                    && source.recorded_at <= receipt.recorded_at
            }) {
                return Err(CivilEvidenceError::InvalidDeliveryCount(receipt.receipt_id.clone()));
            }
        } else if receipt.duplicate_of.is_some() {
            return Err(CivilEvidenceError::InvalidDeliveryCount(receipt.receipt_id.clone()));
        }
    }

    let mut assessments = Vec::new();
    for target in &profile.delivery_targets {
        let mut counts = DeliveryCounts {
            intended_receivers: target.intended_receivers,
            ..DeliveryCounts::default()
        };
        let mut violations = BTreeSet::new();
        let receipts: Vec<&DeliveryReceipt> = profile
            .delivery_receipts
            .iter()
            .filter(|receipt| {
                receipt.cohort_id == target.cohort_id
                    && receipt.channel_id == target.channel_id
            })
            .collect();

        for receipt in receipts {
            if receipt.message_id != profile.delivery_message_id
                || receipt.message_version != profile.delivery_message_version
            {
                violations.insert(CivilViolation::DeliveryReceiptIdentityMismatch);
            }
            if receipt.recorded_at > profile.as_of_cutoff {
                violations.insert(CivilViolation::DeliveryReceiptAfterCutoff);
                continue;
            }
            if receipt.count == 0 {
                return Err(CivilEvidenceError::InvalidDeliveryCount(receipt.receipt_id.clone()));
            }
            match receipt.state {
                DeliveryState::ProviderAccepted => counts.provider_accepted += receipt.count,
                DeliveryState::DeviceDelivered => counts.device_delivered += receipt.count,
                DeliveryState::Failed => counts.failed += receipt.count,
                DeliveryState::Delayed => counts.delayed += receipt.count,
                DeliveryState::Unknown => counts.unknown += receipt.count,
                DeliveryState::Duplicate => counts.duplicates += receipt.count,
            }
        }

        if counts.intended_receivers == 0
            || counts.provider_accepted > counts.intended_receivers
            || counts.device_delivered > counts.intended_receivers
            || counts.failed > counts.intended_receivers
            || counts.delayed > counts.intended_receivers
            || counts.unknown > counts.intended_receivers
        {
            violations.insert(CivilViolation::DeliveryCountExceedsTarget);
        }
        if counts.device_delivered > counts.provider_accepted {
            violations.insert(CivilViolation::DeviceDeliveryExceedsProviderAcceptance);
        }
        counts.not_confirmed_device_delivered = counts
            .intended_receivers
            .saturating_sub(counts.device_delivered);

        let disposition = if !violations.is_empty() {
            ClaimDisposition::Indeterminate
        } else if counts.device_delivered == counts.intended_receivers {
            ClaimDisposition::Supports
        } else if receipts.is_empty() {
            ClaimDisposition::Unavailable
        } else {
            ClaimDisposition::Indeterminate
        };

        assessments.push(DeliveryAssessment {
            cohort_id: target.cohort_id.clone(),
            channel_id: target.channel_id.clone(),
            disposition,
            counts,
            violations: violations.into_iter().collect(),
        });
    }

    // Receipts for an undeclared target are never silently ignored.
    if profile.delivery_receipts.iter().any(|receipt| {
        !target_keys.contains(&(receipt.cohort_id.as_str(), receipt.channel_id.as_str()))
    }) {
        return Err(CivilEvidenceError::InvalidDeliveryCount(
            "receipt for undeclared cohort/channel".into(),
        ));
    }

    Ok(assessments)
}

fn assess_accessibility(profile: &CivilEvidenceProfile) -> Vec<AccessibilityAssessment> {
    let mut output = Vec::new();
    for requirement in &profile.accessibility_requirements {
        let mut violations = BTreeSet::new();
        let evidence: Vec<&AccessibilityEvidence> = profile
            .accessibility_evidence
            .iter()
            .filter(|record| {
                record.cohort_id == requirement.cohort_id
                    && record.language_id == requirement.language_id
                    && record.format_id == requirement.format_id
                    && record.assistive_technology_profile_id
                        == requirement.assistive_technology_profile_id
            })
            .collect();

        let state = if evidence.is_empty() {
            violations.insert(CivilViolation::AccessibilityEvidenceMissing);
            None
        } else {
            let mut selected = None;
            for record in &evidence {
                if record.recorded_at > profile.as_of_cutoff
                    || record.tested_at > profile.as_of_cutoff
                {
                    violations.insert(CivilViolation::AccessibilityEvidenceAfterCutoff);
                    continue;
                }
                if !valid_identifier(&record.test_protocol_id) {
                    violations.insert(CivilViolation::AccessibilityNotSupported);
                    continue;
                }
                selected = Some(record.state);
            }
            selected
        };

        let disposition = if violations.contains(&CivilViolation::AccessibilityEvidenceAfterCutoff) {
            ClaimDisposition::Indeterminate
        } else {
            match state {
            Some(AccessibilityState::Accessible) if violations.is_empty() => {
                ClaimDisposition::Supports
            }
            Some(AccessibilityState::NotAccessible) => {
                violations.insert(CivilViolation::AccessibilityNotSupported);
                ClaimDisposition::Refutes
            }
            Some(AccessibilityState::Unknown) => ClaimDisposition::Unavailable,
            _ if violations.contains(&CivilViolation::AccessibilityEvidenceMissing) => {
                ClaimDisposition::Unavailable
            }
            _ => ClaimDisposition::Indeterminate,
            }
        };

        output.push(AccessibilityAssessment {
            cohort_id: requirement.cohort_id.clone(),
            language_id: requirement.language_id.clone(),
            format_id: requirement.format_id.clone(),
            disposition,
            state,
            violations: violations.into_iter().collect(),
        });
    }
    output
}

fn assess_comprehension(
    profile: &CivilEvidenceProfile,
) -> Result<Vec<ComprehensionAssessment>, CivilEvidenceError> {
    let mut keys = BTreeSet::new();
    for evidence in &profile.comprehension_evidence {
        let key = (
            evidence.cohort_id.as_str(),
            evidence.language_id.as_str(),
            evidence.test_protocol_id.as_str(),
        );
        if !keys.insert(key) {
            return Err(CivilEvidenceError::DuplicateComprehensionKey(format!(
                "{}:{}:{}",
                evidence.cohort_id, evidence.language_id, evidence.test_protocol_id
            )));
        }
        if evidence.understood_count > evidence.tested_count
            || evidence.intended_action_count > evidence.tested_count
        {
            return Err(CivilEvidenceError::InvalidComprehensionCount(
                evidence.cohort_id.clone(),
            ));
        }
    }

    let mut output = Vec::new();
    for requirement in &profile.comprehension_requirements {
        let evidence = profile.comprehension_evidence.iter().find(|record| {
            record.cohort_id == requirement.cohort_id
                && record.language_id == requirement.language_id
                && record.test_protocol_id == requirement.test_protocol_id
        });
        let mut violations = BTreeSet::new();
        let Some(evidence) = evidence else {
            violations.insert(CivilViolation::ComprehensionEvidenceMissing);
            output.push(ComprehensionAssessment {
                cohort_id: requirement.cohort_id.clone(),
                language_id: requirement.language_id.clone(),
                disposition: ClaimDisposition::Unavailable,
                tested_count: 0,
                comprehension_rate: None,
                intended_action_rate: None,
                violations: violations.into_iter().collect(),
            });
            continue;
        };

        let within_cutoff = evidence.recorded_at <= profile.as_of_cutoff
            && evidence.tested_at <= profile.as_of_cutoff;
        if !within_cutoff {
            violations.insert(CivilViolation::ComprehensionEvidenceAfterCutoff);
        }
        let comprehension_rate = (within_cutoff && evidence.tested_count > 0)
            .then_some(evidence.understood_count as f64 / evidence.tested_count as f64);
        let intended_action_rate = (within_cutoff && evidence.tested_count > 0)
            .then_some(evidence.intended_action_count as f64 / evidence.tested_count as f64);

        let disposition = if !violations.is_empty() {
            ClaimDisposition::Indeterminate
        } else if let (Some(comprehension), Some(intent)) =
            (comprehension_rate, intended_action_rate)
        {
            if comprehension >= requirement.minimum_comprehension_rate
                && intent >= requirement.minimum_intent_rate
            {
                ClaimDisposition::Supports
            } else {
                violations.insert(CivilViolation::ComprehensionBelowThreshold);
                ClaimDisposition::Refutes
            }
        } else {
            ClaimDisposition::Unavailable
        };

        output.push(ComprehensionAssessment {
            cohort_id: requirement.cohort_id.clone(),
            language_id: requirement.language_id.clone(),
            disposition,
            tested_count: evidence.tested_count,
            comprehension_rate,
            intended_action_rate,
            violations: violations.into_iter().collect(),
        });
    }
    Ok(output)
}

fn assess_actions(
    profile: &CivilEvidenceProfile,
) -> Result<Vec<ActionAssessment>, CivilEvidenceError> {
    let mut receipt_ids = BTreeSet::new();
    for receipt in &profile.action_receipts {
        for value in [
            receipt.receipt_id.as_str(),
            receipt.action_id.as_str(),
            receipt.decision_id.as_str(),
            receipt.authority_assertion_id.as_str(),
            receipt.executor_agency_id.as_str(),
            receipt.action_type_id.as_str(),
        ] {
            check_identifier(value)?;
        }
        if !receipt_ids.insert(receipt.receipt_id.as_str()) {
            return Err(CivilEvidenceError::DuplicateReceiptId(receipt.receipt_id.clone()));
        }
    }

    let mut required = BTreeSet::new();
    let mut output = Vec::new();
    for requirement in &profile.action_requirements {
        if !required.insert(requirement.action_id.as_str()) {
            return Err(CivilEvidenceError::DuplicateActionId(requirement.action_id.clone()));
        }
        let mut violations = BTreeSet::new();
        let mut receipts: Vec<&ActionReceipt> = profile
            .action_receipts
            .iter()
            .filter(|receipt| receipt.action_id == requirement.action_id)
            .collect();
        receipts.sort_by_key(|receipt| (receipt.event_time, receipt.recorded_at));

        let mut state = None;
        let mut last_time = None;
        let mut as_of_receipt_count = 0usize;
        for receipt in &receipts {
            if receipt.decision_id != requirement.decision_id
                || receipt.authority_assertion_id != requirement.authority_assertion_id
                || receipt.executor_agency_id != requirement.executor_agency_id
                || receipt.action_type_id != requirement.action_type_id
            {
                violations.insert(CivilViolation::ActionReceiptBindingMismatch);
            }
            if receipt.recorded_at > profile.as_of_cutoff || receipt.event_time > profile.as_of_cutoff {
                violations.insert(CivilViolation::ActionReceiptAfterCutoff);
                continue;
            }
            if let (Some(previous_time), Some(previous_state)) = (last_time, state) {
                if receipt.event_time < previous_time
                    || !previous_state.allows_transition_to(receipt.state)
                {
                    violations.insert(CivilViolation::ActionTransitionRegression);
                }
            }
            last_time = Some(receipt.event_time);
            state = Some(receipt.state);
            as_of_receipt_count += 1;
        }

        let disposition = if receipts.is_empty() {
            violations.insert(CivilViolation::ActionReceiptMissing);
            ClaimDisposition::Unavailable
        } else if !violations.is_empty() {
            ClaimDisposition::Indeterminate
        } else {
            match state {
                Some(ActionState::Completed) => ClaimDisposition::Supports,
                Some(ActionState::Failed | ActionState::Cancelled) => {
                    violations.insert(CivilViolation::ActionNotCompleted);
                    ClaimDisposition::Refutes
                }
                Some(ActionState::Unknown) => ClaimDisposition::Unavailable,
                _ => {
                    violations.insert(CivilViolation::ActionNotCompleted);
                    ClaimDisposition::Indeterminate
                }
            }
        };

        output.push(ActionAssessment {
            action_id: requirement.action_id.clone(),
            disposition,
            latest_state: state,
            receipt_count: as_of_receipt_count,
            violations: violations.into_iter().collect(),
        });
    }

    if profile.action_receipts.iter().any(|receipt| {
        !required.contains(receipt.action_id.as_str())
    }) {
        return Err(CivilEvidenceError::InvalidActionSequence(
            "receipt for undeclared action".into(),
        ));
    }
    Ok(output)
}

fn assess_case_frame(profile: &CivilEvidenceProfile) -> Result<CaseFrameAssessment, CivilEvidenceError> {
    let mut seen = BTreeSet::new();
    let mut observed = 0u64;
    let mut missing = 0u64;
    let mut disputed = 0u64;
    let mut excluded = 0u64;
    let mut violations = BTreeSet::new();

    for case in &profile.case_frame.cases {
        check_identifier(&case.case_key)?;
        if !seen.insert(case.case_key.as_str()) {
            return Err(CivilEvidenceError::DuplicateCaseKey(case.case_key.clone()));
        }
        if case.recorded_at > profile.as_of_cutoff {
            violations.insert(CivilViolation::CaseFrameAfterCutoff);
            disputed += 1;
            continue;
        }
        match case.disposition {
            CaseDisposition::Observed => observed += 1,
            CaseDisposition::Missing => missing += 1,
            CaseDisposition::Disputed => disputed += 1,
            CaseDisposition::Excluded => excluded += 1,
        }
    }

    let eligible = observed + missing + disputed;
    if eligible != profile.case_frame.declared_eligible_cases {
        violations.insert(CivilViolation::CaseFrameDenominatorMismatch);
    }
    if missing > 0 {
        violations.insert(CivilViolation::CaseFrameMissingOutcomes);
    }
    if disputed > 0 {
        violations.insert(CivilViolation::CaseFrameDisputedOutcomes);
    }

    let disposition = if eligible == 0 && violations.is_empty() {
        ClaimDisposition::Unavailable
    } else if violations.is_empty() && missing == 0 && disputed == 0 {
        ClaimDisposition::Supports
    } else if violations.contains(&CivilViolation::CaseFrameDenominatorMismatch) {
        ClaimDisposition::Indeterminate
    } else if missing > 0 || disputed > 0 {
        ClaimDisposition::Indeterminate
    } else {
        ClaimDisposition::Supports
    };

    Ok(CaseFrameAssessment {
        frame_id: profile.case_frame.frame_id.clone(),
        disposition,
        eligible_cases: eligible,
        observed_cases: observed,
        missing_cases: missing,
        disputed_cases: disputed,
        excluded_cases: excluded,
        violations: violations.into_iter().collect(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use gcpn_compositional_assurance::{
        CapReceiptLink, ClaimRequirement, EvidenceValidity, ForecastTiming,
        PopulationAccounting, PostProcessingFacts, Scenario, ScenarioFacts,
        StageEvidence, VerificationAccounting, CausalClaim, PairedComparison,
    };

    fn base_audit() -> AuditReport {
        let requirements = vec![
            requirement(Stage::Forecast, "forecast-valid", 1),
            requirement(Stage::CapLifecycle, "receipt-valid", 1),
            requirement(Stage::PopulationReach, "reach-valid", 1),
            requirement(Stage::Decision, "decision-valid", 1),
        ];
        let evidence = vec![
            record("forecast-e", Stage::Forecast, "forecast-valid", "forecast-root"),
            record("cap-e", Stage::CapLifecycle, "receipt-valid", "cap-root"),
            record("reach-e", Stage::PopulationReach, "reach-valid", "reach-root"),
            record("decision-e", Stage::Decision, "decision-valid", "decision-root"),
        ];
        Scenario {
            scenario_id: "base-scenario".into(),
            specification_digest: "gcpn-compositional-assurance-v0.1".into(),
            as_of_cutoff: 100,
            requirements,
            evidence,
            facts: ScenarioFacts {
                forecast_timing: Some(ForecastTiming {
                    issued_at: 10,
                    decision_cutoff: 20,
                    event_valid_at: 100,
                    minimum_action_lead: 10,
                }),
                cap_receipt: Some(CapReceiptLink {
                    current_message_id: "cap-1".into(),
                    current_version: "1".into(),
                    receipt_message_id: "cap-1".into(),
                    receipt_version: "1".into(),
                }),
                population: Some(PopulationAccounting {
                    eligible_denominator: Some(100),
                    observed_cases: 100,
                    typed_missing_cases: 0,
                    unique_reach: 80,
                }),
                verification: Some(VerificationAccounting {
                    eligible_cases: 100,
                    observed_cases: 100,
                    typed_missing_cases: 0,
                }),
                paired_comparison: Some(PairedComparison {
                    candidate_case_frame_digest: "frame".into(),
                    reference_case_frame_digest: "frame".into(),
                    candidate_source: "source".into(),
                    reference_source: "source".into(),
                    candidate_version: "v1".into(),
                    reference_version: "v1".into(),
                }),
                causal_claim: Some(CausalClaim {
                    claim_id: "review".into(),
                    claim_made: false,
                    identification_method_id: None,
                    design_id: None,
                    supporting_evidence_ids: Vec::new(),
                }),
                post_processing: Some(PostProcessingFacts {
                    intrinsic_skill_claim: true,
                    calibration_uses_evaluation_outcomes: false,
                    calibration_trained_through: Some(5),
                    evaluation_period_start: Some(20),
                }),
            },
        }
        .audit()
        .expect("base scenario must audit")
    }

    fn requirement(stage: Stage, claim_id: &str, minimum: usize) -> ClaimRequirement {
        ClaimRequirement {
            stage,
            claim_id: claim_id.into(),
            subject_digest: "subject-1".into(),
            expected_source: Some("source".into()),
            expected_version: Some("v1".into()),
            minimum_independent_supporting_lineages: minimum,
        }
    }

    fn record(id: &str, stage: Stage, claim_id: &str, lineage: &str) -> StageEvidence {
        StageEvidence {
            evidence_id: id.into(),
            stage,
            claim_id: claim_id.into(),
            subject_digest: "subject-1".into(),
            source: "source".into(),
            version: "v1".into(),
            event_time: 10,
            recorded_at: 20,
            validity: EvidenceValidity::Current,
            disposition: ClaimDisposition::Supports,
            lineage_root: lineage.into(),
            evaluator_id: format!("evaluator-{id}"),
        }
    }

    fn profile() -> CivilEvidenceProfile {
        CivilEvidenceProfile {
            scenario_id: "civil-evidence-happy-path".into(),
            as_of_cutoff: 100,
            base_audit: base_audit(),
            required_base_claims: vec![
                BaseClaimBinding { stage: Stage::Forecast, claim_id: "forecast-valid".into() },
                BaseClaimBinding { stage: Stage::CapLifecycle, claim_id: "receipt-valid".into() },
                BaseClaimBinding { stage: Stage::PopulationReach, claim_id: "reach-valid".into() },
                BaseClaimBinding { stage: Stage::Decision, claim_id: "decision-valid".into() },
            ],
            authority_requirement: AuthorityRequirement {
                authority_id: "authority-1".into(),
                jurisdiction_id: "district-1".into(),
                hazard_class_id: "flood".into(),
                cap_originator: "national-alert-centre".into(),
                cap_message_id: "cap-1".into(),
                cap_version: "1".into(),
                decision_id: "decision-1".into(),
                event_time: 50,
            },
            authority_assertion: Some(AuthorityAssertion {
                assertion_id: "assertion-1".into(),
                authority_id: "authority-1".into(),
                jurisdiction_id: "district-1".into(),
                hazard_class_id: "flood".into(),
                cap_originator: "national-alert-centre".into(),
                cap_message_id: "cap-1".into(),
                cap_version: "1".into(),
                decision_id: "decision-1".into(),
                valid_from: 1,
                valid_until: 100,
                recorded_at: 20,
                signature_status: SignatureStatus::Verified,
                registry_status: RegistryAuthorityStatus::Recognized,
            }),
            delivery_message_id: "cap-1".into(),
            delivery_message_version: "1".into(),
            delivery_targets: vec![ChannelCohortTarget {
                cohort_id: "cohort-a".into(),
                channel_id: "cell-broadcast".into(),
                intended_receivers: 100,
            }],
            delivery_receipts: vec![
                DeliveryReceipt {
                    receipt_id: "receipt-provider".into(),
                    message_id: "cap-1".into(),
                    message_version: "1".into(),
                    cohort_id: "cohort-a".into(),
                    channel_id: "cell-broadcast".into(),
                    state: DeliveryState::ProviderAccepted,
                    count: 100,
                    duplicate_of: None,
                    recorded_at: 30,
                },
                DeliveryReceipt {
                    receipt_id: "receipt-device".into(),
                    message_id: "cap-1".into(),
                    message_version: "1".into(),
                    cohort_id: "cohort-a".into(),
                    channel_id: "cell-broadcast".into(),
                    state: DeliveryState::DeviceDelivered,
                    count: 95,
                    duplicate_of: None,
                    recorded_at: 35,
                },
            ],
            accessibility_requirements: vec![AccessibilityRequirement {
                cohort_id: "cohort-a".into(),
                language_id: "en".into(),
                format_id: "text".into(),
                assistive_technology_profile_id: "screen-reader".into(),
            }],
            accessibility_evidence: vec![AccessibilityEvidence {
                cohort_id: "cohort-a".into(),
                language_id: "en".into(),
                format_id: "text".into(),
                assistive_technology_profile_id: "screen-reader".into(),
                test_protocol_id: "wcag-test-v1".into(),
                state: AccessibilityState::Accessible,
                tested_at: 40,
                recorded_at: 45,
            }],
            comprehension_requirements: vec![ComprehensionRequirement {
                cohort_id: "cohort-a".into(),
                language_id: "en".into(),
                test_protocol_id: "human-test-v1".into(),
                minimum_comprehension_rate: 0.8,
                minimum_intent_rate: 0.7,
            }],
            comprehension_evidence: vec![ComprehensionEvidence {
                cohort_id: "cohort-a".into(),
                language_id: "en".into(),
                test_protocol_id: "human-test-v1".into(),
                tested_count: 50,
                understood_count: 45,
                intended_action_count: 40,
                tested_at: 50,
                recorded_at: 55,
            }],
            action_requirements: vec![ActionRequirement {
                action_id: "evacuation-1".into(),
                decision_id: "decision-1".into(),
                authority_assertion_id: "assertion-1".into(),
                executor_agency_id: "response-agency".into(),
                action_type_id: "evacuation".into(),
            }],
            action_receipts: vec![
                ActionReceipt {
                    receipt_id: "action-approved".into(),
                    action_id: "evacuation-1".into(),
                    decision_id: "decision-1".into(),
                    authority_assertion_id: "assertion-1".into(),
                    executor_agency_id: "response-agency".into(),
                    action_type_id: "evacuation".into(),
                    state: ActionState::Approved,
                    event_time: 25,
                    recorded_at: 25,
                },
                ActionReceipt {
                    receipt_id: "action-dispatched".into(),
                    action_id: "evacuation-1".into(),
                    decision_id: "decision-1".into(),
                    authority_assertion_id: "assertion-1".into(),
                    executor_agency_id: "response-agency".into(),
                    action_type_id: "evacuation".into(),
                    state: ActionState::Dispatched,
                    event_time: 30,
                    recorded_at: 30,
                },
                ActionReceipt {
                    receipt_id: "action-started".into(),
                    action_id: "evacuation-1".into(),
                    decision_id: "decision-1".into(),
                    authority_assertion_id: "assertion-1".into(),
                    executor_agency_id: "response-agency".into(),
                    action_type_id: "evacuation".into(),
                    state: ActionState::Started,
                    event_time: 40,
                    recorded_at: 40,
                },
                ActionReceipt {
                    receipt_id: "action-completed".into(),
                    action_id: "evacuation-1".into(),
                    decision_id: "decision-1".into(),
                    authority_assertion_id: "assertion-1".into(),
                    executor_agency_id: "response-agency".into(),
                    action_type_id: "evacuation".into(),
                    state: ActionState::Completed,
                    event_time: 70,
                    recorded_at: 70,
                },
            ],
            case_frame: CaseFrame {
                frame_id: "outcome-frame-1".into(),
                inclusion_criteria_id: "eligible-households-v1".into(),
                declared_eligible_cases: 3,
                cases: vec![
                    FrameCase { case_key: "case-1".into(), disposition: CaseDisposition::Observed, recorded_at: 80 },
                    FrameCase { case_key: "case-2".into(), disposition: CaseDisposition::Observed, recorded_at: 80 },
                    FrameCase { case_key: "case-3".into(), disposition: CaseDisposition::Missing, recorded_at: 80 },
                    FrameCase { case_key: "case-4".into(), disposition: CaseDisposition::Excluded, recorded_at: 80 },
                ],
            },
        }
    }

    #[test]
    fn clean_civil_evidence_chain_supports_each_separate_claim() {
        let report = audit_civil_evidence(&profile()).unwrap();
        assert_eq!(report.base_audit.disposition, ClaimDisposition::Supports);
        assert_eq!(report.authority.disposition, ClaimDisposition::Supports);
        assert_eq!(report.delivery[0].disposition, ClaimDisposition::Indeterminate);
        assert_eq!(report.accessibility[0].disposition, ClaimDisposition::Supports);
        assert_eq!(report.comprehension[0].disposition, ClaimDisposition::Supports);
        assert_eq!(report.actions[0].disposition, ClaimDisposition::Supports);
        assert_eq!(report.case_frame.disposition, ClaimDisposition::Indeterminate);
    }

    #[test]
    fn cryptographic_signature_does_not_imply_registry_authority() {
        let mut data = profile();
        data.authority_assertion.as_mut().unwrap().registry_status = RegistryAuthorityStatus::Unknown;
        let report = audit_civil_evidence(&data).unwrap();
        assert_eq!(report.authority.disposition, ClaimDisposition::Indeterminate);
        assert!(
            report.authority.violations.contains(&CivilViolation::AuthorityRegistryUnknown)
        );
    }

    #[test]
    fn expired_authority_assertion_cannot_support_authority_claim() {
        let mut data = profile();
        data.authority_assertion.as_mut().unwrap().valid_until = 50;
        let report = audit_civil_evidence(&data).unwrap();
        assert_eq!(report.authority.disposition, ClaimDisposition::Indeterminate);
        assert!(report.authority.violations.contains(&CivilViolation::AuthorityExpired));
    }

    #[test]
    fn cap_message_or_decision_binding_mismatch_is_rejected() {
        let mut data = profile();
        data.authority_assertion.as_mut().unwrap().cap_message_id = "other-message".into();
        let report = audit_civil_evidence(&data).unwrap();
        assert_eq!(report.authority.disposition, ClaimDisposition::Indeterminate);
        assert!(
            report.authority.violations.contains(&CivilViolation::AuthorityMessageBindingMismatch)
        );
    }

    #[test]
    fn provider_acceptance_is_not_device_delivery() {
        let mut data = profile();
        data.delivery_receipts.retain(|receipt| receipt.state != DeliveryState::DeviceDelivered);
        let report = audit_civil_evidence(&data).unwrap();
        assert_eq!(report.delivery[0].counts.provider_accepted, 100);
        assert_eq!(report.delivery[0].counts.device_delivered, 0);
        assert_eq!(report.delivery[0].disposition, ClaimDisposition::Indeterminate);
    }

    #[test]
    fn duplicate_delivery_receipt_does_not_inflate_unique_device_delivery() {
        let mut data = profile();
        data.delivery_receipts.push(DeliveryReceipt {
            receipt_id: "receipt-retry".into(),
            message_id: "cap-1".into(),
            message_version: "1".into(),
            cohort_id: "cohort-a".into(),
            channel_id: "cell-broadcast".into(),
            state: DeliveryState::Duplicate,
            count: 95,
            duplicate_of: Some("receipt-device".into()),
            recorded_at: 36,
        });
        let report = audit_civil_evidence(&data).unwrap();
        assert_eq!(report.delivery[0].counts.device_delivered, 95);
        assert_eq!(report.delivery[0].counts.duplicates, 95);
    }

    #[test]
    fn unknown_delivery_is_not_reclassified_as_delivered() {
        let mut data = profile();
        data.delivery_receipts.retain(|receipt| receipt.state != DeliveryState::DeviceDelivered);
        data.delivery_receipts.push(DeliveryReceipt {
            receipt_id: "receipt-unknown".into(),
            message_id: "cap-1".into(),
            message_version: "1".into(),
            cohort_id: "cohort-a".into(),
            channel_id: "cell-broadcast".into(),
            state: DeliveryState::Unknown,
            count: 100,
            duplicate_of: None,
            recorded_at: 36,
        });
        let report = audit_civil_evidence(&data).unwrap();
        assert_eq!(report.delivery[0].counts.device_delivered, 0);
        assert_eq!(report.delivery[0].counts.unknown, 100);
        assert_eq!(report.delivery[0].disposition, ClaimDisposition::Indeterminate);
    }

    #[test]
    fn accessibility_rendering_does_not_imply_comprehension() {
        let mut data = profile();
        data.comprehension_evidence.clear();
        let report = audit_civil_evidence(&data).unwrap();
        assert_eq!(report.accessibility[0].disposition, ClaimDisposition::Supports);
        assert_eq!(report.comprehension[0].disposition, ClaimDisposition::Unavailable);
    }

    #[test]
    fn accessibility_not_tested_is_unavailable_not_accessible() {
        let mut data = profile();
        data.accessibility_evidence.clear();
        let report = audit_civil_evidence(&data).unwrap();
        assert_eq!(report.accessibility[0].disposition, ClaimDisposition::Unavailable);
    }

    #[test]
    fn comprehension_and_intent_rates_are_separate() {
        let mut data = profile();
        data.comprehension_evidence[0].understood_count = 45;
        data.comprehension_evidence[0].intended_action_count = 20;
        let report = audit_civil_evidence(&data).unwrap();
        assert_eq!(report.comprehension[0].comprehension_rate, Some(0.9));
        assert_eq!(report.comprehension[0].intended_action_rate, Some(0.4));
        assert_eq!(report.comprehension[0].disposition, ClaimDisposition::Refutes);
    }

    #[test]
    fn approved_action_is_not_completed_action() {
        let mut data = profile();
        data.action_receipts.retain(|receipt| receipt.state == ActionState::Approved);
        let report = audit_civil_evidence(&data).unwrap();
        assert_eq!(report.actions[0].latest_state, Some(ActionState::Approved));
        assert_eq!(report.actions[0].disposition, ClaimDisposition::Indeterminate);
    }

    #[test]
    fn action_receipt_requires_exact_decision_authority_and_executor_binding() {
        let mut data = profile();
        data.action_receipts[3].authority_assertion_id = "other-assertion".into();
        let report = audit_civil_evidence(&data).unwrap();
        assert_eq!(report.actions[0].disposition, ClaimDisposition::Indeterminate);
        assert!(report.actions[0].violations.contains(&CivilViolation::ActionReceiptBindingMismatch));
    }

    #[test]
    fn completed_action_can_be_supported_without_claiming_population_action_rate() {
        let report = audit_civil_evidence(&profile()).unwrap();
        assert_eq!(report.actions[0].disposition, ClaimDisposition::Supports);
        assert_eq!(report.comprehension[0].intended_action_rate, Some(0.8));
    }

    #[test]
    fn missing_outcomes_remain_missing_and_in_denominator() {
        let report = audit_civil_evidence(&profile()).unwrap();
        assert_eq!(report.case_frame.eligible_cases, 3);
        assert_eq!(report.case_frame.observed_cases, 2);
        assert_eq!(report.case_frame.missing_cases, 1);
        assert_eq!(report.case_frame.excluded_cases, 1);
        assert_eq!(report.case_frame.disposition, ClaimDisposition::Indeterminate);
    }

    #[test]
    fn case_frame_duplicate_keys_fail_closed() {
        let mut data = profile();
        data.case_frame.cases.push(data.case_frame.cases[0].clone());
        assert!(matches!(
            audit_civil_evidence(&data),
            Err(CivilEvidenceError::DuplicateCaseKey(_))
        ));
    }

    #[test]
    fn declared_case_denominator_mismatch_is_not_supported() {
        let mut data = profile();
        data.case_frame.declared_eligible_cases = 99;
        let report = audit_civil_evidence(&data).unwrap();
        assert_eq!(report.case_frame.disposition, ClaimDisposition::Indeterminate);
        assert!(report.case_frame.violations.contains(&CivilViolation::CaseFrameDenominatorMismatch));
    }

    #[test]
    fn cross_channel_receipts_are_not_collapsed_into_unique_population_reach() {
        let mut data = profile();
        data.delivery_targets.push(ChannelCohortTarget {
            cohort_id: "cohort-a".into(),
            channel_id: "sms".into(),
            intended_receivers: 100,
        });
        data.delivery_receipts.push(DeliveryReceipt {
            receipt_id: "sms-accepted".into(),
            message_id: "cap-1".into(),
            message_version: "1".into(),
            cohort_id: "cohort-a".into(),
            channel_id: "sms".into(),
            state: DeliveryState::ProviderAccepted,
            count: 100,
            duplicate_of: None,
            recorded_at: 36,
        });
        let report = audit_civil_evidence(&data).unwrap();
        assert_eq!(report.delivery.len(), 2);
        assert_eq!(report.delivery[0].counts.device_delivered, 95);
        assert_eq!(report.delivery[1].counts.device_delivered, 0);
        assert_eq!(report.delivery[0].cohort_id, report.delivery[1].cohort_id);
    }

    #[test]
    fn as_of_cutoff_is_enforced_on_receipts() {
        let mut data = profile();
        data.delivery_receipts[1].recorded_at = 101;
        let report = audit_civil_evidence(&data).unwrap();
        assert_eq!(report.delivery[0].disposition, ClaimDisposition::Indeterminate);
        assert!(report.delivery[0].violations.contains(&CivilViolation::DeliveryReceiptAfterCutoff));
    }

    #[test]
    fn action_state_regression_is_detected() {
        let mut data = profile();
        data.action_receipts[3].state = ActionState::Dispatched;
        let report = audit_civil_evidence(&data).unwrap();
        assert_eq!(report.actions[0].disposition, ClaimDisposition::Indeterminate);
        assert!(report.actions[0].violations.contains(&CivilViolation::ActionTransitionRegression));
    }

    #[test]
    fn base_compositional_claims_are_a_separate_required_gate() {
        let mut data = profile();
        data.required_base_claims.push(BaseClaimBinding {
            stage: Stage::AfterEventReview,
            claim_id: "missing-claim".into(),
        });
        let report = audit_civil_evidence(&data).unwrap();
        assert_eq!(report.base_audit.disposition, ClaimDisposition::Indeterminate);
        assert!(report.base_audit.violations.contains(&CivilViolation::BaseClaimMissing));
    }

    #[test]
    fn no_aggregate_score_hides_missing_receipts_or_actions() {
        let mut data = profile();
        data.action_receipts.clear();
        data.delivery_receipts.retain(|receipt| receipt.state == DeliveryState::ProviderAccepted);
        let report = audit_civil_evidence(&data).unwrap();
        assert_eq!(report.actions[0].disposition, ClaimDisposition::Unavailable);
        assert_eq!(report.delivery[0].disposition, ClaimDisposition::Indeterminate);
        assert!(report.to_csv().contains("action"));
        assert!(report.to_csv().contains("delivery"));
    }
}
