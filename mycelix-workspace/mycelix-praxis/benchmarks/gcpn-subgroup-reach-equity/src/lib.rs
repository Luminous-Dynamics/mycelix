//! Aggregate-only subgroup reach/equity assurance.
//!
//! This crate consumes the delivery assessments already produced by the civil
//! evidence auditor. It does not store person-level recipient identifiers or
//! infer unique people by summing channel delivery totals. Multi-channel reach
//! requires an external privacy-governed deduplication attestation.

use std::collections::BTreeSet;

use gcpn_civil_evidence_contracts::{CivilEvidenceReport, DeliveryAssessment};
use gcpn_compositional_assurance::ClaimDisposition;

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SubgroupDimension {
    /// A coarse, predeclared dimension key, for example "rurality" or
    /// "language". This is not a person identifier.
    pub dimension_id: String,
    /// A coarse category value, for example "rural" or "xitsonga".
    pub category_id: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DenominatorState {
    KnownInterval {
        lower: u64,
        upper: u64,
        source_id: String,
        source_version: String,
        recorded_at: u64,
    },
    Unknown {
        reason_id: String,
    },
    Suppressed {
        suppression_policy_id: String,
    },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum DeduplicationVerification {
    Verified,
    Unverified,
    Invalid,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DeduplicationAttestation {
    pub attestation_id: String,
    pub source_id: String,
    pub source_version: String,
    pub method_id: String,
    pub method_version: String,
    /// Exact channel set supplied to the external privacy-governed union/dedup
    /// process. No recipient-level tokens are retained in this contract.
    pub input_channel_ids: Vec<String>,
    /// Sum of the unique-per-channel delivered counts in the linked delivery
    /// assessments. This value can exceed the union because of overlap.
    pub channel_count_sum: u64,
    /// Attested unique union across all listed channels.
    pub unique_union_count: u64,
    /// Count of repeated deliveries removed from the channel sum:
    /// channel_count_sum - unique_union_count.
    pub overlap_removed: u64,
    /// Opaque digest/reference returned by the externally governed dedup
    /// service. This crate checks identity/format, not the cryptographic hash.
    pub attestation_digest: String,
    pub verification: DeduplicationVerification,
    pub event_time: u64,
    pub recorded_at: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct SubgroupReachEvidence {
    pub group_id: String,
    pub cohort_id: String,
    pub dimensions: Vec<SubgroupDimension>,
    /// Unique channel IDs expected to contribute to this subgroup's reach.
    pub channel_ids: Vec<String>,
    pub denominator: DenominatorState,
    /// Single-channel groups can derive unique reach directly from that
    /// channel's device-delivery count. Multi-channel groups require this.
    pub deduplication: Option<DeduplicationAttestation>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SubgroupReachPolicy {
    pub as_of_cutoff: u64,
    pub expected_denominator_source_id: String,
    pub expected_denominator_source_version: String,
    pub expected_dedup_source_id: String,
    pub expected_dedup_source_version: String,
    pub expected_dedup_method_id: String,
    pub expected_dedup_method_version: String,
    /// Policy target, not a universal regulatory standard.
    pub minimum_group_coverage: f64,
    /// Policy limit for the conservative upper bound on the maximum
    /// between-group coverage gap.
    pub maximum_coverage_gap: f64,
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum ReachViolation {
    InvalidIdentifier,
    DenominatorUnknown,
    DenominatorSuppressed,
    DenominatorSourceMismatch,
    DenominatorAfterCutoff,
    DenominatorIntervalInvalid,
    DeliveryAssessmentMissing,
    DeliveryChannelSetMismatch,
    DeliveryEvidenceUnavailable,
    DeliveryEvidenceNotSupporting,
    ChannelCountOverflow,
    DeliveryEvidenceNotAsOf,
    DeliveryEvidenceHasViolations,
    DuplicateChannelId,
    DeduplicationMissing,
    DeduplicationSourceMismatch,
    DeduplicationMethodMismatch,
    DeduplicationChannelSetMismatch,
    DeduplicationChannelSumMismatch,
    DeduplicationOverlapMismatch,
    DeduplicationUnverified,
    DeduplicationAfterCutoff,
    UniqueReachExceedsDenominator,
    NoSubgroups,
    InsufficientSubgroupsForGap,
    GroupEvidenceUnavailable,
}

impl ReachViolation {
    pub const fn label(self) -> &'static str {
        match self {
            Self::InvalidIdentifier => "invalid_identifier",
            Self::DenominatorUnknown => "denominator_unknown",
            Self::DenominatorSuppressed => "denominator_suppressed",
            Self::DenominatorSourceMismatch => "denominator_source_mismatch",
            Self::DenominatorAfterCutoff => "denominator_after_cutoff",
            Self::DenominatorIntervalInvalid => "denominator_interval_invalid",
            Self::DeliveryAssessmentMissing => "delivery_assessment_missing",
            Self::DeliveryChannelSetMismatch => "delivery_channel_set_mismatch",
            Self::DeliveryEvidenceUnavailable => "delivery_evidence_unavailable",
            Self::DeliveryEvidenceNotSupporting => "delivery_evidence_not_supporting",
            Self::ChannelCountOverflow => "channel_count_overflow",
            Self::DeliveryEvidenceNotAsOf => "delivery_evidence_not_as_of",
            Self::DeliveryEvidenceHasViolations => "delivery_evidence_has_violations",
            Self::DuplicateChannelId => "duplicate_channel_id",
            Self::DeduplicationMissing => "deduplication_missing",
            Self::DeduplicationSourceMismatch => "deduplication_source_mismatch",
            Self::DeduplicationMethodMismatch => "deduplication_method_mismatch",
            Self::DeduplicationChannelSetMismatch => "deduplication_channel_set_mismatch",
            Self::DeduplicationChannelSumMismatch => "deduplication_channel_sum_mismatch",
            Self::DeduplicationOverlapMismatch => "deduplication_overlap_mismatch",
            Self::DeduplicationUnverified => "deduplication_unverified",
            Self::DeduplicationAfterCutoff => "deduplication_after_cutoff",
            Self::UniqueReachExceedsDenominator => "unique_reach_exceeds_denominator",
            Self::NoSubgroups => "no_subgroups",
            Self::InsufficientSubgroupsForGap => "insufficient_subgroups_for_gap",
            Self::GroupEvidenceUnavailable => "group_evidence_unavailable",
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ReachError {
    InvalidIdentifier,
    DuplicateGroupId(String),
    DuplicateDimensionKey(String),
    DuplicateChannelId(String),
    DuplicateAttestationChannelId(String),
    InvalidPolicyThreshold,
}

impl std::fmt::Display for ReachError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidIdentifier => write!(formatter, "identifier is not canonical"),
            Self::DuplicateGroupId(value) => write!(formatter, "duplicate group ID: {value}"),
            Self::DuplicateDimensionKey(value) => write!(formatter, "duplicate subgroup dimension: {value}"),
            Self::DuplicateChannelId(value) => write!(formatter, "duplicate channel ID: {value}"),
            Self::DuplicateAttestationChannelId(value) => {
                write!(formatter, "duplicate attested channel ID: {value}")
            }
            Self::InvalidPolicyThreshold => write!(formatter, "coverage policy threshold must be finite and between 0 and 1"),
        }
    }
}

impl std::error::Error for ReachError {}

#[derive(Clone, Debug, PartialEq)]
pub struct SubgroupReachAssessment {
    pub group_id: String,
    pub cohort_id: String,
    pub dimensions: Vec<SubgroupDimension>,
    pub disposition: ClaimDisposition,
    pub denominator_lower: Option<u64>,
    pub denominator_upper: Option<u64>,
    pub channel_count_sum: Option<u64>,
    pub unique_reached: Option<u64>,
    pub overlap_removed: Option<u64>,
    /// Conservative coverage interval:
    /// lower = unique_reached / denominator_upper;
    /// upper = min(1, unique_reached / denominator_lower).
    pub coverage_lower_bound: Option<f64>,
    pub coverage_upper_bound: Option<f64>,
    pub violations: Vec<ReachViolation>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct SubgroupEquityReport {
    pub as_of_cutoff: u64,
    pub minimum_group_coverage_policy: f64,
    pub maximum_coverage_gap_policy: f64,
    pub groups: Vec<SubgroupReachAssessment>,
    pub assessed_group_count: usize,
    pub supports_group_count: usize,
    pub refutes_group_count: usize,
    pub indeterminate_group_count: usize,
    pub unavailable_group_count: usize,
    pub minimum_group_coverage_lower_bound: Option<f64>,
    pub maximum_group_coverage_upper_bound: Option<f64>,
    pub coverage_gap_lower_bound: Option<f64>,
    pub coverage_gap_upper_bound: Option<f64>,
    pub equity_disposition: ClaimDisposition,
    pub equity_violations: Vec<ReachViolation>,
}

impl SubgroupEquityReport {
    /// Flat deterministic CSV. Suppressed/unknown populations have no
    /// denominator or reach counts in their output row.
    pub fn to_csv(&self) -> String {
        let mut output = String::from(
            "record_type,group_id,cohort_id,dimensions,disposition,denominator_lower,denominator_upper,channel_count_sum,unique_reached,overlap_removed,coverage_lower,coverage_upper,violations\n",
        );
        for group in &self.groups {
            let dimensions = group
                .dimensions
                .iter()
                .map(|item| format!("{}={}", item.dimension_id, item.category_id))
                .collect::<Vec<_>>()
                .join("|");
            output.push_str(&format!(
                "group,{},{},{},{},{},{},{},{},{},{:.8},{:.8},{}\n",
                group.group_id,
                group.cohort_id,
                dimensions,
                disposition_label(group.disposition),
                optional_u64(group.denominator_lower),
                optional_u64(group.denominator_upper),
                optional_u64(group.channel_count_sum),
                optional_u64(group.unique_reached),
                optional_u64(group.overlap_removed),
                group.coverage_lower_bound.unwrap_or(-1.0),
                group.coverage_upper_bound.unwrap_or(-1.0),
                violation_labels(&group.violations)
            ));
        }
        output.push_str(&format!(
            "summary,equity,all_groups,as_of={}, {},{},{},{},{},{:.8},{:.8},{}\n",
            self.as_of_cutoff,
            disposition_label(self.equity_disposition),
            self.assessed_group_count,
            self.supports_group_count,
            self.refutes_group_count,
            self.indeterminate_group_count,
            self.unavailable_group_count,
            self.minimum_group_coverage_lower_bound.unwrap_or(-1.0),
            self.coverage_gap_upper_bound.unwrap_or(-1.0),
            violation_labels(&self.equity_violations)
        ));
        output.push_str(&format!(
            "bounds,equity_gap,conservative,{},{},{},{},{},{},{:.8},{:.8},{}\n",
            self.as_of_cutoff,
            disposition_label(self.equity_disposition),
            self.assessed_group_count,
            self.supports_group_count,
            self.refutes_group_count,
            self.unavailable_group_count,
            self.coverage_gap_lower_bound.unwrap_or(-1.0),
            self.coverage_gap_upper_bound.unwrap_or(-1.0),
            violation_labels(&self.equity_violations)
        ));
        output
    }
}

pub fn audit_subgroup_reach(
    delivery_report: &CivilEvidenceReport,
    groups: &[SubgroupReachEvidence],
    policy: &SubgroupReachPolicy,
) -> Result<SubgroupEquityReport, ReachError> {
    validate_policy(policy)?;
    validate_group_identifiers(groups)?;

    if groups.is_empty() {
        return Ok(SubgroupEquityReport {
            as_of_cutoff: policy.as_of_cutoff,
            minimum_group_coverage_policy: policy.minimum_group_coverage,
            maximum_coverage_gap_policy: policy.maximum_coverage_gap,
            groups: Vec::new(),
            assessed_group_count: 0,
            supports_group_count: 0,
            refutes_group_count: 0,
            indeterminate_group_count: 0,
            unavailable_group_count: 0,
            minimum_group_coverage_lower_bound: None,
            maximum_group_coverage_upper_bound: None,
            coverage_gap_lower_bound: None,
            coverage_gap_upper_bound: None,
            equity_disposition: ClaimDisposition::Unavailable,
            equity_violations: vec![ReachViolation::NoSubgroups],
        });
    }

    let assessments: Vec<_> = groups
        .iter()
        .map(|group| assess_group(delivery_report, group, policy))
        .collect();

    Ok(aggregate_equity(assessments, policy))
}

fn validate_policy(policy: &SubgroupReachPolicy) -> Result<(), ReachError> {
    if !policy.minimum_group_coverage.is_finite()
        || !policy.maximum_coverage_gap.is_finite()
        || !(0.0..=1.0).contains(&policy.minimum_group_coverage)
        || !(0.0..=1.0).contains(&policy.maximum_coverage_gap)
    {
        return Err(ReachError::InvalidPolicyThreshold);
    }
    for value in [
        policy.expected_denominator_source_id.as_str(),
        policy.expected_denominator_source_version.as_str(),
        policy.expected_dedup_source_id.as_str(),
        policy.expected_dedup_source_version.as_str(),
        policy.expected_dedup_method_id.as_str(),
        policy.expected_dedup_method_version.as_str(),
    ] {
        if !valid_identifier(value) {
            return Err(ReachError::InvalidIdentifier);
        }
    }
    Ok(())
}

fn validate_group_identifiers(groups: &[SubgroupReachEvidence]) -> Result<(), ReachError> {
    let mut group_ids = BTreeSet::new();
    for group in groups {
        for value in [group.group_id.as_str(), group.cohort_id.as_str()] {
            if !valid_identifier(value) {
                return Err(ReachError::InvalidIdentifier);
            }
        }
        if !group_ids.insert(group.group_id.as_str()) {
            return Err(ReachError::DuplicateGroupId(group.group_id.clone()));
        }

        let mut dimension_ids = BTreeSet::new();
        for dimension in &group.dimensions {
            if !valid_identifier(&dimension.dimension_id)
                || !valid_identifier(&dimension.category_id)
            {
                return Err(ReachError::InvalidIdentifier);
            }
            if !dimension_ids.insert(dimension.dimension_id.as_str()) {
                return Err(ReachError::DuplicateDimensionKey(
                    dimension.dimension_id.clone(),
                ));
            }
        }

        let mut channels = BTreeSet::new();
        for channel in &group.channel_ids {
            if !valid_identifier(channel) {
                return Err(ReachError::InvalidIdentifier);
            }
            if !channels.insert(channel.as_str()) {
                return Err(ReachError::DuplicateChannelId(channel.clone()));
            }
        }
        if group.channel_ids.is_empty() {
            return Err(ReachError::InvalidIdentifier);
        }

        if let Some(attestation) = &group.deduplication {
            let mut attested_channels = BTreeSet::new();
            for channel in &attestation.input_channel_ids {
                if !valid_identifier(channel) {
                    return Err(ReachError::InvalidIdentifier);
                }
                if !attested_channels.insert(channel.as_str()) {
                    return Err(ReachError::DuplicateAttestationChannelId(
                        channel.clone(),
                    ));
                }
            }
            for value in [
                attestation.attestation_id.as_str(),
                attestation.source_id.as_str(),
                attestation.source_version.as_str(),
                attestation.method_id.as_str(),
                attestation.method_version.as_str(),
                attestation.attestation_digest.as_str(),
            ] {
                if !valid_identifier(value) {
                    return Err(ReachError::InvalidIdentifier);
                }
            }
        }
        if let DenominatorState::KnownInterval {
            lower,
            upper,
            source_id,
            source_version,
            ..
        } = &group.denominator
        {
            if !valid_identifier(source_id) || !valid_identifier(source_version) {
                return Err(ReachError::InvalidIdentifier);
            }
            if *lower == 0 || *upper < *lower {
                // Malformed interval is represented in the claim output, not
                // as a panicking or silently repaired population estimate.
            }
        }
        if let DenominatorState::Unknown { reason_id } = &group.denominator {
            if !valid_identifier(reason_id) {
                return Err(ReachError::InvalidIdentifier);
            }
        }
        if let DenominatorState::Suppressed {
            suppression_policy_id,
        } = &group.denominator
        {
            if !valid_identifier(suppression_policy_id) {
                return Err(ReachError::InvalidIdentifier);
            }
        }
    }
    Ok(())
}

fn assess_group(
    delivery_report: &CivilEvidenceReport,
    group: &SubgroupReachEvidence,
    policy: &SubgroupReachPolicy,
) -> SubgroupReachAssessment {
    let mut violations = BTreeSet::new();

    let (denominator_lower, denominator_upper) = match &group.denominator {
            DenominatorState::KnownInterval {
                lower,
                upper,
                source_id,
                source_version,
                recorded_at,
            } => {
                if source_id != &policy.expected_denominator_source_id
                    || source_version != &policy.expected_denominator_source_version
                {
                    violations.insert(ReachViolation::DenominatorSourceMismatch);
                }
                if *recorded_at > policy.as_of_cutoff {
                    violations.insert(ReachViolation::DenominatorAfterCutoff);
                }
                if *lower == 0 || *upper < *lower {
                    violations.insert(ReachViolation::DenominatorIntervalInvalid);
                }
                (Some(*lower), Some(*upper))
            }
            DenominatorState::Unknown { .. } => {
                return unavailable_assessment(
                    group,
                    ReachViolation::DenominatorUnknown,
                );
            }
            DenominatorState::Suppressed { .. } => {
                return unavailable_assessment(
                    group,
                    ReachViolation::DenominatorSuppressed,
                );
            }
        };

    if !violations.is_empty() {
        return assessment_without_coverage(
            group,
            ClaimDisposition::Indeterminate,
            denominator_lower,
            denominator_upper,
            violations.into_iter().collect(),
        );
    }
    if delivery_report.as_of_cutoff != policy.as_of_cutoff {
        return assessment_without_coverage(
            group,
            ClaimDisposition::Indeterminate,
            denominator_lower,
            denominator_upper,
            vec![ReachViolation::DeliveryEvidenceNotAsOf],
        );
    }

    let reported_channels: BTreeSet<_> = delivery_report
        .delivery
        .iter()
        .filter(|assessment| assessment.cohort_id == group.cohort_id)
        .map(|assessment| assessment.channel_id.as_str())
        .collect();
    let expected_channels: BTreeSet<_> =
        group.channel_ids.iter().map(String::as_str).collect();
    if reported_channels != expected_channels {
        return assessment_without_coverage(
            group,
            ClaimDisposition::Indeterminate,
            denominator_lower,
            denominator_upper,
            vec![ReachViolation::DeliveryChannelSetMismatch],
        );
    }

    let mut channel_count_sum = 0u64;
    for channel_id in &group.channel_ids {
        let Some(assessment) = delivery_report.delivery.iter().find(|assessment| {
            assessment.cohort_id == group.cohort_id
                && assessment.channel_id == *channel_id
        }) else {
            violations.insert(ReachViolation::DeliveryAssessmentMissing);
            continue;
        };
        if assessment.disposition == ClaimDisposition::Unavailable
            || assessment.disposition == ClaimDisposition::OutOfScope
        {
            violations.insert(ReachViolation::DeliveryEvidenceUnavailable);
        }
        if assessment.disposition == ClaimDisposition::Refutes {
            violations.insert(ReachViolation::DeliveryEvidenceNotSupporting);
        }
        if !assessment.violations.is_empty() {
            violations.insert(ReachViolation::DeliveryEvidenceHasViolations);
        }
        let Some(next_sum) = channel_count_sum.checked_add(assessment.counts.device_delivered) else {
            violations.insert(ReachViolation::ChannelCountOverflow);
            continue;
        };
        channel_count_sum = next_sum;
    }

    if violations.contains(&ReachViolation::DeliveryAssessmentMissing)
        || violations.contains(&ReachViolation::DeliveryEvidenceUnavailable)
    {
        return assessment_without_coverage(
            group,
            ClaimDisposition::Unavailable,
            denominator_lower,
            denominator_upper,
            violations.into_iter().collect(),
        );
    }
    if !violations.is_empty() {
        return assessment_without_coverage(
            group,
            ClaimDisposition::Indeterminate,
            denominator_lower,
            denominator_upper,
            violations.into_iter().collect(),
        );
    }

    let unique_reached = if group.channel_ids.len() == 1 {
        match &group.deduplication {
            None => channel_count_sum,
            Some(attestation) => match validate_deduplication(
                attestation,
                group,
                policy,
                channel_count_sum,
            ) {
                Ok(()) => attestation.unique_union_count,
                Err(violation) => {
                    return assessment_without_coverage(
                        group,
                        ClaimDisposition::Indeterminate,
                        denominator_lower,
                        denominator_upper,
                        vec![violation],
                    );
                }
            },
        }
    } else {
        let Some(attestation) = &group.deduplication else {
            return assessment_without_coverage(
                group,
                ClaimDisposition::Unavailable,
                denominator_lower,
                denominator_upper,
                vec![ReachViolation::DeduplicationMissing],
            );
        };
        match validate_deduplication(attestation, group, policy, channel_count_sum) {
            Ok(()) => attestation.unique_union_count,
            Err(violation) => {
                return assessment_without_coverage(
                    group,
                    ClaimDisposition::Indeterminate,
                    denominator_lower,
                    denominator_upper,
                    vec![violation],
                );
            }
        }
    };

    let denominator_lower_value = denominator_lower.unwrap_or(0);
    let denominator_upper_value = denominator_upper.unwrap_or(0);
    if unique_reached > denominator_upper_value {
        return assessment_without_coverage(
            group,
            ClaimDisposition::Indeterminate,
            denominator_lower,
            denominator_upper,
            vec![ReachViolation::UniqueReachExceedsDenominator],
        );
    }

    let coverage_lower = unique_reached as f64 / denominator_upper_value as f64;
    let coverage_upper = (unique_reached as f64 / denominator_lower_value as f64).min(1.0);
    let disposition = if coverage_lower >= policy.minimum_group_coverage {
        ClaimDisposition::Supports
    } else if coverage_upper < policy.minimum_group_coverage {
        ClaimDisposition::Refutes
    } else {
        ClaimDisposition::Indeterminate
    };
    let overlap_removed = channel_count_sum.saturating_sub(unique_reached);

    SubgroupReachAssessment {
        group_id: group.group_id.clone(),
        cohort_id: group.cohort_id.clone(),
        dimensions: group.dimensions.clone(),
        disposition,
        denominator_lower,
        denominator_upper,
        channel_count_sum: Some(channel_count_sum),
        unique_reached: Some(unique_reached),
        overlap_removed: Some(overlap_removed),
        coverage_lower_bound: Some(coverage_lower),
        coverage_upper_bound: Some(coverage_upper),
        violations: Vec::new(),
    }
}

fn validate_deduplication(
    attestation: &DeduplicationAttestation,
    group: &SubgroupReachEvidence,
    policy: &SubgroupReachPolicy,
    expected_channel_sum: u64,
) -> Result<(), ReachViolation> {
    if attestation.source_id != policy.expected_dedup_source_id
        || attestation.source_version != policy.expected_dedup_source_version
    {
        return Err(ReachViolation::DeduplicationSourceMismatch);
    }
    if attestation.method_id != policy.expected_dedup_method_id
        || attestation.method_version != policy.expected_dedup_method_version
    {
        return Err(ReachViolation::DeduplicationMethodMismatch);
    }
    if attestation.verification != DeduplicationVerification::Verified {
        return Err(ReachViolation::DeduplicationUnverified);
    }
    let expected_channels: BTreeSet<_> = group.channel_ids.iter().collect();
    let attested_channels: BTreeSet<_> = attestation.input_channel_ids.iter().collect();
    if expected_channels != attested_channels {
        return Err(ReachViolation::DeduplicationChannelSetMismatch);
    }
    if attestation.recorded_at > policy.as_of_cutoff
        || attestation.event_time > policy.as_of_cutoff
    {
        return Err(ReachViolation::DeduplicationAfterCutoff);
    }
    if attestation.channel_count_sum != expected_channel_sum {
        return Err(ReachViolation::DeduplicationChannelSumMismatch);
    }
    if attestation.unique_union_count > attestation.channel_count_sum
        || attestation.overlap_removed
            != attestation
                .channel_count_sum
                .saturating_sub(attestation.unique_union_count)
        || (group.channel_ids.len() == 1 && attestation.overlap_removed != 0)
    {
        return Err(ReachViolation::DeduplicationOverlapMismatch);
    }
    Ok(())
}

fn unavailable_assessment(
    group: &SubgroupReachEvidence,
    violation: ReachViolation,
) -> SubgroupReachAssessment {
    assessment_without_coverage(
        group,
        ClaimDisposition::Unavailable,
        None,
        None,
        vec![violation],
    )
}

fn assessment_without_coverage(
    group: &SubgroupReachEvidence,
    disposition: ClaimDisposition,
    denominator_lower: Option<u64>,
    denominator_upper: Option<u64>,
    violations: Vec<ReachViolation>,
) -> SubgroupReachAssessment {
    SubgroupReachAssessment {
        group_id: group.group_id.clone(),
        cohort_id: group.cohort_id.clone(),
        dimensions: group.dimensions.clone(),
        disposition,
        denominator_lower,
        denominator_upper,
        channel_count_sum: None,
        unique_reached: None,
        overlap_removed: None,
        coverage_lower_bound: None,
        coverage_upper_bound: None,
        violations,
    }
}

fn aggregate_equity(
    groups: Vec<SubgroupReachAssessment>,
    policy: &SubgroupReachPolicy,
) -> SubgroupEquityReport {
    let assessed_group_count = groups.len();
    let supports_group_count = groups
        .iter()
        .filter(|group| group.disposition == ClaimDisposition::Supports)
        .count();
    let refutes_group_count = groups
        .iter()
        .filter(|group| group.disposition == ClaimDisposition::Refutes)
        .count();
    let indeterminate_group_count = groups
        .iter()
        .filter(|group| group.disposition == ClaimDisposition::Indeterminate)
        .count();
    let unavailable_group_count = groups
        .iter()
        .filter(|group| group.disposition == ClaimDisposition::Unavailable)
        .count();

    let intervals: Vec<(f64, f64)> = groups
        .iter()
        .filter_map(|group| {
            Some((
                group.coverage_lower_bound?,
                group.coverage_upper_bound?,
            ))
        })
        .collect();

    let minimum_group_coverage_lower_bound = intervals
        .iter()
        .map(|interval| interval.0)
        .min_by(f64::total_cmp);
    let maximum_group_coverage_upper_bound = intervals
        .iter()
        .map(|interval| interval.1)
        .max_by(f64::total_cmp);

    let coverage_gap_lower_bound = if intervals.len() >= 2 {
        let maximum_lower = intervals
            .iter()
            .map(|interval| interval.0)
            .max_by(f64::total_cmp)
            .unwrap_or(0.0);
        let minimum_upper = intervals
            .iter()
            .map(|interval| interval.1)
            .min_by(f64::total_cmp)
            .unwrap_or(0.0);
        Some((maximum_lower - minimum_upper).max(0.0))
    } else {
        None
    };

    let coverage_gap_upper_bound = if intervals.len() >= 2 {
        let maximum_upper = intervals
            .iter()
            .map(|interval| interval.1)
            .max_by(f64::total_cmp)
            .unwrap_or(0.0);
        let minimum_lower = intervals
            .iter()
            .map(|interval| interval.0)
            .min_by(f64::total_cmp)
            .unwrap_or(0.0);
        Some((maximum_upper - minimum_lower).clamp(0.0, 1.0))
    } else {
        None
    };

    let mut equity_violations = BTreeSet::new();
    let unqualified_groups = indeterminate_group_count + unavailable_group_count;
    if unqualified_groups > 0 {
        equity_violations.insert(ReachViolation::GroupEvidenceUnavailable);
    }
    if intervals.len() < 2 {
        equity_violations.insert(ReachViolation::InsufficientSubgroupsForGap);
    }

    let equity_disposition = if intervals.is_empty() {
        ClaimDisposition::Unavailable
    } else if unqualified_groups > 0 || intervals.len() < 2 {
        ClaimDisposition::Indeterminate
    } else if refutes_group_count > 0
        || coverage_gap_lower_bound
            .is_some_and(|gap| gap > policy.maximum_coverage_gap)
    {
        ClaimDisposition::Refutes
    } else if supports_group_count == assessed_group_count
        && coverage_gap_upper_bound
            .is_some_and(|gap| gap <= policy.maximum_coverage_gap)
    {
        ClaimDisposition::Supports
    } else {
        ClaimDisposition::Indeterminate
    };

    SubgroupEquityReport {
        as_of_cutoff: policy.as_of_cutoff,
        minimum_group_coverage_policy: policy.minimum_group_coverage,
        maximum_coverage_gap_policy: policy.maximum_coverage_gap,
        groups,
        assessed_group_count,
        supports_group_count,
        refutes_group_count,
        indeterminate_group_count,
        unavailable_group_count,
        minimum_group_coverage_lower_bound,
        maximum_group_coverage_upper_bound,
        coverage_gap_lower_bound,
        coverage_gap_upper_bound,
        equity_disposition,
        equity_violations: equity_violations.into_iter().collect(),
    }
}

fn optional_u64(value: Option<u64>) -> String {
    value.map(|number| number.to_string()).unwrap_or_default()
}

fn violation_labels(violations: &[ReachViolation]) -> String {
    violations
        .iter()
        .map(|violation| violation.label())
        .collect::<Vec<_>>()
        .join("|")
}

fn disposition_label(disposition: ClaimDisposition) -> &'static str {
    match disposition {
        ClaimDisposition::Supports => "supports",
        ClaimDisposition::Refutes => "refutes",
        ClaimDisposition::Indeterminate => "indeterminate",
        ClaimDisposition::OutOfScope => "out_of_scope",
        ClaimDisposition::Unavailable => "unavailable",
    }
}

fn valid_identifier(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 128
        && value.bytes().all(|byte| byte.is_ascii_alphanumeric() || b"-_.:".contains(&byte))
}

#[cfg(test)]
mod tests {
    use super::*;
    use gcpn_civil_evidence_contracts::{DeliveryCounts, DeliveryAssessment};

    fn delivery_report(deliveries: Vec<DeliveryAssessment>) -> CivilEvidenceReport {
        CivilEvidenceReport {
            scenario_id: "synthetic-reach".into(),
            as_of_cutoff: 100,
            base_audit: gcpn_civil_evidence_contracts::BaseAuditAssessment {
                disposition: ClaimDisposition::Supports,
                required_claims: 1,
                supporting_claims: 1,
                violations: Vec::new(),
            },
            authority: gcpn_civil_evidence_contracts::AuthorityAssessment {
                disposition: ClaimDisposition::Supports,
                assertion_id: Some("authority-1".into()),
                registry_status: Some(gcpn_civil_evidence_contracts::RegistryAuthorityStatus::Recognized),
                signature_status: Some(gcpn_civil_evidence_contracts::SignatureStatus::Verified),
                violations: Vec::new(),
            },
            delivery: deliveries,
            accessibility: Vec::new(),
            comprehension: Vec::new(),
            actions: Vec::new(),
            case_frame: gcpn_civil_evidence_contracts::CaseFrameAssessment {
                frame_id: "frame-1".into(),
                disposition: ClaimDisposition::Supports,
                eligible_cases: 1,
                observed_cases: 1,
                missing_cases: 0,
                disputed_cases: 0,
                excluded_cases: 0,
                violations: Vec::new(),
            },
        }
    }

    fn delivery(cohort: &str, channel: &str, delivered: u64) -> DeliveryAssessment {
        DeliveryAssessment {
            cohort_id: cohort.into(),
            channel_id: channel.into(),
            disposition: if delivered == 100 {
                ClaimDisposition::Supports
            } else {
                ClaimDisposition::Indeterminate
            },
            counts: DeliveryCounts {
                intended_receivers: 100,
                provider_accepted: 100,
                device_delivered: delivered,
                failed: 0,
                delayed: 0,
                unknown: 0,
                duplicates: 0,
                not_confirmed_device_delivered: 100 - delivered,
            },
            violations: Vec::new(),
        }
    }

    fn policy() -> SubgroupReachPolicy {
        SubgroupReachPolicy {
            as_of_cutoff: 100,
            expected_denominator_source_id: "census-source".into(),
            expected_denominator_source_version: "release-1".into(),
            expected_dedup_source_id: "privacy-dedup".into(),
            expected_dedup_source_version: "v1".into(),
            expected_dedup_method_id: "aggregate-union-v1".into(),
            expected_dedup_method_version: "1".into(),
            minimum_group_coverage: 0.95,
            maximum_coverage_gap: 0.05,
        }
    }

    fn subgroup(
        id: &str,
        cohort: &str,
        channels: &[&str],
        denominator_lower: u64,
        denominator_upper: u64,
    ) -> SubgroupReachEvidence {
        SubgroupReachEvidence {
            group_id: id.into(),
            cohort_id: cohort.into(),
            dimensions: vec![SubgroupDimension {
                dimension_id: "rurality".into(),
                category_id: id.into(),
            }],
            channel_ids: channels.iter().map(|channel| (*channel).into()).collect(),
            denominator: DenominatorState::KnownInterval {
                lower: denominator_lower,
                upper: denominator_upper,
                source_id: "census-source".into(),
                source_version: "release-1".into(),
                recorded_at: 80,
            },
            deduplication: None,
        }
    }

    fn attestation(
        channels: &[&str],
        channel_sum: u64,
        unique: u64,
    ) -> DeduplicationAttestation {
        DeduplicationAttestation {
            attestation_id: "dedup-attestation-1".into(),
            source_id: "privacy-dedup".into(),
            source_version: "v1".into(),
            method_id: "aggregate-union-v1".into(),
            method_version: "1".into(),
            input_channel_ids: channels.iter().map(|channel| (*channel).into()).collect(),
            channel_count_sum: channel_sum,
            unique_union_count: unique,
            overlap_removed: channel_sum - unique,
            attestation_digest: "sha256-opaque-digest".into(),
            verification: DeduplicationVerification::Verified,
            event_time: 90,
            recorded_at: 95,
        }
    }

    #[test]
    fn single_channel_reach_uses_device_receipt_count_without_deduplication() {
        let report = delivery_report(vec![delivery("group-rural", "cell-broadcast", 98)]);
        let group = subgroup("rural", "group-rural", &["cell-broadcast"], 100, 100);
        let result = audit_subgroup_reach(&report, &[group], &policy()).unwrap();
        let assessment = &result.groups[0];
        assert_eq!(assessment.unique_reached, Some(98));
        assert_eq!(assessment.coverage_lower_bound, Some(0.98));
        assert_eq!(assessment.disposition, ClaimDisposition::Supports);
    }

    #[test]
    fn multi_channel_reach_uses_attested_union_not_naive_sum() {
        let report = delivery_report(vec![
            delivery("group-rural", "cell-broadcast", 100),
            delivery("group-rural", "sms", 80),
        ]);
        let mut group = subgroup(
            "rural",
            "group-rural",
            &["cell-broadcast", "sms"],
            200,
            200,
        );
        group.deduplication = Some(attestation(&["cell-broadcast", "sms"], 180, 140));
        let result = audit_subgroup_reach(&report, &[group], &policy()).unwrap();
        let assessment = &result.groups[0];
        assert_eq!(assessment.channel_count_sum, Some(180));
        assert_eq!(assessment.unique_reached, Some(140));
        assert_eq!(assessment.overlap_removed, Some(40));
        assert_eq!(assessment.coverage_lower_bound, Some(0.7));
        assert_eq!(assessment.disposition, ClaimDisposition::Refutes);
    }

    #[test]
    fn unknown_population_denominator_is_unavailable_not_imputed() {
        let report = delivery_report(vec![delivery("group-unknown", "sms", 90)]);
        let mut group = subgroup("unknown", "group-unknown", &["sms"], 100, 100);
        group.denominator = DenominatorState::Unknown {
            reason_id: "census-gap".into(),
        };
        let result = audit_subgroup_reach(&report, &[group], &policy()).unwrap();
        let assessment = &result.groups[0];
        assert_eq!(assessment.disposition, ClaimDisposition::Unavailable);
        assert_eq!(assessment.denominator_lower, None);
        assert_eq!(assessment.unique_reached, None);
    }

    #[test]
    fn suppressed_small_group_does_not_leak_counts() {
        let report = delivery_report(vec![delivery("small-group", "sms", 7)]);
        let mut group = subgroup("small", "small-group", &["sms"], 10, 12);
        group.denominator = DenominatorState::Suppressed {
            suppression_policy_id: "small-cell-threshold-10".into(),
        };
        let result = audit_subgroup_reach(&report, &[group], &policy()).unwrap();
        let assessment = &result.groups[0];
        assert_eq!(assessment.disposition, ClaimDisposition::Unavailable);
        assert_eq!(assessment.denominator_lower, None);
        assert_eq!(assessment.unique_reached, None);
        assert_eq!(assessment.channel_count_sum, None);
    }

    #[test]
    fn missing_multi_channel_deduplication_is_unavailable() {
        let report = delivery_report(vec![
            delivery("cohort-a", "cell-broadcast", 100),
            delivery("cohort-a", "sms", 90),
        ]);
        let group = subgroup("group-a", "cohort-a", &["cell-broadcast", "sms"], 100, 110);
        let result = audit_subgroup_reach(&report, &[group], &policy()).unwrap();
        assert_eq!(result.groups[0].disposition, ClaimDisposition::Unavailable);
        assert!(result.groups[0].violations.contains(&ReachViolation::DeduplicationMissing));
    }

    #[test]
    fn mismatched_dedup_channel_set_is_indeterminate() {
        let report = delivery_report(vec![
            delivery("cohort-a", "cell-broadcast", 100),
            delivery("cohort-a", "sms", 90),
        ]);
        let mut group = subgroup("group-a", "cohort-a", &["cell-broadcast", "sms"], 100, 110);
        group.deduplication = Some(attestation(&["cell-broadcast", "other-channel"], 190, 150));
        let result = audit_subgroup_reach(&report, &[group], &policy()).unwrap();
        assert_eq!(result.groups[0].disposition, ClaimDisposition::Indeterminate);
        assert!(result.groups[0]
            .violations
            .contains(&ReachViolation::DeduplicationChannelSetMismatch));
    }

    #[test]
    fn mismatched_channel_sum_is_indeterminate() {
        let report = delivery_report(vec![
            delivery("cohort-a", "cell-broadcast", 100),
            delivery("cohort-a", "sms", 90),
        ]);
        let mut group = subgroup("group-a", "cohort-a", &["cell-broadcast", "sms"], 100, 110);
        group.deduplication = Some(attestation(&["cell-broadcast", "sms"], 180, 150));
        let result = audit_subgroup_reach(&report, &[group], &policy()).unwrap();
        assert_eq!(result.groups[0].disposition, ClaimDisposition::Indeterminate);
        assert!(result.groups[0]
            .violations
            .contains(&ReachViolation::DeduplicationChannelSumMismatch));
    }

    #[test]
    fn inconsistent_overlap_adjustment_is_indeterminate() {
        let report = delivery_report(vec![
            delivery("cohort-a", "cell-broadcast", 100),
            delivery("cohort-a", "sms", 90),
        ]);
        let mut record = attestation(&["cell-broadcast", "sms"], 190, 150);
        record.overlap_removed = 20;
        let mut group = subgroup("group-a", "cohort-a", &["cell-broadcast", "sms"], 100, 110);
        group.deduplication = Some(record);
        let result = audit_subgroup_reach(&report, &[group], &policy()).unwrap();
        assert_eq!(result.groups[0].disposition, ClaimDisposition::Indeterminate);
        assert!(result.groups[0]
            .violations
            .contains(&ReachViolation::DeduplicationOverlapMismatch));
    }

    #[test]
    fn unverified_dedup_attestation_cannot_support_reach() {
        let report = delivery_report(vec![
            delivery("cohort-a", "cell-broadcast", 100),
            delivery("cohort-a", "sms", 90),
        ]);
        let mut record = attestation(&["cell-broadcast", "sms"], 190, 150);
        record.verification = DeduplicationVerification::Unverified;
        let mut group = subgroup("group-a", "cohort-a", &["cell-broadcast", "sms"], 100, 110);
        group.deduplication = Some(record);
        let result = audit_subgroup_reach(&report, &[group], &policy()).unwrap();
        assert_eq!(result.groups[0].disposition, ClaimDisposition::Indeterminate);
        assert!(result.groups[0]
            .violations
            .contains(&ReachViolation::DeduplicationUnverified));
    }

    #[test]
    fn post_cutoff_dedup_attestation_is_not_used() {
        let report = delivery_report(vec![
            delivery("cohort-a", "cell-broadcast", 100),
            delivery("cohort-a", "sms", 90),
        ]);
        let mut record = attestation(&["cell-broadcast", "sms"], 190, 150);
        record.recorded_at = 101;
        let mut group = subgroup("group-a", "cohort-a", &["cell-broadcast", "sms"], 100, 110);
        group.deduplication = Some(record);
        let result = audit_subgroup_reach(&report, &[group], &policy()).unwrap();
        assert_eq!(result.groups[0].disposition, ClaimDisposition::Indeterminate);
        assert!(result.groups[0]
            .violations
            .contains(&ReachViolation::DeduplicationAfterCutoff));
    }

    #[test]
    fn denominator_interval_produces_conservative_coverage_bounds() {
        let report = delivery_report(vec![delivery("cohort-a", "sms", 90)]);
        let group = subgroup("group-a", "cohort-a", &["sms"], 90, 120);
        let result = audit_subgroup_reach(&report, &[group], &policy()).unwrap();
        let assessment = &result.groups[0];
        assert!((assessment.coverage_lower_bound.unwrap() - 0.75).abs() < 1e-12);
        assert!((assessment.coverage_upper_bound.unwrap() - 1.0).abs() < 1e-12);
        assert_eq!(assessment.disposition, ClaimDisposition::Indeterminate);
    }

    #[test]
    fn overall_high_reach_cannot_hide_one_subgroup_below_policy() {
        let report = delivery_report(vec![
            delivery("urban", "sms", 100),
            delivery("rural", "sms", 30),
        ]);
        let groups = vec![
            subgroup("urban", "urban", &["sms"], 100, 100),
            subgroup("rural", "rural", &["sms"], 100, 100),
        ];
        let result = audit_subgroup_reach(&report, &groups, &policy()).unwrap();
        assert_eq!(result.groups[0].disposition, ClaimDisposition::Supports);
        assert_eq!(result.groups[1].disposition, ClaimDisposition::Refutes);
        assert_eq!(result.minimum_group_coverage_lower_bound, Some(0.3));
        assert_eq!(result.coverage_gap_upper_bound, Some(0.7));
        assert_eq!(result.equity_disposition, ClaimDisposition::Refutes);
    }

    #[test]
    fn exact_target_boundary_supports_when_lower_bound_meets_policy() {
        let report = delivery_report(vec![delivery("cohort-a", "sms", 95)]);
        let group = subgroup("group-a", "cohort-a", &["sms"], 100, 100);
        let result = audit_subgroup_reach(&report, &[group], &policy()).unwrap();
        assert_eq!(result.groups[0].coverage_lower_bound, Some(0.95));
        assert_eq!(result.groups[0].disposition, ClaimDisposition::Supports);
    }

    #[test]
    fn unknown_group_prevents_overall_equity_support() {
        let report = delivery_report(vec![
            delivery("urban", "sms", 100),
            delivery("rural", "sms", 95),
        ]);
        let mut rural = subgroup("rural", "rural", &["sms"], 100, 100);
        rural.denominator = DenominatorState::Unknown {
            reason_id: "outdated-local-register".into(),
        };
        let groups = vec![
            subgroup("urban", "urban", &["sms"], 100, 100),
            rural,
        ];
        let result = audit_subgroup_reach(&report, &groups, &policy()).unwrap();
        assert_eq!(result.groups[0].disposition, ClaimDisposition::Supports);
        assert_eq!(result.groups[1].disposition, ClaimDisposition::Unavailable);
        assert_eq!(result.equity_disposition, ClaimDisposition::Indeterminate);
    }

    #[test]
    fn delivery_evidence_with_violations_is_not_counted_as_reach() {
        let mut receipt = delivery("cohort-a", "sms", 100);
        receipt.violations = vec![
            gcpn_civil_evidence_contracts::CivilViolation::DeliveryReceiptAfterCutoff,
        ];
        let report = delivery_report(vec![receipt]);
        let group = subgroup("group-a", "cohort-a", &["sms"], 100, 100);
        let result = audit_subgroup_reach(&report, &[group], &policy()).unwrap();
        assert_eq!(result.groups[0].disposition, ClaimDisposition::Indeterminate);
        assert_eq!(result.groups[0].unique_reached, None);
        assert!(result.groups[0]
            .violations
            .contains(&ReachViolation::DeliveryEvidenceHasViolations));
    }

    #[test]
    fn duplicate_subgroup_ids_and_channel_ids_fail_closed() {
        let report = delivery_report(vec![delivery("cohort-a", "sms", 100)]);
        let group = subgroup("group-a", "cohort-a", &["sms"], 100, 100);
        assert!(matches!(
            audit_subgroup_reach(&report, &[group.clone(), group], &policy()),
            Err(ReachError::DuplicateGroupId(_))
        ));

        let mut duplicate_channel = subgroup(
            "group-b",
            "cohort-a",
            &["sms", "sms"],
            100,
            100,
        );
        duplicate_channel.group_id = "group-b".into();
        assert!(matches!(
            audit_subgroup_reach(&report, &[duplicate_channel], &policy()),
            Err(ReachError::DuplicateChannelId(_))
        ));
    }

    #[test]
    fn no_groups_is_unavailable_not_zero_coverage() {
        let report = delivery_report(Vec::new());
        let result = audit_subgroup_reach(&report, &[], &policy()).unwrap();
        assert_eq!(result.equity_disposition, ClaimDisposition::Unavailable);
        assert_eq!(result.minimum_group_coverage_lower_bound, None);
        assert!(result.equity_violations.contains(&ReachViolation::NoSubgroups));
    }

    #[test]
    fn subgroup_equity_csv_is_deterministic_and_has_no_person_identifiers() {
        let report = delivery_report(vec![
            delivery("urban", "sms", 100),
            delivery("rural", "sms", 92),
        ]);
        let groups = vec![
            subgroup("urban", "urban", &["sms"], 100, 100),
            subgroup("rural", "rural", &["sms"], 100, 100),
        ];
        let first = audit_subgroup_reach(&report, &groups, &policy()).unwrap();
        let second = audit_subgroup_reach(&report, &groups, &policy()).unwrap();
        assert_eq!(first.to_csv(), second.to_csv());
        assert_eq!(first.to_csv().lines().count(), 4);
        assert!(first.to_csv().starts_with(
            "record_type,group_id,cohort_id,dimensions,disposition,"
        ));
        assert!(!first.to_csv().contains("phone_number"));
    }

    #[test]
    fn invalid_coverage_policy_is_rejected() {
        let mut invalid = policy();
        invalid.minimum_group_coverage = 1.2;
        assert_eq!(
            audit_subgroup_reach(
                &delivery_report(Vec::new()),
                &[],
                &invalid
            ),
            Err(ReachError::InvalidPolicyThreshold)
        );
    }
}
