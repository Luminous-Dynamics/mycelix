//! Conservative per-subgroup action-opportunity assessment.
//!
//! This module evaluates whether an evidenced protective-action opportunity
//! exists under declared time, capacity and interval-net-benefit constraints.
//! It does not select, order, recommend, dispatch or execute an action.

use std::collections::BTreeSet;

use gcpn_compositional_assurance::ClaimDisposition;
use gcpn_subgroup_reach_equity::{SubgroupEquityReport, SubgroupReachAssessment};

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EventTimeInterval {
    pub hazard_id: String,
    pub earliest: u64,
    pub latest: u64,
    pub source_id: String,
    pub source_version: String,
    pub recorded_at: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DurationInterval {
    pub action_id: String,
    pub lower_seconds: u64,
    pub upper_seconds: u64,
    pub source_id: String,
    pub source_version: String,
    pub recorded_at: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CountInterval {
    pub lower: u64,
    pub upper: u64,
    pub resource_id: String,
    pub unit_id: String,
    pub source_id: String,
    pub source_version: String,
    pub recorded_at: u64,
}

#[derive(Clone, Debug, PartialEq)]
pub struct MetricInterval {
    pub lower: f64,
    pub upper: f64,
    pub unit_id: String,
    pub source_id: String,
    pub source_version: String,
    pub recorded_at: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct WarningDeliveryEvidence {
    pub message_id: String,
    pub message_version: String,
    pub channel_ids: Vec<String>,
    pub earliest_delivery_at: u64,
    pub latest_delivery_at: u64,
    pub source_id: String,
    pub source_version: String,
    pub recorded_at: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ActionOpportunityCase {
    pub group_id: String,
    pub cohort_id: String,
    pub action_id: String,
    pub decision_id: String,
    pub hazard_id: String,
    pub message_id: String,
    pub message_version: String,
    /// Caller-declared channel set used for this action opportunity. It must
    /// match the delivery-time evidence's exact channel set.
    pub channel_ids: Vec<String>,
    pub warning_delivery: Option<WarningDeliveryEvidence>,
    pub hazard_arrival: Option<EventTimeInterval>,
    pub action_duration: Option<DurationInterval>,
    /// Aggregate need/capacity intervals; never person-level allocations.
    pub required_resource_count: Option<CountInterval>,
    pub available_resource_count: Option<CountInterval>,
    /// All three metrics must use the policy's same utility unit.
    pub loss_without_action: Option<MetricInterval>,
    pub loss_with_action: Option<MetricInterval>,
    pub action_burden: Option<MetricInterval>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ActionOpportunityPolicy {
    pub as_of_cutoff: u64,
    pub expected_warning_source_id: String,
    pub expected_warning_source_version: String,
    pub expected_hazard_source_id: String,
    pub expected_hazard_source_version: String,
    pub expected_duration_source_id: String,
    pub expected_duration_source_version: String,
    pub expected_resource_source_id: String,
    pub expected_resource_source_version: String,
    pub expected_loss_source_id: String,
    pub expected_loss_source_version: String,
    pub expected_utility_unit_id: String,
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum OpportunityViolation {
    InvalidIdentifier,
    ReachReportAsOfMismatch,
    ReachAssessmentMissing,
    ReachCountUnavailable,
    WarningDeliveryMissing,
    WarningMessageBindingMismatch,
    WarningChannelSetMismatch,
    WarningEvidenceAfterCutoff,
    WarningSourceMismatch,
    HazardArrivalMissing,
    HazardIdentityMismatch,
    HazardEvidenceAfterCutoff,
    HazardSourceMismatch,
    ActionDurationMissing,
    ActionDurationEvidenceAfterCutoff,
    ActionDurationSourceMismatch,
    ActionDurationIdentityMismatch,
    ResourceRequirementMissing,
    ResourceCapacityMissing,
    ResourceEvidenceAfterCutoff,
    ResourceSourceMismatch,
    ResourceIdentityMismatch,
    ResourceCapacityOverflow,
    LossWithoutActionMissing,
    LossWithActionMissing,
    ActionBurdenMissing,
    LossEvidenceAfterCutoff,
    LossSourceMismatch,
    UtilityUnitMismatch,
    InvalidTimeInterval,
    InvalidDurationInterval,
    InvalidCountInterval,
    InvalidMetricInterval,
}

impl OpportunityViolation {
    pub const fn label(self) -> &'static str {
        match self {
            Self::InvalidIdentifier => "invalid_identifier",
            Self::ReachReportAsOfMismatch => "reach_report_as_of_mismatch",
            Self::ReachAssessmentMissing => "reach_assessment_missing",
            Self::ReachCountUnavailable => "reach_count_unavailable",
            Self::WarningDeliveryMissing => "warning_delivery_missing",
            Self::WarningMessageBindingMismatch => "warning_message_binding_mismatch",
            Self::WarningChannelSetMismatch => "warning_channel_set_mismatch",
            Self::WarningEvidenceAfterCutoff => "warning_evidence_after_cutoff",
            Self::WarningSourceMismatch => "warning_source_mismatch",
            Self::HazardArrivalMissing => "hazard_arrival_missing",
            Self::HazardIdentityMismatch => "hazard_identity_mismatch",
            Self::HazardEvidenceAfterCutoff => "hazard_evidence_after_cutoff",
            Self::HazardSourceMismatch => "hazard_source_mismatch",
            Self::ActionDurationMissing => "action_duration_missing",
            Self::ActionDurationEvidenceAfterCutoff => "action_duration_evidence_after_cutoff",
            Self::ActionDurationSourceMismatch => "action_duration_source_mismatch",
            Self::ActionDurationIdentityMismatch => "action_duration_identity_mismatch",
            Self::ResourceRequirementMissing => "resource_requirement_missing",
            Self::ResourceCapacityMissing => "resource_capacity_missing",
            Self::ResourceEvidenceAfterCutoff => "resource_evidence_after_cutoff",
            Self::ResourceSourceMismatch => "resource_source_mismatch",
            Self::ResourceIdentityMismatch => "resource_identity_mismatch",
            Self::ResourceCapacityOverflow => "resource_capacity_overflow",
            Self::LossWithoutActionMissing => "loss_without_action_missing",
            Self::LossWithActionMissing => "loss_with_action_missing",
            Self::ActionBurdenMissing => "action_burden_missing",
            Self::LossEvidenceAfterCutoff => "loss_evidence_after_cutoff",
            Self::LossSourceMismatch => "loss_source_mismatch",
            Self::UtilityUnitMismatch => "utility_unit_mismatch",
            Self::InvalidTimeInterval => "invalid_time_interval",
            Self::InvalidDurationInterval => "invalid_duration_interval",
            Self::InvalidCountInterval => "invalid_count_interval",
            Self::InvalidMetricInterval => "invalid_metric_interval",
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum OpportunityError {
    InvalidIdentifier,
    DuplicateGroupAction(String),
    DuplicateChannel(String),
}

impl std::fmt::Display for OpportunityError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidIdentifier => write!(formatter, "identifier is not canonical"),
            Self::DuplicateGroupAction(value) => {
                write!(formatter, "duplicate group/action case: {value}")
            }
            Self::DuplicateChannel(value) => {
                write!(formatter, "duplicate channel in action case: {value}")
            }
        }
    }
}

impl std::error::Error for OpportunityError {}

#[derive(Clone, Debug, PartialEq)]
pub struct TimeFeasibilityAssessment {
    pub disposition: ClaimDisposition,
    pub conservative_time_budget_lower_seconds: Option<u64>,
    pub possible_time_budget_upper_seconds: Option<u64>,
    pub required_duration_lower_seconds: Option<u64>,
    pub required_duration_upper_seconds: Option<u64>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ResourceFeasibilityAssessment {
    pub disposition: ClaimDisposition,
    pub required_lower: Option<u64>,
    pub required_upper: Option<u64>,
    pub available_lower: Option<u64>,
    pub available_upper: Option<u64>,
    pub resource_id: Option<String>,
    pub unit_id: Option<String>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct NetBenefitAssessment {
    pub disposition: ClaimDisposition,
    pub unit_id: String,
    pub lower_bound: Option<f64>,
    pub upper_bound: Option<f64>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ActionOpportunityAssessment {
    pub group_id: String,
    pub cohort_id: String,
    pub action_id: String,
    pub decision_id: String,
    pub hazard_id: String,
    /// Reach coverage is retained separately from action opportunity.
    pub reach_disposition: ClaimDisposition,
    pub unique_reached_count: Option<u64>,
    pub time: TimeFeasibilityAssessment,
    pub resources: ResourceFeasibilityAssessment,
    pub net_benefit: NetBenefitAssessment,
    pub disposition: ClaimDisposition,
    pub violations: Vec<OpportunityViolation>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct ActionOpportunityReport {
    pub as_of_cutoff: u64,
    pub assessments: Vec<ActionOpportunityAssessment>,
    pub assessed_case_count: usize,
    pub supports: usize,
    pub refutes: usize,
    pub indeterminate: usize,
    pub unavailable: usize,
}

impl ActionOpportunityReport {
    /// Per-group/per-action record. No global score or selected action is emitted.
    pub fn to_csv(&self) -> String {
        let mut output = String::from(
            "record_type,group_id,cohort_id,action_id,decision_id,hazard_id,reach_disposition,unique_reached,opportunity_scope,time_disposition,time_budget_lower,time_budget_upper,required_duration_upper,resource_disposition,required_capacity_upper,available_capacity_lower,net_benefit_disposition,net_benefit_lower,net_benefit_upper,utility_unit,opportunity_disposition,violations\n",
        );
        for assessment in &self.assessments {
            output.push_str(&format!(
                "case,{},{},{},{},{},{},{},reached_population_only,{},{},{},{},{},{},{},{:.8},{:.8},{},{},{}\n",
                assessment.group_id,
                assessment.cohort_id,
                assessment.action_id,
                assessment.decision_id,
                assessment.hazard_id,
                disposition_label(assessment.reach_disposition),
                optional_u64(assessment.unique_reached_count),
                disposition_label(assessment.time.disposition),
                optional_u64(assessment.time.conservative_time_budget_lower_seconds),
                optional_u64(assessment.time.possible_time_budget_upper_seconds),
                optional_u64(assessment.time.required_duration_upper_seconds),
                disposition_label(assessment.resources.disposition),
                optional_u64(assessment.resources.required_upper),
                optional_u64(assessment.resources.available_lower),
                disposition_label(assessment.net_benefit.disposition),
                assessment.net_benefit.lower_bound.unwrap_or(-1.0),
                assessment.net_benefit.upper_bound.unwrap_or(-1.0),
                assessment.net_benefit.unit_id,
                disposition_label(assessment.disposition),
                violation_labels(&assessment.violations)
            ));
        }
        let mut summary_fields = vec![
            "summary".to_string(),
            "all_groups".to_string(),
            "all_cohorts".to_string(),
            "all_actions".to_string(),
            "all_decisions".to_string(),
            "all_hazards".to_string(),
        ];
        summary_fields.extend((0..14).map(|_| String::new()));
        summary_fields.push("not_aggregated".into());
        summary_fields.push(format!(
            "cases={};supports={};refutes={};indeterminate={};unavailable={}",
            self.assessed_case_count,
            self.supports,
            self.refutes,
            self.indeterminate,
            self.unavailable
        ));
        output.push_str(&summary_fields.join(","));
        output.push('\n');
        output
    }
}

pub fn audit_action_opportunity(
    reach_report: &SubgroupEquityReport,
    cases: &[ActionOpportunityCase],
    policy: &ActionOpportunityPolicy,
) -> Result<ActionOpportunityReport, OpportunityError> {
    validate_policy(policy)?;
    validate_cases(cases)?;

    let assessments = cases
        .iter()
        .map(|case| assess_case(reach_report, case, policy))
        .collect::<Vec<_>>();

    let supports = assessments
        .iter()
        .filter(|assessment| assessment.disposition == ClaimDisposition::Supports)
        .count();
    let refutes = assessments
        .iter()
        .filter(|assessment| assessment.disposition == ClaimDisposition::Refutes)
        .count();
    let indeterminate = assessments
        .iter()
        .filter(|assessment| assessment.disposition == ClaimDisposition::Indeterminate)
        .count();
    let unavailable = assessments
        .iter()
        .filter(|assessment| assessment.disposition == ClaimDisposition::Unavailable)
        .count();

    Ok(ActionOpportunityReport {
        as_of_cutoff: policy.as_of_cutoff,
        assessed_case_count: assessments.len(),
        assessments,
        supports,
        refutes,
        indeterminate,
        unavailable,
    })
}

fn validate_policy(policy: &ActionOpportunityPolicy) -> Result<(), OpportunityError> {
    for value in [
        policy.expected_warning_source_id.as_str(),
        policy.expected_warning_source_version.as_str(),
        policy.expected_hazard_source_id.as_str(),
        policy.expected_hazard_source_version.as_str(),
        policy.expected_duration_source_id.as_str(),
        policy.expected_duration_source_version.as_str(),
        policy.expected_resource_source_id.as_str(),
        policy.expected_resource_source_version.as_str(),
        policy.expected_loss_source_id.as_str(),
        policy.expected_loss_source_version.as_str(),
        policy.expected_utility_unit_id.as_str(),
    ] {
        check_identifier(value)?;
    }
    Ok(())
}

fn validate_cases(cases: &[ActionOpportunityCase]) -> Result<(), OpportunityError> {
    let mut keys = BTreeSet::new();
    for case in cases {
        for value in [
            case.group_id.as_str(),
            case.cohort_id.as_str(),
            case.action_id.as_str(),
            case.decision_id.as_str(),
            case.hazard_id.as_str(),
            case.message_id.as_str(),
            case.message_version.as_str(),
        ] {
            check_identifier(value)?;
        }
        if !keys.insert((case.group_id.as_str(), case.action_id.as_str())) {
            return Err(OpportunityError::DuplicateGroupAction(format!(
                "{}:{}",
                case.group_id, case.action_id
            )));
        }
        let mut channels = BTreeSet::new();
        for channel in &case.channel_ids {
            check_identifier(channel)?;
            if !channels.insert(channel.as_str()) {
                return Err(OpportunityError::DuplicateChannel(channel.clone()));
            }
        }
        if case.channel_ids.is_empty() {
            return Err(OpportunityError::InvalidIdentifier);
        }
        if let Some(warning) = &case.warning_delivery {
            for value in [
                warning.message_id.as_str(),
                warning.message_version.as_str(),
                warning.source_id.as_str(),
                warning.source_version.as_str(),
            ] {
                check_identifier(value)?;
            }
            for channel in &warning.channel_ids {
                check_identifier(channel)?;
            }
        }
        if let Some(hazard) = &case.hazard_arrival {
            check_identifier(&hazard.hazard_id)?;
            check_identifier(&hazard.source_id)?;
            check_identifier(&hazard.source_version)?;
        }
        if let Some(duration) = &case.action_duration {
            check_identifier(&duration.action_id)?;
            check_identifier(&duration.source_id)?;
            check_identifier(&duration.source_version)?;
        }
        for interval in [
            case.required_resource_count.as_ref(),
            case.available_resource_count.as_ref(),
        ].into_iter().flatten() {
            for value in [
                interval.resource_id.as_str(),
                interval.unit_id.as_str(),
                interval.source_id.as_str(),
                interval.source_version.as_str(),
            ] {
                check_identifier(value)?;
            }
        }
        for metric in [
            case.loss_without_action.as_ref(),
            case.loss_with_action.as_ref(),
            case.action_burden.as_ref(),
        ].into_iter().flatten() {
            for value in [
                metric.unit_id.as_str(),
                metric.source_id.as_str(),
                metric.source_version.as_str(),
            ] {
                check_identifier(value)?;
            }
        }
        if let Some(hazard) = &case.hazard_arrival {
            check_identifier(&hazard.hazard_id)?;
            check_identifier(&hazard.source_id)?;
            check_identifier(&hazard.source_version)?;
        }
        if let Some(duration) = &case.action_duration {
            check_identifier(&duration.action_id)?;
            check_identifier(&duration.source_id)?;
            check_identifier(&duration.source_version)?;
        }
    }
    Ok(())
}

fn assess_case(
    reach_report: &SubgroupEquityReport,
    case: &ActionOpportunityCase,
    policy: &ActionOpportunityPolicy,
) -> ActionOpportunityAssessment {
    let mut violations = BTreeSet::new();

    let reach = if reach_report.as_of_cutoff != policy.as_of_cutoff {
        violations.insert(OpportunityViolation::ReachReportAsOfMismatch);
        None
    } else {
        reach_report
            .groups
            .iter()
            .find(|group| group.group_id == case.group_id && group.cohort_id == case.cohort_id)
    };

    let (reach_disposition, unique_reached_count) = match reach {
        None => {
            violations.insert(OpportunityViolation::ReachAssessmentMissing);
            (ClaimDisposition::Unavailable, None)
        }
        Some(group) => {
            if group.unique_reached.unwrap_or(0) == 0 {
                violations.insert(OpportunityViolation::ReachCountUnavailable);
                (group.disposition, group.unique_reached)
            } else {
                (group.disposition, group.unique_reached)
            }
        }
    };

    let mut time_violations = BTreeSet::new();
    let time = assess_time(case, policy, &mut time_violations);
    violations.extend(time_violations);

    let mut resource_violations = BTreeSet::new();
    let resources = assess_resources(case, policy, &mut resource_violations);
    violations.extend(resource_violations);

    let mut benefit_violations = BTreeSet::new();
    let net_benefit = assess_net_benefit(case, policy, &mut benefit_violations);
    violations.extend(benefit_violations);

    let dimensions = [time.disposition, resources.disposition, net_benefit.disposition];
    let disposition = if unique_reached_count.is_none_or(|count| count == 0) {
        ClaimDisposition::Unavailable
    } else if dimensions.contains(&ClaimDisposition::Refutes) {
        ClaimDisposition::Refutes
    } else if dimensions.contains(&ClaimDisposition::Unavailable) {
        ClaimDisposition::Unavailable
    } else if dimensions.iter().all(|value| *value == ClaimDisposition::Supports) {
        ClaimDisposition::Supports
    } else {
        ClaimDisposition::Indeterminate
    };

    ActionOpportunityAssessment {
        group_id: case.group_id.clone(),
        cohort_id: case.cohort_id.clone(),
        action_id: case.action_id.clone(),
        decision_id: case.decision_id.clone(),
        hazard_id: case.hazard_id.clone(),
        reach_disposition,
        unique_reached_count,
        time,
        resources,
        net_benefit,
        disposition,
        violations: violations.into_iter().collect(),
    }
}

fn assess_time(
    case: &ActionOpportunityCase,
    policy: &ActionOpportunityPolicy,
    violations: &mut BTreeSet<OpportunityViolation>,
) -> TimeFeasibilityAssessment {
    let Some(warning) = &case.warning_delivery else {
        violations.insert(OpportunityViolation::WarningDeliveryMissing);
        return unavailable_time();
    };
    let Some(hazard) = &case.hazard_arrival else {
        violations.insert(OpportunityViolation::HazardArrivalMissing);
        return unavailable_time();
    };
    let Some(duration) = &case.action_duration else {
        violations.insert(OpportunityViolation::ActionDurationMissing);
        return unavailable_time();
    };

    if warning.message_id != case.message_id || warning.message_version != case.message_version {
        violations.insert(OpportunityViolation::WarningMessageBindingMismatch);
    }
    let expected_channels: BTreeSet<_> = case.channel_ids.iter().collect();
    let observed_channels: BTreeSet<_> = warning.channel_ids.iter().collect();
    if expected_channels != observed_channels
        || observed_channels.len() != warning.channel_ids.len()
    {
        violations.insert(OpportunityViolation::WarningChannelSetMismatch);
    }
    if hazard.hazard_id != case.hazard_id {
        violations.insert(OpportunityViolation::HazardIdentityMismatch);
    }
    if duration.action_id != case.action_id {
        violations.insert(OpportunityViolation::ActionDurationIdentityMismatch);
    }
    if hazard.earliest > hazard.latest || warning.earliest_delivery_at > warning.latest_delivery_at {
        violations.insert(OpportunityViolation::InvalidTimeInterval);
    }
    if duration.lower_seconds == 0 || duration.lower_seconds > duration.upper_seconds {
        violations.insert(OpportunityViolation::InvalidDurationInterval);
    }
    if warning.recorded_at > policy.as_of_cutoff || hazard.recorded_at > policy.as_of_cutoff
        || duration.recorded_at > policy.as_of_cutoff
        || warning.earliest_delivery_at > policy.as_of_cutoff
        || warning.latest_delivery_at > policy.as_of_cutoff
    {
        violations.insert(OpportunityViolation::WarningEvidenceAfterCutoff);
    }
    if warning.source_id != policy.expected_warning_source_id
        || warning.source_version != policy.expected_warning_source_version
    {
        violations.insert(OpportunityViolation::WarningSourceMismatch);
    }
    if hazard.source_id != policy.expected_hazard_source_id
        || hazard.source_version != policy.expected_hazard_source_version
    {
        violations.insert(OpportunityViolation::HazardSourceMismatch);
    }
    if duration.source_id != policy.expected_duration_source_id
        || duration.source_version != policy.expected_duration_source_version
    {
        violations.insert(OpportunityViolation::ActionDurationSourceMismatch);
    }
    if !violations.is_empty() {
        return TimeFeasibilityAssessment {
            disposition: ClaimDisposition::Indeterminate,
            conservative_time_budget_lower_seconds: None,
            possible_time_budget_upper_seconds: None,
            required_duration_lower_seconds: Some(duration.lower_seconds),
            required_duration_upper_seconds: Some(duration.upper_seconds),
        };
    }

    let budget_lower = hazard.earliest.saturating_sub(warning.latest_delivery_at);
    let budget_upper = hazard.latest.saturating_sub(warning.earliest_delivery_at);
    let disposition = if budget_lower >= duration.upper_seconds {
        ClaimDisposition::Supports
    } else if budget_upper < duration.lower_seconds {
        ClaimDisposition::Refutes
    } else {
        ClaimDisposition::Indeterminate
    };

    TimeFeasibilityAssessment {
        disposition,
        conservative_time_budget_lower_seconds: Some(budget_lower),
        possible_time_budget_upper_seconds: Some(budget_upper),
        required_duration_lower_seconds: Some(duration.lower_seconds),
        required_duration_upper_seconds: Some(duration.upper_seconds),
    }
}

fn assess_resources(
    case: &ActionOpportunityCase,
    policy: &ActionOpportunityPolicy,
    violations: &mut BTreeSet<OpportunityViolation>,
) -> ResourceFeasibilityAssessment {
    let Some(required) = &case.required_resource_count else {
        violations.insert(OpportunityViolation::ResourceRequirementMissing);
        return unavailable_resources();
    };
    let Some(available) = &case.available_resource_count else {
        violations.insert(OpportunityViolation::ResourceCapacityMissing);
        return unavailable_resources();
    };
    if required.lower > required.upper
        || available.lower > available.upper
        || required.unit_id != available.unit_id
    {
        violations.insert(OpportunityViolation::InvalidCountInterval);
    }
    if required.resource_id != available.resource_id {
        violations.insert(OpportunityViolation::ResourceIdentityMismatch);
    }
    if required.source_id != policy.expected_resource_source_id
        || available.source_id != policy.expected_resource_source_id
        || required.source_version != policy.expected_resource_source_version
        || available.source_version != policy.expected_resource_source_version
    {
        violations.insert(OpportunityViolation::ResourceSourceMismatch);
    }
    if required.recorded_at > policy.as_of_cutoff || available.recorded_at > policy.as_of_cutoff {
        violations.insert(OpportunityViolation::ResourceEvidenceAfterCutoff);
    }
    if !violations.is_empty() {
        return ResourceFeasibilityAssessment {
            disposition: ClaimDisposition::Indeterminate,
            required_lower: Some(required.lower),
            required_upper: Some(required.upper),
            available_lower: Some(available.lower),
            available_upper: Some(available.upper),
            resource_id: Some(required.resource_id.clone()),
            unit_id: Some(required.unit_id.clone()),
        };
    }
    let disposition = if available.lower >= required.upper {
        ClaimDisposition::Supports
    } else if available.upper < required.lower {
        ClaimDisposition::Refutes
    } else {
        ClaimDisposition::Indeterminate
    };
    ResourceFeasibilityAssessment {
        disposition,
        required_lower: Some(required.lower),
        required_upper: Some(required.upper),
        available_lower: Some(available.lower),
        available_upper: Some(available.upper),
        resource_id: Some(required.resource_id.clone()),
        unit_id: Some(required.unit_id.clone()),
    }
}

fn assess_net_benefit(
    case: &ActionOpportunityCase,
    policy: &ActionOpportunityPolicy,
    violations: &mut BTreeSet<OpportunityViolation>,
) -> NetBenefitAssessment {
    let Some(no_action) = &case.loss_without_action else {
        violations.insert(OpportunityViolation::LossWithoutActionMissing);
        return unavailable_benefit(&policy.expected_utility_unit_id);
    };
    let Some(with_action) = &case.loss_with_action else {
        violations.insert(OpportunityViolation::LossWithActionMissing);
        return unavailable_benefit(&policy.expected_utility_unit_id);
    };
    let Some(burden) = &case.action_burden else {
        violations.insert(OpportunityViolation::ActionBurdenMissing);
        return unavailable_benefit(&policy.expected_utility_unit_id);
    };
    for metric in [no_action, with_action, burden] {
        if !metric.lower.is_finite()
            || !metric.upper.is_finite()
            || metric.lower > metric.upper
            || metric.lower < 0.0
        {
            violations.insert(OpportunityViolation::InvalidMetricInterval);
        }
        if metric.unit_id != policy.expected_utility_unit_id {
            violations.insert(OpportunityViolation::UtilityUnitMismatch);
        }
        if metric.recorded_at > policy.as_of_cutoff {
            violations.insert(OpportunityViolation::LossEvidenceAfterCutoff);
        }
        if metric.source_id != policy.expected_loss_source_id
            || metric.source_version != policy.expected_loss_source_version
        {
            violations.insert(OpportunityViolation::LossSourceMismatch);
        }
    }
    if !violations.is_empty() {
        return NetBenefitAssessment {
            disposition: ClaimDisposition::Indeterminate,
            unit_id: policy.expected_utility_unit_id.clone(),
            lower_bound: None,
            upper_bound: None,
        };
    }

    let lower = no_action.lower - with_action.upper - burden.upper;
    let upper = no_action.upper - with_action.lower - burden.lower;
    if !lower.is_finite() || !upper.is_finite() {
        violations.insert(OpportunityViolation::InvalidMetricInterval);
        return NetBenefitAssessment {
            disposition: ClaimDisposition::Indeterminate,
            unit_id: policy.expected_utility_unit_id.clone(),
            lower_bound: None,
            upper_bound: None,
        };
    }
    let disposition = if lower > 0.0 {
        ClaimDisposition::Supports
    } else if upper < 0.0 {
        ClaimDisposition::Refutes
    } else {
        ClaimDisposition::Indeterminate
    };
    NetBenefitAssessment {
        disposition,
        unit_id: policy.expected_utility_unit_id.clone(),
        lower_bound: Some(lower),
        upper_bound: Some(upper),
    }
}

fn unavailable_time() -> TimeFeasibilityAssessment {
    TimeFeasibilityAssessment {
        disposition: ClaimDisposition::Unavailable,
        conservative_time_budget_lower_seconds: None,
        possible_time_budget_upper_seconds: None,
        required_duration_lower_seconds: None,
        required_duration_upper_seconds: None,
    }
}

fn unavailable_resources() -> ResourceFeasibilityAssessment {
    ResourceFeasibilityAssessment {
        disposition: ClaimDisposition::Unavailable,
        required_lower: None,
        required_upper: None,
        available_lower: None,
        available_upper: None,
        resource_id: None,
        unit_id: None,
    }
}

fn unavailable_benefit(unit_id: &str) -> NetBenefitAssessment {
    NetBenefitAssessment {
        disposition: ClaimDisposition::Unavailable,
        unit_id: unit_id.into(),
        lower_bound: None,
        upper_bound: None,
    }
}

fn check_identifier(value: &str) -> Result<(), OpportunityError> {
    if value.is_empty()
        || value.len() > 128
        || !value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || b"-_.:".contains(&byte))
    {
        Err(OpportunityError::InvalidIdentifier)
    } else {
        Ok(())
    }
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

fn optional_u64(value: Option<u64>) -> String {
    value.map(|number| number.to_string()).unwrap_or_default()
}

fn violation_labels(violations: &[OpportunityViolation]) -> String {
    violations
        .iter()
        .map(|violation| violation.label())
        .collect::<Vec<_>>()
        .join("|")
}

#[cfg(test)]
mod tests {
    use super::*;
    use gcpn_subgroup_reach_equity::{SubgroupReachAssessment, SubgroupEquityReport};

    fn policy() -> ActionOpportunityPolicy {
        ActionOpportunityPolicy {
            as_of_cutoff: 100,
            expected_warning_source_id: "delivery-network".into(),
            expected_warning_source_version: "receipt-v1".into(),
            expected_hazard_source_id: "hazard-model".into(),
            expected_hazard_source_version: "forecast-v3".into(),
            expected_duration_source_id: "local-drill".into(),
            expected_duration_source_version: "drill-2026".into(),
            expected_resource_source_id: "resource-ledger".into(),
            expected_resource_source_version: "snapshot-8".into(),
            expected_loss_source_id: "impact-model".into(),
            expected_loss_source_version: "loss-v2".into(),
            expected_utility_unit_id: "usd-2026".into(),
        }
    }

    fn reach_report() -> SubgroupEquityReport {
        SubgroupEquityReport {
            as_of_cutoff: 100,
            minimum_group_coverage_policy: 0.95,
            maximum_coverage_gap_policy: 0.05,
            groups: vec![SubgroupReachAssessment {
                group_id: "rural".into(),
                cohort_id: "cohort-rural".into(),
                dimensions: Vec::new(),
                disposition: ClaimDisposition::Refutes,
                denominator_lower: Some(100),
                denominator_upper: Some(110),
                channel_count_sum: Some(100),
                unique_reached: Some(85),
                overlap_removed: Some(15),
                coverage_lower_bound: Some(85.0 / 110.0),
                coverage_upper_bound: Some(0.85),
                violations: Vec::new(),
            }],
            assessed_group_count: 1,
            supports_group_count: 0,
            refutes_group_count: 1,
            indeterminate_group_count: 0,
            unavailable_group_count: 0,
            minimum_group_coverage_lower_bound: Some(85.0 / 110.0),
            maximum_group_coverage_upper_bound: Some(0.85),
            coverage_gap_lower_bound: None,
            coverage_gap_upper_bound: None,
            equity_disposition: ClaimDisposition::Indeterminate,
            equity_violations: Vec::new(),
        }
    }

    fn valid_case() -> ActionOpportunityCase {
        ActionOpportunityCase {
            group_id: "rural".into(),
            cohort_id: "cohort-rural".into(),
            action_id: "evacuation-support".into(),
            decision_id: "decision-1".into(),
            hazard_id: "flood-1".into(),
            message_id: "cap-1".into(),
            message_version: "2".into(),
            channel_ids: vec!["cell-broadcast".into(), "sms".into()],
            warning_delivery: Some(WarningDeliveryEvidence {
                message_id: "cap-1".into(),
                message_version: "2".into(),
                channel_ids: vec!["cell-broadcast".into(), "sms".into()],
                earliest_delivery_at: 10,
                latest_delivery_at: 20,
                source_id: "delivery-network".into(),
                source_version: "receipt-v1".into(),
                recorded_at: 25,
            }),
            hazard_arrival: Some(EventTimeInterval {
                hazard_id: "flood-1".into(),
                earliest: 1_000,
                latest: 1_200,
                source_id: "hazard-model".into(),
                source_version: "forecast-v3".into(),
                recorded_at: 30,
            }),
            action_duration: Some(DurationInterval {
                action_id: "evacuation-support".into(),
                lower_seconds: 300,
                upper_seconds: 600,
                source_id: "local-drill".into(),
                source_version: "drill-2026".into(),
                recorded_at: 40,
            }),
            required_resource_count: Some(CountInterval {
                lower: 80,
                upper: 85,
                resource_id: "accessible-transport".into(),
                unit_id: "seats".into(),
                source_id: "resource-ledger".into(),
                source_version: "snapshot-8".into(),
                recorded_at: 40,
            }),
            available_resource_count: Some(CountInterval {
                lower: 90,
                upper: 100,
                resource_id: "accessible-transport".into(),
                unit_id: "seats".into(),
                source_id: "resource-ledger".into(),
                source_version: "snapshot-8".into(),
                recorded_at: 40,
            }),
            loss_without_action: Some(metric(100_000.0, 120_000.0)),
            loss_with_action: Some(metric(30_000.0, 40_000.0)),
            action_burden: Some(metric(10_000.0, 15_000.0)),
        }
    }

    fn metric(lower: f64, upper: f64) -> MetricInterval {
        MetricInterval {
            lower,
            upper,
            unit_id: "usd-2026".into(),
            source_id: "impact-model".into(),
            source_version: "loss-v2".into(),
            recorded_at: 40,
        }
    }

    fn evaluate(case: ActionOpportunityCase) -> ActionOpportunityAssessment {
        audit_action_opportunity(&reach_report(), &[case], &policy())
            .unwrap()
            .assessments
            .remove(0)
    }

    #[test]
    fn all_three_robust_constraints_support_action_opportunity() {
        let assessment = evaluate(valid_case());
        assert_eq!(assessment.time.disposition, ClaimDisposition::Supports);
        assert_eq!(assessment.resources.disposition, ClaimDisposition::Supports);
        assert_eq!(assessment.net_benefit.disposition, ClaimDisposition::Supports);
        assert_eq!(assessment.disposition, ClaimDisposition::Supports);
        assert_eq!(assessment.reach_disposition, ClaimDisposition::Refutes);
        assert_eq!(assessment.unique_reached_count, Some(85));
    }

    #[test]
    fn insufficient_time_refutes_opportunity_even_with_positive_net_benefit() {
        let mut case = valid_case();
        case.hazard_arrival.as_mut().unwrap().earliest = 300;
        case.hazard_arrival.as_mut().unwrap().latest = 350;
        let assessment = evaluate(case);
        assert_eq!(assessment.time.disposition, ClaimDisposition::Refutes);
        assert_eq!(assessment.net_benefit.disposition, ClaimDisposition::Supports);
        assert_eq!(assessment.disposition, ClaimDisposition::Refutes);
    }

    #[test]
    fn insufficient_resource_capacity_refutes_opportunity() {
        let mut case = valid_case();
        case.available_resource_count.as_mut().unwrap().lower = 10;
        case.available_resource_count.as_mut().unwrap().upper = 40;
        let assessment = evaluate(case);
        assert_eq!(assessment.resources.disposition, ClaimDisposition::Refutes);
        assert_eq!(assessment.disposition, ClaimDisposition::Refutes);
    }

    #[test]
    fn uncertain_resource_overlap_is_indeterminate() {
        let mut case = valid_case();
        case.required_resource_count.as_mut().unwrap().upper = 95;
        case.available_resource_count.as_mut().unwrap().lower = 90;
        case.available_resource_count.as_mut().unwrap().upper = 100;
        let assessment = evaluate(case);
        assert_eq!(assessment.resources.disposition, ClaimDisposition::Indeterminate);
        assert_eq!(assessment.disposition, ClaimDisposition::Indeterminate);
    }

    #[test]
    fn positive_net_benefit_requires_strictly_positive_lower_bound() {
        let mut case = valid_case();
        case.loss_without_action = Some(metric(40.0, 70.0));
        case.loss_with_action = Some(metric(30.0, 40.0));
        case.action_burden = Some(metric(10.0, 30.0));
        let assessment = evaluate(case);
        assert_eq!(assessment.net_benefit.lower_bound, Some(-30.0));
        assert_eq!(assessment.net_benefit.upper_bound, Some(30.0));
        assert_eq!(assessment.net_benefit.disposition, ClaimDisposition::Indeterminate);
    }

    #[test]
    fn negative_upper_net_benefit_refutes_action_value() {
        let mut case = valid_case();
        case.loss_without_action = Some(metric(10.0, 20.0));
        case.loss_with_action = Some(metric(30.0, 40.0));
        case.action_burden = Some(metric(5.0, 10.0));
        let assessment = evaluate(case);
        assert!(assessment.net_benefit.upper_bound.unwrap() < 0.0);
        assert_eq!(assessment.net_benefit.disposition, ClaimDisposition::Refutes);
        assert_eq!(assessment.disposition, ClaimDisposition::Refutes);
    }

    #[test]
    fn missing_local_drill_duration_is_unavailable_not_assumed() {
        let mut case = valid_case();
        case.action_duration = None;
        let assessment = evaluate(case);
        assert_eq!(assessment.time.disposition, ClaimDisposition::Unavailable);
        assert_eq!(assessment.disposition, ClaimDisposition::Unavailable);
    }

    #[test]
    fn warning_delivery_must_match_exact_message_and_version() {
        let mut case = valid_case();
        case.warning_delivery.as_mut().unwrap().message_version = "1".into();
        let assessment = evaluate(case);
        assert_eq!(assessment.time.disposition, ClaimDisposition::Indeterminate);
        assert!(assessment.violations.contains(&OpportunityViolation::WarningMessageBindingMismatch));
    }

    #[test]
    fn warning_delivery_channel_set_must_match() {
        let mut case = valid_case();
        case.warning_delivery.as_mut().unwrap().channel_ids.pop();
        let assessment = evaluate(case);
        assert_eq!(assessment.time.disposition, ClaimDisposition::Indeterminate);
        assert!(assessment.violations.contains(&OpportunityViolation::WarningChannelSetMismatch));
    }

    #[test]
    fn post_cutoff_evidence_is_indeterminate() {
        let mut case = valid_case();
        case.available_resource_count.as_mut().unwrap().recorded_at = 101;
        let assessment = evaluate(case);
        assert_eq!(assessment.resources.disposition, ClaimDisposition::Indeterminate);
        assert!(assessment.violations.contains(&OpportunityViolation::ResourceEvidenceAfterCutoff));
    }

    #[test]
    fn reach_report_cutoff_mismatch_prevents_opportunity_support() {
        let mut reach = reach_report();
        reach.as_of_cutoff = 99;
        let report = audit_action_opportunity(&reach, &[valid_case()], &policy()).unwrap();
        assert_eq!(report.assessments[0].disposition, ClaimDisposition::Unavailable);
        assert!(report.assessments[0].violations.contains(&OpportunityViolation::ReachReportAsOfMismatch));
    }

    #[test]
    fn positive_action_value_cannot_compensate_for_time_failure() {
        let mut case = valid_case();
        case.hazard_arrival.as_mut().unwrap().earliest = 200;
        case.hazard_arrival.as_mut().unwrap().latest = 250;
        let assessment = evaluate(case);
        assert_eq!(assessment.net_benefit.disposition, ClaimDisposition::Supports);
        assert_eq!(assessment.disposition, ClaimDisposition::Refutes);
    }

    #[test]
    fn action_opportunity_is_separate_from_reach_coverage_disposition() {
        let assessment = evaluate(valid_case());
        assert_eq!(assessment.reach_disposition, ClaimDisposition::Refutes);
        assert_eq!(assessment.disposition, ClaimDisposition::Supports);
    }

    #[test]
    fn duplicate_group_action_cases_fail_closed() {
        let case = valid_case();
        assert!(matches!(
            audit_action_opportunity(&reach_report(), &[case.clone(), case], &policy()),
            Err(OpportunityError::DuplicateGroupAction(_))
        ));
    }

    #[test]
    fn no_reached_population_makes_action_opportunity_unavailable() {
        let mut reach = reach_report();
        reach.groups[0].unique_reached = Some(0);
        let report = audit_action_opportunity(&reach, &[valid_case()], &policy()).unwrap();
        assert_eq!(report.assessments[0].disposition, ClaimDisposition::Unavailable);
        assert_eq!(report.assessments[0].unique_reached_count, Some(0));
    }

    #[test]
    fn action_opportunity_csv_is_deterministic_and_per_case() {
        let first = audit_action_opportunity(&reach_report(), &[valid_case()], &policy()).unwrap();
        let second = audit_action_opportunity(&reach_report(), &[valid_case()], &policy()).unwrap();
        assert_eq!(first.to_csv(), second.to_csv());
        assert_eq!(first.to_csv().lines().count(), 3);
        assert!(first.to_csv().lines().all(|line| line.matches(',').count() == 21));
        assert!(first.to_csv().contains("reached_population_only"));
    }

    #[test]
    fn unit_mismatch_prevents_net_benefit_support() {
        let mut case = valid_case();
        case.action_burden.as_mut().unwrap().unit_id = "eur-2026".into();
        let assessment = evaluate(case);
        assert_eq!(assessment.net_benefit.disposition, ClaimDisposition::Indeterminate);
        assert!(assessment.violations.contains(&OpportunityViolation::UtilityUnitMismatch));
    }
}
