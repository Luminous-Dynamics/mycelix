//! Structural qualification for the 2x2x2 inquiry × RER × ACB tournament.
//!
//! This module validates one complete, matched eight-arm block. It keeps
//! authorized rights/provenance and generation-exposure observation mandatory
//! in every arm; only the inquiry, relational-refinement, and response-use
//! constraint treatments vary. It does not estimate factorial effects, prove
//! statistical power, or qualify human-subject research.

use std::collections::BTreeSet;

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct FactorialTreatment {
    pub inquiry: bool,
    pub rer: bool,
    pub acb: bool,
}

impl FactorialTreatment {
    pub const ALL: [Self; 8] = [
        Self { inquiry: false, rer: false, acb: false },
        Self { inquiry: false, rer: false, acb: true },
        Self { inquiry: false, rer: true, acb: false },
        Self { inquiry: false, rer: true, acb: true },
        Self { inquiry: true, rer: false, acb: false },
        Self { inquiry: true, rer: false, acb: true },
        Self { inquiry: true, rer: true, acb: false },
        Self { inquiry: true, rer: true, acb: true },
    ];

    pub fn code(self) -> u8 {
        u8::from(self.inquiry) << 2 | u8::from(self.rer) << 1 | u8::from(self.acb)
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FactorialStudyIdentity {
    pub study_id: String,
    pub benchmark_id: String,
    pub benchmark_version: String,
    pub benchmark_certificate_digest: String,
    pub split_digest: String,
    pub model_snapshot_digest: String,
    pub tokenizer_digest: String,
    pub system_prompt_digest: String,
    pub user_prompt_template_digest: String,
    pub toolchain_digest: String,
    pub decoding_digest: String,
    pub scorer_digest: String,
    pub evaluator_digest: String,
    pub inference_plan_digest: String,
    pub randomization_policy_digest: String,
    pub random_seed_digest: String,
    /// Identical within a matched eight-arm block.
    pub context_budget_tokens: u64,
    pub output_budget_tokens: u64,
    pub compute_budget_microusd: u64,
    pub wall_clock_budget_ms: u64,
    pub memory_budget_bytes: u64,
    pub agent_count: u32,
    /// Same independently qualified witness profile in every arm.
    pub exposure_trust_profile_digest: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FactorialBlockIdentity {
    pub block_id: String,
    /// Cluster unit for uncertainty estimation; requests within this group
    /// must not be treated as independent observations.
    pub source_group_id: String,
    pub task_id: String,
    pub task_digest: String,
    /// The common starting candidate pool before any inquiry treatment.
    pub initial_candidate_pool_digest: String,
    /// The common initial retrieval seeds before any treatment.
    pub initial_retrieval_seed_digest: String,
    pub frozen_history_cutoff_ms: u64,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TreatmentExecutionReceipt {
    /// Must equal the corresponding treatment bit in FactorialTreatment.
    pub applied: bool,
    /// A receipt is required even when the treatment is OFF. It attests that
    /// the disabled control was actually enforced rather than simply omitted.
    pub receipt_digest: String,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FactorialGateOutcome {
    Pass,
    Fail,
    Indeterminate,
    NotApplied,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FactorialOutcome {
    /// Official benchmark scorer output, normalized to [0, 1].
    pub l2_quality: f64,
    pub verified_evidence_coverage: f64,
    /// Paired recalled-memory/L2-required-memory observations.
    pub recalled_pairs: u64,
    pub recalled_but_unused_pairs: u64,
    pub unsupported_material_claims: u64,
    pub conflict_preservation_failures: u64,
    pub unauthorized_disclosure_count: u64,
    pub unauthorized_action_count: u64,
    pub over_refusal_rate: f64,
    /// Measured operational outcomes; no weighted aggregate is constructed.
    pub latency_ms: u64,
    pub total_cost_microusd: u64,
    pub input_tokens: u64,
    pub output_tokens: u64,
    pub inquiry_requests: u64,
    pub newly_verified_source_origins: u64,
    pub rer_added_evidence_count: u64,
    pub generator_visible_evidence_count: u64,
    pub constraint_items_processed: u64,
    pub unaccounted_visible_evidence_count: u64,
    /// Failures remain observations, not invalid-run filters.
    pub rights_provenance_gate: FactorialGateOutcome,
    pub constraint_gate: FactorialGateOutcome,
    pub response_audit_gate: FactorialGateOutcome,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FactorialArmRun {
    pub run_id: String,
    pub treatment: FactorialTreatment,
    pub study: FactorialStudyIdentity,
    pub block: FactorialBlockIdentity,
    /// Pool after the inquiry phase; may vary only when inquiry is ON.
    pub post_inquiry_candidate_pool_digest: String,
    /// All treatments have explicit on/off receipts.
    pub inquiry_execution: TreatmentExecutionReceipt,
    pub rer_execution: TreatmentExecutionReceipt,
    pub acb_execution: TreatmentExecutionReceipt,
    /// A CBP/ACB OFF arm still requires rights/provenance and input-exposure
    /// receipts. Those safety boundaries are not treatment variables.
    pub rights_provenance_receipt_digest: String,
    pub generation_exposure_receipt_digest: String,
    pub execution_receipt_digest: String,
    pub rer_bundle_receipt_digest: String,
    pub constraint_receipt_digest: String,
    pub independent_response_audit_receipt_digest: String,
    pub evaluator_receipt_digest: String,
    pub outcome: FactorialOutcome,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FactorialBlock {
    pub study: FactorialStudyIdentity,
    pub block: FactorialBlockIdentity,
    pub arms: Vec<FactorialArmRun>,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FactorialQualificationStatus {
    Pass,
    Fail,
    Indeterminate,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FactorialArmObservation {
    pub treatment: FactorialTreatment,
    pub run_id: String,
    pub l2_quality: f64,
    pub verified_evidence_coverage: f64,
    pub recalled_but_unused_rate: Option<f64>,
    pub unsupported_material_claims: u64,
    pub conflict_preservation_failures: u64,
    pub unauthorized_disclosure_count: u64,
    pub unauthorized_action_count: u64,
    pub over_refusal_rate: f64,
    pub latency_ms: u64,
    pub total_cost_microusd: u64,
    pub input_tokens: u64,
    pub output_tokens: u64,
    pub inquiry_requests: u64,
    pub newly_verified_source_origins: u64,
    pub rer_added_evidence_count: u64,
    pub generator_visible_evidence_count: u64,
    pub constraint_items_processed: u64,
    pub unaccounted_visible_evidence_count: u64,
    pub rights_provenance_gate: FactorialGateOutcome,
    pub constraint_gate: FactorialGateOutcome,
    pub response_audit_gate: FactorialGateOutcome,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FactorialQualificationReport {
    pub status: FactorialQualificationStatus,
    pub block_id: String,
    pub source_group_id: String,
    pub qualified_arm_count: usize,
    pub observations: Vec<FactorialArmObservation>,
    pub reasons: Vec<String>,
}

fn nonempty(value: &str) -> bool {
    !value.trim().is_empty()
}

fn valid_digest(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn valid_unit_interval(value: f64) -> bool {
    value.is_finite() && (0.0..=1.0).contains(&value)
}

fn study_identity_errors(study: &FactorialStudyIdentity) -> Vec<String> {
    let mut errors = Vec::new();
    for (name, value) in [
        ("study ID", study.study_id.as_str()),
        ("benchmark ID", study.benchmark_id.as_str()),
        ("benchmark version", study.benchmark_version.as_str()),
    ] {
        if !nonempty(value) {
            errors.push(format!("{name} must be non-empty"));
        }
    }
    for (name, value) in [
        ("benchmark certificate", study.benchmark_certificate_digest.as_str()),
        ("split", study.split_digest.as_str()),
        ("model snapshot", study.model_snapshot_digest.as_str()),
        ("tokenizer", study.tokenizer_digest.as_str()),
        ("system prompt", study.system_prompt_digest.as_str()),
        ("user prompt template", study.user_prompt_template_digest.as_str()),
        ("toolchain", study.toolchain_digest.as_str()),
        ("decoding", study.decoding_digest.as_str()),
        ("scorer", study.scorer_digest.as_str()),
        ("evaluator", study.evaluator_digest.as_str()),
        ("inference plan", study.inference_plan_digest.as_str()),
        ("randomization policy", study.randomization_policy_digest.as_str()),
        ("random seed", study.random_seed_digest.as_str()),
        ("exposure trust profile", study.exposure_trust_profile_digest.as_str()),
    ] {
        if !valid_digest(value) {
            errors.push(format!("{name} digest is malformed"));
        }
    }
    if study.context_budget_tokens == 0
        || study.output_budget_tokens == 0
        || study.compute_budget_microusd == 0
        || study.wall_clock_budget_ms == 0
        || study.memory_budget_bytes == 0
        || study.agent_count == 0
    {
        errors.push("all resource budgets and agent count must be positive".into());
    }
    errors
}

fn block_identity_errors(block: &FactorialBlockIdentity) -> Vec<String> {
    let mut errors = Vec::new();
    for (name, value) in [
        ("block ID", block.block_id.as_str()),
        ("source group ID", block.source_group_id.as_str()),
        ("task ID", block.task_id.as_str()),
    ] {
        if !nonempty(value) {
            errors.push(format!("{name} must be non-empty"));
        }
    }
    for (name, value) in [
        ("task", block.task_digest.as_str()),
        ("initial candidate pool", block.initial_candidate_pool_digest.as_str()),
        ("initial retrieval seed", block.initial_retrieval_seed_digest.as_str()),
    ] {
        if !valid_digest(value) {
            errors.push(format!("{name} digest is malformed"));
        }
    }
    errors
}

fn run_errors(run: &FactorialArmRun) -> Vec<String> {
    let mut errors = Vec::new();
    if !nonempty(&run.run_id) {
        errors.push("run ID must be non-empty".into());
    }
    for (name, value) in [
        ("post-inquiry candidate pool", run.post_inquiry_candidate_pool_digest.as_str()),
        ("rights/provenance receipt", run.rights_provenance_receipt_digest.as_str()),
        ("generation exposure receipt", run.generation_exposure_receipt_digest.as_str()),
        ("execution receipt", run.execution_receipt_digest.as_str()),
        ("RER bundle receipt", run.rer_bundle_receipt_digest.as_str()),
        ("constraint receipt", run.constraint_receipt_digest.as_str()),
        ("independent response audit receipt", run.independent_response_audit_receipt_digest.as_str()),
        ("evaluator receipt", run.evaluator_receipt_digest.as_str()),
    ] {
        if !valid_digest(value) {
            errors.push(format!("{name} digest is malformed or missing"));
        }
    }
    for (name, receipt) in [
        ("inquiry", &run.inquiry_execution),
        ("RER", &run.rer_execution),
        ("ACB", &run.acb_execution),
    ] {
        if !valid_digest(&receipt.receipt_digest) {
            errors.push(format!("{name} treatment receipt digest is malformed"));
        }
    }
    if run.inquiry_execution.applied != run.treatment.inquiry
        || run.rer_execution.applied != run.treatment.rer
        || run.acb_execution.applied != run.treatment.acb
    {
        errors.push("one or more treatment execution receipts disagree with the assigned arm".into());
    }
    if !run.treatment.inquiry {
        if run.outcome.inquiry_requests != 0
            || run.outcome.newly_verified_source_origins != 0
            || run.post_inquiry_candidate_pool_digest != run.block.initial_candidate_pool_digest
        {
            errors.push("inquiry-OFF arm contains inquiry activity, new verified origins, or a changed candidate pool".into());
        }
    }
    if !run.treatment.rer && run.outcome.rer_added_evidence_count != 0 {
        errors.push("RER-OFF arm reports relationally added evidence".into());
    }
    if run.treatment.acb {
        if run.outcome.constraint_items_processed != run.outcome.generator_visible_evidence_count {
            errors.push("ACB-ON arm did not process the complete generator-visible evidence inventory".into());
        }
        if run.outcome.constraint_gate == FactorialGateOutcome::NotApplied {
            errors.push("ACB-ON arm reports that the constraint gate was not applied".into());
        }
    } else if run.outcome.constraint_items_processed != 0
        || run.outcome.unaccounted_visible_evidence_count != 0
        || run.outcome.constraint_gate != FactorialGateOutcome::NotApplied
    {
        errors.push("ACB-OFF arm reports constraint processing, ACB failures, or an applied constraint gate".into());
    }
    if run.outcome.rights_provenance_gate == FactorialGateOutcome::NotApplied
        || run.outcome.response_audit_gate == FactorialGateOutcome::NotApplied
    {
        errors.push("rights/provenance and independent response-audit controls cannot be disabled by the factorial treatment".into());
    }
    if run.outcome.unaccounted_visible_evidence_count > run.outcome.generator_visible_evidence_count {
        errors.push("unaccounted visible evidence exceeds the generator-visible inventory".into());
    }
    if run.outcome.recalled_but_unused_pairs > run.outcome.recalled_pairs {
        errors.push("recalled-but-unused pair count exceeds recalled pair count".into());
    }
    if !valid_unit_interval(run.outcome.l2_quality)
        || !valid_unit_interval(run.outcome.verified_evidence_coverage)
        || !valid_unit_interval(run.outcome.over_refusal_rate)
    {
        errors.push("normalized quality, evidence-coverage, and over-refusal measures must be finite and within [0, 1]".into());
    }
    errors
}

fn observations(arms: &[FactorialArmRun]) -> Vec<FactorialArmObservation> {
    let mut output = arms
        .iter()
        .map(|run| FactorialArmObservation {
            treatment: run.treatment,
            run_id: run.run_id.clone(),
            l2_quality: run.outcome.l2_quality,
            verified_evidence_coverage: run.outcome.verified_evidence_coverage,
            recalled_but_unused_rate: (run.outcome.recalled_pairs > 0).then(|| {
                run.outcome.recalled_but_unused_pairs as f64 / run.outcome.recalled_pairs as f64
            }),
            unsupported_material_claims: run.outcome.unsupported_material_claims,
            conflict_preservation_failures: run.outcome.conflict_preservation_failures,
            unauthorized_disclosure_count: run.outcome.unauthorized_disclosure_count,
            unauthorized_action_count: run.outcome.unauthorized_action_count,
            over_refusal_rate: run.outcome.over_refusal_rate,
            latency_ms: run.outcome.latency_ms,
            total_cost_microusd: run.outcome.total_cost_microusd,
            input_tokens: run.outcome.input_tokens,
            output_tokens: run.outcome.output_tokens,
            inquiry_requests: run.outcome.inquiry_requests,
            newly_verified_source_origins: run.outcome.newly_verified_source_origins,
            rer_added_evidence_count: run.outcome.rer_added_evidence_count,
            generator_visible_evidence_count: run.outcome.generator_visible_evidence_count,
            constraint_items_processed: run.outcome.constraint_items_processed,
            unaccounted_visible_evidence_count: run.outcome.unaccounted_visible_evidence_count,
            rights_provenance_gate: run.outcome.rights_provenance_gate,
            constraint_gate: run.outcome.constraint_gate,
            response_audit_gate: run.outcome.response_audit_gate,
        })
        .collect::<Vec<_>>();
    output.sort_by_key(|item| item.treatment.code());
    output
}

/// Qualifies one matched eight-arm factorial block.
///
/// PASS means that all eight treatment combinations were executed with matched
/// identity, fixed starting candidate pool/seeds, explicit treatment receipts,
/// mandatory rights/provenance and exposure receipts, and consistent treatment
/// application. It does not estimate factorial effects or determine whether the
/// sample has sufficient independent source groups for confirmatory inference.
pub fn qualify_factorial_block(block: &FactorialBlock) -> FactorialQualificationReport {
    let mut errors = study_identity_errors(&block.study);
    errors.extend(block_identity_errors(&block.block));
    let mut status = if errors.is_empty() {
        FactorialQualificationStatus::Pass
    } else {
        FactorialQualificationStatus::Fail
    };

    if !errors.is_empty() {
        return FactorialQualificationReport {
            status,
            block_id: block.block.block_id.clone(),
            source_group_id: block.block.source_group_id.clone(),
            qualified_arm_count: 0,
            observations: Vec::new(),
            reasons: errors,
        };
    }

    if block.arms.len() != FactorialTreatment::ALL.len() {
        errors.push("factorial block must contain exactly eight treatment arms".into());
    }

    let mut seen_codes = BTreeSet::new();
    let mut seen_run_ids = BTreeSet::new();
    for run in &block.arms {
        if run.study != block.study {
            errors.push(format!("run {} has a mismatched study/model/evaluator/budget identity", run.run_id));
        }
        if run.block != block.block {
            errors.push(format!("run {} has a mismatched task/source-group/initial-pool identity", run.run_id));
        }
        if !seen_codes.insert(run.treatment.code()) {
            errors.push(format!("duplicate factorial treatment arm {}", run.treatment.code()));
        }
        if !seen_run_ids.insert(run.run_id.as_str()) {
            errors.push("factorial arm run IDs must be unique".into());
        }
        errors.extend(run_errors(run).into_iter().map(|reason| {
            format!("arm {}: {reason}", run.treatment.code())
        }));
    }
    for treatment in FactorialTreatment::ALL {
        if !seen_codes.contains(&treatment.code()) {
            errors.push(format!("missing factorial treatment arm {}", treatment.code()));
        }
    }

    if !errors.is_empty() {
        status = FactorialQualificationStatus::Fail;
    }
    FactorialQualificationReport {
        status,
        block_id: block.block.block_id.clone(),
        source_group_id: block.block.source_group_id.clone(),
        qualified_arm_count: if status == FactorialQualificationStatus::Pass { 8 } else { 0 },
        observations: if status == FactorialQualificationStatus::Pass {
            observations(&block.arms)
        } else {
            Vec::new()
        },
        reasons: errors,
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FactorialStudyPlan {
    pub study_id: String,
    /// Exact block identities expected from the preregistered schedule.
    pub expected_block_ids: Vec<String>,
    /// Set by the independent pilot/power plan; the validator checks execution
    /// against it but does not calculate statistical power or ICC.
    pub minimum_source_groups: usize,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FactorialTournament {
    pub plan: FactorialStudyPlan,
    pub blocks: Vec<FactorialBlock>,
}

#[derive(Clone, Debug, PartialEq)]
pub struct FactorialTournamentReport {
    pub status: FactorialQualificationStatus,
    pub qualified_block_count: usize,
    pub independent_source_group_count: usize,
    pub qualified_arm_count: usize,
    pub block_reports: Vec<FactorialQualificationReport>,
    pub reasons: Vec<String>,
}

fn same_study_configuration(left: &FactorialStudyIdentity, right: &FactorialStudyIdentity) -> bool {
    left.study_id == right.study_id
        && left.benchmark_id == right.benchmark_id
        && left.benchmark_version == right.benchmark_version
        && left.benchmark_certificate_digest == right.benchmark_certificate_digest
        && left.split_digest == right.split_digest
        && left.model_snapshot_digest == right.model_snapshot_digest
        && left.tokenizer_digest == right.tokenizer_digest
        && left.system_prompt_digest == right.system_prompt_digest
        && left.user_prompt_template_digest == right.user_prompt_template_digest
        && left.toolchain_digest == right.toolchain_digest
        && left.decoding_digest == right.decoding_digest
        && left.scorer_digest == right.scorer_digest
        && left.evaluator_digest == right.evaluator_digest
        && left.inference_plan_digest == right.inference_plan_digest
        && left.randomization_policy_digest == right.randomization_policy_digest
        && left.context_budget_tokens == right.context_budget_tokens
        && left.output_budget_tokens == right.output_budget_tokens
        && left.compute_budget_microusd == right.compute_budget_microusd
        && left.wall_clock_budget_ms == right.wall_clock_budget_ms
        && left.memory_budget_bytes == right.memory_budget_bytes
        && left.agent_count == right.agent_count
        && left.exposure_trust_profile_digest == right.exposure_trust_profile_digest
}

/// Qualifies a collection of matched eight-arm blocks against the preregistered
/// study schedule and minimum independent source-group count.
///
/// Missing scheduled blocks or too few source groups are INDETERMINATE. Duplicate
/// or unexpected block identities and structurally invalid blocks are FAIL.
/// PASS is a structural eligibility result, not a sample-size/power determination.
pub fn qualify_factorial_tournament(
    tournament: &FactorialTournament,
) -> FactorialTournamentReport {
    let mut reasons = Vec::new();
    if !nonempty(&tournament.plan.study_id) {
        reasons.push("factorial study ID must be non-empty".into());
    }
    if tournament.plan.expected_block_ids.is_empty()
        || !unique_nonempty(&tournament.plan.expected_block_ids)
    {
        reasons.push("expected block IDs must be non-empty and unique".into());
    }
    if tournament.plan.minimum_source_groups == 0 {
        reasons.push("minimum independent source-group count must be positive".into());
    }
    if !reasons.is_empty() {
        return FactorialTournamentReport {
            status: FactorialQualificationStatus::Fail,
            qualified_block_count: 0,
            independent_source_group_count: 0,
            qualified_arm_count: 0,
            block_reports: Vec::new(),
            reasons,
        };
    }

    let expected: BTreeSet<&str> = tournament
        .plan
        .expected_block_ids
        .iter()
        .map(String::as_str)
        .collect();
    let reference_study = tournament.blocks.first().map(|block| &block.study);
    let mut seen_blocks = BTreeSet::new();
    let mut source_groups = BTreeSet::new();
    let mut block_reports = Vec::new();
    let mut invalid_block = false;

    for block in &tournament.blocks {
        if block.study.study_id != tournament.plan.study_id {
            reasons.push(format!(
                "block {} belongs to a different study",
                block.block.block_id
            ));
        }
        if reference_study.is_some_and(|reference| {
            !same_study_configuration(reference, &block.study)
        }) {
            reasons.push(format!(
                "block {} changes the shared model, prompt, evaluator, resource, randomization or witness configuration",
                block.block.block_id
            ));
        }
        if !seen_blocks.insert(block.block.block_id.as_str()) {
            reasons.push(format!(
                "duplicate factorial block ID {}",
                block.block.block_id
            ));
        }
        if !expected.contains(block.block.block_id.as_str()) {
            reasons.push(format!(
                "unexpected factorial block ID {}",
                block.block.block_id
            ));
        }
        source_groups.insert(block.block.source_group_id.as_str());
        let result = qualify_factorial_block(block);
        if result.status != FactorialQualificationStatus::Pass {
            invalid_block = true;
            reasons.push(format!(
                "block {} did not qualify: {}",
                block.block.block_id,
                result.reasons.join("; ")
            ));
        }
        block_reports.push(result);
    }

    for expected_id in &tournament.plan.expected_block_ids {
        if !seen_blocks.contains(expected_id.as_str()) {
            reasons.push(format!("scheduled factorial block {expected_id} is missing"));
        }
    }

    let unexpected_or_duplicate = reasons.iter().any(|reason| {
        reason.starts_with("duplicate factorial block ID")
            || reason.starts_with("unexpected factorial block ID")
            || reason.contains("belongs to a different study")
            || reason.contains("changes the shared model, prompt, evaluator, resource, randomization or witness configuration")
    });
    let missing_scheduled = seen_blocks.len() < expected.len();
    let insufficient_clusters =
        source_groups.len() < tournament.plan.minimum_source_groups;

    let status = if unexpected_or_duplicate || invalid_block {
        FactorialQualificationStatus::Fail
    } else if missing_scheduled || insufficient_clusters {
        FactorialQualificationStatus::Indeterminate
    } else {
        FactorialQualificationStatus::Pass
    };
    let qualified_block_count = block_reports
        .iter()
        .filter(|report| report.status == FactorialQualificationStatus::Pass)
        .count();
    let qualified_arm_count = qualified_block_count * FactorialTreatment::ALL.len();

    if insufficient_clusters {
        reasons.push(format!(
            "only {} independent source groups are present; preregistered minimum is {}",
            source_groups.len(),
            tournament.plan.minimum_source_groups
        ));
    }

    FactorialTournamentReport {
        status,
        qualified_block_count,
        independent_source_group_count: source_groups.len(),
        qualified_arm_count,
        block_reports,
        reasons,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(ch: char) -> String {
        std::iter::repeat_n(ch, 64).collect()
    }

    fn study() -> FactorialStudyIdentity {
        FactorialStudyIdentity {
            study_id: "inquiry-rer-acb-v0.1".into(),
            benchmark_id: "GroupAssistBench".into(),
            benchmark_version: "reproduction-v1".into(),
            benchmark_certificate_digest: digest('1'),
            split_digest: digest('2'),
            model_snapshot_digest: digest('3'),
            tokenizer_digest: digest('4'),
            system_prompt_digest: digest('5'),
            user_prompt_template_digest: digest('6'),
            toolchain_digest: digest('7'),
            decoding_digest: digest('8'),
            scorer_digest: digest('9'),
            evaluator_digest: digest('a'),
            inference_plan_digest: digest('b'),
            randomization_policy_digest: digest('c'),
            random_seed_digest: digest('d'),
            context_budget_tokens: 16000,
            output_budget_tokens: 2000,
            compute_budget_microusd: 100000,
            wall_clock_budget_ms: 60000,
            memory_budget_bytes: 1_000_000,
            agent_count: 1,
            exposure_trust_profile_digest: digest('e'),
        }
    }

    fn block_identity() -> FactorialBlockIdentity {
        FactorialBlockIdentity {
            block_id: "block-001".into(),
            source_group_id: "group-001".into(),
            task_id: "task-001".into(),
            task_digest: digest('f'),
            initial_candidate_pool_digest: digest('1'),
            initial_retrieval_seed_digest: digest('2'),
            frozen_history_cutoff_ms: 1000,
        }
    }

    fn run(treatment: FactorialTreatment, index: usize) -> FactorialArmRun {
        let initial = block_identity();
        let post_pool = if treatment.inquiry { digest('3') } else { initial.initial_candidate_pool_digest.clone() };
        FactorialArmRun {
            run_id: format!("run-{index}"),
            treatment,
            study: study(),
            block: initial,
            post_inquiry_candidate_pool_digest: post_pool,
            inquiry_execution: TreatmentExecutionReceipt { applied: treatment.inquiry, receipt_digest: digest('4') },
            rer_execution: TreatmentExecutionReceipt { applied: treatment.rer, receipt_digest: digest('5') },
            acb_execution: TreatmentExecutionReceipt { applied: treatment.acb, receipt_digest: digest('6') },
            rights_provenance_receipt_digest: digest('7'),
            generation_exposure_receipt_digest: digest('8'),
            execution_receipt_digest: digest('9'),
            rer_bundle_receipt_digest: digest('a'),
            constraint_receipt_digest: digest('b'),
            independent_response_audit_receipt_digest: digest('c'),
            evaluator_receipt_digest: digest('d'),
            outcome: FactorialOutcome {
                l2_quality: 0.70 + (treatment.code() as f64 * 0.01),
                verified_evidence_coverage: 0.80,
                recalled_pairs: 10,
                recalled_but_unused_pairs: if treatment.acb { 1 } else { 3 },
                unsupported_material_claims: 1,
                conflict_preservation_failures: 0,
                unauthorized_disclosure_count: 0,
                unauthorized_action_count: 0,
                over_refusal_rate: 0.02,
                latency_ms: 1000 + u64::from(treatment.code()),
                total_cost_microusd: 50000 + u64::from(treatment.code()),
                input_tokens: 5000,
                output_tokens: 1000,
                inquiry_requests: if treatment.inquiry { 2 } else { 0 },
                newly_verified_source_origins: if treatment.inquiry { 1 } else { 0 },
                rer_added_evidence_count: if treatment.rer { 2 } else { 0 },
                generator_visible_evidence_count: 4,
                constraint_items_processed: if treatment.acb { 4 } else { 0 },
                unaccounted_visible_evidence_count: if treatment.acb { 0 } else { 0 },
                rights_provenance_gate: FactorialGateOutcome::Pass,
                constraint_gate: if treatment.acb {
                    FactorialGateOutcome::Pass
                } else {
                    FactorialGateOutcome::NotApplied
                },
                response_audit_gate: FactorialGateOutcome::Pass,
            },
        }
    }

    fn valid_block() -> FactorialBlock {
        FactorialBlock {
            study: study(),
            block: block_identity(),
            arms: FactorialTreatment::ALL
                .into_iter()
                .enumerate()
                .map(|(index, treatment)| run(treatment, index))
                .collect(),
        }
    }

    #[test]
    fn complete_eight_arm_block_passes() {
        let report = qualify_factorial_block(&valid_block());
        assert_eq!(report.status, FactorialQualificationStatus::Pass, "{:?}", report.reasons);
        assert_eq!(report.qualified_arm_count, 8);
        assert_eq!(report.observations.len(), 8);
    }

    #[test]
    fn missing_arm_fails() {
        let mut block = valid_block();
        block.arms.pop();
        assert_eq!(qualify_factorial_block(&block).status, FactorialQualificationStatus::Fail);
    }

    #[test]
    fn duplicate_arm_fails() {
        let mut block = valid_block();
        block.arms[7].treatment = block.arms[6].treatment;
        assert_eq!(qualify_factorial_block(&block).status, FactorialQualificationStatus::Fail);
    }

    #[test]
    fn every_arm_requires_same_model_and_budget_identity() {
        let mut block = valid_block();
        block.arms[4].study.model_snapshot_digest = digest('0');
        assert_eq!(qualify_factorial_block(&block).status, FactorialQualificationStatus::Fail);
    }

    #[test]
    fn source_group_and_task_are_fixed_within_block() {
        let mut block = valid_block();
        block.arms[3].block.source_group_id = "other-group".into();
        assert_eq!(qualify_factorial_block(&block).status, FactorialQualificationStatus::Fail);
    }

    #[test]
    fn inquiry_off_cannot_acquire_or_change_candidate_pool() {
        let mut block = valid_block();
        block.arms[0].outcome.inquiry_requests = 1;
        let report = qualify_factorial_block(&block);
        assert_eq!(report.status, FactorialQualificationStatus::Fail);
    }

    #[test]
    fn inquiry_on_may_return_no_new_fact_without_being_invalid() {
        let mut block = valid_block();
        block.arms[4].outcome.newly_verified_source_origins = 0;
        block.arms[4].outcome.inquiry_requests = 1;
        assert_eq!(qualify_factorial_block(&block).status, FactorialQualificationStatus::Pass);
    }

    #[test]
    fn rer_off_cannot_report_relational_additions() {
        let mut block = valid_block();
        block.arms[0].outcome.rer_added_evidence_count = 1;
        assert_eq!(qualify_factorial_block(&block).status, FactorialQualificationStatus::Fail);
    }

    #[test]
    fn acb_on_must_account_for_complete_visible_inventory() {
        let mut block = valid_block();
        block.arms[1].outcome.constraint_items_processed = 3;
        assert_eq!(qualify_factorial_block(&block).status, FactorialQualificationStatus::Fail);
    }

    #[test]
    fn rights_and_exposure_receipts_are_mandatory_in_every_arm() {
        let mut block = valid_block();
        block.arms[0].generation_exposure_receipt_digest = "missing".into();
        assert_eq!(qualify_factorial_block(&block).status, FactorialQualificationStatus::Fail);
    }

    #[test]
    fn malformed_normalized_measurement_fails() {
        let mut block = valid_block();
        block.arms[2].outcome.l2_quality = f64::NAN;
        assert_eq!(qualify_factorial_block(&block).status, FactorialQualificationStatus::Fail);
    }

    #[test]
    fn recalled_but_unused_count_cannot_exceed_recalled_pairs() {
        let mut block = valid_block();
        block.arms[5].outcome.recalled_but_unused_pairs = 11;
        assert_eq!(qualify_factorial_block(&block).status, FactorialQualificationStatus::Fail);
    }

    #[test]
    fn constraint_off_does_not_disable_rights_or_independent_response_audit() {
        let mut block = valid_block();
        block.arms[0].outcome.rights_provenance_gate = FactorialGateOutcome::NotApplied;
        let report = qualify_factorial_block(&block);
        assert_eq!(report.status, FactorialQualificationStatus::Fail);
    }

    #[test]
    fn observed_rights_or_response_gate_failures_do_not_erase_valid_arm() {
        let mut block = valid_block();
        block.arms[7].outcome.rights_provenance_gate = FactorialGateOutcome::Fail;
        block.arms[7].outcome.constraint_gate = FactorialGateOutcome::Fail;
        let report = qualify_factorial_block(&block);
        assert_eq!(report.status, FactorialQualificationStatus::Pass);
        assert_eq!(report.observations[7].rights_provenance_gate, FactorialGateOutcome::Fail);
    }
    fn second_block(source_group: &str) -> FactorialBlock {
        let mut block = valid_block();
        block.study.random_seed_digest = digest('e');
        block.block.block_id = "block-002".into();
        block.block.source_group_id = source_group.into();
        block.block.task_id = "task-002".into();
        block.block.task_digest = digest('e');
        block.block.initial_candidate_pool_digest = digest('8');
        block.block.initial_retrieval_seed_digest = digest('9');
        let block_identity = block.block.clone();
        let study_identity = block.study.clone();
        for (index, arm) in block.arms.iter_mut().enumerate() {
            arm.block = block_identity.clone();
            arm.study = study_identity.clone();
            arm.run_id = format!("second-run-{index}");
        }
        block
    }

    #[test]
    fn multi_block_tournament_requires_complete_eight_arm_blocks_and_passes() {
        let tournament = FactorialTournament {
            plan: FactorialStudyPlan {
                study_id: "inquiry-rer-acb-v0.1".into(),
                expected_block_ids: vec!["block-001".into(), "block-002".into()],
                minimum_source_groups: 2,
            },
            blocks: vec![valid_block(), second_block("group-002")],
        };
        let report = qualify_factorial_tournament(&tournament);
        assert_eq!(report.status, FactorialQualificationStatus::Pass, "{:?}", report.reasons);
        assert_eq!(report.qualified_block_count, 2);
        assert_eq!(report.independent_source_group_count, 2);
        assert_eq!(report.qualified_arm_count, 16);
    }

    #[test]
    fn missing_scheduled_block_is_indeterminate() {
        let tournament = FactorialTournament {
            plan: FactorialStudyPlan {
                study_id: "inquiry-rer-acb-v0.1".into(),
                expected_block_ids: vec!["block-001".into(), "block-002".into()],
                minimum_source_groups: 2,
            },
            blocks: vec![valid_block()],
        };
        let report = qualify_factorial_tournament(&tournament);
        assert_eq!(report.status, FactorialQualificationStatus::Indeterminate);
    }

    #[test]
    fn repeated_source_group_does_not_inflate_independent_cluster_count() {
        let tournament = FactorialTournament {
            plan: FactorialStudyPlan {
                study_id: "inquiry-rer-acb-v0.1".into(),
                expected_block_ids: vec!["block-001".into(), "block-002".into()],
                minimum_source_groups: 2,
            },
            blocks: vec![valid_block(), second_block("group-001")],
        };
        let report = qualify_factorial_tournament(&tournament);
        assert_eq!(report.status, FactorialQualificationStatus::Indeterminate);
        assert_eq!(report.independent_source_group_count, 1);
    }

    #[test]
    fn unexpected_block_identity_fails() {
        let mut block = valid_block();
        block.block.block_id = "unregistered-block".into();
        let block_identity = block.block.clone();
        for arm in &mut block.arms {
            arm.block = block_identity.clone();
        }
        let tournament = FactorialTournament {
            plan: FactorialStudyPlan {
                study_id: "inquiry-rer-acb-v0.1".into(),
                expected_block_ids: vec!["block-001".into()],
                minimum_source_groups: 1,
            },
            blocks: vec![block],
        };
        assert_eq!(
            qualify_factorial_tournament(&tournament).status,
            FactorialQualificationStatus::Fail
        );
    }

    #[test]
    fn different_model_or_evaluator_between_blocks_fails() {
        let mut second = second_block("group-002");
        second.study.evaluator_digest = digest('0');
        for arm in &mut second.arms {
            arm.study = second.study.clone();
        }
        let tournament = FactorialTournament {
            plan: FactorialStudyPlan {
                study_id: "inquiry-rer-acb-v0.1".into(),
                expected_block_ids: vec!["block-001".into(), "block-002".into()],
                minimum_source_groups: 2,
            },
            blocks: vec![valid_block(), second],
        };
        assert_eq!(
            qualify_factorial_tournament(&tournament).status,
            FactorialQualificationStatus::Fail
        );
    }

    #[test]
    fn independent_block_random_seeds_are_allowed() {
        let tournament = FactorialTournament {
            plan: FactorialStudyPlan {
                study_id: "inquiry-rer-acb-v0.1".into(),
                expected_block_ids: vec!["block-001".into(), "block-002".into()],
                minimum_source_groups: 2,
            },
            blocks: vec![valid_block(), second_block("group-002")],
        };
        assert_eq!(
            qualify_factorial_tournament(&tournament).status,
            FactorialQualificationStatus::Pass
        );
    }

    #[test]
    fn invalid_arm_block_fails_tournament() {
        let mut block = valid_block();
        block.arms[0].study.model_snapshot_digest = digest('0');
        let tournament = FactorialTournament {
            plan: FactorialStudyPlan {
                study_id: "inquiry-rer-acb-v0.1".into(),
                expected_block_ids: vec!["block-001".into()],
                minimum_source_groups: 1,
            },
            blocks: vec![block],
        };
        assert_eq!(
            qualify_factorial_tournament(&tournament).status,
            FactorialQualificationStatus::Fail
        );
    }

}
