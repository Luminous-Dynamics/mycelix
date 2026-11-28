#![forbid(unsafe_code)]

//! DJCC V0.1 structural composition verifier.
//!
//! This crate verifies stage identity, exact dependency edges, digest equality,
//! profile-limited NotApplicable states, and claim-ceiling non-widening.
//! It does not authenticate digest producers, verify signatures, prove owner
//! universe completeness, establish source truth, execute policy, authorize an
//! effect, or verify a physical outcome.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt;

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum StageId {
    VerifierFrontier,
    SourceTrust,
    SemanticProjection,
    Query,
    Policy,
    Authorization,
    Execution,
    Outcome,
}

pub const ALL_STAGE_IDS: [StageId; 8] = [
    StageId::VerifierFrontier,
    StageId::SourceTrust,
    StageId::SemanticProjection,
    StageId::Query,
    StageId::Policy,
    StageId::Authorization,
    StageId::Execution,
    StageId::Outcome,
];

const SOURCE_TRUST_DEPS: &[StageId] = &[StageId::VerifierFrontier];
const SEMANTIC_DEPS: &[StageId] = &[StageId::VerifierFrontier, StageId::SourceTrust];
const QUERY_DEPS: &[StageId] = &[
    StageId::VerifierFrontier,
    StageId::SourceTrust,
    StageId::SemanticProjection,
];
const POLICY_DEPS: &[StageId] = &[StageId::Query];
const AUTHORIZATION_DEPS: &[StageId] = &[StageId::VerifierFrontier, StageId::Policy];
const EXECUTION_DEPS: &[StageId] = &[StageId::Authorization];
const OUTCOME_DEPS: &[StageId] = &[
    StageId::SourceTrust,
    StageId::SemanticProjection,
    StageId::Execution,
];

impl StageId {
    pub fn required_dependencies(self) -> &'static [StageId] {
        match self {
            StageId::VerifierFrontier => &[],
            StageId::SourceTrust => SOURCE_TRUST_DEPS,
            StageId::SemanticProjection => SEMANTIC_DEPS,
            StageId::Query => QUERY_DEPS,
            StageId::Policy => POLICY_DEPS,
            StageId::Authorization => AUTHORIZATION_DEPS,
            StageId::Execution => EXECUTION_DEPS,
            StageId::Outcome => OUTCOME_DEPS,
        }
    }

    pub fn as_str(self) -> &'static str {
        match self {
            StageId::VerifierFrontier => "verifier_frontier",
            StageId::SourceTrust => "source_trust",
            StageId::SemanticProjection => "semantic_projection",
            StageId::Query => "query",
            StageId::Policy => "policy",
            StageId::Authorization => "authorization",
            StageId::Execution => "execution",
            StageId::Outcome => "outcome",
        }
    }
}

impl fmt::Display for StageId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StageDisposition {
    VerifiedForDeclaredClaim,
    Rejected,
    Indeterminate,
    NotEvaluated,
    UnsupportedByProfile,
    NotApplicable,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ClaimState {
    EstablishedWithinProfile,
    Rejected,
    Indeterminate,
    NotEvaluated,
    UnsupportedByProfile,
    NotApplicable,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CompositionDisposition {
    DecisionJustifiedAtAcceptedFrontier,
    DecisionNotJustifiedAtAcceptedFrontier,
    Indeterminate,
    HistoricalReplayMismatch,
    UnsupportedByProfile,
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub struct Digest(pub [u8; 32]);

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Producer {
    pub owner_ref: String,
    pub version_ref: String,
    pub source_commit: String,
    pub build_digest: Digest,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct InputBinding {
    pub name: String,
    pub digest: Digest,
    pub schema_ref: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Dependency {
    pub stage_id: StageId,
    pub output_digest: Digest,
    pub schema_ref: String,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ClaimCeiling {
    pub authentication: ClaimState,
    pub completeness: ClaimState,
    pub truth: ClaimState,
    pub authority: ClaimState,
    pub effect: ClaimState,
}

impl ClaimCeiling {
    pub const fn established() -> Self {
        Self {
            authentication: ClaimState::EstablishedWithinProfile,
            completeness: ClaimState::EstablishedWithinProfile,
            truth: ClaimState::EstablishedWithinProfile,
            authority: ClaimState::EstablishedWithinProfile,
            effect: ClaimState::EstablishedWithinProfile,
        }
    }
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct StageEvidence {
    pub disposition: StageDisposition,
    pub profile_ref: String,
    pub producer: Producer,
    pub input_bindings: Vec<InputBinding>,
    pub output_digest: Option<Digest>,
    pub claim_ceiling: ClaimCeiling,
    pub dependencies: Vec<Dependency>,
    pub reason_codes: Vec<String>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CompositionInput {
    pub certificate_id: String,
    pub stages: BTreeMap<StageId, StageEvidence>,
    pub declared_disposition: CompositionDisposition,
    pub declared_claim_ceiling: ClaimCeiling,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CompositionProfile {
    pub profile_ref: String,
    pub profile_digest: Digest,
    pub allowed_not_applicable: BTreeSet<StageId>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Violation {
    pub code: &'static str,
    pub message: String,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct VerificationReport {
    pub derived_disposition: CompositionDisposition,
    pub violations: Vec<Violation>,
}

impl VerificationReport {
    pub fn is_valid(&self) -> bool {
        self.violations.is_empty()
    }
}

fn violation(code: &'static str, message: impl Into<String>) -> Violation {
    Violation {
        code,
        message: message.into(),
    }
}

fn requires_output(disposition: StageDisposition) -> bool {
    matches!(
        disposition,
        StageDisposition::VerifiedForDeclaredClaim
            | StageDisposition::Rejected
            | StageDisposition::Indeterminate
    )
}

fn valid_commit(value: &str) -> bool {
    (value.len() == 40 || value.len() == 64)
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn validate_producer(stage_id: StageId, producer: &Producer, out: &mut Vec<Violation>) {
    for (field, value) in [
        ("owner_ref", producer.owner_ref.as_str()),
        ("version_ref", producer.version_ref.as_str()),
    ] {
        if value.trim().is_empty() {
            out.push(violation(
                "DJCC-PRODUCER-IDENTITY",
                format!("{stage_id}: producer {field} is empty"),
            ));
        }
    }
    if !valid_commit(&producer.source_commit) {
        out.push(violation(
            "DJCC-PRODUCER-COMMIT",
            format!("{stage_id}: source_commit must be a lowercase 40- or 64-character hexadecimal SHA"),
        ));
    }
}

fn validate_bindings(stage_id: StageId, bindings: &[InputBinding], out: &mut Vec<Violation>) {
    let mut previous: Option<&str> = None;
    for binding in bindings {
        if binding.name.trim().is_empty() || binding.schema_ref.trim().is_empty() {
            out.push(violation(
                "DJCC-INPUT-BINDING-SHAPE",
                format!("{stage_id}: input binding name/schema_ref is empty"),
            ));
        }
        if let Some(prior) = previous {
            if prior >= binding.name.as_str() {
                out.push(violation(
                    "DJCC-INPUT-BINDING-ORDER",
                    format!("{stage_id}: input bindings must be strictly sorted by name and unique"),
                ));
                break;
            }
        }
        previous = Some(&binding.name);
    }
}

fn validate_stage_dependencies(
    stage_id: StageId,
    stage: &StageEvidence,
    stages: &BTreeMap<StageId, StageEvidence>,
    out: &mut Vec<Violation>,
) {
    let expected = stage_id.required_dependencies();

    if !requires_output(stage.disposition) {
        if !stage.dependencies.is_empty() {
            out.push(violation(
                "DJCC-UNRESOLVED-STAGE-DEPENDENCY",
                format!(
                    "{stage_id}: a stage without an output digest cannot claim consumed stage dependencies"
                ),
            ));
        }
        return;
    }

    let actual_ids: Vec<_> = stage.dependencies.iter().map(|dep| dep.stage_id).collect();
    if actual_ids != expected {
        out.push(violation(
            "DJCC-DEPENDENCY-SET",
            format!(
                "{stage_id}: dependencies must exactly match the profile edge set; expected {expected:?}, got {actual_ids:?}"
            ),
        ));
        return;
    }

    for dependency in &stage.dependencies {
        if dependency.schema_ref.trim().is_empty() {
            out.push(violation(
                "DJCC-DEPENDENCY-SCHEMA",
                format!("{stage_id}: dependency {} has an empty schema_ref", dependency.stage_id),
            ));
        }
        match stages.get(&dependency.stage_id) {
            Some(parent) => match parent.output_digest {
                Some(parent_digest) if parent_digest == dependency.output_digest => {}
                Some(_) => out.push(violation(
                    "DJCC-EDGE-DIGEST-MISMATCH",
                    format!(
                        "{stage_id}: consumed digest for {} differs from producer output",
                        dependency.stage_id
                    ),
                )),
                None => out.push(violation(
                    "DJCC-DEPENDENCY-WITHOUT-OUTPUT",
                    format!(
                        "{stage_id}: dependency {} has no producer output digest",
                        dependency.stage_id
                    ),
                )),
            },
            None => out.push(violation(
                "DJCC-DEPENDENCY-MISSING",
                format!("{stage_id}: dependency {} is absent", dependency.stage_id),
            )),
        }
    }
}

fn dimension_values(ceiling: ClaimCeiling) -> [ClaimState; 5] {
    [
        ceiling.authentication,
        ceiling.completeness,
        ceiling.truth,
        ceiling.authority,
        ceiling.effect,
    ]
}

fn validate_claim_ceiling(
    input: &CompositionInput,
    out: &mut Vec<Violation>,
) {
    let declared = dimension_values(input.declared_claim_ceiling);
    for (index, stage_id) in ALL_STAGE_IDS.iter().enumerate() {
        let Some(stage) = input.stages.get(stage_id) else {
            continue;
        };
        let owner_claims = dimension_values(stage.claim_ceiling);
        for dimension in 0..5 {
            if declared[dimension] == ClaimState::EstablishedWithinProfile
                && !matches!(
                    owner_claims[dimension],
                    ClaimState::EstablishedWithinProfile | ClaimState::NotApplicable
                )
            {
                out.push(violation(
                    "DJCC-CLAIM-CEILING-WIDENED",
                    format!(
                        "composition claim dimension {dimension} is established despite {stage_id} ceiling {:?}",
                        owner_claims[dimension]
                    ),
                ));
            }
        }
        let _ = index;
    }
}

fn derive_disposition(stages: &BTreeMap<StageId, StageEvidence>) -> CompositionDisposition {
    if stages
        .get(&StageId::Policy)
        .is_some_and(|stage| stage.disposition == StageDisposition::Rejected)
    {
        return CompositionDisposition::HistoricalReplayMismatch;
    }
    if stages
        .get(&StageId::Authorization)
        .is_some_and(|stage| stage.disposition == StageDisposition::Rejected)
    {
        return CompositionDisposition::DecisionNotJustifiedAtAcceptedFrontier;
    }
    if ALL_STAGE_IDS.iter().any(|stage_id| {
        stages
            .get(stage_id)
            .is_some_and(|stage| stage.disposition == StageDisposition::Rejected)
    }) {
        return CompositionDisposition::DecisionNotJustifiedAtAcceptedFrontier;
    }
    if ALL_STAGE_IDS.iter().any(|stage_id| {
        stages.get(stage_id).is_some_and(|stage| {
            stage.disposition == StageDisposition::UnsupportedByProfile
        })
    }) {
        return CompositionDisposition::UnsupportedByProfile;
    }
    if ALL_STAGE_IDS.iter().any(|stage_id| {
        stages.get(stage_id).is_some_and(|stage| {
            matches!(
                stage.disposition,
                StageDisposition::Indeterminate | StageDisposition::NotEvaluated
            )
        })
    }) {
        return CompositionDisposition::Indeterminate;
    }
    CompositionDisposition::DecisionJustifiedAtAcceptedFrontier
}

/// Verify stage composition structure and claim-ceiling monotonicity.
///
/// The caller must independently authenticate producer receipts and verify each
/// stage's native claim. This function only checks that the declared stage
/// artifacts compose without digest substitution or claim-ceiling widening.
pub fn verify_composition(
    input: &CompositionInput,
    profile: &CompositionProfile,
) -> VerificationReport {
    let mut violations = Vec::new();

    if input.certificate_id.trim().is_empty() {
        violations.push(violation(
            "DJCC-CERTIFICATE-ID",
            "certificate_id is empty",
        ));
    }
    if profile.profile_ref.trim().is_empty() {
        violations.push(violation(
            "DJCC-PROFILE-REF",
            "composition profile_ref is empty",
        ));
    }

    for stage_id in ALL_STAGE_IDS {
        if !input.stages.contains_key(&stage_id) {
            violations.push(violation(
                "DJCC-STAGE-MISSING",
                format!("required stage {stage_id} is absent"),
            ));
        }
    }
    for stage_id in input.stages.keys() {
        if !ALL_STAGE_IDS.contains(stage_id) {
            violations.push(violation(
                "DJCC-STAGE-UNKNOWN",
                format!("unknown stage {stage_id} is present"),
            ));
        }
    }

    for (stage_id, stage) in &input.stages {
        validate_producer(*stage_id, &stage.producer, &mut violations);
        if stage.profile_ref.trim().is_empty() {
            violations.push(violation(
                "DJCC-STAGE-PROFILE",
                format!("{stage_id}: profile_ref is empty"),
            ));
        }

        if requires_output(stage.disposition) != stage.output_digest.is_some() {
            violations.push(violation(
                "DJCC-STAGE-OUTPUT-DISPOSITION",
                format!(
                    "{stage_id}: output digest presence is inconsistent with disposition {:?}",
                    stage.disposition
                ),
            ));
        }

        if stage.disposition == StageDisposition::NotApplicable
            && !profile.allowed_not_applicable.contains(stage_id)
        {
            violations.push(violation(
                "DJCC-NOT-APPLICABLE-OUT-OF-PROFILE",
                format!("{stage_id}: NotApplicable is not permitted by this composition profile"),
            ));
        }

        validate_bindings(*stage_id, &stage.input_bindings, &mut violations);
        validate_stage_dependencies(*stage_id, stage, &input.stages, &mut violations);

        let mut previous: Option<&str> = None;
        for reason in &stage.reason_codes {
            if reason.trim().is_empty() {
                violations.push(violation(
                    "DJCC-REASON-CODE-EMPTY",
                    format!("{stage_id}: reason code is empty"),
                ));
            }
            if let Some(prior) = previous {
                if prior >= reason.as_str() {
                    violations.push(violation(
                        "DJCC-REASON-CODE-ORDER",
                        format!("{stage_id}: reason codes must be sorted and unique"),
                    ));
                    break;
                }
            }
            previous = Some(reason);
        }
    }

    validate_claim_ceiling(input, &mut violations);
    let derived_disposition = derive_disposition(&input.stages);
    if input.declared_disposition != derived_disposition {
        violations.push(violation(
            "DJCC-COMPOSITION-DISPOSITION-MISMATCH",
            format!(
                "declared composition disposition {:?} differs from derived {:?}",
                input.declared_disposition, derived_disposition
            ),
        ));
    }

    VerificationReport {
        derived_disposition,
        violations,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(value: u8) -> Digest {
        Digest([value; 32])
    }

    fn producer() -> Producer {
        Producer {
            owner_ref: "owner:test".to_owned(),
            version_ref: "owner-api:v1".to_owned(),
            source_commit: "0123456789abcdef0123456789abcdef01234567".to_owned(),
            build_digest: digest(90),
        }
    }

    fn claim_ceiling() -> ClaimCeiling {
        ClaimCeiling::established()
    }

    fn stage(
        disposition: StageDisposition,
        output_digest: Option<Digest>,
        dependencies: Vec<Dependency>,
    ) -> StageEvidence {
        StageEvidence {
            disposition,
            profile_ref: "profile:test:v1".to_owned(),
            producer: producer(),
            input_bindings: vec![InputBinding {
                name: "scope".to_owned(),
                digest: digest(1),
                schema_ref: "scope:v1".to_owned(),
            }],
            output_digest,
            claim_ceiling: claim_ceiling(),
            dependencies,
            reason_codes: Vec::new(),
        }
    }

    fn dependency(stage_id: StageId, output_digest: Digest) -> Dependency {
        Dependency {
            stage_id,
            output_digest,
            schema_ref: "stage-output:v1".to_owned(),
        }
    }

    fn valid_profile() -> CompositionProfile {
        CompositionProfile {
            profile_ref: "profile:test:v1".to_owned(),
            profile_digest: digest(99),
            allowed_not_applicable: BTreeSet::from([
                StageId::Execution,
                StageId::Outcome,
            ]),
        }
    }

    fn positive_input() -> CompositionInput {
        let d = [
            digest(10),
            digest(11),
            digest(12),
            digest(13),
            digest(14),
            digest(15),
            digest(16),
            digest(17),
        ];
        let mut stages = BTreeMap::new();
        stages.insert(
            StageId::VerifierFrontier,
            stage(
                StageDisposition::VerifiedForDeclaredClaim,
                Some(d[0]),
                vec![],
            ),
        );
        stages.insert(
            StageId::SourceTrust,
            stage(
                StageDisposition::VerifiedForDeclaredClaim,
                Some(d[1]),
                vec![dependency(StageId::VerifierFrontier, d[0])],
            ),
        );
        stages.insert(
            StageId::SemanticProjection,
            stage(
                StageDisposition::VerifiedForDeclaredClaim,
                Some(d[2]),
                vec![
                    dependency(StageId::VerifierFrontier, d[0]),
                    dependency(StageId::SourceTrust, d[1]),
                ],
            ),
        );
        stages.insert(
            StageId::Query,
            stage(
                StageDisposition::VerifiedForDeclaredClaim,
                Some(d[3]),
                vec![
                    dependency(StageId::VerifierFrontier, d[0]),
                    dependency(StageId::SourceTrust, d[1]),
                    dependency(StageId::SemanticProjection, d[2]),
                ],
            ),
        );
        stages.insert(
            StageId::Policy,
            stage(
                StageDisposition::VerifiedForDeclaredClaim,
                Some(d[4]),
                vec![dependency(StageId::Query, d[3])],
            ),
        );
        stages.insert(
            StageId::Authorization,
            stage(
                StageDisposition::VerifiedForDeclaredClaim,
                Some(d[5]),
                vec![
                    dependency(StageId::VerifierFrontier, d[0]),
                    dependency(StageId::Policy, d[4]),
                ],
            ),
        );
        stages.insert(
            StageId::Execution,
            stage(
                StageDisposition::VerifiedForDeclaredClaim,
                Some(d[6]),
                vec![dependency(StageId::Authorization, d[5])],
            ),
        );
        stages.insert(
            StageId::Outcome,
            stage(
                StageDisposition::VerifiedForDeclaredClaim,
                Some(d[7]),
                vec![
                    dependency(StageId::SourceTrust, d[1]),
                    dependency(StageId::SemanticProjection, d[2]),
                    dependency(StageId::Execution, d[6]),
                ],
            ),
        );
        CompositionInput {
            certificate_id: "djcc:test:positive".to_owned(),
            stages,
            declared_disposition: CompositionDisposition::DecisionJustifiedAtAcceptedFrontier,
            declared_claim_ceiling: ClaimCeiling::established(),
        }
    }

    #[test]
    fn complete_chain_composes_when_all_edges_and_claims_match() {
        let report = verify_composition(&positive_input(), &valid_profile());
        assert!(report.is_valid(), "{:?}", report.violations);
        assert_eq!(
            report.derived_disposition,
            CompositionDisposition::DecisionJustifiedAtAcceptedFrontier
        );
    }

    #[test]
    fn supplied_root_without_authentication_cannot_be_promoted() {
        let mut input = positive_input();
        input.stages.get_mut(&StageId::SourceTrust).unwrap().disposition =
            StageDisposition::Indeterminate;
        input.stages.get_mut(&StageId::SourceTrust).unwrap().claim_ceiling.authentication =
            ClaimState::Indeterminate;
        input.declared_disposition = CompositionDisposition::Indeterminate;
        input.declared_claim_ceiling.authentication = ClaimState::Indeterminate;
        let report = verify_composition(&input, &valid_profile());
        assert!(report.is_valid(), "{:?}", report.violations);
    }

    #[test]
    fn hidden_owner_frontier_cannot_be_promoted_to_complete() {
        let mut input = positive_input();
        input.stages.get_mut(&StageId::VerifierFrontier).unwrap().disposition =
            StageDisposition::Indeterminate;
        input.stages.get_mut(&StageId::VerifierFrontier).unwrap().claim_ceiling.completeness =
            ClaimState::Indeterminate;
        input.declared_disposition = CompositionDisposition::Indeterminate;
        input.declared_claim_ceiling.completeness = ClaimState::Indeterminate;
        let report = verify_composition(&input, &valid_profile());
        assert!(report.is_valid(), "{:?}", report.violations);
    }

    #[test]
    fn changed_dependency_digest_is_rejected() {
        let mut input = positive_input();
        input
            .stages
            .get_mut(&StageId::Policy)
            .unwrap()
            .dependencies[0]
            .output_digest = digest(222);
        let report = verify_composition(&input, &valid_profile());
        assert!(report
            .violations
            .iter()
            .any(|item| item.code == "DJCC-EDGE-DIGEST-MISMATCH"));
    }

    #[test]
    fn claim_ceiling_cannot_be_widened_by_composition() {
        let mut input = positive_input();
        input.stages.get_mut(&StageId::Outcome).unwrap().claim_ceiling.effect =
            ClaimState::Indeterminate;
        let report = verify_composition(&input, &valid_profile());
        assert!(report
            .violations
            .iter()
            .any(|item| item.code == "DJCC-CLAIM-CEILING-WIDENED"));
    }

    #[test]
    fn not_applicable_requires_profile_permission() {
        let mut input = positive_input();
        let execution = input.stages.get_mut(&StageId::Execution).unwrap();
        execution.disposition = StageDisposition::NotApplicable;
        execution.output_digest = None;
        execution.dependencies.clear();
        let outcome = input.stages.get_mut(&StageId::Outcome).unwrap();
        outcome.disposition = StageDisposition::NotApplicable;
        outcome.output_digest = None;
        outcome.dependencies.clear();
        input.declared_claim_ceiling.effect = ClaimState::NotApplicable;
        let mut profile = valid_profile();
        profile.allowed_not_applicable.clear();
        let report = verify_composition(&input, &profile);
        assert!(report
            .violations
            .iter()
            .any(|item| item.code == "DJCC-NOT-APPLICABLE-OUT-OF-PROFILE"));
    }

    #[test]
    fn not_applicable_stages_can_be_explicitly_permitted() {
        let mut input = positive_input();
        let execution = input.stages.get_mut(&StageId::Execution).unwrap();
        execution.disposition = StageDisposition::NotApplicable;
        execution.output_digest = None;
        execution.dependencies.clear();
        let outcome = input.stages.get_mut(&StageId::Outcome).unwrap();
        outcome.disposition = StageDisposition::NotApplicable;
        outcome.output_digest = None;
        outcome.dependencies.clear();
        input.declared_claim_ceiling.effect = ClaimState::NotApplicable;
        let report = verify_composition(&input, &valid_profile());
        assert!(report.is_valid(), "{:?}", report.violations);
    }

    #[test]
    fn policy_epoch_rejection_derives_historical_replay_mismatch() {
        let mut input = positive_input();
        input.stages.get_mut(&StageId::Policy).unwrap().disposition =
            StageDisposition::Rejected;
        input.declared_disposition = CompositionDisposition::HistoricalReplayMismatch;
        let report = verify_composition(&input, &valid_profile());
        assert!(report.is_valid(), "{:?}", report.violations);
    }

    #[test]
    fn action_authorization_rejection_derives_not_justified() {
        let mut input = positive_input();
        input.stages.get_mut(&StageId::Authorization).unwrap().disposition =
            StageDisposition::Rejected;
        input.declared_disposition = CompositionDisposition::DecisionNotJustifiedAtAcceptedFrontier;
        let report = verify_composition(&input, &valid_profile());
        assert!(report.is_valid(), "{:?}", report.violations);
    }

    #[test]
    fn output_digest_presence_is_bound_to_stage_disposition() {
        let mut input = positive_input();
        let policy = input.stages.get_mut(&StageId::Policy).unwrap();
        policy.disposition = StageDisposition::NotEvaluated;
        policy.output_digest = Some(digest(14));
        let report = verify_composition(&input, &valid_profile());
        assert!(report
            .violations
            .iter()
            .any(|item| item.code == "DJCC-STAGE-OUTPUT-DISPOSITION"));
    }

    #[test]
    fn dependency_arrays_must_be_in_canonical_order() {
        let mut input = positive_input();
        input
            .stages
            .get_mut(&StageId::SemanticProjection)
            .unwrap()
            .dependencies
            .reverse();
        let report = verify_composition(&input, &valid_profile());
        assert!(report
            .violations
            .iter()
            .any(|item| item.code == "DJCC-DEPENDENCY-SET"));
    }

    #[test]
    fn policy_rejection_cannot_be_averaged_away_by_other_green_stages() {
        let mut input = positive_input();
        input.stages.get_mut(&StageId::Policy).unwrap().disposition =
            StageDisposition::Rejected;
        input.declared_disposition =
            CompositionDisposition::DecisionJustifiedAtAcceptedFrontier;
        let report = verify_composition(&input, &valid_profile());
        assert_eq!(
            report.derived_disposition,
            CompositionDisposition::HistoricalReplayMismatch
        );
        assert!(report
            .violations
            .iter()
            .any(|item| item.code == "DJCC-COMPOSITION-DISPOSITION-MISMATCH"));
    }
}
