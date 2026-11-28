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

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum ClaimDimension {
    Authentication,
    Authority,
    Completeness,
    Effect,
    Independence,
    Truth,
}

impl ClaimDimension {
    pub fn as_str(self) -> &'static str {
        match self {
            ClaimDimension::Authentication => "authentication",
            ClaimDimension::Completeness => "completeness",
            ClaimDimension::Truth => "truth",
            ClaimDimension::Authority => "authority",
            ClaimDimension::Effect => "effect",
            ClaimDimension::Independence => "independence",
        }
    }
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
pub struct StageEdgeV1 {
    pub producer_stage: String,
    pub consumer_stage: String,
    pub input_name: String,
    pub output_schema_ref: String,
    pub required_claims: Vec<ClaimDimension>,
    pub output_digest: Digest,
}

/// Encode one producer-output to consumer-input edge using the DJCC V1
/// domain-separated byte contract. Strings use UTF-8 plus big-endian u32 byte
/// lengths; the 32-byte output digest is appended raw.
pub fn canonical_stage_edge_bytes_v1(edge: &StageEdgeV1) -> Result<Vec<u8>, String> {
    const STAGES: [&str; 8] = [
        "verifier_frontier",
        "source_trust",
        "semantic_projection",
        "query",
        "policy",
        "authorization",
        "execution",
        "outcome",
    ];
    if !STAGES.contains(&edge.producer_stage.as_str())
        || !STAGES.contains(&edge.consumer_stage.as_str())
    {
        return Err("stage edge contains an unknown producer or consumer stage".to_owned());
    }

    fn write_string(buffer: &mut Vec<u8>, value: &str) -> Result<(), String> {
        if value.trim().is_empty() {
            return Err("canonical DJCC string is empty".to_owned());
        }
        let bytes = value.as_bytes();
        if bytes.len() > 4096 {
            return Err("canonical DJCC string exceeds 4096 UTF-8 bytes".to_owned());
        }
        let length = u32::try_from(bytes.len())
            .map_err(|_| "canonical DJCC string length exceeds u32".to_owned())?;
        buffer.extend_from_slice(&length.to_be_bytes());
        buffer.extend_from_slice(bytes);
        Ok(())
    }

    let mut output = Vec::new();
    write_string(&mut output, "MYCELIX-DJCC-STAGE-EDGE-V1")?;
    for value in [
        edge.producer_stage.as_str(),
        edge.consumer_stage.as_str(),
        edge.input_name.as_str(),
        edge.output_schema_ref.as_str(),
    ] {
        write_string(&mut output, value)?;
    }
    if edge.required_claims.is_empty()
        || edge
            .required_claims
            .windows(2)
            .any(|pair| pair[0] >= pair[1])
    {
        return Err("required claim dimensions must be non-empty, sorted and unique".to_owned());
    }
    let claim_count = u32::try_from(edge.required_claims.len())
        .map_err(|_| "required claim dimension count exceeds u32".to_owned())?;
    output.extend_from_slice(&claim_count.to_be_bytes());
    for claim in &edge.required_claims {
        write_string(&mut output, claim.as_str())?;
    }
    output.extend_from_slice(&edge.output_digest.0);
    Ok(output)
}

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
    pub input_name: String,
    pub required_claims: Vec<ClaimDimension>,
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
    pub independence: ClaimState,
}

impl ClaimCeiling {
    pub const fn established() -> Self {
        Self {
            authentication: ClaimState::EstablishedWithinProfile,
            completeness: ClaimState::EstablishedWithinProfile,
            truth: ClaimState::EstablishedWithinProfile,
            authority: ClaimState::EstablishedWithinProfile,
            effect: ClaimState::EstablishedWithinProfile,
            independence: ClaimState::NotEvaluated,
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
    pub output_schema_ref: String,
    pub claim_ceiling: ClaimCeiling,
    pub dependencies: Vec<Dependency>,
    pub reason_codes: Vec<String>,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CompositionInput {
    pub certificate_id: String,
    pub profile_ref: String,
    pub profile_digest: Digest,
    pub scope_digest: Digest,
    pub stages: BTreeMap<StageId, StageEvidence>,
    pub declared_disposition: CompositionDisposition,
    pub declared_claim_ceiling: ClaimCeiling,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CompositionProfile {
    pub profile_ref: String,
    pub profile_digest: Digest,
    pub scope_schema_ref: String,
    pub allowed_not_applicable: BTreeSet<StageId>,
    pub required_claims: BTreeSet<ClaimDimension>,
    pub claim_stages: BTreeMap<ClaimDimension, BTreeSet<StageId>>,
    pub dependency_claims: BTreeMap<StageId, BTreeMap<StageId, BTreeSet<ClaimDimension>>>,
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
        if binding.name.starts_with("stage:") {
            let matches = bindings.iter().filter(|candidate| candidate.name == binding.name).count();
            if matches != 1 {
                out.push(violation(
                    "DJCC-STAGE-INPUT-NONUNIQUE",
                    format!("{stage_id}: stage input binding {} is not unique", binding.name),
                ));
            }
        }
        previous = Some(&binding.name);
    }
}

fn validate_stage_dependencies(
    stage_id: StageId,
    stage: &StageEvidence,
    stages: &BTreeMap<StageId, StageEvidence>,
    profile: &CompositionProfile,
    out: &mut Vec<Violation>,
) {
    let expected = stage_id.required_dependencies();

    for binding in &stage.input_bindings {
        if binding.name.starts_with("stage:") {
            let matched = stage.dependencies.iter().any(|dependency| {
                dependency.input_name == binding.name
                    && dependency.output_digest == binding.digest
                    && dependency.schema_ref == binding.schema_ref
            });
            if !matched {
                out.push(violation(
                    "DJCC-UNBOUND-STAGE-INPUT",
                    format!("{stage_id}: stage input {} has no matching dependency edge", binding.name),
                ));
            }
        }
    }

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
    if actual_ids.as_slice() != expected {
        out.push(violation(
            "DJCC-DEPENDENCY-SET",
            format!(
                "{stage_id}: dependencies must exactly match the profile edge set; expected {expected:?}, got {actual_ids:?}"
            ),
        ));
        return;
    }

    for dependency in &stage.dependencies {
        if dependency.input_name.trim().is_empty() || dependency.schema_ref.trim().is_empty() {
            out.push(violation(
                "DJCC-DEPENDENCY-SCHEMA",
                format!("{stage_id}: dependency {} has an empty input_name/schema_ref", dependency.stage_id),
            ));
        }
        if dependency.required_claims.is_empty()
            || dependency.required_claims.windows(2).any(|pair| pair[0] >= pair[1])
        {
            out.push(violation(
                "DJCC-DEPENDENCY-CLAIMS",
                format!("{stage_id}: dependency {} must name a non-empty sorted unique claim requirement set", dependency.stage_id),
            ));
        }
        match profile
            .dependency_claims
            .get(&stage_id)
            .and_then(|parents| parents.get(&dependency.stage_id))
        {
            Some(profile_claims)
                if dependency.required_claims.iter().copied().collect::<BTreeSet<_>>()
                    == profile_claims.clone() => {}
            Some(_) => out.push(violation(
                "DJCC-EDGE-CLAIMS-WEAKENED",
                format!("{stage_id}: dependency {} claim requirements differ from the selected profile", dependency.stage_id),
            )),
            None => out.push(violation(
                "DJCC-EDGE-OUTSIDE-PROFILE",
                format!("{stage_id}: dependency {} is not declared by the selected profile", dependency.stage_id),
            )),
        }
        let matching_input = stage.input_bindings.iter().filter(|binding| {
            binding.name == dependency.input_name
                && binding.digest == dependency.output_digest
                && binding.schema_ref == dependency.schema_ref
        }).count();
        if matching_input != 1 {
            out.push(violation(
                "DJCC-DEPENDENCY-INPUT-BINDING",
                format!("{stage_id}: dependency {} is not bound to exactly one matching input binding", dependency.stage_id),
            ));
        }
        match stages.get(&dependency.stage_id) {
            Some(parent) => {
                if stage.disposition == StageDisposition::VerifiedForDeclaredClaim {
                    for required_claim in &dependency.required_claims {
                        if claim_state(parent.claim_ceiling, *required_claim)
                            != ClaimState::EstablishedWithinProfile
                        {
                            out.push(violation(
                                "DJCC-DEPENDENCY-CLAIM-UNMET",
                                format!(
                                    "{stage_id}: dependency {} does not establish required claim {}",
                                    dependency.stage_id,
                                    required_claim.as_str()
                                ),
                            ));
                        }
                    }
                }
                if parent.output_schema_ref != dependency.schema_ref {
                    out.push(violation(
                        "DJCC-EDGE-SCHEMA-MISMATCH",
                        format!("{stage_id}: consumed schema for {} differs from producer output schema", dependency.stage_id),
                    ));
                }
                match parent.output_digest {
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
                }
            }
            None => out.push(violation(
                "DJCC-DEPENDENCY-MISSING",
                format!("{stage_id}: dependency {} is absent", dependency.stage_id),
            )),
        }
    }
}

fn claim_state(ceiling: ClaimCeiling, dimension: ClaimDimension) -> ClaimState {
    match dimension {
        ClaimDimension::Authentication => ceiling.authentication,
        ClaimDimension::Completeness => ceiling.completeness,
        ClaimDimension::Truth => ceiling.truth,
        ClaimDimension::Authority => ceiling.authority,
        ClaimDimension::Effect => ceiling.effect,
        ClaimDimension::Independence => ceiling.independence,
    }
}

fn dimension_values(ceiling: ClaimCeiling) -> [ClaimState; 6] {
    [
        ceiling.authentication,
        ceiling.completeness,
        ceiling.truth,
        ceiling.authority,
        ceiling.effect,
        ceiling.independence,
    ]
}

fn validate_profile_dependency_graph(profile: &CompositionProfile, out: &mut Vec<Violation>) {
    for stage_id in ALL_STAGE_IDS {
        let expected = stage_id.required_dependencies();
        let actual = profile.dependency_claims.get(&stage_id);
        let actual_ids: Vec<_> = actual
            .map(|parents| parents.keys().copied().collect())
            .unwrap_or_default();
        if actual_ids.as_slice() != expected {
            out.push(violation(
                "DJCC-PROFILE-EDGE-SET",
                format!(
                    "profile dependency graph for {stage_id} must contain exactly {expected:?}, got {actual_ids:?}"
                ),
            ));
        }
        if let Some(parents) = actual {
            for (parent_id, claims) in parents {
                if claims.is_empty() {
                    out.push(violation(
                        "DJCC-PROFILE-EDGE-CLAIMS-EMPTY",
                        format!("profile edge {stage_id} <- {parent_id} has no required claims"),
                    ));
                }
            }
        }
    }
}

fn validate_claim_ceiling(
    input: &CompositionInput,
    profile: &CompositionProfile,
    out: &mut Vec<Violation>,
) {
    let declared = input.declared_claim_ceiling;
    for dimension in &profile.required_claims {
        let Some(required_stages) = profile.claim_stages.get(dimension) else {
            out.push(violation(
                "DJCC-PROFILE-CLAIM-STAGES-MISSING",
                format!("profile has no stage map for required claim {}", dimension.as_str()),
            ));
            continue;
        };
        if required_stages.is_empty() {
            out.push(violation(
                "DJCC-PROFILE-CLAIM-STAGES-EMPTY",
                format!("profile claim {} has no producing stages", dimension.as_str()),
            ));
        }
    }
    for (dimension, required_stages) in &profile.claim_stages {
        if !profile.required_claims.contains(dimension) {
            out.push(violation(
                "DJCC-PROFILE-CLAIM-STAGES-UNUSED",
                format!("profile maps claim {} that is not required", dimension.as_str()),
            ));
        }
        if claim_state(declared, *dimension) != ClaimState::EstablishedWithinProfile {
            // A conservative root disposition may remain Indeterminate,
            // Rejected, Unsupported or NotEvaluated even when some owner
            // stages have stronger partial claims. Never force a wider root
            // conclusion merely because a producing stage is green.
            continue;
        }
        for stage_id in required_stages {
            match input.stages.get(stage_id) {
                Some(stage)
                    if claim_state(stage.claim_ceiling, *dimension)
                        == ClaimState::EstablishedWithinProfile => {}
                Some(stage) => out.push(violation(
                    "DJCC-CLAIM-CEILING-WIDENED",
                    format!(
                        "composition claim {} is established despite required stage {} ceiling {:?}",
                        dimension.as_str(),
                        stage_id,
                        claim_state(stage.claim_ceiling, *dimension)
                    ),
                )),
                None => out.push(violation(
                    "DJCC-CLAIM-STAGE-MISSING",
                    format!(
                        "composition claim {} requires absent stage {}",
                        dimension.as_str(),
                        stage_id
                    ),
                )),
            }
        }
    }
    for dimension in [
        ClaimDimension::Authentication,
        ClaimDimension::Authority,
        ClaimDimension::Completeness,
        ClaimDimension::Effect,
        ClaimDimension::Independence,
        ClaimDimension::Truth,
    ] {
        if claim_state(declared, dimension) == ClaimState::EstablishedWithinProfile
            && !profile.required_claims.contains(&dimension)
        {
            out.push(violation(
                "DJCC-CLAIM-NOT-IN-PROFILE",
                format!("certificate establishes unrequested claim {}", dimension.as_str()),
            ));
        }
    }
}

fn validate_stage_disposition_claims(
    stage_id: StageId,
    stage: &StageEvidence,
    out: &mut Vec<Violation>,
) {
    let values = dimension_values(stage.claim_ceiling);
    match stage.disposition {
        StageDisposition::NotApplicable => {
            if values.iter().any(|value| *value != ClaimState::NotApplicable) {
                out.push(violation(
                    "DJCC-NOT-APPLICABLE-CLAIM-STATE",
                    format!("{stage_id}: NotApplicable stage must mark every claim dimension NotApplicable"),
                ));
            }
        }
        StageDisposition::NotEvaluated => {
            if values.iter().any(|value| {
                !matches!(value, ClaimState::NotEvaluated | ClaimState::NotApplicable)
            }) {
                out.push(violation(
                    "DJCC-NOT-EVALUATED-CLAIM-STATE",
                    format!("{stage_id}: NotEvaluated stage cannot claim an evaluated dimension"),
                ));
            }
        }
        StageDisposition::UnsupportedByProfile => {
            if values.iter().any(|value| {
                !matches!(
                    value,
                    ClaimState::UnsupportedByProfile
                        | ClaimState::NotEvaluated
                        | ClaimState::NotApplicable
                )
            }) {
                out.push(violation(
                    "DJCC-UNSUPPORTED-CLAIM-STATE",
                    format!("{stage_id}: UnsupportedByProfile stage cannot establish or reject a claim"),
                ));
            }
        }
        StageDisposition::VerifiedForDeclaredClaim
        | StageDisposition::Rejected
        | StageDisposition::Indeterminate => {}
    }
}

fn validate_scope_binding(
    stage_id: StageId,
    stage: &StageEvidence,
    scope_digest: Digest,
    scope_schema_ref: &str,
    out: &mut Vec<Violation>,
) {
    let matches: Vec<_> = stage
        .input_bindings
        .iter()
        .filter(|binding| binding.name == "scope")
        .collect();
    if matches.len() != 1 {
        out.push(violation(
            "DJCC-SCOPE-BINDING-COUNT",
            format!("{stage_id}: exactly one scope input binding is required"),
        ));
        return;
    }
    let binding = matches[0];
    if binding.digest != scope_digest || binding.schema_ref != scope_schema_ref {
        out.push(violation(
            "DJCC-SCOPE-DIGEST-MISMATCH",
            format!("{stage_id}: scope binding differs from the certificate scope/profile"),
        ));
    }
}

fn derive_disposition(input: &CompositionInput, profile: &CompositionProfile) -> CompositionDisposition {
    if profile.required_claims.contains(&ClaimDimension::Truth)
        && input
            .stages
            .get(&StageId::Policy)
            .is_some_and(|stage| stage.disposition == StageDisposition::Rejected)
    {
        return CompositionDisposition::HistoricalReplayMismatch;
    }
    if profile.required_claims.contains(&ClaimDimension::Authority)
        && input
            .stages
            .get(&StageId::Authorization)
            .is_some_and(|stage| stage.disposition == StageDisposition::Rejected)
    {
        return CompositionDisposition::DecisionNotJustifiedAtAcceptedFrontier;
    }
    if profile.required_claims.iter().any(|dimension| {
        claim_state(input.declared_claim_ceiling, *dimension) == ClaimState::Rejected
    }) {
        return CompositionDisposition::DecisionNotJustifiedAtAcceptedFrontier;
    }
    if profile.required_claims.iter().any(|dimension| {
        matches!(
            claim_state(input.declared_claim_ceiling, *dimension),
            ClaimState::UnsupportedByProfile | ClaimState::NotApplicable
        )
    }) {
        return CompositionDisposition::UnsupportedByProfile;
    }
    if profile.required_claims.iter().any(|dimension| {
        matches!(
            claim_state(input.declared_claim_ceiling, *dimension),
            ClaimState::Indeterminate | ClaimState::NotEvaluated
        )
    }) {
        return CompositionDisposition::Indeterminate;
    }
    if profile.required_claims.is_empty() {
        return CompositionDisposition::UnsupportedByProfile;
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
    if profile.profile_ref.trim().is_empty() || input.profile_ref.trim().is_empty() {
        violations.push(violation(
            "DJCC-PROFILE-REF",
            "composition profile_ref is empty",
        ));
    }
    if input.profile_ref != profile.profile_ref || input.profile_digest != profile.profile_digest {
        violations.push(violation(
            "DJCC-PROFILE-BINDING-MISMATCH",
            "certificate profile reference/digest differs from selected composition profile",
        ));
    }
    if profile.scope_schema_ref.trim().is_empty() {
        violations.push(violation(
            "DJCC-SCOPE-SCHEMA",
            "composition profile scope schema reference is empty",
        ));
    }
    validate_profile_dependency_graph(profile, &mut violations);

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
        if stage.profile_ref.trim().is_empty() || stage.output_schema_ref.trim().is_empty() {
            violations.push(violation(
                "DJCC-STAGE-PROFILE",
                format!("{stage_id}: profile_ref/output_schema_ref is empty"),
            ));
        }
        validate_scope_binding(
            *stage_id,
            stage,
            input.scope_digest,
            &profile.scope_schema_ref,
            &mut violations,
        );

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
        validate_stage_disposition_claims(*stage_id, stage, &mut violations);

        validate_bindings(*stage_id, &stage.input_bindings, &mut violations);
        validate_stage_dependencies(
            *stage_id,
            stage,
            &input.stages,
            profile,
            &mut violations,
        );

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

    validate_claim_ceiling(input, profile, &mut violations);
    let derived_disposition = derive_disposition(input, profile);
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
            input_bindings: {
                let mut bindings = vec![InputBinding {
                    name: "scope".to_owned(),
                    digest: digest(1),
                    schema_ref: "scope:v1".to_owned(),
                }];
                bindings.extend(dependencies.iter().map(|dependency| InputBinding {
                    name: dependency.input_name.clone(),
                    digest: dependency.output_digest,
                    schema_ref: dependency.schema_ref.clone(),
                }));
                bindings.sort_by(|left, right| left.name.cmp(&right.name));
                bindings
            },
            output_digest,
            claim_ceiling: claim_ceiling(),
            output_schema_ref: "stage-output:v1".to_owned(),
            dependencies,
            reason_codes: Vec::new(),
        }
    }

    fn dependency_with_claims(
        stage_id: StageId,
        output_digest: Digest,
        required_claims: &[ClaimDimension],
    ) -> Dependency {
        Dependency {
            stage_id,
            input_name: format!("stage:{}", stage_id.as_str()),
            required_claims: required_claims.to_vec(),
            output_digest,
            schema_ref: "stage-output:v1".to_owned(),
        }
    }

    fn dependency(stage_id: StageId, output_digest: Digest) -> Dependency {
        dependency_with_claims(stage_id, output_digest, &[ClaimDimension::Authentication])
    }

    fn valid_profile() -> CompositionProfile {
        CompositionProfile {
            profile_ref: "profile:test:v1".to_owned(),
            profile_digest: digest(99),
            scope_schema_ref: "scope:v1".to_owned(),
            allowed_not_applicable: BTreeSet::new(),
            required_claims: BTreeSet::from([
                ClaimDimension::Authentication,
                ClaimDimension::Authority,
                ClaimDimension::Completeness,
                ClaimDimension::Effect,
                ClaimDimension::Truth,
            ]),
            claim_stages: BTreeMap::from([
                (
                    ClaimDimension::Authentication,
                    BTreeSet::from([StageId::VerifierFrontier, StageId::SourceTrust]),
                ),
                (
                    ClaimDimension::Authority,
                    BTreeSet::from([StageId::Authorization]),
                ),
                (
                    ClaimDimension::Completeness,
                    BTreeSet::from([
                        StageId::VerifierFrontier,
                        StageId::SourceTrust,
                        StageId::SemanticProjection,
                        StageId::Query,
                    ]),
                ),
                (
                    ClaimDimension::Effect,
                    BTreeSet::from([StageId::Execution, StageId::Outcome]),
                ),
                (
                    ClaimDimension::Truth,
                    BTreeSet::from([
                        StageId::Query,
                        StageId::Policy,
                        StageId::Outcome,
                    ]),
                ),
            ]),
            dependency_claims: BTreeMap::from([
                (
                    StageId::SourceTrust,
                    BTreeMap::from([(
                        StageId::VerifierFrontier,
                        BTreeSet::from([
                            ClaimDimension::Authentication,
                            ClaimDimension::Completeness,
                        ]),
                    )]),
                ),
                (
                    StageId::SemanticProjection,
                    BTreeMap::from([
                        (
                            StageId::VerifierFrontier,
                            BTreeSet::from([ClaimDimension::Authentication]),
                        ),
                        (
                            StageId::SourceTrust,
                            BTreeSet::from([ClaimDimension::Authentication]),
                        ),
                    ]),
                ),
                (
                    StageId::Query,
                    BTreeMap::from([
                        (
                            StageId::VerifierFrontier,
                            BTreeSet::from([
                                ClaimDimension::Authentication,
                                ClaimDimension::Completeness,
                            ]),
                        ),
                        (
                            StageId::SourceTrust,
                            BTreeSet::from([
                                ClaimDimension::Authentication,
                                ClaimDimension::Completeness,
                            ]),
                        ),
                        (
                            StageId::SemanticProjection,
                            BTreeSet::from([
                                ClaimDimension::Authentication,
                                ClaimDimension::Completeness,
                            ]),
                        ),
                    ]),
                ),
                (
                    StageId::Policy,
                    BTreeMap::from([(
                        StageId::Query,
                        BTreeSet::from([
                            ClaimDimension::Authentication,
                            ClaimDimension::Completeness,
                            ClaimDimension::Truth,
                        ]),
                    )]),
                ),
                (
                    StageId::Authorization,
                    BTreeMap::from([
                        (
                            StageId::VerifierFrontier,
                            BTreeSet::from([ClaimDimension::Authentication]),
                        ),
                        (
                            StageId::Policy,
                            BTreeSet::from([
                                ClaimDimension::Authentication,
                                ClaimDimension::Truth,
                            ]),
                        ),
                    ]),
                ),
                (
                    StageId::Execution,
                    BTreeMap::from([(
                        StageId::Authorization,
                        BTreeSet::from([
                            ClaimDimension::Authentication,
                            ClaimDimension::Authority,
                        ]),
                    )]),
                ),
                (
                    StageId::Outcome,
                    BTreeMap::from([
                        (
                            StageId::SourceTrust,
                            BTreeSet::from([ClaimDimension::Authentication]),
                        ),
                        (
                            StageId::SemanticProjection,
                            BTreeSet::from([ClaimDimension::Completeness]),
                        ),
                        (
                            StageId::Execution,
                            BTreeSet::from([ClaimDimension::Effect]),
                        ),
                    ]),
                ),
            ]),
        }
    }

    fn independent_corroboration_profile() -> CompositionProfile {
        let mut profile = valid_profile();
        profile.profile_ref = "profile:independent-corroboration:v1".to_owned();
        profile.profile_digest = digest(101);
        profile.allowed_not_applicable =
            BTreeSet::from([StageId::Execution, StageId::Outcome]);

        profile.required_claims.remove(&ClaimDimension::Effect);
        profile.required_claims.insert(ClaimDimension::Independence);
        profile.claim_stages.remove(&ClaimDimension::Effect);
        profile.claim_stages.insert(
            ClaimDimension::Independence,
            BTreeSet::from([
                StageId::SourceTrust,
                StageId::SemanticProjection,
                StageId::Query,
            ]),
        );
        profile
            .claim_stages
            .get_mut(&ClaimDimension::Truth)
            .unwrap()
            .remove(&StageId::Outcome);

        for stage_id in [StageId::Execution, StageId::Outcome] {
            profile.dependency_claims.remove(&stage_id);
        }
        {
            let query_edges = profile.dependency_claims.get_mut(&StageId::Query).unwrap();
            for producer in [StageId::SourceTrust, StageId::SemanticProjection] {
                query_edges
                    .get_mut(&producer)
                    .unwrap()
                    .insert(ClaimDimension::Independence);
            }
        }
        profile
            .dependency_claims
            .get_mut(&StageId::Policy)
            .unwrap()
            .get_mut(&StageId::Query)
            .unwrap()
            .insert(ClaimDimension::Independence);
        profile
    }

    fn positive_independence_input() -> (CompositionInput, CompositionProfile) {
        let mut input = positive_input();
        let profile = independent_corroboration_profile();

        for stage_id in [
            StageId::SourceTrust,
            StageId::SemanticProjection,
            StageId::Query,
        ] {
            input
                .stages
                .get_mut(&stage_id)
                .unwrap()
                .claim_ceiling
                .independence = ClaimState::EstablishedWithinProfile;
        }

        for dependency in input
            .stages
            .get_mut(&StageId::Query)
            .unwrap()
            .dependencies
            .iter_mut()
        {
            if matches!(
                dependency.stage_id,
                StageId::SourceTrust | StageId::SemanticProjection
            ) {
                dependency.required_claims.push(ClaimDimension::Independence);
                dependency.required_claims.sort();
            }
        }
        let policy_query_dependency = input
            .stages
            .get_mut(&StageId::Policy)
            .unwrap()
            .dependencies
            .iter_mut()
            .find(|dependency| dependency.stage_id == StageId::Query)
            .unwrap();
        policy_query_dependency
            .required_claims
            .push(ClaimDimension::Independence);
        policy_query_dependency.required_claims.sort();

        mark_not_applicable(&mut input, StageId::Execution);
        mark_not_applicable(&mut input, StageId::Outcome);
        input.declared_claim_ceiling.effect = ClaimState::NotApplicable;
        input.declared_claim_ceiling.independence = ClaimState::EstablishedWithinProfile;
        input.declared_disposition =
            CompositionDisposition::DecisionJustifiedAtAcceptedFrontier;
        (input, profile)
    }

    fn mark_not_evaluated(input: &mut CompositionInput, stage_id: StageId) {
        let stage = input.stages.get_mut(&stage_id).expect("required fixture stage");
        stage.disposition = StageDisposition::NotEvaluated;
        stage.output_digest = None;
        stage.dependencies.clear();
        stage.input_bindings.retain(|binding| binding.name == "scope");
        stage.claim_ceiling = ClaimCeiling {
            authentication: ClaimState::NotEvaluated,
            completeness: ClaimState::NotEvaluated,
            truth: ClaimState::NotEvaluated,
            authority: ClaimState::NotEvaluated,
            effect: ClaimState::NotEvaluated,
            independence: ClaimState::NotEvaluated,
        };
    }

    fn mark_not_applicable(input: &mut CompositionInput, stage_id: StageId) {
        let stage = input.stages.get_mut(&stage_id).expect("required fixture stage");
        stage.disposition = StageDisposition::NotApplicable;
        stage.output_digest = None;
        stage.dependencies.clear();
        stage.input_bindings.retain(|binding| binding.name == "scope");
        stage.claim_ceiling = ClaimCeiling {
            authentication: ClaimState::NotApplicable,
            completeness: ClaimState::NotApplicable,
            truth: ClaimState::NotApplicable,
            authority: ClaimState::NotApplicable,
            effect: ClaimState::NotApplicable,
            independence: ClaimState::NotApplicable,
        };
    }

    fn remove_profile_claim(profile: &mut CompositionProfile, dimension: ClaimDimension) {
        profile.required_claims.remove(&dimension);
        profile.claim_stages.remove(&dimension);
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
                vec![dependency_with_claims(
                    StageId::VerifierFrontier,
                    d[0],
                    &[ClaimDimension::Authentication, ClaimDimension::Completeness],
                )],
            ),
        );
        stages.insert(
            StageId::SemanticProjection,
            stage(
                StageDisposition::VerifiedForDeclaredClaim,
                Some(d[2]),
                vec![
                    dependency_with_claims(
                        StageId::VerifierFrontier,
                        d[0],
                        &[ClaimDimension::Authentication],
                    ),
                    dependency_with_claims(
                        StageId::SourceTrust,
                        d[1],
                        &[ClaimDimension::Authentication],
                    ),
                ],
            ),
        );
        stages.insert(
            StageId::Query,
            stage(
                StageDisposition::VerifiedForDeclaredClaim,
                Some(d[3]),
                vec![
                    dependency_with_claims(
                        StageId::VerifierFrontier,
                        d[0],
                        &[ClaimDimension::Authentication, ClaimDimension::Completeness],
                    ),
                    dependency_with_claims(
                        StageId::SourceTrust,
                        d[1],
                        &[ClaimDimension::Authentication, ClaimDimension::Completeness],
                    ),
                    dependency_with_claims(
                        StageId::SemanticProjection,
                        d[2],
                        &[ClaimDimension::Authentication, ClaimDimension::Completeness],
                    ),
                ],
            ),
        );
        stages.insert(
            StageId::Policy,
            stage(
                StageDisposition::VerifiedForDeclaredClaim,
                Some(d[4]),
                vec![dependency_with_claims(
                    StageId::Query,
                    d[3],
                    &[
                        ClaimDimension::Authentication,
                        ClaimDimension::Completeness,
                        ClaimDimension::Truth,
                    ],
                )],
            ),
        );
        stages.insert(
            StageId::Authorization,
            stage(
                StageDisposition::VerifiedForDeclaredClaim,
                Some(d[5]),
                vec![
                    dependency_with_claims(
                        StageId::VerifierFrontier,
                        d[0],
                        &[ClaimDimension::Authentication],
                    ),
                    dependency_with_claims(
                        StageId::Policy,
                        d[4],
                        &[ClaimDimension::Authentication, ClaimDimension::Truth],
                    ),
                ],
            ),
        );
        stages.insert(
            StageId::Execution,
            stage(
                StageDisposition::VerifiedForDeclaredClaim,
                Some(d[6]),
                vec![dependency_with_claims(
                    StageId::Authorization,
                    d[5],
                    &[ClaimDimension::Authentication, ClaimDimension::Authority],
                )],
            ),
        );
        stages.insert(
            StageId::Outcome,
            stage(
                StageDisposition::VerifiedForDeclaredClaim,
                Some(d[7]),
                vec![
                    dependency_with_claims(
                        StageId::SourceTrust,
                        d[1],
                        &[ClaimDimension::Authentication],
                    ),
                    dependency_with_claims(
                        StageId::SemanticProjection,
                        d[2],
                        &[ClaimDimension::Completeness],
                    ),
                    dependency_with_claims(
                        StageId::Execution,
                        d[6],
                        &[ClaimDimension::Effect],
                    ),
                ],
            ),
        );
        CompositionInput {
            certificate_id: "djcc:test:positive".to_owned(),
            profile_ref: "profile:test:v1".to_owned(),
            profile_digest: digest(99),
            scope_digest: digest(1),
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
        for stage_id in [
            StageId::Query,
            StageId::Policy,
            StageId::Authorization,
            StageId::Execution,
            StageId::Outcome,
        ] {
            mark_not_evaluated(&mut input, stage_id);
        }
        input.declared_claim_ceiling.truth = ClaimState::NotEvaluated;
        input.declared_claim_ceiling.authority = ClaimState::NotEvaluated;
        input.declared_claim_ceiling.effect = ClaimState::NotEvaluated;
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
        for stage_id in [
            StageId::SourceTrust,
            StageId::SemanticProjection,
            StageId::Query,
            StageId::Policy,
            StageId::Authorization,
            StageId::Execution,
            StageId::Outcome,
        ] {
            mark_not_evaluated(&mut input, stage_id);
        }
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
        mark_not_applicable(&mut input, StageId::Execution);
        mark_not_applicable(&mut input, StageId::Outcome);
        input.declared_claim_ceiling.effect = ClaimState::NotApplicable;
        let mut profile = valid_profile();
        remove_profile_claim(&mut profile, ClaimDimension::Effect);
        profile
            .claim_stages
            .get_mut(&ClaimDimension::Truth)
            .unwrap()
            .remove(&StageId::Outcome);
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
        mark_not_applicable(&mut input, StageId::Execution);
        mark_not_applicable(&mut input, StageId::Outcome);
        input.declared_claim_ceiling.effect = ClaimState::NotApplicable;
        let mut profile = valid_profile();
        profile.allowed_not_applicable = BTreeSet::from([
            StageId::Execution,
            StageId::Outcome,
        ]);
        remove_profile_claim(&mut profile, ClaimDimension::Effect);
        profile
            .claim_stages
            .get_mut(&ClaimDimension::Truth)
            .unwrap()
            .remove(&StageId::Outcome);
        let report = verify_composition(&input, &profile);
        assert!(report.is_valid(), "{:?}", report.violations);
    }

    #[test]
    fn policy_epoch_rejection_derives_historical_replay_mismatch() {
        let mut input = positive_input();
        let policy = input.stages.get_mut(&StageId::Policy).unwrap();
        policy.disposition = StageDisposition::Rejected;
        policy.claim_ceiling.truth = ClaimState::Rejected;
        for stage_id in [
            StageId::Authorization,
            StageId::Execution,
            StageId::Outcome,
        ] {
            mark_not_evaluated(&mut input, stage_id);
        }
        input.declared_claim_ceiling.truth = ClaimState::Rejected;
        input.declared_claim_ceiling.authority = ClaimState::NotEvaluated;
        input.declared_claim_ceiling.effect = ClaimState::NotEvaluated;
        input.declared_disposition = CompositionDisposition::HistoricalReplayMismatch;
        let report = verify_composition(&input, &valid_profile());
        assert!(report.is_valid(), "{:?}", report.violations);
    }

    #[test]
    fn action_authorization_rejection_derives_not_justified() {
        let mut input = positive_input();
        let authorization = input.stages.get_mut(&StageId::Authorization).unwrap();
        authorization.disposition = StageDisposition::Rejected;
        authorization.claim_ceiling.authority = ClaimState::Rejected;
        input.declared_claim_ceiling.authority = ClaimState::Rejected;
        mark_not_evaluated(&mut input, StageId::Execution);
        mark_not_evaluated(&mut input, StageId::Outcome);
        input.declared_claim_ceiling.truth = ClaimState::NotEvaluated;
        input.declared_claim_ceiling.effect = ClaimState::NotEvaluated;
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
    #[test]
    fn not_applicable_stage_cannot_support_an_established_effect_claim() {
        let mut input = positive_input();
        let execution = input.stages.get_mut(&StageId::Execution).unwrap();
        execution.disposition = StageDisposition::NotApplicable;
        execution.output_digest = None;
        execution.dependencies.clear();
        execution.input_bindings.retain(|binding| binding.name == "scope");
        execution.claim_ceiling.effect = ClaimState::NotApplicable;
        let report = verify_composition(&input, &valid_profile());
        assert!(report
            .violations
            .iter()
            .any(|item| item.code == "DJCC-CLAIM-CEILING-WIDENED"));
    }

    #[test]
    fn unevaluated_stage_cannot_retain_orphaned_stage_input() {
        let mut input = positive_input();
        let policy = input.stages.get_mut(&StageId::Policy).unwrap();
        policy.disposition = StageDisposition::NotEvaluated;
        policy.output_digest = None;
        policy.dependencies.clear();
        // Deliberately retain the prior Query dependency binding.
        let report = verify_composition(&input, &valid_profile());
        assert!(report
            .violations
            .iter()
            .any(|item| item.code == "DJCC-UNBOUND-STAGE-INPUT"));
    }
 
    fn decode_hex(value: &str) -> Vec<u8> {
        assert_eq!(value.len() % 2, 0, "hex input has odd length");
        value
            .as_bytes()
            .chunks_exact(2)
            .map(|pair| {
                let high = char::from(pair[0]).to_digit(16).expect("hex digit");
                let low = char::from(pair[1]).to_digit(16).expect("hex digit");
                ((high << 4) | low) as u8
            })
            .collect()
    }

    fn parse_claim_dimension(value: &str) -> ClaimDimension {
        match value {
            "authentication" => ClaimDimension::Authentication,
            "authority" => ClaimDimension::Authority,
            "completeness" => ClaimDimension::Completeness,
            "effect" => ClaimDimension::Effect,
            "truth" => ClaimDimension::Truth,
            other => panic!("unknown claim dimension {other}"),
        }
    }

    #[test]
    fn shared_go_rust_stage_edge_canonicalization_vectors_match() {
        const VECTORS: &str = include_str!(
            "../../../../tools/economic/evidence-execution-composition/DJCC_STAGE_EDGE_V1_KAT.tsv"
        );
        let mut seen = 0;
        for (line_index, line) in VECTORS.lines().enumerate() {
            let line = line.trim_end_matches('\r');
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let fields: Vec<_> = line.split('\t').collect();
            assert_eq!(fields.len(), 8, "vector line {}", line_index + 1);
            let digest_bytes = decode_hex(fields[6]);
            let digest: [u8; 32] = digest_bytes
                .try_into()
                .expect("stage-edge output digest must be 32 bytes");
            let required_claims = fields[5]
                .split(',')
                .map(parse_claim_dimension)
                .collect();
            let edge = StageEdgeV1 {
                producer_stage: fields[1].to_owned(),
                consumer_stage: fields[2].to_owned(),
                input_name: fields[3].to_owned(),
                output_schema_ref: fields[4].to_owned(),
                required_claims,
                output_digest: Digest(digest),
            };
            let actual = canonical_stage_edge_bytes_v1(&edge)
                .expect("shared stage-edge fixture is valid");
            let expected = decode_hex(fields[7]);
            assert_eq!(
                actual,
                expected,
                "canonical stage-edge bytes differ for {}",
                fields[0]
            );
            seen += 1;
        }
        assert_eq!(seen, 3, "all shared stage-edge vectors must execute");
    }

    #[test]
    fn canonical_stage_edge_rejects_unknown_stage_empty_input_and_unsorted_claims() {
        let mut edge = StageEdgeV1 {
            producer_stage: "unknown".to_owned(),
            consumer_stage: "query".to_owned(),
            input_name: "frontier".to_owned(),
            output_schema_ref: "schema:v1".to_owned(),
            required_claims: vec![ClaimDimension::Authentication],
            output_digest: Digest([1; 32]),
        };
        assert!(canonical_stage_edge_bytes_v1(&edge).is_err());
        edge.producer_stage = "verifier_frontier".to_owned();
        edge.input_name = "  ".to_owned();
        assert!(canonical_stage_edge_bytes_v1(&edge).is_err());
        edge.input_name = "frontier".to_owned();
        edge.required_claims = vec![ClaimDimension::Completeness, ClaimDimension::Authentication];
        assert!(canonical_stage_edge_bytes_v1(&edge).is_err());
    }

    #[test]
    fn verified_child_can_use_an_established_claim_from_partially_indeterminate_parent() {
        let mut input = positive_input();
        let source = input.stages.get_mut(&StageId::SourceTrust).unwrap();
        source.disposition = StageDisposition::Indeterminate;
        source.claim_ceiling.completeness = ClaimState::Indeterminate;
        input.declared_claim_ceiling.completeness = ClaimState::Indeterminate;
        input.declared_disposition = CompositionDisposition::Indeterminate;
        // The semantic projection edge requires only source authentication,
        // which remains established. Incomplete universe coverage must not
        // erase the narrower, independently verified parsing claim.
        let report = verify_composition(&input, &valid_profile());
        assert!(!report.violations.iter().any(|item| {
            item.code == "DJCC-DEPENDENCY-CLAIM-UNMET"
        }));
        assert_eq!(report.derived_disposition, CompositionDisposition::Indeterminate);
    }

    #[test]
    fn verified_child_cannot_use_an_unmet_required_claim_from_parent() {
        let mut input = positive_input();
        let semantic = input.stages.get_mut(&StageId::SemanticProjection).unwrap();
        semantic.disposition = StageDisposition::Indeterminate;
        semantic.claim_ceiling.completeness = ClaimState::Indeterminate;
        let query = input.stages.get_mut(&StageId::Query).unwrap();
        query.dependencies[2].required_claims = vec![ClaimDimension::Completeness];
        let report = verify_composition(&input, &valid_profile());
        assert!(report
            .violations
            .iter()
            .any(|item| item.code == "DJCC-DEPENDENCY-CLAIM-UNMET"));
    }

    #[test]
    fn dependency_claim_requirements_must_be_sorted_unique_and_nonempty() {
        let mut input = positive_input();
        let query = input.stages.get_mut(&StageId::Query).unwrap();
        query.dependencies[0].required_claims = vec![
            ClaimDimension::Truth,
            ClaimDimension::Authentication,
        ];
        let report = verify_composition(&input, &valid_profile());
        assert!(report
            .violations
            .iter()
            .any(|item| item.code == "DJCC-DEPENDENCY-CLAIMS"));
    }


    #[test]
    fn not_applicable_stage_cannot_claim_an_established_dimension() {
        let mut input = positive_input();
        let stage = input.stages.get_mut(&StageId::Execution).unwrap();
        stage.disposition = StageDisposition::NotApplicable;
        stage.output_digest = None;
        stage.dependencies.clear();
        stage.input_bindings.retain(|binding| binding.name == "scope");
        // Leave the effect ceiling established to test the attempted laundering.
        let mut profile = valid_profile();
        remove_profile_claim(&mut profile, ClaimDimension::Effect);
        let report = verify_composition(&input, &profile);
        assert!(report
            .violations
            .iter()
            .any(|item| item.code == "DJCC-NOT-APPLICABLE-CLAIM-STATE"));
    }


    #[test]
    fn profile_cannot_weaken_or_expand_the_fixed_dependency_graph() {
        let mut profile = valid_profile();
        profile
            .dependency_claims
            .get_mut(&StageId::Query)
            .unwrap()
            .remove(&StageId::SourceTrust);
        let report = verify_composition(&positive_input(), &profile);
        assert!(report
            .violations
            .iter()
            .any(|item| item.code == "DJCC-PROFILE-EDGE-SET"));
    }

    #[test]
    fn certificate_cannot_weaken_profile_required_edge_claims() {
        let mut input = positive_input();
        let query = input.stages.get_mut(&StageId::Query).unwrap();
        let source = query
            .dependencies
            .iter_mut()
            .find(|dependency| dependency.stage_id == StageId::SourceTrust)
            .unwrap();
        source.required_claims = vec![ClaimDimension::Authentication];
        let report = verify_composition(&input, &valid_profile());
        assert!(report
            .violations
            .iter()
            .any(|item| item.code == "DJCC-EDGE-CLAIMS-WEAKENED"));
    }


    #[test]
    fn indeterminate_root_claim_does_not_require_all_producers_to_be_green() {
        let mut input = positive_input();
        let source = input.stages.get_mut(&StageId::SourceTrust).unwrap();
        source.claim_ceiling.completeness = ClaimState::Indeterminate;
        source.disposition = StageDisposition::Indeterminate;
        input.declared_claim_ceiling.completeness = ClaimState::Indeterminate;
        input.declared_disposition = CompositionDisposition::Indeterminate;
        let report = verify_composition(&input, &valid_profile());
        assert!(!report.violations.iter().any(|item| {
            item.code == "DJCC-CLAIM-CEILING-WIDENED"
                && item.message.contains("completeness")
        }));
        assert!(report.is_valid(), "{:?}", report.violations);
    }

    #[test]
    fn established_root_claim_requires_every_profile_owner_stage() {
        let mut input = positive_input();
        input.stages.get_mut(&StageId::SourceTrust).unwrap().claim_ceiling.completeness =
            ClaimState::Indeterminate;
        input.stages.get_mut(&StageId::SourceTrust).unwrap().disposition =
            StageDisposition::Indeterminate;
        // Leave the certificate claiming that Completeness is established.
        let report = verify_composition(&input, &valid_profile());
        assert!(report.violations.iter().any(|item| {
            item.code == "DJCC-CLAIM-CEILING-WIDENED"
                && item.message.contains("completeness")
        }));
    }


    #[test]
    fn independent_corroboration_profile_requires_an_established_independence_claim() {
        let (input, profile) = positive_independence_input();
        let report = verify_composition(&input, &profile);
        assert!(report.is_valid(), "{:?}", report.violations);
        assert_eq!(
            report.derived_disposition,
            CompositionDisposition::DecisionJustifiedAtAcceptedFrontier
        );
    }

    #[test]
    fn common_cause_provenance_keeps_independence_indeterminate() {
        let (mut input, profile) = positive_independence_input();
        let source = input.stages.get_mut(&StageId::SourceTrust).unwrap();
        source.disposition = StageDisposition::Indeterminate;
        source.claim_ceiling.independence = ClaimState::Indeterminate;
        let semantic = input
            .stages
            .get_mut(&StageId::SemanticProjection)
            .unwrap();
        semantic.disposition = StageDisposition::Indeterminate;
        semantic.claim_ceiling.independence = ClaimState::Indeterminate;
        let query = input.stages.get_mut(&StageId::Query).unwrap();
        query.disposition = StageDisposition::Indeterminate;
        query.claim_ceiling.independence = ClaimState::Indeterminate;
        for stage_id in [StageId::Policy, StageId::Authorization] {
            mark_not_evaluated(&mut input, stage_id);
        }
        input.declared_claim_ceiling.independence = ClaimState::Indeterminate;
        input.declared_claim_ceiling.truth = ClaimState::NotEvaluated;
        input.declared_claim_ceiling.authority = ClaimState::NotEvaluated;
        input.declared_disposition = CompositionDisposition::Indeterminate;
        let report = verify_composition(&input, &profile);
        assert!(report.is_valid(), "{:?}", report.violations);
        assert_eq!(report.derived_disposition, CompositionDisposition::Indeterminate);
    }

    #[test]
    fn independence_claim_cannot_be_omitted_from_a_required_edge() {
        let (mut input, profile) = positive_independence_input();
        let query = input.stages.get_mut(&StageId::Query).unwrap();
        let source_dependency = query
            .dependencies
            .iter_mut()
            .find(|dependency| dependency.stage_id == StageId::SourceTrust)
            .unwrap();
        source_dependency
            .required_claims
            .retain(|dimension| *dimension != ClaimDimension::Independence);
        let report = verify_composition(&input, &profile);
        assert!(report
            .violations
            .iter()
            .any(|item| item.code == "DJCC-EDGE-CLAIMS-WEAKENED"));
    }


}
