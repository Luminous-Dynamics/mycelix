// Copyright (C) 2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//
// This crate is a finite-set reference algebra, not a truth oracle or authority service.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::{collections::BTreeSet, process::Command};

pub const REQUEST_SCHEMA_VERSION: &str = "mycelix.claim-ceiling-meet.request.v1";
pub const RECEIPT_SCHEMA_VERSION: &str = "mycelix.claim-ceiling-meet.receipt.v1";
pub const UNPINNED_LOCAL_RUN: &str = "UNPINNED_LOCAL_RUN";

const REQUEST_SCHEMA: &[u8] = include_bytes!("../schemas/claim-ceiling-request-v1.schema.json");
const RECEIPT_SCHEMA: &[u8] = include_bytes!("../schemas/claim-ceiling-receipt-v1.schema.json");
#[cfg(test)]
const GOLDEN_VECTORS: &[u8] = include_bytes!("../fixtures/claim-ceiling-meet-golden-v1.json");
const NONCLAIMS: &[&str] = &[
    "A well-formed ceiling or meet receipt does not establish that any claim is true.",
    "A digest binds exact bytes but does not authenticate the source, signer, timestamp, or policy.",
    "This reference does not admit evidence, establish source independence, validate qualifications, or grant action authority.",
    "An empty action-scope set means no action authority is represented; a non-empty set is still only a ceiling, not an authorization grant.",
    "Temporal scope is a declared interval, not proof of trusted time or present-tense currentness.",
    "Policy authors remain responsible for defining allowed claim atoms; the algebra cannot repair an incorrectly permissive policy.",
];

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ClaimPredicate {
    Observation,
    Measurement,
    ModelEstimate,
    SimulationResult,
    Forecast,
    FormalProperty,
    CausalClaim,
    OwnershipClaim,
    AuthorizationClaim,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EvidenceClass {
    DirectObservation,
    InstrumentMeasurement,
    ModelOutput,
    SimulationOutput,
    ForecastOutput,
    FormalProof,
    IndependentReplication,
    SourceAssertion,
    CorrelationalAnalysis,
    CausalIdentification,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum DerivationKind {
    Direct,
    DeterministicTransform,
    StatisticalInference,
    Simulation,
    FormalVerification,
    CrossSourceSynthesis,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IntendedUse {
    Research,
    Display,
    Planning,
    DecisionSupport,
    OperationalExecution,
}

/// One allowed combination. Keeping predicate/evidence/derivation/use coupled
/// prevents an independent per-field intersection from inventing combinations.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ClaimAtom {
    pub predicate: ClaimPredicate,
    pub evidence_class: EvidenceClass,
    pub derivation: DerivationKind,
    pub intended_use: IntendedUse,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IndependenceControl {
    DistinctRecord,
    DistinctAcquisition,
    IndependentOrigin,
    IndependentMeasurement,
    IndependentVerifier,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ProvenanceRequirement {
    SourceAttribution,
    ExactArtifactDigest,
    TrustedTimeEvidence,
    MeasurementCalibration,
    ToolchainIdentity,
    DependencyClosure,
    ExternalWitness,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TimeWindow {
    /// Inclusive lower bound in Unix seconds. This is a declared boundary only.
    pub start_unix_seconds: i64,
    /// Exclusive upper bound in Unix seconds.
    pub end_unix_seconds_exclusive: i64,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ClaimCeilingV1 {
    pub claim_atoms: BTreeSet<ClaimAtom>,
    pub subject_ids: BTreeSet<String>,
    pub context_ids: BTreeSet<String>,
    pub valid_time: TimeWindow,
    /// Explicit action-scope ceilings. Empty means no action scopes are permitted.
    pub action_scopes: BTreeSet<String>,
    /// Composition adds mandatory requirements; it never drops one silently.
    pub required_qualifications: BTreeSet<String>,
    pub required_independence_controls: BTreeSet<IndependenceControl>,
    pub required_provenance: BTreeSet<ProvenanceRequirement>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MeetRequestV1 {
    pub schema_version: String,
    pub request_id: String,
    pub left: ClaimCeilingV1,
    pub right: ClaimCeilingV1,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum EmptyDimension {
    ClaimAtoms,
    SubjectIds,
    ContextIds,
    TimeWindow,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum MeetDisposition {
    UsableIntersection,
    BottomNoCommonPermittedClaim,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct NonAmplificationChecks {
    pub no_broader_than_left: bool,
    pub no_broader_than_right: bool,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MeetReceiptV1 {
    pub schema_version: String,
    pub request_id: String,
    pub source_commit: String,
    pub source_commit_pinned: bool,
    pub input_sha256: String,
    pub request_schema_sha256: String,
    pub receipt_schema_sha256: String,
    pub disposition: MeetDisposition,
    pub ceiling: Option<ClaimCeilingV1>,
    pub empty_dimensions: Vec<EmptyDimension>,
    pub non_amplification: NonAmplificationChecks,
    pub claim_ceiling_only: bool,
    pub nonclaims: Vec<String>,
}

/// A ceiling is no broader when its allowed scopes are subsets and it preserves
/// at least all of the other's mandatory qualification, independence and
/// provenance requirements.
pub fn is_no_broader_than(self_ceiling: &ClaimCeilingV1, other: &ClaimCeilingV1) -> bool {
    self_ceiling.claim_atoms.is_subset(&other.claim_atoms)
        && self_ceiling.subject_ids.is_subset(&other.subject_ids)
        && self_ceiling.context_ids.is_subset(&other.context_ids)
        && self_ceiling.valid_time.start_unix_seconds >= other.valid_time.start_unix_seconds
        && self_ceiling.valid_time.end_unix_seconds_exclusive
            <= other.valid_time.end_unix_seconds_exclusive
        && self_ceiling.action_scopes.is_subset(&other.action_scopes)
        && self_ceiling
            .required_qualifications
            .is_superset(&other.required_qualifications)
        && self_ceiling
            .required_independence_controls
            .is_superset(&other.required_independence_controls)
        && self_ceiling
            .required_provenance
            .is_superset(&other.required_provenance)
}

/// Conservative meet over finite scopes.
///
/// Claim atoms and scope sets intersect; temporal windows intersect;
/// requirements combine by union because both operands' requirements remain
/// mandatory. Empty action authority is allowed and does not erase an otherwise
/// admissible analytical claim. Empty claim/subject/context/time scope produces
/// an explicit bottom result with no usable ceiling.
pub fn meet(
    left: &ClaimCeilingV1,
    right: &ClaimCeilingV1,
) -> (MeetDisposition, Option<ClaimCeilingV1>, Vec<EmptyDimension>) {
    let valid_time = TimeWindow {
        start_unix_seconds: left
            .valid_time
            .start_unix_seconds
            .max(right.valid_time.start_unix_seconds),
        end_unix_seconds_exclusive: left
            .valid_time
            .end_unix_seconds_exclusive
            .min(right.valid_time.end_unix_seconds_exclusive),
    };
    let ceiling = ClaimCeilingV1 {
        claim_atoms: left
            .claim_atoms
            .intersection(&right.claim_atoms)
            .cloned()
            .collect(),
        subject_ids: left
            .subject_ids
            .intersection(&right.subject_ids)
            .cloned()
            .collect(),
        context_ids: left
            .context_ids
            .intersection(&right.context_ids)
            .cloned()
            .collect(),
        valid_time,
        action_scopes: left
            .action_scopes
            .intersection(&right.action_scopes)
            .cloned()
            .collect(),
        required_qualifications: left
            .required_qualifications
            .union(&right.required_qualifications)
            .cloned()
            .collect(),
        required_independence_controls: left
            .required_independence_controls
            .union(&right.required_independence_controls)
            .cloned()
            .collect(),
        required_provenance: left
            .required_provenance
            .union(&right.required_provenance)
            .cloned()
            .collect(),
    };

    let mut empty_dimensions = Vec::new();
    if ceiling.claim_atoms.is_empty() {
        empty_dimensions.push(EmptyDimension::ClaimAtoms);
    }
    if ceiling.subject_ids.is_empty() {
        empty_dimensions.push(EmptyDimension::SubjectIds);
    }
    if ceiling.context_ids.is_empty() {
        empty_dimensions.push(EmptyDimension::ContextIds);
    }
    if ceiling.valid_time.start_unix_seconds >= ceiling.valid_time.end_unix_seconds_exclusive {
        empty_dimensions.push(EmptyDimension::TimeWindow);
    }
    empty_dimensions.sort();

    if empty_dimensions.is_empty() {
        (
            MeetDisposition::UsableIntersection,
            Some(ceiling),
            empty_dimensions,
        )
    } else {
        (
            MeetDisposition::BottomNoCommonPermittedClaim,
            None,
            empty_dimensions,
        )
    }
}

fn validate_ceiling(name: &str, ceiling: &ClaimCeilingV1) -> Result<(), String> {
    if ceiling.claim_atoms.is_empty() {
        return Err(format!("{name}.claim_atoms must not be empty"));
    }
    if ceiling.subject_ids.is_empty() || ceiling.subject_ids.iter().any(|id| id.trim().is_empty()) {
        return Err(format!(
            "{name}.subject_ids must contain non-empty explicit identities"
        ));
    }
    if ceiling.context_ids.is_empty() || ceiling.context_ids.iter().any(|id| id.trim().is_empty()) {
        return Err(format!(
            "{name}.context_ids must contain non-empty explicit contexts"
        ));
    }
    if ceiling.action_scopes.iter().any(|id| id.trim().is_empty())
        || ceiling
            .required_qualifications
            .iter()
            .any(|id| id.trim().is_empty())
    {
        return Err(format!(
            "{name} contains an empty action scope or qualification"
        ));
    }
    if ceiling.valid_time.start_unix_seconds >= ceiling.valid_time.end_unix_seconds_exclusive {
        return Err(format!(
            "{name}.valid_time must be a non-empty half-open interval"
        ));
    }
    Ok(())
}

fn sha256(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

fn validate_json_against_schema(
    schema_bytes: &[u8],
    json_bytes: &[u8],
    name: &str,
) -> Result<(), String> {
    let schema: serde_json::Value = serde_json::from_slice(schema_bytes)
        .map_err(|error| format!("{name} schema is invalid JSON: {error}"))?;
    let validator = jsonschema::JSONSchema::options()
        .with_draft(jsonschema::Draft::Draft202012)
        .compile(&schema)
        .map_err(|error| format!("{name} schema compilation failed: {error}"))?;
    let instance: serde_json::Value = serde_json::from_slice(json_bytes)
        .map_err(|error| format!("{name} instance is invalid JSON: {error}"))?;
    match validator.validate(&instance) {
        Ok(()) => Ok(()),
        Err(errors) => Err(errors
            .map(|error| format!("{} at {}", error, error.instance_path))
            .collect::<Vec<_>>()
            .join("\n")),
    }
}

pub fn evaluate_request_bytes(input: &[u8], source_commit: &str) -> Result<MeetReceiptV1, String> {
    let source_commit_pinned = validate_source_commit(source_commit)?;
    validate_json_against_schema(REQUEST_SCHEMA, input, "request")?;
    let request: MeetRequestV1 = serde_json::from_slice(input)
        .map_err(|error| format!("request deserialization failed: {error}"))?;
    if request.schema_version != REQUEST_SCHEMA_VERSION {
        return Err(format!(
            "unsupported request schema version: {}",
            request.schema_version
        ));
    }
    if request.request_id.trim().is_empty() {
        return Err("request_id must not be empty".to_owned());
    }
    validate_ceiling("left", &request.left)?;
    validate_ceiling("right", &request.right)?;

    let (disposition, ceiling, empty_dimensions) = meet(&request.left, &request.right);
    let no_broader_than_left = ceiling
        .as_ref()
        .is_none_or(|result| is_no_broader_than(result, &request.left));
    let no_broader_than_right = ceiling
        .as_ref()
        .is_none_or(|result| is_no_broader_than(result, &request.right));
    if !no_broader_than_left || !no_broader_than_right {
        return Err("internal non-amplification invariant violated".to_owned());
    }

    let receipt = MeetReceiptV1 {
        schema_version: RECEIPT_SCHEMA_VERSION.to_owned(),
        request_id: request.request_id,
        source_commit: source_commit.to_owned(),
        source_commit_pinned,
        input_sha256: sha256(input),
        request_schema_sha256: sha256(REQUEST_SCHEMA),
        receipt_schema_sha256: sha256(RECEIPT_SCHEMA),
        disposition,
        ceiling,
        empty_dimensions,
        non_amplification: NonAmplificationChecks {
            no_broader_than_left,
            no_broader_than_right,
        },
        claim_ceiling_only: true,
        nonclaims: NONCLAIMS.iter().map(|text| (*text).to_owned()).collect(),
    };
    let receipt_bytes = serde_json::to_vec(&receipt)
        .map_err(|error| format!("receipt serialization failed: {error}"))?;
    validate_json_against_schema(RECEIPT_SCHEMA, &receipt_bytes, "receipt")?;
    Ok(receipt)
}

fn checkout_commit() -> Result<String, String> {
    let output = Command::new("git")
        .arg("-C")
        .arg(env!("CARGO_MANIFEST_DIR"))
        .args(["rev-parse", "HEAD"])
        .output()
        .map_err(|error| format!("cannot launch git to verify source identity: {error}"))?;
    if !output.status.success() {
        return Err("cannot resolve checked-out source commit; use UNPINNED_LOCAL_RUN for an explicitly unpinned receipt".to_owned());
    }
    let actual = String::from_utf8_lossy(&output.stdout).trim().to_owned();

    let dirty = Command::new("git")
        .arg("-C")
        .arg(env!("CARGO_MANIFEST_DIR"))
        .args(["status", "--porcelain", "--untracked-files=no"])
        .output()
        .map_err(|error| format!("cannot launch git to check source modifications: {error}"))?;
    if !dirty.status.success() || !String::from_utf8_lossy(&dirty.stdout).trim().is_empty() {
        return Err("tracked source tree is modified; refusing a pinned source receipt".to_owned());
    }
    Ok(actual)
}

fn validate_source_commit(source_commit: &str) -> Result<bool, String> {
    if source_commit == UNPINNED_LOCAL_RUN {
        return Ok(false);
    }
    if source_commit.len() != 40
        || !source_commit
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return Err(
            "source commit must be 40 lowercase hexadecimal characters or UNPINNED_LOCAL_RUN"
                .to_owned(),
        );
    }
    let actual = checkout_commit()?;
    if actual != source_commit {
        return Err(format!(
            "source commit mismatch: requested {source_commit}, checked-out source is {actual}"
        ));
    }
    Ok(true)
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::Deserialize;
    use serde_json::json;

    #[derive(Debug, Deserialize)]
    struct GoldenVectorCorpus {
        schema_version: String,
        vectors: Vec<GoldenVector>,
    }

    #[derive(Debug, Deserialize)]
    struct GoldenVector {
        id: String,
        request: MeetRequestV1,
        expected: GoldenExpectation,
    }

    #[derive(Debug, Deserialize)]
    struct GoldenExpectation {
        disposition: MeetDisposition,
        ceiling: Option<ClaimCeilingV1>,
        empty_dimensions: Vec<EmptyDimension>,
    }

    fn atom(
        predicate: ClaimPredicate,
        evidence_class: EvidenceClass,
        derivation: DerivationKind,
        intended_use: IntendedUse,
    ) -> ClaimAtom {
        ClaimAtom {
            predicate,
            evidence_class,
            derivation,
            intended_use,
        }
    }

    fn base_ceiling() -> ClaimCeilingV1 {
        ClaimCeilingV1 {
            claim_atoms: BTreeSet::from([
                atom(
                    ClaimPredicate::Measurement,
                    EvidenceClass::InstrumentMeasurement,
                    DerivationKind::Direct,
                    IntendedUse::Research,
                ),
                atom(
                    ClaimPredicate::Forecast,
                    EvidenceClass::ForecastOutput,
                    DerivationKind::StatisticalInference,
                    IntendedUse::Planning,
                ),
            ]),
            subject_ids: BTreeSet::from(["asset-1".to_owned(), "asset-2".to_owned()]),
            context_ids: BTreeSet::from(["zone-a".to_owned(), "zone-b".to_owned()]),
            valid_time: TimeWindow {
                start_unix_seconds: 100,
                end_unix_seconds_exclusive: 200,
            },
            action_scopes: BTreeSet::from([
                "publish-report".to_owned(),
                "operate-asset".to_owned(),
            ]),
            required_qualifications: BTreeSet::from(["source-attributed".to_owned()]),
            required_independence_controls: BTreeSet::from([IndependenceControl::DistinctRecord]),
            required_provenance: BTreeSet::from([ProvenanceRequirement::ExactArtifactDigest]),
        }
    }

    fn request(left: ClaimCeilingV1, right: ClaimCeilingV1) -> MeetRequestV1 {
        MeetRequestV1 {
            schema_version: REQUEST_SCHEMA_VERSION.to_owned(),
            request_id: "test-request-1".to_owned(),
            left,
            right,
        }
    }

    #[test]
    fn meet_is_idempotent_commutative_and_narrower_than_both_inputs() {
        let left = base_ceiling();
        let idempotent = meet(&left, &left);
        assert_eq!(idempotent.0, MeetDisposition::UsableIntersection);
        assert_eq!(idempotent.1.as_ref(), Some(&left));
        assert!(is_no_broader_than(&left, &left));

        let mut right = base_ceiling();
        right.subject_ids = BTreeSet::from(["asset-1".to_owned()]);
        right.context_ids = BTreeSet::from(["zone-a".to_owned()]);
        right.valid_time = TimeWindow {
            start_unix_seconds: 120,
            end_unix_seconds_exclusive: 180,
        };
        right.action_scopes = BTreeSet::from(["publish-report".to_owned()]);
        right
            .required_qualifications
            .insert("calibrated-tool".to_owned());
        right
            .required_independence_controls
            .insert(IndependenceControl::IndependentOrigin);
        right
            .required_provenance
            .insert(ProvenanceRequirement::SourceAttribution);

        let (forward_disposition, forward, _) = meet(&left, &right);
        let (reverse_disposition, reverse, _) = meet(&right, &left);
        assert_eq!(forward_disposition, reverse_disposition);
        assert_eq!(forward, reverse);
        let result = forward.expect("overlapping ceilings retain a usable intersection");
        assert!(is_no_broader_than(&result, &left));
        assert!(is_no_broader_than(&result, &right));
        assert_eq!(result.subject_ids, BTreeSet::from(["asset-1".to_owned()]));
        assert_eq!(result.context_ids, BTreeSet::from(["zone-a".to_owned()]));
        assert_eq!(
            result.valid_time,
            TimeWindow {
                start_unix_seconds: 120,
                end_unix_seconds_exclusive: 180
            }
        );
        assert_eq!(
            result.action_scopes,
            BTreeSet::from(["publish-report".to_owned()])
        );
        assert_eq!(
            result.required_qualifications,
            BTreeSet::from(["source-attributed".to_owned(), "calibrated-tool".to_owned()])
        );
    }

    #[test]
    fn meet_is_associative_for_three_overlapping_ceilings() {
        let a = base_ceiling();
        let mut b = base_ceiling();
        b.subject_ids = BTreeSet::from(["asset-1".to_owned()]);
        b.valid_time = TimeWindow {
            start_unix_seconds: 110,
            end_unix_seconds_exclusive: 190,
        };
        b.required_qualifications.insert("qualified-b".to_owned());
        let mut c = base_ceiling();
        c.context_ids = BTreeSet::from(["zone-b".to_owned()]);
        c.valid_time = TimeWindow {
            start_unix_seconds: 120,
            end_unix_seconds_exclusive: 180,
        };
        c.required_independence_controls
            .insert(IndependenceControl::IndependentVerifier);
        let ab = meet(&a, &b).1.expect("first meet");
        let left = meet(&ab, &c).1.expect("left-associated meet");
        let bc = meet(&b, &c).1.expect("second meet");
        let right = meet(&a, &bc).1.expect("right-associated meet");
        assert_eq!(left, right);
    }

    fn next_random(seed: &mut u64) -> u64 {
        *seed = seed
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        *seed
    }

    fn selected_set<T: Clone + Ord>(values: &[T], seed: &mut u64) -> BTreeSet<T> {
        let mut result = values
            .iter()
            .filter(|_| next_random(seed) & 1 == 1)
            .cloned()
            .collect::<BTreeSet<_>>();
        if result.is_empty() {
            let index = (next_random(seed) as usize) % values.len();
            result.insert(values[index].clone());
        }
        result
    }

    fn generated_ceiling(seed: &mut u64) -> ClaimCeilingV1 {
        let atoms = [
            atom(
                ClaimPredicate::Measurement,
                EvidenceClass::InstrumentMeasurement,
                DerivationKind::Direct,
                IntendedUse::Research,
            ),
            atom(
                ClaimPredicate::Forecast,
                EvidenceClass::ForecastOutput,
                DerivationKind::StatisticalInference,
                IntendedUse::Planning,
            ),
            atom(
                ClaimPredicate::CausalClaim,
                EvidenceClass::CausalIdentification,
                DerivationKind::StatisticalInference,
                IntendedUse::DecisionSupport,
            ),
            atom(
                ClaimPredicate::FormalProperty,
                EvidenceClass::FormalProof,
                DerivationKind::FormalVerification,
                IntendedUse::Research,
            ),
        ];
        let subjects = ["asset-1", "asset-2", "asset-3", "asset-4"].map(str::to_owned);
        let contexts = ["zone-a", "zone-b", "zone-c"].map(str::to_owned);
        let actions = ["publish-report", "plan-repair", "operate-asset"].map(str::to_owned);
        let qualifications =
            ["qualified-source", "calibrated-tool", "reviewed-method"].map(str::to_owned);
        let controls = [
            IndependenceControl::DistinctRecord,
            IndependenceControl::DistinctAcquisition,
            IndependenceControl::IndependentOrigin,
            IndependenceControl::IndependentVerifier,
        ];
        let provenance = [
            ProvenanceRequirement::SourceAttribution,
            ProvenanceRequirement::ExactArtifactDigest,
            ProvenanceRequirement::TrustedTimeEvidence,
            ProvenanceRequirement::DependencyClosure,
        ];
        let start = (next_random(seed) % 1_000) as i64;
        let duration = (next_random(seed) % 100 + 1) as i64;
        ClaimCeilingV1 {
            claim_atoms: selected_set(&atoms, seed),
            subject_ids: selected_set(&subjects, seed),
            context_ids: selected_set(&contexts, seed),
            valid_time: TimeWindow {
                start_unix_seconds: start,
                end_unix_seconds_exclusive: start + duration,
            },
            action_scopes: actions
                .iter()
                .filter(|_| next_random(seed) & 1 == 1)
                .cloned()
                .collect(),
            required_qualifications: qualifications
                .iter()
                .filter(|_| next_random(seed) & 1 == 1)
                .cloned()
                .collect(),
            required_independence_controls: selected_set(&controls, seed),
            required_provenance: selected_set(&provenance, seed),
        }
    }

    #[test]
    fn deterministic_10k_property_sweep_finds_no_lattice_violation() {
        let mut seed = 0x9e37_79b9_7f4a_7c15_u64;
        for iteration in 0..10_000 {
            let a = generated_ceiling(&mut seed);
            let b = generated_ceiling(&mut seed);
            let c = generated_ceiling(&mut seed);

            let ab = meet(&a, &b).1;
            let ba = meet(&b, &a).1;
            assert_eq!(ab, ba, "commutativity failed on pair {iteration}");

            if let Some(result) = &ab {
                assert!(
                    is_no_broader_than(result, &a),
                    "left non-amplification failed on pair {iteration}"
                );
                assert!(
                    is_no_broader_than(result, &b),
                    "right non-amplification failed on pair {iteration}"
                );
            }
            assert_eq!(
                meet(&a, &a).1.as_ref(),
                Some(&a),
                "idempotence failed on item {iteration}"
            );

            let left_associated = ab.and_then(|first| meet(&first, &c).1);
            let bc = meet(&b, &c).1;
            let right_associated = bc.and_then(|second| meet(&a, &second).1);
            assert_eq!(
                left_associated, right_associated,
                "associativity failed on triple {iteration}"
            );
        }
    }

    #[test]
    fn disjoint_subjects_produce_bottom_without_a_fake_ceiling() {
        let mut left = base_ceiling();
        let mut right = base_ceiling();
        left.subject_ids = BTreeSet::from(["asset-1".to_owned()]);
        right.subject_ids = BTreeSet::from(["asset-2".to_owned()]);
        let (disposition, ceiling, empty) = meet(&left, &right);
        assert_eq!(disposition, MeetDisposition::BottomNoCommonPermittedClaim);
        assert_eq!(ceiling, None);
        assert_eq!(empty, vec![EmptyDimension::SubjectIds]);
    }

    #[test]
    fn disjoint_time_windows_produce_bottom() {
        let mut left = base_ceiling();
        let mut right = base_ceiling();
        left.valid_time = TimeWindow {
            start_unix_seconds: 100,
            end_unix_seconds_exclusive: 120,
        };
        right.valid_time = TimeWindow {
            start_unix_seconds: 120,
            end_unix_seconds_exclusive: 140,
        };
        let (disposition, ceiling, empty) = meet(&left, &right);
        assert_eq!(disposition, MeetDisposition::BottomNoCommonPermittedClaim);
        assert_eq!(ceiling, None);
        assert_eq!(empty, vec![EmptyDimension::TimeWindow]);
    }

    #[test]
    fn incompatible_claim_atoms_produce_bottom_instead_of_cross_product_invention() {
        let mut left = base_ceiling();
        let mut right = base_ceiling();
        left.claim_atoms = BTreeSet::from([atom(
            ClaimPredicate::CausalClaim,
            EvidenceClass::CorrelationalAnalysis,
            DerivationKind::StatisticalInference,
            IntendedUse::DecisionSupport,
        )]);
        right.claim_atoms = BTreeSet::from([atom(
            ClaimPredicate::CausalClaim,
            EvidenceClass::CausalIdentification,
            DerivationKind::StatisticalInference,
            IntendedUse::DecisionSupport,
        )]);
        let (disposition, ceiling, empty) = meet(&left, &right);
        assert_eq!(disposition, MeetDisposition::BottomNoCommonPermittedClaim);
        assert_eq!(ceiling, None);
        assert_eq!(empty, vec![EmptyDimension::ClaimAtoms]);
    }

    #[test]
    fn no_shared_action_scope_retains_analysis_but_grants_no_action() {
        let mut left = base_ceiling();
        let mut right = base_ceiling();
        left.action_scopes = BTreeSet::from(["operate-asset".to_owned()]);
        right.action_scopes = BTreeSet::from(["publish-report".to_owned()]);
        let (disposition, ceiling, empty) = meet(&left, &right);
        assert_eq!(disposition, MeetDisposition::UsableIntersection);
        assert!(empty.is_empty());
        assert_eq!(
            ceiling.expect("claim may remain assessable").action_scopes,
            BTreeSet::new()
        );
    }

    #[test]
    fn requirements_are_never_dropped_during_composition() {
        let left = base_ceiling();
        let mut right = base_ceiling();
        right
            .required_qualifications
            .insert("qualified-tool".to_owned());
        right
            .required_independence_controls
            .insert(IndependenceControl::IndependentOrigin);
        right
            .required_provenance
            .insert(ProvenanceRequirement::TrustedTimeEvidence);
        let result = meet(&left, &right).1.expect("non-empty claim scope");
        assert!(result.required_qualifications.contains("qualified-tool"));
        assert!(result.required_qualifications.contains("source-attributed"));
        assert!(
            result
                .required_independence_controls
                .contains(&IndependenceControl::DistinctRecord)
        );
        assert!(
            result
                .required_independence_controls
                .contains(&IndependenceControl::IndependentOrigin)
        );
        assert!(
            result
                .required_provenance
                .contains(&ProvenanceRequirement::ExactArtifactDigest)
        );
        assert!(
            result
                .required_provenance
                .contains(&ProvenanceRequirement::TrustedTimeEvidence)
        );
    }

    #[test]
    fn malformed_json_and_unpinned_commit_are_explicitly_rejected_or_marked() {
        assert!(evaluate_request_bytes(b"{", UNPINNED_LOCAL_RUN).is_err());
        assert!(evaluate_request_bytes(b"{}", "not-a-commit").is_err());

        let request = request(base_ceiling(), base_ceiling());
        let bytes = serde_json::to_vec(&request).unwrap();
        let receipt =
            evaluate_request_bytes(&bytes, UNPINNED_LOCAL_RUN).expect("valid unpinned local run");
        assert!(!receipt.source_commit_pinned);
        assert_eq!(receipt.source_commit, UNPINNED_LOCAL_RUN);
        assert!(receipt.non_amplification.no_broader_than_left);
        assert!(receipt.non_amplification.no_broader_than_right);
    }

    #[test]
    fn a_pinned_receipt_requires_the_actual_clean_checkout_commit() {
        let request = serde_json::to_vec(&request(base_ceiling(), base_ceiling())).unwrap();
        let actual = checkout_commit().expect("test runs in a Git checkout");
        let receipt =
            evaluate_request_bytes(&request, &actual).expect("exact checkout pin accepted");
        assert!(receipt.source_commit_pinned);

        let replacement_prefix = if actual.starts_with('0') { "1" } else { "0" };
        let mut wrong = actual.clone();
        wrong.replace_range(0..1, replacement_prefix);
        assert!(
            evaluate_request_bytes(&request, &wrong)
                .expect_err("different source commit must not be labeled pinned")
                .contains("source commit mismatch")
        );
    }

    #[test]
    fn frozen_portable_golden_vectors_match_the_reference_algebra() {
        let corpus: GoldenVectorCorpus =
            serde_json::from_slice(GOLDEN_VECTORS).expect("golden-vector corpus parses");
        assert_eq!(
            corpus.schema_version,
            "mycelix.claim-ceiling-meet.golden.v1"
        );
        assert_eq!(corpus.vectors.len(), 5);

        for vector in corpus.vectors {
            let request_bytes =
                serde_json::to_vec(&vector.request).expect("golden request serializes");
            let receipt = evaluate_request_bytes(&request_bytes, UNPINNED_LOCAL_RUN)
                .unwrap_or_else(|error| panic!("golden vector {} failed: {error}", vector.id));
            assert_eq!(
                receipt.disposition, vector.expected.disposition,
                "disposition mismatch for {}",
                vector.id
            );
            assert_eq!(
                receipt.ceiling, vector.expected.ceiling,
                "ceiling mismatch for {}",
                vector.id
            );
            assert_eq!(
                receipt.empty_dimensions, vector.expected.empty_dimensions,
                "empty dimensions mismatch for {}",
                vector.id
            );
            assert!(receipt.non_amplification.no_broader_than_left);
            assert!(receipt.non_amplification.no_broader_than_right);
        }
    }

    #[test]
    fn schema_rejects_unknown_fields_and_schema_files_are_valid_draft_2020_12() {
        let schema_req: serde_json::Value = serde_json::from_slice(REQUEST_SCHEMA).unwrap();
        let schema_rec: serde_json::Value = serde_json::from_slice(RECEIPT_SCHEMA).unwrap();
        jsonschema::JSONSchema::options()
            .with_draft(jsonschema::Draft::Draft202012)
            .compile(&schema_req)
            .expect("request schema is valid Draft 2020-12");
        jsonschema::JSONSchema::options()
            .with_draft(jsonschema::Draft::Draft202012)
            .compile(&schema_rec)
            .expect("receipt schema is valid Draft 2020-12");

        let mut value = serde_json::to_value(request(base_ceiling(), base_ceiling())).unwrap();
        value["unexpected"] = json!(true);
        assert!(
            evaluate_request_bytes(&serde_json::to_vec(&value).unwrap(), UNPINNED_LOCAL_RUN)
                .is_err()
        );
    }
}

pub mod policy_profile;
pub mod policy_lifecycle;
