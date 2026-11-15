#![forbid(unsafe_code)]

// Independent verifier for the bounded FSRE decision-defeater E0 format.
//
// This crate intentionally does not depend on, import, or call the producer
// crate. It independently parses the frozen input, computes decision
// signatures, enumerates the finite perturbation universe, checks the
// producer's witnesses and verifies a content-addressed evidence package.
//
// Claim ceiling: SHA-256 proves byte integrity relative to the manifest. The
// fixture manifest has no signature, and the verifier does not authenticate
// source truth, institutional authority, rights, causal truth or physical
// realizability.

use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::fmt::Write as _;
use std::fs;
use std::path::Path;

const MANIFEST_NAME: &str = "manifest.psv";
const MANIFEST_HEADER: &str = "FSRE-DECISION-EVIDENCE-MANIFEST|1";
const INPUT_HEADER: &str = "world_id|action_id|decision_value|safety_value";
const MUTATION_HEADER: &str =
    "mutation_id|mutation_kind|world_id|action_id|delta|cost|evidence_ref|admissibility";
const MAX_MANIFEST_BYTES: usize = 64 * 1024;
const MAX_INPUT_BYTES: usize = 16 * 1024 * 1024;
const MAX_REPORT_BYTES: usize = 8 * 1024 * 1024;
const MAX_EVIDENCE_BYTES: usize = 8 * 1024 * 1024;
const MAX_PERTURBATIONS: usize = 22;
const MAX_ROWS: usize = 1_000_000;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Disposition {
    VerifiedWitnessAndMinimum,
    DigestMismatch,
    SchemaUnsupported,
    EvidenceReferenceUnresolved,
    MutationNotAdmissible,
    SourceUseBlockedOrUnknown,
    AuthorityBlockedOrUnknown,
    InputOrWorldSetIncomplete,
    WitnessReplayMismatch,
    MinimumCertificateInvalid,
    VerifierResourceLimit,
    Indeterminate,
}

impl Disposition {
    pub fn as_str(self) -> &'static str {
        match self {
            Self::VerifiedWitnessAndMinimum => "VERIFIED_WITNESS_AND_MINIMUM",
            Self::DigestMismatch => "DIGEST_MISMATCH",
            Self::SchemaUnsupported => "SCHEMA_UNSUPPORTED",
            Self::EvidenceReferenceUnresolved => "EVIDENCE_REFERENCE_UNRESOLVED",
            Self::MutationNotAdmissible => "MUTATION_NOT_ADMISSIBLE",
            Self::SourceUseBlockedOrUnknown => "SOURCE_USE_BLOCKED_OR_UNKNOWN",
            Self::AuthorityBlockedOrUnknown => "AUTHORITY_BLOCKED_OR_UNKNOWN",
            Self::InputOrWorldSetIncomplete => "INPUT_OR_WORLD_SET_INCOMPLETE",
            Self::WitnessReplayMismatch => "WITNESS_REPLAY_MISMATCH",
            Self::MinimumCertificateInvalid => "MINIMUM_CERTIFICATE_INVALID",
            Self::VerifierResourceLimit => "VERIFIER_RESOURCE_LIMIT",
            Self::Indeterminate => "INDETERMINATE",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VerifyError {
    pub disposition: Disposition,
    pub detail: String,
}

impl VerifyError {
    fn new(disposition: Disposition, detail: impl Into<String>) -> Self {
        Self {
            disposition,
            detail: detail.into(),
        }
    }
}

impl std::fmt::Display for VerifyError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}: {}", self.disposition.as_str(), self.detail)
    }
}

impl std::error::Error for VerifyError {}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VerificationReceipt {
    pub disposition: Disposition,
    pub manifest_sha256: String,
    pub input_sha256: String,
    pub report_sha256: String,
    pub evidence_object_count: usize,
    pub mutation_count: usize,
    pub evaluated_subsets: u64,
    pub verified_minimum_cost: u128,
    pub verified_witness_count: u64,
    pub analysis_id: String,
    pub input_snapshot_id: String,
}

impl VerificationReceipt {
    pub fn render(&self) -> String {
        let mut out = String::new();
        writeln!(out, "verifier_schema=fsre.independent-decision-defeater-verifier")
            .expect("String write");
        writeln!(out, "verifier_version=0.1.0").expect("String write");
        writeln!(out, "disposition={}", self.disposition.as_str()).expect("String write");
        writeln!(out, "analysis_id={}", self.analysis_id).expect("String write");
        writeln!(out, "input_snapshot_id={}", self.input_snapshot_id).expect("String write");
        writeln!(out, "manifest_sha256={}", self.manifest_sha256).expect("String write");
        writeln!(out, "input_sha256={}", self.input_sha256).expect("String write");
        writeln!(out, "report_sha256={}", self.report_sha256).expect("String write");
        writeln!(out, "evidence_object_count={}", self.evidence_object_count)
            .expect("String write");
        writeln!(out, "mutation_count={}", self.mutation_count).expect("String write");
        writeln!(out, "evaluated_subsets={}", self.evaluated_subsets).expect("String write");
        writeln!(out, "verified_minimum_cost={}", self.verified_minimum_cost)
            .expect("String write");
        writeln!(out, "verified_witness_count={}", self.verified_witness_count)
            .expect("String write");
        out.push_str("input_binding=EXACT_RAW_BYTES\n");
        out.push_str("canonical_manifest=FIXED_SCHEMA_LENGTH_BOUNDED_PSV_V1\n");
        out.push_str("witness_replay=INDEPENDENT_IMPLEMENTATION\n");
        out.push_str("minimum_verification=INDEPENDENT_EXHAUSTIVE_ENUMERATION\n");
        out.push_str("evidence_integrity=CONTENT_INTEGRITY_ONLY\n");
        out.push_str("source_use=ASSERTED_NOT_AUTHENTICATED\n");
        out.push_str("authority=ASSERTED_NOT_AUTHENTICATED\n");
        out.push_str("issuer_authenticity=NOT_ESTABLISHED\n");
        out.push_str("legal_rights=NOT_ESTABLISHED\n");
        out.push_str("population_completeness=NOT_ESTABLISHED\n");
        out.push_str("causal_truth=NOT_ESTABLISHED\n");
        out.push_str("physical_realizability=NOT_ESTABLISHED\n");
        out
    }
}

#[derive(Clone, Debug)]
struct EvidenceEntry {
    reference: String,
    role: String,
    filename: String,
    digest: String,
}

#[derive(Clone, Debug)]
struct Manifest {
    input_filename: String,
    input_digest: String,
    report_filename: String,
    report_digest: String,
    evidence: Vec<EvidenceEntry>,
}

#[derive(Clone, Debug)]
struct Cell {
    decision_value: i64,
    safety_value: i64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum MutationKind {
    DecisionValueDelta,
    SafetyValueDelta,
}

#[derive(Clone, Debug)]
struct Mutation {
    id: String,
    kind: MutationKind,
    world_id: String,
    action_id: String,
    delta: i64,
    cost: u64,
    evidence_ref: String,
}

#[derive(Clone, Debug)]
struct Problem {
    analysis_id: String,
    snapshot_id: String,
    property: String,
    target_action: String,
    safety_floor: i64,
    cost_unit: String,
    worlds: Vec<String>,
    actions: Vec<String>,
    cells: BTreeMap<(String, String), Cell>,
    mutations: Vec<Mutation>,
    all_evidence_refs: BTreeSet<String>,
    blocked_mutations: usize,
}

#[derive(Clone, Debug, PartialEq, Eq)]
struct Signature {
    robustly_safe_actions: Vec<String>,
    invariant_optimal_set: Vec<String>,
    unique_optimum: Option<String>,
    minimax_regret_actions: Vec<String>,
}

#[derive(Clone, Debug)]
struct VerifiedWitness {
    mutation_ids: Vec<String>,
    evidence_refs: Vec<String>,
    cost: u128,
    mutated: Signature,
}

fn invalid(detail: impl Into<String>) -> VerifyError {
    VerifyError::new(Disposition::SchemaUnsupported, detail)
}

fn valid_token(value: &str) -> bool {
    !value.is_empty()
        && value != "-"
        && value.len() <= 200
        && value.bytes().all(|byte| {
            byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'.' | b':' | b'/')
        })
}

fn valid_filename(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 128
        && value != "."
        && value != ".."
        && value.bytes().all(|byte| {
            byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'.')
        })
}

fn required<'a>(
    fields: &'a BTreeMap<String, String>,
    key: &str,
) -> Result<&'a str, VerifyError> {
    fields
        .get(key)
        .map(String::as_str)
        .ok_or_else(|| invalid(format!("missing field {key}")))
}

fn parse_u64(value: &str, name: &str) -> Result<u64, VerifyError> {
    value
        .parse::<u64>()
        .map_err(|error| invalid(format!("invalid {name}: {error}")))
}

fn parse_i64(value: &str, name: &str) -> Result<i64, VerifyError> {
    value
        .parse::<i64>()
        .map_err(|error| invalid(format!("invalid {name}: {error}")))
}

fn hash_artifact(domain: &str, bytes: &[u8]) -> String {
    let mut digest = Sha256::new();
    digest.update(b"FSRE-CONTENT-DIGEST\0");
    digest.update((domain.len() as u32).to_be_bytes());
    digest.update(domain.as_bytes());
    digest.update((bytes.len() as u64).to_be_bytes());
    digest.update(bytes);
    hex_lower(&digest.finalize())
}

fn hex_lower(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        out.push(HEX[(byte >> 4) as usize] as char);
        out.push(HEX[(byte & 0x0f) as usize] as char);
    }
    out
}

fn parse_digest(value: &str) -> Result<(), VerifyError> {
    let Some(hex) = value.strip_prefix("sha256:") else {
        return Err(invalid("digest must use sha256:<64 lowercase hex>"));
    };
    if hex.len() != 64
        || !hex
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return Err(invalid("malformed SHA-256 digest"));
    }
    Ok(())
}

fn parse_manifest(bytes: &[u8]) -> Result<Manifest, VerifyError> {
    if bytes.len() > MAX_MANIFEST_BYTES {
        return Err(VerifyError::new(
            Disposition::VerifierResourceLimit,
            "manifest exceeds size limit",
        ));
    }
    let source = std::str::from_utf8(bytes).map_err(|_| invalid("manifest is not UTF-8"))?;
    if !source.ends_with('\n') || source.contains('\r') {
        return Err(invalid("manifest must be LF-terminated with no CR bytes"));
    }
    let mut lines = source.lines();
    if lines.next() != Some(MANIFEST_HEADER) {
        return Err(invalid("unsupported manifest schema header"));
    }
    let input_line = lines
        .next()
        .ok_or_else(|| invalid("missing input manifest row"))?;
    let report_line = lines
        .next()
        .ok_or_else(|| invalid("missing report manifest row"))?;

    let input_fields: Vec<&str> = input_line.split('|').collect();
    let report_fields: Vec<&str> = report_line.split('|').collect();
    if input_fields.len() != 3 || input_fields[0] != "input" {
        return Err(invalid("expected input|filename|sha256 row"));
    }
    if report_fields.len() != 3 || report_fields[0] != "report" {
        return Err(invalid("expected report|filename|sha256 row"));
    }
    if !valid_filename(input_fields[1]) || !valid_filename(report_fields[1]) {
        return Err(invalid("unsafe input or report filename"));
    }
    parse_digest(input_fields[2])?;
    parse_digest(report_fields[2])?;
    if input_fields[1] == report_fields[1] || input_fields[1] == MANIFEST_NAME
        || report_fields[1] == MANIFEST_NAME
    {
        return Err(invalid("manifest, input and report filenames must be distinct"));
    }

    let mut evidence = Vec::new();
    let mut seen_refs = BTreeSet::new();
    let mut previous_ref: Option<String> = None;
    let mut seen_files = BTreeSet::from([
        input_fields[1].to_owned(),
        report_fields[1].to_owned(),
        MANIFEST_NAME.to_owned(),
    ]);
    for line in lines {
        let fields: Vec<&str> = line.split('|').collect();
        if fields.len() != 5 || fields[0] != "evidence" {
            return Err(invalid("expected evidence|reference|role|filename|sha256 row"));
        }
        if !valid_token(fields[1]) || !valid_token(fields[2]) || !valid_filename(fields[3]) {
            return Err(invalid("invalid evidence reference, role or filename"));
        }
        parse_digest(fields[4])?;
        if fields[2] != "SYNTHETIC_FIXTURE_ASSERTION" {
            return Err(invalid("v0.1 verifier accepts synthetic fixture evidence only"));
        }
        if !seen_refs.insert(fields[1].to_owned()) || !seen_files.insert(fields[3].to_owned()) {
            return Err(invalid("duplicate evidence reference or artifact filename"));
        }
        if previous_ref.as_deref().is_some_and(|previous| previous >= fields[1]) {
            return Err(invalid("evidence rows must be strictly sorted by reference"));
        }
        previous_ref = Some(fields[1].to_owned());
        evidence.push(EvidenceEntry {
            reference: fields[1].to_owned(),
            role: fields[2].to_owned(),
            filename: fields[3].to_owned(),
            digest: fields[4].to_owned(),
        });
    }

    if evidence.is_empty() {
        return Err(VerifyError::new(
            Disposition::EvidenceReferenceUnresolved,
            "manifest contains no evidence objects",
        ));
    }
    Ok(Manifest {
        input_filename: input_fields[1].to_owned(),
        input_digest: input_fields[2].to_owned(),
        report_filename: report_fields[1].to_owned(),
        report_digest: report_fields[2].to_owned(),
        evidence,
    })
}

fn read_regular_file(
    root: &Path,
    filename: &str,
    limit: usize,
) -> Result<Vec<u8>, VerifyError> {
    if !valid_filename(filename) {
        return Err(invalid("unsafe artifact filename"));
    }
    let path = root.join(filename);
    let metadata = fs::symlink_metadata(&path).map_err(|error| {
        VerifyError::new(
            Disposition::Indeterminate,
            format!("cannot stat artifact {filename}: {error}"),
        )
    })?;
    if !metadata.file_type().is_file() {
        return Err(VerifyError::new(
            Disposition::SchemaUnsupported,
            format!("artifact {filename} must be a regular non-symlink file"),
        ));
    }
    if metadata.len() > limit as u64 {
        return Err(VerifyError::new(
            Disposition::VerifierResourceLimit,
            format!("artifact {filename} exceeds size limit"),
        ));
    }
    fs::read(path).map_err(|error| {
        VerifyError::new(
            Disposition::Indeterminate,
            format!("cannot read artifact {filename}: {error}"),
        )
    })
}

fn parse_problem(bytes: &[u8]) -> Result<Problem, VerifyError> {
    let source = std::str::from_utf8(bytes).map_err(|_| invalid("input is not UTF-8"))?;
    let lines: Vec<&str> = source.lines().collect();
    let mut metadata = BTreeMap::<String, String>::new();
    let mut header_index = None;
    let allowed = [
        "analysis_id",
        "input_snapshot_id",
        "decision_cutoff",
        "decision_direction",
        "decision_property",
        "target_action",
        "safety_floor",
        "source_use_status",
        "authority_status",
        "world_set_completeness",
        "perturbation_universe_completeness",
        "cost_unit",
        "max_subsets",
        "max_witnesses",
    ];
    for (index, raw) in lines.iter().enumerate() {
        let line = raw.trim();
        if line.is_empty() {
            continue;
        }
        if let Some(comment) = line.strip_prefix('#') {
            let (key, value) = comment
                .trim()
                .split_once('=')
                .ok_or_else(|| invalid(format!("line {}: malformed metadata", index + 1)))?;
            if !allowed.contains(&key) || value.is_empty() {
                return Err(invalid(format!("line {}: unsupported metadata", index + 1)));
            }
            if metadata.insert(key.to_owned(), value.to_owned()).is_some() {
                return Err(invalid(format!("line {}: duplicate metadata", index + 1)));
            }
            continue;
        }
        if line != INPUT_HEADER {
            return Err(invalid(format!(
                "line {}: expected exact input header",
                index + 1
            )));
        }
        header_index = Some(index);
        break;
    }
    let header_index = header_index.ok_or_else(|| invalid("missing input header"))?;

    let analysis_id = required(&metadata, "analysis_id")?.to_owned();
    let snapshot_id = required(&metadata, "input_snapshot_id")?.to_owned();
    let _cutoff = parse_u64(required(&metadata, "decision_cutoff")?, "decision_cutoff")?;
    if !valid_token(&analysis_id) || !valid_token(&snapshot_id) {
        return Err(invalid("invalid analysis or snapshot identifier"));
    }
    if required(&metadata, "source_use_status")? != "AUTHORIZED" {
        return Err(VerifyError::new(
            Disposition::SourceUseBlockedOrUnknown,
            "source-use status is blocked or unknown",
        ));
    }
    if required(&metadata, "authority_status")? != "ESTABLISHED" {
        return Err(VerifyError::new(
            Disposition::AuthorityBlockedOrUnknown,
            "authority status is blocked or unknown",
        ));
    }
    if required(&metadata, "world_set_completeness")? != "DECLARED_COMPLETE"
        || required(&metadata, "perturbation_universe_completeness")? != "DECLARED_COMPLETE"
    {
        return Err(VerifyError::new(
            Disposition::InputOrWorldSetIncomplete,
            "declared world set or perturbation universe is not complete",
        ));
    }
    if required(&metadata, "decision_direction")? != "MAXIMIZE" {
        return Err(invalid("v0.1 verifier supports MAXIMIZE only"));
    }
    let property = required(&metadata, "decision_property")?.to_owned();
    if property != "ROBUST_SAFE_ACTION_RETAINED" {
        return Err(VerifyError::new(
            Disposition::SchemaUnsupported,
            "v0.1 independent verifier supports ROBUST_SAFE_ACTION_RETAINED only",
        ));
    }
    let target_action = required(&metadata, "target_action")?.to_owned();
    if !valid_token(&target_action) {
        return Err(invalid("invalid target action"));
    }
    let safety_floor = parse_i64(required(&metadata, "safety_floor")?, "safety_floor")?;
    let cost_unit = required(&metadata, "cost_unit")?.to_owned();
    if !valid_token(&cost_unit) {
        return Err(invalid("invalid cost unit"));
    }

    let mut worlds = BTreeSet::<String>::new();
    let mut actions = BTreeSet::<String>::new();
    let mut cells = BTreeMap::<(String, String), Cell>::new();
    let mut mutations = Vec::new();
    let mut all_evidence_refs = BTreeSet::new();
    let mut ids = BTreeSet::new();
    let mut blocked_mutations = 0usize;
    let mut in_mutations = false;
    let mut mutation_header_seen = false;

    let mut index = header_index + 1;
    while index < lines.len() {
        let line = lines[index].trim();
        index += 1;
        if line.is_empty() {
            continue;
        }
        if !in_mutations && line == "MUTATIONS" {
            in_mutations = true;
            if lines.get(index).map(|line| line.trim()) != Some(MUTATION_HEADER) {
                return Err(invalid("MUTATIONS must be followed by exact mutation header"));
            }
            mutation_header_seen = true;
            index += 1;
            continue;
        }
        if line.starts_with('#') {
            return Err(invalid("metadata after the input header is not allowed"));
        }
        let fields: Vec<&str> = line.split('|').map(str::trim).collect();
        if !in_mutations {
            if fields.len() != 4 || !valid_token(fields[0]) || !valid_token(fields[1]) {
                return Err(invalid("invalid world/action row"));
            }
            let key = (fields[0].to_owned(), fields[1].to_owned());
            let cell = Cell {
                decision_value: parse_i64(fields[2], "decision_value")?,
                safety_value: parse_i64(fields[3], "safety_value")?,
            };
            if cells.insert(key, cell).is_some() {
                return Err(invalid("duplicate world/action cell"));
            }
            worlds.insert(fields[0].to_owned());
            actions.insert(fields[1].to_owned());
            if cells.len() > MAX_ROWS {
                return Err(VerifyError::new(
                    Disposition::VerifierResourceLimit,
                    "world/action rows exceed verifier limit",
                ));
            }
        } else {
            if fields.len() != 8
                || !valid_token(fields[0])
                || !valid_token(fields[2])
                || !valid_token(fields[3])
                || !valid_token(fields[6])
            {
                return Err(invalid("invalid mutation row"));
            }
            if !ids.insert(fields[0].to_owned()) {
                return Err(invalid("duplicate mutation identifier"));
            }
            all_evidence_refs.insert(fields[6].to_owned());
            let kind = match fields[1] {
                "DECISION_VALUE_DELTA" => MutationKind::DecisionValueDelta,
                "SAFETY_VALUE_DELTA" => MutationKind::SafetyValueDelta,
                _ => {
                    return Err(VerifyError::new(
                        Disposition::SchemaUnsupported,
                        format!("unsupported mutation kind {}", fields[1]),
                    ));
                }
            };
            let delta = parse_i64(fields[4], "mutation delta")?;
            let cost = parse_u64(fields[5], "mutation cost")?;
            match fields[7] {
                "ADMISSIBLE" => mutations.push(Mutation {
                    id: fields[0].to_owned(),
                    kind,
                    world_id: fields[2].to_owned(),
                    action_id: fields[3].to_owned(),
                    delta,
                    cost,
                    evidence_ref: fields[6].to_owned(),
                }),
                "BLOCKED" => blocked_mutations += 1,
                "UNKNOWN" => {
                    return Err(VerifyError::new(
                        Disposition::MutationNotAdmissible,
                        format!("mutation {} has unknown admissibility", fields[0]),
                    ));
                }
                other => return Err(invalid(format!("invalid admissibility {other}"))),
            }
            if mutations.len() + blocked_mutations > MAX_ROWS {
                return Err(VerifyError::new(
                    Disposition::VerifierResourceLimit,
                    "mutation rows exceed verifier limit",
                ));
            }
        }
    }

    if !in_mutations || !mutation_header_seen {
        return Err(invalid("missing mutation section"));
    }
    if worlds.is_empty() || actions.is_empty() {
        return Err(VerifyError::new(
            Disposition::InputOrWorldSetIncomplete,
            "empty finite world/action set",
        ));
    }
    let expected_cells = worlds
        .len()
        .checked_mul(actions.len())
        .ok_or_else(|| invalid("world/action grid size overflow"))?;
    if expected_cells != cells.len() {
        return Err(VerifyError::new(
            Disposition::InputOrWorldSetIncomplete,
            "world/action grid is incomplete",
        ));
    }
    if !actions.contains(&target_action) {
        return Err(invalid("target action is absent from the action set"));
    }
    for mutation in &mutations {
        if !worlds.contains(&mutation.world_id) || !actions.contains(&mutation.action_id) {
            return Err(invalid(format!(
                "mutation {} references an unknown world/action cell",
                mutation.id
            )));
        }
    }
    if mutations.len() > MAX_PERTURBATIONS {
        return Err(VerifyError::new(
            Disposition::VerifierResourceLimit,
            format!(
                "{} admissible mutations exceed verifier cap {MAX_PERTURBATIONS}",
                mutations.len()
            ),
        ));
    }
    mutations.sort_by_key(|mutation| mutation.id.clone());

    let problem = Problem {
        analysis_id,
        snapshot_id,
        property,
        target_action,
        safety_floor,
        cost_unit,
        worlds: worlds.into_iter().collect(),
        actions: actions.into_iter().collect(),
        cells,
        mutations,
        all_evidence_refs,
        blocked_mutations,
    };
    let baseline = compute_signature(&problem, &BTreeMap::new());
    if !baseline
        .robustly_safe_actions
        .iter()
        .any(|action| action == &problem.target_action)
    {
        return Err(VerifyError::new(
            Disposition::WitnessReplayMismatch,
            "declared target action is not robustly safe in the baseline",
        ));
    }
    Ok(problem)
}

fn compute_signature(
    problem: &Problem,
    deltas: &BTreeMap<(String, String), (i128, i128)>,
) -> Signature {
    let robustly_safe_actions: Vec<String> = problem
        .actions
        .iter()
        .filter(|action| {
            problem.worlds.iter().all(|world| {
                let cell = problem
                    .cells
                    .get(&(world.clone(), (*action).clone()))
                    .expect("complete grid checked");
                let safety_delta = deltas
                    .get(&(world.clone(), (*action).clone()))
                    .map_or(0, |value| value.1);
                i128::from(cell.safety_value) + safety_delta >= i128::from(problem.safety_floor)
            })
        })
        .cloned()
        .collect();

    let mut common_top: Option<Vec<String>> = None;
    let mut invariant = true;
    let mut worst_regret = BTreeMap::<String, i128>::new();
    for world in &problem.worlds {
        let mut best: Option<i128> = None;
        let mut top = Vec::new();
        for action in &robustly_safe_actions {
            let cell = problem
                .cells
                .get(&(world.clone(), action.clone()))
                .expect("complete grid checked");
            let delta = deltas
                .get(&(world.clone(), action.clone()))
                .map_or(0, |value| value.0);
            let value = i128::from(cell.decision_value) + delta;
            match best {
                None => {
                    best = Some(value);
                    top.push(action.clone());
                }
                Some(current) if value > current => {
                    best = Some(value);
                    top.clear();
                    top.push(action.clone());
                }
                Some(current) if value == current => top.push(action.clone()),
                Some(_) => {}
            }
        }
        if let Some(previous) = &common_top {
            if previous != &top {
                invariant = false;
            }
        } else {
            common_top = Some(top);
        }
        if let Some(best_value) = best {
            for action in &robustly_safe_actions {
                let cell = problem
                    .cells
                    .get(&(world.clone(), action.clone()))
                    .expect("complete grid checked");
                let delta = deltas
                    .get(&(world.clone(), action.clone()))
                    .map_or(0, |value| value.0);
                let regret = best_value - (i128::from(cell.decision_value) + delta);
                let entry = worst_regret.entry(action.clone()).or_insert(0);
                *entry = (*entry).max(regret);
            }
        }
    }
    let invariant_optimal_set = if invariant {
        common_top.unwrap_or_default()
    } else {
        Vec::new()
    };
    let unique_optimum = if invariant_optimal_set.len() == 1 {
        Some(invariant_optimal_set[0].clone())
    } else {
        None
    };
    let minimum_regret = worst_regret.values().copied().min();
    let minimax_regret_actions = minimum_regret.map_or_else(Vec::new, |minimum| {
        worst_regret
            .iter()
            .filter(|(_, regret)| **regret == minimum)
            .map(|(action, _)| action.clone())
            .collect()
    });
    Signature {
        robustly_safe_actions,
        invariant_optimal_set,
        unique_optimum,
        minimax_regret_actions,
    }
}

fn apply_subset(
    problem: &Problem,
    mask: u64,
) -> (
    BTreeMap<(String, String), (i128, i128)>,
    u128,
    Vec<String>,
    Vec<String>,
) {
    let mut deltas = BTreeMap::<(String, String), (i128, i128)>::new();
    let mut total_cost = 0_u128;
    let mut mutation_ids = Vec::new();
    let mut evidence_refs = Vec::new();
    for (index, mutation) in problem.mutations.iter().enumerate() {
        if mask & (1_u64 << index) == 0 {
            continue;
        }
        let value = deltas
            .entry((mutation.world_id.clone(), mutation.action_id.clone()))
            .or_insert((0, 0));
        match mutation.kind {
            MutationKind::DecisionValueDelta => value.0 += i128::from(mutation.delta),
            MutationKind::SafetyValueDelta => value.1 += i128::from(mutation.delta),
        }
        total_cost += u128::from(mutation.cost);
        mutation_ids.push(mutation.id.clone());
        evidence_refs.push(mutation.evidence_ref.clone());
    }
    (deltas, total_cost, mutation_ids, evidence_refs)
}

fn signature_value(signature: &Signature, field: &str) -> Option<String> {
    match field {
        "robustly_safe_actions" => Some(signature.robustly_safe_actions.join(",")),
        "invariant_optimal_set" => Some(signature.invariant_optimal_set.join(",")),
        "unique_optimum" => Some(signature.unique_optimum.as_deref().unwrap_or("NONE").to_owned()),
        "minimax_regret_actions" => Some(signature.minimax_regret_actions.join(",")),
        _ => None,
    }
}

fn verify_report(problem: &Problem, report_bytes: &[u8]) -> Result<(u64, u128, u64), VerifyError> {
    let source = std::str::from_utf8(report_bytes).map_err(|_| invalid("report is not UTF-8"))?;
    let mut fields = BTreeMap::<String, String>::new();
    for (index, line) in source.lines().enumerate() {
        let Some((key, value)) = line.split_once('=') else {
            return Err(invalid(format!("report line {} is not key=value", index + 1)));
        };
        if key.is_empty() || fields.insert(key.to_owned(), value.to_owned()).is_some() {
            return Err(invalid(format!("duplicate or empty report key at line {}", index + 1)));
        }
    }
    if required(&fields, "analysis_id")? != problem.analysis_id
        || required(&fields, "input_snapshot_id")? != problem.snapshot_id
        || required(&fields, "decision_property")? != problem.property
        || required(&fields, "target_action")? != problem.target_action
        || required(&fields, "cost_unit")? != problem.cost_unit
        || required(&fields, "search_status")? != "DEFEATER_FOUND"
        || required(&fields, "minimality_status")? != "PROVEN_MINIMAL"
    {
        return Err(VerifyError::new(
            Disposition::MinimumCertificateInvalid,
            "report identity, property or complete-search claim does not match input",
        ));
    }
    if required(&fields, "source_use")? != "ASSERTED_NOT_AUTHENTICATED"
        || required(&fields, "authority")? != "ASSERTED_NOT_AUTHENTICATED"
        || required(&fields, "perturbation_admissibility")? != "ASSERTED_NOT_AUTHENTICATED"
        || required(&fields, "causal_truth")? != "NOT_ESTABLISHED"
        || required(&fields, "physical_realizability")? != "NOT_ESTABLISHED"
    {
        return Err(VerifyError::new(
            Disposition::MinimumCertificateInvalid,
            "report claim-boundary fields are missing or inconsistent",
        ));
    }

    let total_subsets = 1_u64 << problem.mutations.len();
    let reported_total = parse_u64(required(&fields, "total_subsets")?, "total_subsets")?;
    let evaluated = parse_u64(required(&fields, "evaluated_subsets")?, "evaluated_subsets")?;
    if reported_total != total_subsets || evaluated != total_subsets {
        return Err(VerifyError::new(
            Disposition::MinimumCertificateInvalid,
            "producer did not report a complete exhaustive subset search",
        ));
    }

    let baseline = compute_signature(problem, &BTreeMap::new());
    verify_signature_fields(&fields, "baseline", &baseline)?;
    let mut minimum_cost: Option<u128> = None;
    let mut expected_witnesses = Vec::<VerifiedWitness>::new();
    for mask in 0..total_subsets {
        let (deltas, cost, ids, evidence_refs) = apply_subset(problem, mask);
        let signature = compute_signature(problem, &deltas);
        let still_retained = signature
            .robustly_safe_actions
            .iter()
            .any(|action| action == &problem.target_action);
        if still_retained {
            continue;
        }
        match minimum_cost {
            None => {
                minimum_cost = Some(cost);
                expected_witnesses.clear();
                expected_witnesses.push(VerifiedWitness {
                    mutation_ids: ids,
                    evidence_refs,
                    cost,
                    mutated: signature,
                });
            }
            Some(current) if cost < current => {
                minimum_cost = Some(cost);
                expected_witnesses.clear();
                expected_witnesses.push(VerifiedWitness {
                    mutation_ids: ids,
                    evidence_refs,
                    cost,
                    mutated: signature,
                });
            }
            Some(current) if cost == current => expected_witnesses.push(VerifiedWitness {
                mutation_ids: ids,
                evidence_refs,
                cost,
                mutated: signature,
            }),
            Some(_) => {}
        }
    }

    expected_witnesses.sort_by_key(|witness| witness.mutation_ids.clone());
    let Some(minimum_cost) = minimum_cost else {
        return Err(VerifyError::new(
            Disposition::WitnessReplayMismatch,
            "report claims a defeater but independent enumeration found none",
        ));
    };
    if required(&fields, "robustness_margin")? != minimum_cost.to_string()
        || required(&fields, "best_witness_cost_upper_bound")? != minimum_cost.to_string()
        || parse_u64(required(&fields, "minimum_witness_count")?, "minimum_witness_count")?
            != expected_witnesses.len() as u64
    {
        return Err(VerifyError::new(
            Disposition::MinimumCertificateInvalid,
            "reported margin or minimum tie count differs from independent exhaustive result",
        ));
    }

    let truncated = match required(&fields, "witnesses_truncated")? {
        "true" => true,
        "false" => false,
        _ => return Err(invalid("witnesses_truncated must be true or false")),
    };
    let mut reported_indices = BTreeSet::new();
    for (key, value) in &fields {
        let Some(rest) = key.strip_prefix("witness.") else {
            continue;
        };
        let Some((index, property)) = rest.split_once('.') else {
            return Err(invalid("malformed witness report key"));
        };
        if index.len() != 4 || !index.bytes().all(|byte| byte.is_ascii_digit()) {
            return Err(invalid("malformed witness index"));
        }
        let parsed_index = index
            .parse::<usize>()
            .map_err(|error| invalid(format!("invalid witness index: {error}")))?;
        reported_indices.insert(parsed_index);
        let expected = expected_witnesses
            .get(parsed_index)
            .ok_or_else(|| {
                VerifyError::new(
                    Disposition::WitnessReplayMismatch,
                    "report includes a witness index absent from the independent minimum set",
                )
            })?;
        let prefix = format!("witness.{index}");
        match property {
            "mutation_ids" => {
                if value != &expected.mutation_ids.join(",") {
                    return Err(VerifyError::new(
                        Disposition::WitnessReplayMismatch,
                        "reported mutation identifiers do not match independent witness",
                    ));
                }
            }
            "evidence_refs" => {
                if value != &expected.evidence_refs.join(",") {
                    return Err(VerifyError::new(
                        Disposition::WitnessReplayMismatch,
                        "reported witness evidence references do not match input mutations",
                    ));
                }
            }
            "total_cost" => {
                if value != &expected.cost.to_string() {
                    return Err(VerifyError::new(
                        Disposition::WitnessReplayMismatch,
                        "reported witness cost does not match independent cost calculation",
                    ));
                }
            }
            "failed_property" => {
                if value != &problem.property {
                    return Err(VerifyError::new(
                        Disposition::WitnessReplayMismatch,
                        "reported failed property does not match input",
                    ));
                }
            }
            _ => {
                let Some((scope, signature_field)) = property.split_once('.') else {
                    return Err(invalid("unknown witness report field"));
                };
                let signature = match scope {
                    "baseline" => &baseline,
                    "mutated" => &expected.mutated,
                    _ => return Err(invalid("unknown witness signature scope")),
                };
                let expected_value = signature_value(signature, signature_field)
                    .ok_or_else(|| invalid("unknown decision signature field"))?;
                if value != &expected_value {
                    return Err(VerifyError::new(
                        Disposition::WitnessReplayMismatch,
                        "reported before/after decision signature differs from independent calculation",
                    ));
                }
            }
        }
        let _ = prefix;
    }
    if reported_indices.is_empty() || reported_indices.len() > expected_witnesses.len() {
        return Err(VerifyError::new(
            Disposition::WitnessReplayMismatch,
            "report contains no valid witnesses or too many witnesses",
        ));
    }
    if !truncated && reported_indices.len() != expected_witnesses.len() {
        return Err(VerifyError::new(
            Disposition::MinimumCertificateInvalid,
            "report omits tied minimum witnesses without declaring truncation",
        ));
    }
    if truncated && reported_indices.len() == expected_witnesses.len() {
        return Err(invalid("witnesses_truncated=true but no witness is omitted"));
    }
    if required(&fields, "minimum_witness_count")? != expected_witnesses.len().to_string() {
        return Err(VerifyError::new(
            Disposition::MinimumCertificateInvalid,
            "minimum witness count mismatch",
        ));
    }
    Ok((evaluated, minimum_cost, expected_witnesses.len() as u64))
}

fn verify_signature_fields(
    fields: &BTreeMap<String, String>,
    prefix: &str,
    signature: &Signature,
) -> Result<(), VerifyError> {
    for field in [
        "robustly_safe_actions",
        "invariant_optimal_set",
        "unique_optimum",
        "minimax_regret_actions",
    ] {
        let key = format!("{prefix}.{field}");
        let value = fields
            .get(&key)
            .ok_or_else(|| invalid(format!("missing signature field {key}")))?;
        if Some(value.as_str()) != signature_value(signature, field).as_deref() {
            return Err(VerifyError::new(
                Disposition::WitnessReplayMismatch,
                format!("baseline signature field {field} differs"),
            ));
        }
    }
    Ok(())
}

/// Verify a complete E0 package using an implementation independent of the
/// decision-defeater producer crate.
pub fn verify_package(root: impl AsRef<Path>) -> Result<VerificationReceipt, VerifyError> {
    let root = root.as_ref();
    let root_metadata = fs::symlink_metadata(root).map_err(|error| {
        VerifyError::new(
            Disposition::Indeterminate,
            format!("cannot stat package directory: {error}"),
        )
    })?;
    if !root_metadata.file_type().is_dir() {
        return Err(invalid("package root must be a real directory, not a symlink"));
    }

    let manifest_bytes = read_regular_file(root, MANIFEST_NAME, MAX_MANIFEST_BYTES)?;
    let manifest = parse_manifest(&manifest_bytes)?;
    let manifest_sha256 = hash_artifact("MANIFEST", &manifest_bytes);

    let input_bytes = read_regular_file(root, &manifest.input_filename, MAX_INPUT_BYTES)?;
    let input_sha256 = hash_artifact("INPUT", &input_bytes);
    if format!("sha256:{input_sha256}") != manifest.input_digest {
        return Err(VerifyError::new(
            Disposition::DigestMismatch,
            "input bytes do not match manifest digest",
        ));
    }
    let report_bytes = read_regular_file(root, &manifest.report_filename, MAX_REPORT_BYTES)?;
    let report_sha256 = hash_artifact("REPORT", &report_bytes);
    if format!("sha256:{report_sha256}") != manifest.report_digest {
        return Err(VerifyError::new(
            Disposition::DigestMismatch,
            "report bytes do not match manifest digest",
        ));
    }

    let mut evidence_refs = BTreeSet::new();
    for entry in &manifest.evidence {
        let bytes = read_regular_file(root, &entry.filename, MAX_EVIDENCE_BYTES)?;
        let digest = hash_artifact("EVIDENCE", &bytes);
        if format!("sha256:{digest}") != entry.digest {
            return Err(VerifyError::new(
                Disposition::DigestMismatch,
                format!("evidence object {} does not match manifest digest", entry.reference),
            ));
        }
        evidence_refs.insert(entry.reference.clone());
        let _ = &entry.role;
    }

    let problem = parse_problem(&input_bytes)?;
    if problem.all_evidence_refs != evidence_refs {
        return Err(VerifyError::new(
            Disposition::EvidenceReferenceUnresolved,
            "manifest evidence reference set differs from input mutation references",
        ));
    }
    let (evaluated_subsets, verified_minimum_cost, verified_witness_count) =
        verify_report(&problem, &report_bytes)?;

    Ok(VerificationReceipt {
        disposition: Disposition::VerifiedWitnessAndMinimum,
        manifest_sha256,
        input_sha256,
        report_sha256,
        evidence_object_count: manifest.evidence.len(),
        mutation_count: problem.mutations.len() + problem.blocked_mutations,
        evaluated_subsets,
        verified_minimum_cost,
        verified_witness_count,
        analysis_id: problem.analysis_id,
        input_snapshot_id: problem.snapshot_id,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicUsize, Ordering};

    const FIXTURE: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/fixtures/pump-station");
    static NEXT_DIR: AtomicUsize = AtomicUsize::new(0);

    fn temp_copy() -> PathBuf {
        let id = NEXT_DIR.fetch_add(1, Ordering::Relaxed);
        let root = std::env::temp_dir().join(format!(
            "fsre-verifier-test-{}-{id}",
            std::process::id()
        ));
        fs::create_dir_all(&root).unwrap();
        for name in [
            "manifest.psv",
            "pump-station.psv",
            "report.txt",
            "evidence-a.txt",
            "evidence-b.txt",
            "evidence-noop.txt",
        ] {
            fs::copy(Path::new(FIXTURE).join(name), root.join(name)).unwrap();
        }
        root
    }

    fn rebuild_manifest(root: &Path) {
        let input = fs::read(root.join("pump-station.psv")).unwrap();
        let report = fs::read(root.join("report.txt")).unwrap();
        let mut refs = Vec::new();
        for name in ["evidence-a", "evidence-b", "evidence-noop"] {
            let file = format!("{name}.txt");
            let bytes = fs::read(root.join(&file)).unwrap();
            refs.push(format!(
                "evidence|{name}|SYNTHETIC_FIXTURE_ASSERTION|{file}|sha256:{}",
                hash_artifact("EVIDENCE", &bytes)
            ));
        }
        let mut manifest = format!(
            "{MANIFEST_HEADER}\ninput|pump-station.psv|sha256:{}\nreport|report.txt|sha256:{}\n",
            hash_artifact("INPUT", &input),
            hash_artifact("REPORT", &report)
        );
        for row in refs {
            manifest.push_str(&row);
            manifest.push('\n');
        }
        fs::write(root.join(MANIFEST_NAME), manifest).unwrap();
    }

    #[test]
    fn sha256_primitive_matches_published_known_answer() {
        assert_eq!(
            hex_lower(&Sha256::digest(b"abc")),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
    }

    #[test]
    fn digest_domain_separation_prevents_cross_type_reuse() {
        assert_ne!(hash_artifact("INPUT", b"abc"), hash_artifact("REPORT", b"abc"));
        assert_ne!(hash_artifact("INPUT", b"abc"), hash_artifact("EVIDENCE", b"abc"));
    }

    #[test]
    fn frozen_fixture_verifies_witness_and_global_minimum_independently() {
        let receipt = verify_package(FIXTURE).unwrap();
        assert_eq!(receipt.disposition, Disposition::VerifiedWitnessAndMinimum);
        assert_eq!(receipt.evaluated_subsets, 8);
        assert_eq!(receipt.verified_minimum_cost, 2);
        assert_eq!(receipt.verified_witness_count, 2);
        assert_eq!(receipt.evidence_object_count, 3);
    }

    #[test]
    fn changing_input_without_updating_manifest_is_detected() {
        let root = temp_copy();
        let path = root.join("pump-station.psv");
        let mut input = fs::read(&path).unwrap();
        input.extend_from_slice(b"\n");
        fs::write(path, input).unwrap();
        let error = verify_package(&root).unwrap_err();
        assert_eq!(error.disposition, Disposition::DigestMismatch);
        fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn changing_evidence_without_updating_manifest_is_detected() {
        let root = temp_copy();
        fs::write(root.join("evidence-a.txt"), b"altered fixture evidence\n").unwrap();
        let error = verify_package(&root).unwrap_err();
        assert_eq!(error.disposition, Disposition::DigestMismatch);
        fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn a_valid_hash_does_not_rescue_a_false_minimum_claim() {
        let root = temp_copy();
        let report_path = root.join("report.txt");
        let report = fs::read_to_string(&report_path)
            .unwrap()
            .replace("robustness_margin=2", "robustness_margin=3");
        fs::write(report_path, report).unwrap();
        rebuild_manifest(&root);
        let error = verify_package(&root).unwrap_err();
        assert_eq!(error.disposition, Disposition::MinimumCertificateInvalid);
        fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn unknown_authority_remains_a_hard_gate_even_after_rehashing() {
        let root = temp_copy();
        let input_path = root.join("pump-station.psv");
        let input = fs::read_to_string(&input_path)
            .unwrap()
            .replace("authority_status=ESTABLISHED", "authority_status=UNKNOWN");
        fs::write(input_path, input).unwrap();
        rebuild_manifest(&root);
        let error = verify_package(&root).unwrap_err();
        assert_eq!(error.disposition, Disposition::AuthorityBlockedOrUnknown);
        fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn unknown_source_use_remains_a_hard_gate_even_after_rehashing() {
        let root = temp_copy();
        let input_path = root.join("pump-station.psv");
        let input = fs::read_to_string(&input_path)
            .unwrap()
            .replace("source_use_status=AUTHORIZED", "source_use_status=UNKNOWN");
        fs::write(input_path, input).unwrap();
        rebuild_manifest(&root);
        let error = verify_package(&root).unwrap_err();
        assert_eq!(error.disposition, Disposition::SourceUseBlockedOrUnknown);
        fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn report_signature_tampering_is_detected_after_valid_rehash() {
        let root = temp_copy();
        let report_path = root.join("report.txt");
        let report = fs::read_to_string(&report_path)
            .unwrap()
            .replace("witness.0000.mutated.robustly_safe_actions=inspect", "witness.0000.mutated.robustly_safe_actions=repair");
        fs::write(report_path, report).unwrap();
        rebuild_manifest(&root);
        let error = verify_package(&root).unwrap_err();
        assert_eq!(error.disposition, Disposition::WitnessReplayMismatch);
        fs::remove_dir_all(root).unwrap();
    }

    #[test]
    fn unsafe_manifest_paths_are_rejected() {
        let root = temp_copy();
        let manifest_path = root.join(MANIFEST_NAME);
        let manifest = fs::read_to_string(&manifest_path)
            .unwrap()
            .replace("input|pump-station.psv|", "input|../pump-station.psv|");
        fs::write(manifest_path, manifest).unwrap();
        let error = verify_package(&root).unwrap_err();
        assert_eq!(error.disposition, Disposition::SchemaUnsupported);
        fs::remove_dir_all(root).unwrap();
    }
}
