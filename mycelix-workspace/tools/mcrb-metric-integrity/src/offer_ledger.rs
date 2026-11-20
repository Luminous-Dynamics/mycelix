//! Offer-first workload ledger verification for MCRB.
//!
//! The denominator is frozen before dispatch. This module verifies internal
//! ledger structure and a domain-separated SHA-256 digest; it does not prove
//! who generated or externally observed the ledger. Runner/orchestrator
//! authenticity remains a separate receipt-signature and custody requirement.

use sha2::{Digest, Sha256};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use crate::operation_contract::{
    OperationOutcomeClass, ValidatedOperationContractRegistry,
};

pub const OFFER_LEDGER_SCHEMA_VERSION: &str = "mycelix-mcrb-offer-ledger-v0.1";
const DOMAIN_SEPARATOR: &[u8] = b"MYCELIX-MCRB-OFFER-LEDGER\0v0.1\0";
const EMBEDDED_SCENARIO_MANIFEST: &str =
    include_str!("../../../../docs/benchmarks/MCRB_V0_1_SCENARIO_MANIFEST.json");

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct OfferLedgerEntry {
    pub sequence: u64,
    pub run_id: String,
    pub scenario_id: String,
    pub operation_id: String,
    pub request_digest_sha256: String,
    pub idempotency_key_digest_sha256: Option<String>,
    pub scheduled_monotonic_tick: u64,
    pub monotonic_clock_id: String,
    pub monotonic_clock_epoch: String,
    pub operation_class: String,
    pub operation_contract_id: String,
    pub operation_contract_version: String,
    pub operation_contract_digest_sha256: String,
    pub requires_idempotency_key: bool,
    pub generator_digest_sha256: String,
    pub workload_manifest_digest_sha256: String,
    pub parameter_set_digest_sha256: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct OfferLedgerEnvelope {
    pub schema_version: String,
    pub run_id: String,
    pub scenario_id: String,
    pub measurement_window_id: String,
    pub scenario_contract_digest_sha256: String,
    pub operation_contract_registry_digest_sha256: String,
    pub requires_cross_host_visibility: bool,
    pub workload_manifest_digest_sha256: String,
    pub generator_digest_sha256: String,
    pub parameter_set_digest_sha256: String,
    pub clock_source: String,
    pub monotonic_clock_id: String,
    pub monotonic_clock_epoch: String,
    pub measurement_start_ticks: u64,
    pub measurement_end_ticks: u64,
    pub ticks_per_second: u64,
    pub entry_count: u64,
    pub entries: Vec<OfferLedgerEntry>,
    pub ledger_digest_sha256: String,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum OfferLedgerIssue {
    UnsupportedSchemaVersion,
    EmptyRunId,
    EmptyScenarioId,
    EmptyMeasurementWindowId,
    EmptyClockSource,
    EmptyMonotonicClockId,
    EmptyMonotonicClockEpoch,
    InvalidWorkloadManifestDigest,
    InvalidGeneratorDigest,
    InvalidParameterSetDigest,
    InvalidScenarioContractDigest,
    ScenarioContractDigestMismatch { expected: String, actual: String },
    EmbeddedScenarioManifestInvalid,
    UnknownScenarioInEmbeddedManifest(String),
    ScenarioVisibilityPolicyMismatch { scenario_id: String, expected: bool, actual: bool },
    InvalidWindowBounds,
    InvalidTickFrequency,
    WindowDurationOverflow,
    EmptyOfferLedger,
    EntryCountMismatch,
    InvalidSequence { index: usize, expected: u64, actual: u64 },
    EmptyOperationId(usize),
    DuplicateOperationId(String),
    InvalidRequestDigest(usize),
    InvalidIdempotencyKeyDigest(usize),
    MissingIdempotencyKeyForExternalEffect(usize),
    DuplicateIdempotencyKey(String),
    EmptyOperationClass(usize),
    InvalidOperationContractRegistryDigest,
    OperationContractRegistryDigestMismatch { expected: String, actual: String },
    MissingOperationContractIdentity(usize),
    UnknownOperationContract(usize),
    OperationContractContextMismatch(usize),
    OperationContractDigestMismatch(usize),
    OperationContractRequiresCrossHostVisibility(usize),
    OperationContractIdempotencyMismatch(usize),
    EntryRunIdMismatch(usize),
    EntryScenarioIdMismatch(usize),
    EntryClockIdMismatch(usize),
    EntryClockEpochMismatch(usize),
    EntryGeneratorDigestMismatch(usize),
    EntryManifestDigestMismatch(usize),
    EntryParameterDigestMismatch(usize),
    OfferOutsideMeasurementWindow(usize),
    InvalidLedgerDigest,
    LedgerDigestMismatch { expected: String, actual: String },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VerifiedOfferLedger {
    run_id: String,
    scenario_id: String,
    measurement_window_id: String,
    scenario_contract_digest_sha256: String,
    operation_contract_registry_digest_sha256: String,
    requires_cross_host_visibility: bool,
    generator_digest_sha256: String,
    workload_manifest_digest_sha256: String,
    parameter_set_digest_sha256: String,
    monotonic_clock_id: String,
    monotonic_clock_epoch: String,
    measurement_start_ticks: u64,
    measurement_end_ticks: u64,
    ticks_per_second: u64,
    measurement_window_ns: u64,
    ledger_digest_sha256: String,
    operation_ids: Vec<String>,
    offered_operations: Vec<VerifiedOfferedOperation>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VerifiedOfferedOperation {
    sequence: u64,
    operation_id: String,
    request_digest_sha256: String,
    idempotency_key_digest_sha256: Option<String>,
    contract_id: String,
    contract_version: String,
    contract_digest_sha256: String,
    expected_qops_outcome: OperationOutcomeClass,
    contract_profile: ValidatedOperationContract,
}

impl VerifiedOfferedOperation {
    pub fn sequence(&self) -> u64 { self.sequence }
    pub fn operation_id(&self) -> &str { &self.operation_id }
    pub fn request_digest_sha256(&self) -> &str { &self.request_digest_sha256 }
    pub fn idempotency_key_digest_sha256(&self) -> Option<&str> { self.idempotency_key_digest_sha256.as_deref() }
    pub fn contract_id(&self) -> &str { &self.contract_id }
    pub fn contract_version(&self) -> &str { &self.contract_version }
    pub fn contract_digest_sha256(&self) -> &str { &self.contract_digest_sha256 }
    pub fn expected_qops_outcome(&self) -> OperationOutcomeClass { self.expected_qops_outcome }
    /// The exact validated pre-dispatch profile bound by this offer.
    /// The profile is immutable and cannot be replaced by a later registry read.
    pub fn contract_profile(&self) -> &ValidatedOperationContract { &self.contract_profile }
}

impl VerifiedOfferLedger {
    pub fn run_id(&self) -> &str { &self.run_id }
    pub fn scenario_id(&self) -> &str { &self.scenario_id }
    pub fn measurement_window_id(&self) -> &str { &self.measurement_window_id }
    pub fn scenario_contract_digest_sha256(&self) -> &str { &self.scenario_contract_digest_sha256 }
    pub fn operation_contract_registry_digest_sha256(&self) -> &str { &self.operation_contract_registry_digest_sha256 }
    pub fn requires_cross_host_visibility(&self) -> bool { self.requires_cross_host_visibility }
    pub fn offered_operations(&self) -> &[VerifiedOfferedOperation] { &self.offered_operations }
    pub fn offered_operation(&self, operation_id: &str) -> Option<&VerifiedOfferedOperation> {
        self.offered_operations.iter().find(|operation| operation.operation_id == operation_id)
    }
    pub fn generator_digest_sha256(&self) -> &str { &self.generator_digest_sha256 }
    pub fn workload_manifest_digest_sha256(&self) -> &str { &self.workload_manifest_digest_sha256 }
    pub fn parameter_set_digest_sha256(&self) -> &str { &self.parameter_set_digest_sha256 }
    pub fn monotonic_clock_id(&self) -> &str { &self.monotonic_clock_id }
    pub fn monotonic_clock_epoch(&self) -> &str { &self.monotonic_clock_epoch }
    pub fn measurement_start_ticks(&self) -> u64 { self.measurement_start_ticks }
    pub fn measurement_end_ticks(&self) -> u64 { self.measurement_end_ticks }
    pub fn ticks_per_second(&self) -> u64 { self.ticks_per_second }
    pub fn measurement_window_ns(&self) -> u64 { self.measurement_window_ns }
    pub fn ledger_digest_sha256(&self) -> &str { &self.ledger_digest_sha256 }
    pub fn entry_count(&self) -> usize { self.operation_ids.len() }
    pub fn operation_ids(&self) -> &[String] { &self.operation_ids }
}

fn valid_sha256_hex(value: &str) -> bool {
    value.len() == 64
        && value.bytes().all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
}
fn push_u64(bytes: &mut Vec<u8>, value: u64) { bytes.extend_from_slice(&value.to_be_bytes()); }
fn push_bytes(bytes: &mut Vec<u8>, value: &[u8]) {
    push_u64(bytes, value.len() as u64);
    bytes.extend_from_slice(value);
}
fn push_string(bytes: &mut Vec<u8>, value: &str) { push_bytes(bytes, value.as_bytes()); }
fn push_optional_string(bytes: &mut Vec<u8>, value: Option<&str>) {
    match value {
        Some(value) => { bytes.push(1); push_string(bytes, value); }
        None => bytes.push(0),
    }
}

const IDEMPOTENCY_KEY_COMMITMENT_DOMAIN: &[u8] =
    b"MYCELIX-MCRB-IDEMPOTENCY-KEY\0v0.1\0";

/// Create the public commitment for an idempotency key without exposing the
/// raw key in offer-ledger artifacts. The caller retains the raw key for the
/// execution adapter; only this digest should cross the benchmark boundary.
pub fn idempotency_key_commitment(raw_key: &[u8]) -> String {
    let mut material = IDEMPOTENCY_KEY_COMMITMENT_DOMAIN.to_vec();
    push_bytes(&mut material, raw_key);
    let digest = Sha256::digest(material);
    let mut hex = String::with_capacity(64);
    for byte in digest {
        use std::fmt::Write as _;
        let _ = write!(&mut hex, "{byte:02x}");
    }
    hex
}

fn canonical_offer_ledger_bytes(envelope: &OfferLedgerEnvelope) -> Vec<u8> {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(DOMAIN_SEPARATOR);
    push_string(&mut bytes, &envelope.schema_version);
    push_string(&mut bytes, &envelope.run_id);
    push_string(&mut bytes, &envelope.scenario_id);
    push_string(&mut bytes, &envelope.measurement_window_id);
    push_string(&mut bytes, &envelope.scenario_contract_digest_sha256);
    push_string(&mut bytes, &envelope.operation_contract_registry_digest_sha256);
    bytes.push(u8::from(envelope.requires_cross_host_visibility));
    push_string(&mut bytes, &envelope.workload_manifest_digest_sha256);
    push_string(&mut bytes, &envelope.generator_digest_sha256);
    push_string(&mut bytes, &envelope.parameter_set_digest_sha256);
    push_string(&mut bytes, &envelope.clock_source);
    push_string(&mut bytes, &envelope.monotonic_clock_id);
    push_string(&mut bytes, &envelope.monotonic_clock_epoch);
    push_u64(&mut bytes, envelope.measurement_start_ticks);
    push_u64(&mut bytes, envelope.measurement_end_ticks);
    push_u64(&mut bytes, envelope.ticks_per_second);
    push_u64(&mut bytes, envelope.entry_count);

    // Entry order is committed and independently checked; never sort this list.
    for entry in &envelope.entries {
        push_u64(&mut bytes, entry.sequence);
        push_string(&mut bytes, &entry.run_id);
        push_string(&mut bytes, &entry.scenario_id);
        push_string(&mut bytes, &entry.operation_id);
        push_string(&mut bytes, &entry.request_digest_sha256);
        push_optional_string(&mut bytes, entry.idempotency_key_digest_sha256.as_deref());
        push_u64(&mut bytes, entry.scheduled_monotonic_tick);
        push_string(&mut bytes, &entry.monotonic_clock_id);
        push_string(&mut bytes, &entry.monotonic_clock_epoch);
        push_string(&mut bytes, &entry.operation_class);
        push_string(&mut bytes, &entry.operation_contract_id);
        push_string(&mut bytes, &entry.operation_contract_version);
        push_string(&mut bytes, &entry.operation_contract_digest_sha256);
        bytes.push(u8::from(entry.requires_idempotency_key));
        push_string(&mut bytes, &entry.generator_digest_sha256);
        push_string(&mut bytes, &entry.workload_manifest_digest_sha256);
        push_string(&mut bytes, &entry.parameter_set_digest_sha256);
    }
    bytes
}


pub fn embedded_scenario_manifest_digest() -> String {
    let digest = Sha256::digest(EMBEDDED_SCENARIO_MANIFEST.as_bytes());
    let mut hex = String::with_capacity(64);
    for byte in digest {
        use std::fmt::Write as _;
        let _ = write!(&mut hex, "{byte:02x}");
    }
    hex
}

fn expected_scenario_visibility(scenario_id: &str) -> Result<bool, OfferLedgerIssue> {
    let manifest: serde_json::Value = serde_json::from_str(EMBEDDED_SCENARIO_MANIFEST)
        .map_err(|_| OfferLedgerIssue::EmbeddedScenarioManifestInvalid)?;
    let scenarios = manifest["scenarios"]
        .as_array()
        .ok_or(OfferLedgerIssue::EmbeddedScenarioManifestInvalid)?;
    let scenario = scenarios
        .iter()
        .find(|scenario| scenario["id"].as_str() == Some(scenario_id))
        .ok_or_else(|| OfferLedgerIssue::UnknownScenarioInEmbeddedManifest(scenario_id.to_string()))?;
    scenario["qops_requires_cross_host_visibility"]
        .as_bool()
        .ok_or(OfferLedgerIssue::EmbeddedScenarioManifestInvalid)
}

/// Compute the canonical, domain-separated SHA-256 digest for an offer ledger.
/// This proves internal consistency, not generator/orchestrator authenticity.
pub fn calculate_offer_ledger_digest(envelope: &OfferLedgerEnvelope) -> String {
    let digest = Sha256::digest(canonical_offer_ledger_bytes(envelope));
    let mut hex = String::with_capacity(64);
    for byte in digest {
        use std::fmt::Write as _;
        let _ = write!(&mut hex, "{byte:02x}");
    }
    hex
}

/// Verify the offer schedule and produce an opaque value accepted by QOPS.
/// Every offer is bound to one run, scenario, manifest, generator, parameter
/// set, clock identity/epoch and predeclared monotonic measurement window.
pub fn verify_offer_ledger(
    envelope: &OfferLedgerEnvelope,
    contracts: &ValidatedOperationContractRegistry,
) -> Result<VerifiedOfferLedger, Vec<OfferLedgerIssue>> {
    let mut issues = Vec::new();
    if envelope.schema_version != OFFER_LEDGER_SCHEMA_VERSION { issues.push(OfferLedgerIssue::UnsupportedSchemaVersion); }
    if envelope.run_id.trim().is_empty() { issues.push(OfferLedgerIssue::EmptyRunId); }
    if envelope.scenario_id.trim().is_empty() { issues.push(OfferLedgerIssue::EmptyScenarioId); }
    if envelope.measurement_window_id.trim().is_empty() { issues.push(OfferLedgerIssue::EmptyMeasurementWindowId); }
    if envelope.clock_source.trim().is_empty() { issues.push(OfferLedgerIssue::EmptyClockSource); }
    if envelope.monotonic_clock_id.trim().is_empty() { issues.push(OfferLedgerIssue::EmptyMonotonicClockId); }
    if envelope.monotonic_clock_epoch.trim().is_empty() { issues.push(OfferLedgerIssue::EmptyMonotonicClockEpoch); }
    if !valid_sha256_hex(&envelope.workload_manifest_digest_sha256) { issues.push(OfferLedgerIssue::InvalidWorkloadManifestDigest); }
    if !valid_sha256_hex(&envelope.generator_digest_sha256) { issues.push(OfferLedgerIssue::InvalidGeneratorDigest); }
    if !valid_sha256_hex(&envelope.parameter_set_digest_sha256) { issues.push(OfferLedgerIssue::InvalidParameterSetDigest); }
    if !valid_sha256_hex(&envelope.scenario_contract_digest_sha256) { issues.push(OfferLedgerIssue::InvalidScenarioContractDigest); }
    if !valid_sha256_hex(&envelope.operation_contract_registry_digest_sha256) {
        issues.push(OfferLedgerIssue::InvalidOperationContractRegistryDigest);
    }
    if envelope.operation_contract_registry_digest_sha256 != contracts.registry_digest_sha256() {
        issues.push(OfferLedgerIssue::OperationContractRegistryDigestMismatch {
            expected: contracts.registry_digest_sha256().to_string(),
            actual: envelope.operation_contract_registry_digest_sha256.clone(),
        });
    }
    let expected_contract_digest = embedded_scenario_manifest_digest();
    if envelope.scenario_contract_digest_sha256 != expected_contract_digest {
        issues.push(OfferLedgerIssue::ScenarioContractDigestMismatch {
            expected: expected_contract_digest,
            actual: envelope.scenario_contract_digest_sha256.clone(),
        });
    }
    match expected_scenario_visibility(&envelope.scenario_id) {
        Ok(expected) if expected != envelope.requires_cross_host_visibility => {
            issues.push(OfferLedgerIssue::ScenarioVisibilityPolicyMismatch {
                scenario_id: envelope.scenario_id.clone(),
                expected,
                actual: envelope.requires_cross_host_visibility,
            });
        }
        Ok(_) => {}
        Err(issue) => issues.push(issue),
    }
    if envelope.measurement_end_ticks <= envelope.measurement_start_ticks { issues.push(OfferLedgerIssue::InvalidWindowBounds); }
    if envelope.ticks_per_second == 0 { issues.push(OfferLedgerIssue::InvalidTickFrequency); }
    if envelope.entries.is_empty() { issues.push(OfferLedgerIssue::EmptyOfferLedger); }
    if envelope.entry_count != envelope.entries.len() as u64 { issues.push(OfferLedgerIssue::EntryCountMismatch); }

    let mut operation_ids = Vec::with_capacity(envelope.entries.len());
    let mut offered_operations = Vec::with_capacity(envelope.entries.len());
    let mut seen_operations = BTreeSet::new();
    let mut seen_idempotency_keys = BTreeSet::new();

    for (index, entry) in envelope.entries.iter().enumerate() {
        let expected_sequence = index as u64 + 1;
        if entry.sequence != expected_sequence {
            issues.push(OfferLedgerIssue::InvalidSequence { index, expected: expected_sequence, actual: entry.sequence });
        }
        if entry.operation_id.trim().is_empty() {
            issues.push(OfferLedgerIssue::EmptyOperationId(index));
        } else if !seen_operations.insert(entry.operation_id.clone()) {
            issues.push(OfferLedgerIssue::DuplicateOperationId(entry.operation_id.clone()));
        } else {
            operation_ids.push(entry.operation_id.clone());
        }
        if !valid_sha256_hex(&entry.request_digest_sha256) { issues.push(OfferLedgerIssue::InvalidRequestDigest(index)); }
        if entry.requires_idempotency_key && entry.idempotency_key_digest_sha256.is_none() {
            issues.push(OfferLedgerIssue::MissingIdempotencyKeyForExternalEffect(index));
        }
        if let Some(key_digest) = entry.idempotency_key_digest_sha256.as_deref() {
            if !valid_sha256_hex(key_digest) {
                issues.push(OfferLedgerIssue::InvalidIdempotencyKeyDigest(index));
            } else if !seen_idempotency_keys.insert(key_digest.to_string()) {
                issues.push(OfferLedgerIssue::DuplicateIdempotencyKey(key_digest.to_string()));
            }
        }
        if entry.operation_class.trim().is_empty() { issues.push(OfferLedgerIssue::EmptyOperationClass(index)); }
        if entry.operation_contract_id.trim().is_empty()
            || entry.operation_contract_version.trim().is_empty()
        {
            issues.push(OfferLedgerIssue::MissingOperationContractIdentity(index));
        } else if let Some(contract) = contracts.find(
            &entry.operation_contract_id,
            &entry.operation_contract_version,
        ) {
            if contract.scenario_id() != envelope.scenario_id
                || contract.operation_class() != entry.operation_class
            {
                issues.push(OfferLedgerIssue::OperationContractContextMismatch(index));
            }
            if contract.digest_sha256() != entry.operation_contract_digest_sha256 {
                issues.push(OfferLedgerIssue::OperationContractDigestMismatch(index));
            }
            if contract.requires_cross_host_visibility() && !envelope.requires_cross_host_visibility {
                issues.push(OfferLedgerIssue::OperationContractRequiresCrossHostVisibility(index));
            }
            if contract.idempotency_required()
                && (!entry.requires_idempotency_key || entry.idempotency_key_digest_sha256.is_none())
            {
                issues.push(OfferLedgerIssue::OperationContractIdempotencyMismatch(index));
            }
            if !entry.operation_id.trim().is_empty() {
                offered_operations.push(VerifiedOfferedOperation {
                    sequence: entry.sequence,
                    operation_id: entry.operation_id.clone(),
                    request_digest_sha256: entry.request_digest_sha256.clone(),
                    idempotency_key_digest_sha256: entry.idempotency_key_digest_sha256.clone(),
                    contract_id: entry.operation_contract_id.clone(),
                    contract_version: entry.operation_contract_version.clone(),
                    contract_digest_sha256: entry.operation_contract_digest_sha256.clone(),
                    expected_qops_outcome: contract.expected_qops_outcome(),
                    contract_profile: contract.clone(),
                });
            }
        } else {
            issues.push(OfferLedgerIssue::UnknownOperationContract(index));
        }
        if entry.run_id != envelope.run_id { issues.push(OfferLedgerIssue::EntryRunIdMismatch(index)); }
        if entry.scenario_id != envelope.scenario_id { issues.push(OfferLedgerIssue::EntryScenarioIdMismatch(index)); }
        if entry.monotonic_clock_id != envelope.monotonic_clock_id { issues.push(OfferLedgerIssue::EntryClockIdMismatch(index)); }
        if entry.monotonic_clock_epoch != envelope.monotonic_clock_epoch { issues.push(OfferLedgerIssue::EntryClockEpochMismatch(index)); }
        if entry.generator_digest_sha256 != envelope.generator_digest_sha256 { issues.push(OfferLedgerIssue::EntryGeneratorDigestMismatch(index)); }
        if entry.workload_manifest_digest_sha256 != envelope.workload_manifest_digest_sha256 { issues.push(OfferLedgerIssue::EntryManifestDigestMismatch(index)); }
        if entry.parameter_set_digest_sha256 != envelope.parameter_set_digest_sha256 { issues.push(OfferLedgerIssue::EntryParameterDigestMismatch(index)); }
        if entry.scheduled_monotonic_tick < envelope.measurement_start_ticks
            || entry.scheduled_monotonic_tick >= envelope.measurement_end_ticks {
            issues.push(OfferLedgerIssue::OfferOutsideMeasurementWindow(index));
        }
    }

    if !valid_sha256_hex(&envelope.ledger_digest_sha256) { issues.push(OfferLedgerIssue::InvalidLedgerDigest); }
    let actual_digest = calculate_offer_ledger_digest(envelope);
    if envelope.ledger_digest_sha256 != actual_digest {
        issues.push(OfferLedgerIssue::LedgerDigestMismatch {
            expected: envelope.ledger_digest_sha256.clone(),
            actual: actual_digest,
        });
    }
    if !issues.is_empty() { return Err(issues); }

    let delta_ticks = (envelope.measurement_end_ticks - envelope.measurement_start_ticks) as u128;
    let window_ns = delta_ticks.checked_mul(1_000_000_000)
        .ok_or_else(|| vec![OfferLedgerIssue::WindowDurationOverflow])?
        / envelope.ticks_per_second as u128;
    if window_ns == 0 || window_ns > u64::MAX as u128 {
        return Err(vec![OfferLedgerIssue::WindowDurationOverflow]);
    }

    Ok(VerifiedOfferLedger {
        run_id: envelope.run_id.clone(),
        scenario_id: envelope.scenario_id.clone(),
        measurement_window_id: envelope.measurement_window_id.clone(),
        scenario_contract_digest_sha256: envelope.scenario_contract_digest_sha256.clone(),
        operation_contract_registry_digest_sha256: envelope.operation_contract_registry_digest_sha256.clone(),
        requires_cross_host_visibility: envelope.requires_cross_host_visibility,
        generator_digest_sha256: envelope.generator_digest_sha256.clone(),
        workload_manifest_digest_sha256: envelope.workload_manifest_digest_sha256.clone(),
        parameter_set_digest_sha256: envelope.parameter_set_digest_sha256.clone(),
        monotonic_clock_id: envelope.monotonic_clock_id.clone(),
        monotonic_clock_epoch: envelope.monotonic_clock_epoch.clone(),
        measurement_start_ticks: envelope.measurement_start_ticks,
        measurement_end_ticks: envelope.measurement_end_ticks,
        ticks_per_second: envelope.ticks_per_second,
        measurement_window_ns: window_ns as u64,
        ledger_digest_sha256: envelope.ledger_digest_sha256.clone(),
        operation_ids,
        offered_operations,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::operation_contract::{
        sha256_hex, validate_operation_contract_registry, ValidatedOperationContractRegistry,
    };
    const EMBEDDED_OPERATION_CONTRACTS: &str =
        include_str!("../../../../docs/benchmarks/MCRB_OPERATION_CONTRACTS_V0_1.json");
    const EXPECTED_SCENARIO_MANIFEST_GIT_BLOB_SHA: &str =
        "e3fb66b94622fd7e4a1f9367ca45ba413a30722c";

    fn embedded_contract_registry() -> ValidatedOperationContractRegistry {
        let raw = EMBEDDED_OPERATION_CONTRACTS.as_bytes();
        let digest = sha256_hex(raw);
        validate_operation_contract_registry(raw, &digest, EXPECTED_SCENARIO_MANIFEST_GIT_BLOB_SHA)
            .expect("embedded operation-contract registry should validate")
    }

    fn digest(seed: &str) -> String {
        let digest = Sha256::digest(seed.as_bytes());
        let mut hex = String::with_capacity(64);
        for byte in digest {
            use std::fmt::Write as _;
            let _ = write!(&mut hex, "{byte:02x}");
        }
        hex
    }

    fn valid_envelope() -> OfferLedgerEnvelope {
        let run_id = "run-001".to_string();
        let scenario_id = "MCRB-03".to_string();
        let generator = digest("generator");
        let manifest = digest("manifest");
        let params = digest("params");
        let clock_id = "clock-peer-a".to_string();
        let clock_epoch = "boot-epoch-7".to_string();
        let contracts = embedded_contract_registry();
        let operation_contract = contracts
            .find("MCRB-OP-REMOTE-DHT-PUBLICATION", "0.1.0")
            .expect("remote DHT contract should be present");
        let mut envelope = OfferLedgerEnvelope {
            schema_version: OFFER_LEDGER_SCHEMA_VERSION.to_string(),
            run_id: run_id.clone(),
            scenario_id: scenario_id.clone(),
            measurement_window_id: "window-1".to_string(),
            scenario_contract_digest_sha256: embedded_scenario_manifest_digest(),
            operation_contract_registry_digest_sha256: contracts.registry_digest_sha256().to_string(),
            requires_cross_host_visibility: true,
            workload_manifest_digest_sha256: manifest.clone(),
            generator_digest_sha256: generator.clone(),
            parameter_set_digest_sha256: params.clone(),
            clock_source: "CLOCK_MONOTONIC_RAW".to_string(),
            monotonic_clock_id: clock_id.clone(),
            monotonic_clock_epoch: clock_epoch.clone(),
            measurement_start_ticks: 1_000,
            measurement_end_ticks: 11_000,
            ticks_per_second: 1_000,
            entry_count: 3,
            entries: (0..3).map(|index| OfferLedgerEntry {
                sequence: index + 1,
                run_id: run_id.clone(),
                scenario_id: scenario_id.clone(),
                operation_id: format!("op-{}", index + 1),
                request_digest_sha256: digest(&format!("request-{index}")),
                idempotency_key_digest_sha256: Some(idempotency_key_commitment(format!("idem-{index}").as_bytes())),
                scheduled_monotonic_tick: 2_000 + index * 1_000,
                monotonic_clock_id: clock_id.clone(),
                monotonic_clock_epoch: clock_epoch.clone(),
                operation_class: operation_contract.operation_class().to_string(),
                operation_contract_id: operation_contract.contract_id().to_string(),
                operation_contract_version: operation_contract.contract_version().to_string(),
                operation_contract_digest_sha256: operation_contract.digest_sha256().to_string(),
                requires_idempotency_key: index == 2,
                generator_digest_sha256: generator.clone(),
                workload_manifest_digest_sha256: manifest.clone(),
                parameter_set_digest_sha256: params.clone(),
            }).collect(),
            ledger_digest_sha256: String::new(),
        };
        envelope.ledger_digest_sha256 = calculate_offer_ledger_digest(&envelope);
        envelope
    }

    #[test]
    fn verifies_complete_offer_ledger_and_derives_window() {
        let envelope = valid_envelope();
        let contracts = embedded_contract_registry();
        let verified = verify_offer_ledger(&envelope, &contracts).unwrap();
        assert_eq!(verified.entry_count(), 3);
        assert_eq!(verified.operation_ids().iter().map(String::as_str).collect::<Vec<_>>(), vec!["op-1", "op-2", "op-3"]);
        assert_eq!(verified.measurement_window_ns(), 10_000_000_000);
        assert_eq!(verified.run_id(), "run-001");
        assert_eq!(verified.scenario_id(), "MCRB-03");
        assert_eq!(verified.operation_contract_registry_digest_sha256(), contracts.registry_digest_sha256());
        assert_eq!(verified.offered_operation("op-1").unwrap().contract_id(), "MCRB-OP-REMOTE-DHT-PUBLICATION");
        assert_eq!(verified.offered_operation("op-1").unwrap().expected_qops_outcome(), OperationOutcomeClass::QualifiedSuccess);
        assert_eq!(verified.ledger_digest_sha256(), envelope.ledger_digest_sha256.as_str());
    }

    #[test]
    fn serde_round_trip_preserves_canonical_offer_ledger() {
        let envelope = valid_envelope();
        let bytes = serde_json::to_vec(&envelope).unwrap();
        let decoded: OfferLedgerEnvelope = serde_json::from_slice(&bytes).unwrap();
        assert_eq!(decoded, envelope);
        assert!(verify_offer_ledger(&decoded, &embedded_contract_registry()).is_ok());
    }

    #[test]
    fn remote_visibility_policy_is_committed_into_the_offer_ledger_digest() {
        let mut envelope = valid_envelope();
        let original_digest = envelope.ledger_digest_sha256.clone();
        envelope.requires_cross_host_visibility = false;
        let changed_digest = calculate_offer_ledger_digest(&envelope);
        assert_ne!(original_digest, changed_digest);
        envelope.ledger_digest_sha256 = changed_digest;
        assert!(verify_offer_ledger(&envelope, &embedded_contract_registry())
            .unwrap_err()
            .contains(&OfferLedgerIssue::ScenarioVisibilityPolicyMismatch {
                scenario_id: "MCRB-03".into(),
                expected: true,
                actual: false,
            }));
    }

    #[test]
    fn remote_visibility_policy_cannot_be_downgraded_even_with_a_recomputed_ledger_digest() {
        let mut envelope = valid_envelope(); // MCRB-03 requires remote visibility in the embedded contract.
        envelope.requires_cross_host_visibility = false;
        envelope.ledger_digest_sha256 = calculate_offer_ledger_digest(&envelope);
        assert!(verify_offer_ledger(&envelope, &embedded_contract_registry())
            .unwrap_err()
            .contains(&OfferLedgerIssue::ScenarioVisibilityPolicyMismatch {
                scenario_id: "MCRB-03".into(),
                expected: true,
                actual: false,
            }));
    }

    #[test]
    fn stale_scenario_contract_digest_is_rejected() {
        let mut envelope = valid_envelope();
        envelope.scenario_contract_digest_sha256 = digest("stale-scenario-contract");
        envelope.ledger_digest_sha256 = calculate_offer_ledger_digest(&envelope);
        assert!(verify_offer_ledger(&envelope, &embedded_contract_registry())
            .unwrap_err()
            .iter()
            .any(|issue| matches!(issue, OfferLedgerIssue::ScenarioContractDigestMismatch { .. })));
    }

    #[test]
    fn unknown_scenario_id_is_rejected_against_embedded_contract() {
        let mut envelope = valid_envelope();
        envelope.scenario_id = "MCRB-UNKNOWN".into();
        envelope.ledger_digest_sha256 = calculate_offer_ledger_digest(&envelope);
        assert!(verify_offer_ledger(&envelope, &embedded_contract_registry())
            .unwrap_err()
            .contains(&OfferLedgerIssue::UnknownScenarioInEmbeddedManifest("MCRB-UNKNOWN".into())));
    }

    #[test]
    fn truncation_cannot_hide_an_offer_from_count_or_digest() {
        let mut envelope = valid_envelope();
        envelope.entries.pop();
        let errors = verify_offer_ledger(&envelope, &embedded_contract_registry()).unwrap_err();
        assert!(errors.contains(&OfferLedgerIssue::EntryCountMismatch));
        assert!(errors.iter().any(|issue| matches!(issue, OfferLedgerIssue::LedgerDigestMismatch { .. })));
    }

    #[test]
    fn duplicate_operation_id_is_rejected_even_with_recomputed_digest() {
        let mut envelope = valid_envelope();
        envelope.entries[1].operation_id = envelope.entries[0].operation_id.clone();
        envelope.ledger_digest_sha256 = calculate_offer_ledger_digest(&envelope);
        assert!(verify_offer_ledger(&envelope, &embedded_contract_registry()).unwrap_err()
            .contains(&OfferLedgerIssue::DuplicateOperationId("op-1".into())));
    }

    #[test]
    fn idempotency_commitment_is_stable_and_domain_separated() {
        let first = idempotency_key_commitment(b"idem-test");
        let second = idempotency_key_commitment(b"idem-test");
        assert_eq!(first, second);
        assert_ne!(first, digest("idem-test"));
        assert_ne!(first, idempotency_key_commitment(b"idem-test-2"));
    }

    #[test]
    fn duplicate_idempotency_key_is_rejected() {
        let mut envelope = valid_envelope();
        envelope.entries[1].idempotency_key_digest_sha256 = envelope.entries[0].idempotency_key_digest_sha256.clone();
        envelope.ledger_digest_sha256 = calculate_offer_ledger_digest(&envelope);
        assert!(verify_offer_ledger(&envelope, &embedded_contract_registry()).unwrap_err()
            .contains(&OfferLedgerIssue::DuplicateIdempotencyKey(idempotency_key_commitment(b"idem-0"))));
    }

    #[test]
    fn operation_contract_registry_digest_is_bound_into_the_offer_envelope() {
        let mut envelope = valid_envelope();
        envelope.operation_contract_registry_digest_sha256 = digest("different-contract-registry");
        envelope.ledger_digest_sha256 = calculate_offer_ledger_digest(&envelope);
        let issues = verify_offer_ledger(&envelope, &embedded_contract_registry()).unwrap_err();
        assert!(issues.iter().any(|issue| matches!(
            issue,
            OfferLedgerIssue::OperationContractRegistryDigestMismatch { .. }
        )));
    }

    #[test]
    fn unknown_operation_contract_is_rejected_even_when_offer_digest_is_valid() {
        let mut envelope = valid_envelope();
        envelope.entries[0].operation_contract_id = "unregistered-contract".to_string();
        envelope.ledger_digest_sha256 = calculate_offer_ledger_digest(&envelope);
        assert!(verify_offer_ledger(&envelope, &embedded_contract_registry())
            .unwrap_err()
            .contains(&OfferLedgerIssue::UnknownOperationContract(0)));
    }

    #[test]
    fn operation_contract_digest_mismatch_is_rejected_even_when_offer_digest_is_recomputed() {
        let mut envelope = valid_envelope();
        envelope.entries[0].operation_contract_digest_sha256 = digest("tampered-contract");
        envelope.ledger_digest_sha256 = calculate_offer_ledger_digest(&envelope);
        assert!(verify_offer_ledger(&envelope, &embedded_contract_registry())
            .unwrap_err()
            .contains(&OfferLedgerIssue::OperationContractDigestMismatch(0)));
    }

    #[test]
    fn operation_contract_class_cannot_be_changed_after_offer_creation() {
        let mut envelope = valid_envelope();
        envelope.entries[0].operation_class = "external-provider-effect".to_string();
        envelope.ledger_digest_sha256 = calculate_offer_ledger_digest(&envelope);
        assert!(verify_offer_ledger(&envelope, &embedded_contract_registry())
            .unwrap_err()
            .contains(&OfferLedgerIssue::OperationContractContextMismatch(0)));
    }

    #[test]
    fn raw_idempotency_key_values_are_rejected_and_never_serialized() {
        let mut envelope = valid_envelope();
        envelope.entries[0].idempotency_key_digest_sha256 = Some("not-a-sha256-digest".to_string());
        envelope.ledger_digest_sha256 = calculate_offer_ledger_digest(&envelope);
        assert!(verify_offer_ledger(&envelope, &embedded_contract_registry())
            .unwrap_err()
            .contains(&OfferLedgerIssue::InvalidIdempotencyKeyDigest(0)));

        let encoded = serde_json::to_string(&valid_envelope()).unwrap();
        assert!(!encoded.contains("\"idempotency_key\":"));
        assert!(encoded.contains("\"idempotency_key_digest_sha256\":"));
    }

    #[test]
    fn reordered_sequence_is_rejected_even_when_digest_is_recomputed() {
        let mut envelope = valid_envelope();
        envelope.entries.swap(0, 1);
        envelope.ledger_digest_sha256 = calculate_offer_ledger_digest(&envelope);
        assert!(verify_offer_ledger(&envelope, &embedded_contract_registry()).unwrap_err()
            .iter().any(|issue| matches!(issue, OfferLedgerIssue::InvalidSequence { .. })));
    }

    #[test]
    fn changed_entry_digest_without_root_update_is_rejected() {
        let mut envelope = valid_envelope();
        envelope.entries[0].request_digest_sha256 = digest("tampered-request");
        assert!(verify_offer_ledger(&envelope, &embedded_contract_registry()).unwrap_err()
            .iter().any(|issue| matches!(issue, OfferLedgerIssue::LedgerDigestMismatch { .. })));
    }

    #[test]
    fn cross_run_entry_reuse_is_rejected() {
        let mut envelope = valid_envelope();
        envelope.entries[0].run_id = "other-run".to_string();
        envelope.ledger_digest_sha256 = calculate_offer_ledger_digest(&envelope);
        assert!(verify_offer_ledger(&envelope, &embedded_contract_registry()).unwrap_err()
            .contains(&OfferLedgerIssue::EntryRunIdMismatch(0)));
    }

    #[test]
    fn offer_outside_measurement_window_is_rejected() {
        let mut envelope = valid_envelope();
        envelope.entries[0].scheduled_monotonic_tick = envelope.measurement_end_ticks;
        envelope.ledger_digest_sha256 = calculate_offer_ledger_digest(&envelope);
        assert!(verify_offer_ledger(&envelope, &embedded_contract_registry()).unwrap_err()
            .contains(&OfferLedgerIssue::OfferOutsideMeasurementWindow(0)));
    }

    #[test]
    fn zero_frequency_and_overflow_are_rejected() {
        let mut envelope = valid_envelope();
        envelope.ticks_per_second = 0;
        envelope.ledger_digest_sha256 = calculate_offer_ledger_digest(&envelope);
        assert!(verify_offer_ledger(&envelope, &embedded_contract_registry()).unwrap_err()
            .contains(&OfferLedgerIssue::InvalidTickFrequency));

        envelope = valid_envelope();
        envelope.measurement_start_ticks = 0;
        envelope.measurement_end_ticks = u64::MAX;
        envelope.ticks_per_second = 1;
        envelope.entries[0].scheduled_monotonic_tick = 0;
        envelope.ledger_digest_sha256 = calculate_offer_ledger_digest(&envelope);
        assert!(verify_offer_ledger(&envelope, &embedded_contract_registry()).unwrap_err()
            .contains(&OfferLedgerIssue::WindowDurationOverflow));
    }

    #[test]
    fn missing_idempotency_key_is_rejected_for_external_effect_offers() {
        let mut envelope = valid_envelope();
        envelope.entries[2].idempotency_key_digest_sha256 = None;
        envelope.ledger_digest_sha256 = calculate_offer_ledger_digest(&envelope);
        assert!(verify_offer_ledger(&envelope, &embedded_contract_registry()).unwrap_err()
            .contains(&OfferLedgerIssue::MissingIdempotencyKeyForExternalEffect(2)));
    }

    #[test]
    fn idempotency_key_may_be_absent_for_non_mutating_operations() {
        let mut envelope = valid_envelope();
        envelope.entries[0].idempotency_key_digest_sha256 = None;
        envelope.ledger_digest_sha256 = calculate_offer_ledger_digest(&envelope);
        assert!(verify_offer_ledger(&envelope, &embedded_contract_registry()).is_ok());
    }
}
