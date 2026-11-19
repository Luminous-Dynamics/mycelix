//! Offer-first workload ledger verification for MCRB.
//!
//! The denominator is frozen before dispatch. This module verifies internal
//! ledger structure and a domain-separated SHA-256 digest; it does not prove
//! who generated or externally observed the ledger. Runner/orchestrator
//! authenticity remains a separate receipt-signature and custody requirement.

use sha2::{Digest, Sha256};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

pub const OFFER_LEDGER_SCHEMA_VERSION: &str = "mycelix-mcrb-offer-ledger-v0.1";
const DOMAIN_SEPARATOR: &[u8] = b"MYCELIX-MCRB-OFFER-LEDGER\0v0.1\0";

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct OfferLedgerEntry {
    pub sequence: u64,
    pub run_id: String,
    pub scenario_id: String,
    pub operation_id: String,
    pub request_digest_sha256: String,
    pub idempotency_key: Option<String>,
    pub scheduled_monotonic_tick: u64,
    pub monotonic_clock_id: String,
    pub monotonic_clock_epoch: String,
    pub operation_class: String,
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
    InvalidWindowBounds,
    InvalidTickFrequency,
    WindowDurationOverflow,
    EmptyOfferLedger,
    EntryCountMismatch,
    InvalidSequence { index: usize, expected: u64, actual: u64 },
    EmptyOperationId(usize),
    DuplicateOperationId(String),
    InvalidRequestDigest(usize),
    EmptyIdempotencyKey(usize),
    MissingIdempotencyKeyForExternalEffect(usize),
    DuplicateIdempotencyKey(String),
    EmptyOperationClass(usize),
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
}

impl VerifiedOfferLedger {
    pub fn run_id(&self) -> &str { &self.run_id }
    pub fn scenario_id(&self) -> &str { &self.scenario_id }
    pub fn measurement_window_id(&self) -> &str { &self.measurement_window_id }
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

fn canonical_offer_ledger_bytes(envelope: &OfferLedgerEnvelope) -> Vec<u8> {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(DOMAIN_SEPARATOR);
    push_string(&mut bytes, &envelope.schema_version);
    push_string(&mut bytes, &envelope.run_id);
    push_string(&mut bytes, &envelope.scenario_id);
    push_string(&mut bytes, &envelope.measurement_window_id);
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
        push_optional_string(&mut bytes, entry.idempotency_key.as_deref());
        push_u64(&mut bytes, entry.scheduled_monotonic_tick);
        push_string(&mut bytes, &entry.monotonic_clock_id);
        push_string(&mut bytes, &entry.monotonic_clock_epoch);
        push_string(&mut bytes, &entry.operation_class);
        bytes.push(u8::from(entry.requires_idempotency_key));
        push_string(&mut bytes, &entry.generator_digest_sha256);
        push_string(&mut bytes, &entry.workload_manifest_digest_sha256);
        push_string(&mut bytes, &entry.parameter_set_digest_sha256);
    }
    bytes
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
    if envelope.measurement_end_ticks <= envelope.measurement_start_ticks { issues.push(OfferLedgerIssue::InvalidWindowBounds); }
    if envelope.ticks_per_second == 0 { issues.push(OfferLedgerIssue::InvalidTickFrequency); }
    if envelope.entries.is_empty() { issues.push(OfferLedgerIssue::EmptyOfferLedger); }
    if envelope.entry_count != envelope.entries.len() as u64 { issues.push(OfferLedgerIssue::EntryCountMismatch); }

    let mut operation_ids = Vec::with_capacity(envelope.entries.len());
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
        if entry.requires_idempotency_key && entry.idempotency_key.is_none() {
            issues.push(OfferLedgerIssue::MissingIdempotencyKeyForExternalEffect(index));
        }
        if let Some(key) = entry.idempotency_key.as_deref() {
            if key.trim().is_empty() {
                issues.push(OfferLedgerIssue::EmptyIdempotencyKey(index));
            } else if !seen_idempotency_keys.insert(key.to_string()) {
                issues.push(OfferLedgerIssue::DuplicateIdempotencyKey(key.to_string()));
            }
        }
        if entry.operation_class.trim().is_empty() { issues.push(OfferLedgerIssue::EmptyOperationClass(index)); }
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
    })
}

#[cfg(test)]
mod tests {
    use super::*;

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
        let mut envelope = OfferLedgerEnvelope {
            schema_version: OFFER_LEDGER_SCHEMA_VERSION.to_string(),
            run_id: run_id.clone(),
            scenario_id: scenario_id.clone(),
            measurement_window_id: "window-1".to_string(),
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
                idempotency_key: Some(format!("idem-{index}")),
                scheduled_monotonic_tick: 2_000 + index * 1_000,
                monotonic_clock_id: clock_id.clone(),
                monotonic_clock_epoch: clock_epoch.clone(),
                operation_class: "read-or-write".to_string(),
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
        let verified = verify_offer_ledger(&envelope).unwrap();
        assert_eq!(verified.entry_count(), 3);
        assert_eq!(verified.operation_ids(), ["op-1", "op-2", "op-3"]);
        assert_eq!(verified.measurement_window_ns(), 10_000_000_000);
        assert_eq!(verified.run_id(), "run-001");
        assert_eq!(verified.scenario_id(), "MCRB-03");
        assert_eq!(verified.ledger_digest_sha256(), envelope.ledger_digest_sha256);
    }

    #[test]
    fn truncation_cannot_hide_an_offer_from_count_or_digest() {
        let mut envelope = valid_envelope();
        envelope.entries.pop();
        let errors = verify_offer_ledger(&envelope).unwrap_err();
        assert!(errors.contains(&OfferLedgerIssue::EntryCountMismatch));
        assert!(errors.iter().any(|issue| matches!(issue, OfferLedgerIssue::LedgerDigestMismatch { .. })));
    }

    #[test]
    fn duplicate_operation_id_is_rejected_even_with_recomputed_digest() {
        let mut envelope = valid_envelope();
        envelope.entries[1].operation_id = envelope.entries[0].operation_id.clone();
        envelope.ledger_digest_sha256 = calculate_offer_ledger_digest(&envelope);
        assert!(verify_offer_ledger(&envelope).unwrap_err()
            .contains(&OfferLedgerIssue::DuplicateOperationId("op-1".into())));
    }

    #[test]
    fn duplicate_idempotency_key_is_rejected() {
        let mut envelope = valid_envelope();
        envelope.entries[1].idempotency_key = envelope.entries[0].idempotency_key.clone();
        envelope.ledger_digest_sha256 = calculate_offer_ledger_digest(&envelope);
        assert!(verify_offer_ledger(&envelope).unwrap_err()
            .contains(&OfferLedgerIssue::DuplicateIdempotencyKey("idem-0".into())));
    }

    #[test]
    fn reordered_sequence_is_rejected_even_when_digest_is_recomputed() {
        let mut envelope = valid_envelope();
        envelope.entries.swap(0, 1);
        envelope.ledger_digest_sha256 = calculate_offer_ledger_digest(&envelope);
        assert!(verify_offer_ledger(&envelope).unwrap_err()
            .iter().any(|issue| matches!(issue, OfferLedgerIssue::InvalidSequence { .. })));
    }

    #[test]
    fn changed_entry_digest_without_root_update_is_rejected() {
        let mut envelope = valid_envelope();
        envelope.entries[0].request_digest_sha256 = digest("tampered-request");
        assert!(verify_offer_ledger(&envelope).unwrap_err()
            .iter().any(|issue| matches!(issue, OfferLedgerIssue::LedgerDigestMismatch { .. })));
    }

    #[test]
    fn cross_run_entry_reuse_is_rejected() {
        let mut envelope = valid_envelope();
        envelope.entries[0].run_id = "other-run".to_string();
        envelope.ledger_digest_sha256 = calculate_offer_ledger_digest(&envelope);
        assert!(verify_offer_ledger(&envelope).unwrap_err()
            .contains(&OfferLedgerIssue::EntryRunIdMismatch(0)));
    }

    #[test]
    fn offer_outside_measurement_window_is_rejected() {
        let mut envelope = valid_envelope();
        envelope.entries[0].scheduled_monotonic_tick = envelope.measurement_end_ticks;
        envelope.ledger_digest_sha256 = calculate_offer_ledger_digest(&envelope);
        assert!(verify_offer_ledger(&envelope).unwrap_err()
            .contains(&OfferLedgerIssue::OfferOutsideMeasurementWindow(0)));
    }

    #[test]
    fn zero_frequency_and_overflow_are_rejected() {
        let mut envelope = valid_envelope();
        envelope.ticks_per_second = 0;
        envelope.ledger_digest_sha256 = calculate_offer_ledger_digest(&envelope);
        assert!(verify_offer_ledger(&envelope).unwrap_err()
            .contains(&OfferLedgerIssue::InvalidTickFrequency));

        envelope = valid_envelope();
        envelope.measurement_start_ticks = 0;
        envelope.measurement_end_ticks = u64::MAX;
        envelope.ticks_per_second = 1;
        envelope.entries[0].scheduled_monotonic_tick = 0;
        envelope.ledger_digest_sha256 = calculate_offer_ledger_digest(&envelope);
        assert!(verify_offer_ledger(&envelope).unwrap_err()
            .contains(&OfferLedgerIssue::WindowDurationOverflow));
    }

    #[test]
    fn idempotency_key_may_be_absent_for_non_mutating_operations() {
        let mut envelope = valid_envelope();
        envelope.entries[0].idempotency_key = None;
        envelope.ledger_digest_sha256 = calculate_offer_ledger_digest(&envelope);
        assert!(verify_offer_ledger(&envelope).is_ok());
    }
}
