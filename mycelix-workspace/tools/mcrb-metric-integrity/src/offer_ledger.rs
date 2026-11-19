//! Verifiable, offer-first workload denominator for MCRB QOPS.
//!
//! The ledger fixes the set of offered operations before their outcomes are
//! observed. It does not authenticate the workload scheduler: callers must
//! still bind the ledger artifact to the independent runner identity and run
//! receipt. This module verifies structure, deterministic IDs and SHA-256
//! content integrity; it does not claim that a scheduler actually dispatched
//! every listed request.

use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::fmt;

pub const OFFER_LEDGER_SCHEMA_VERSION: &str = "mycelix-mcrb-offer-ledger-v0.1";

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OfferMeasurementWindow {
    pub window_id: String,
    pub clock_source: String,
    pub monotonic_clock_id: String,
    pub monotonic_clock_epoch: String,
    pub start_ticks: u64,
    pub end_ticks: u64,
    pub ticks_per_second: u64,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OfferEntry {
    /// Zero-based, contiguous sequence assigned before dispatch.
    pub sequence: u64,
    /// Deterministic identifier derived from the run contract and this offer.
    pub operation_id: String,
    /// SHA-256 digest of the exact request bytes or canonical request envelope.
    pub request_digest_sha256: String,
    /// Required by effectful operations; absent only for read-only operations.
    pub idempotency_key: Option<String>,
    /// Scheduled monotonic tick on the declared measurement clock.
    pub scheduled_at_ticks: u64,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct OfferLedger {
    pub schema_version: String,
    pub run_id: String,
    pub scenario_id: String,
    pub workload_manifest_digest_sha256: String,
    pub generator_digest_sha256: String,
    pub seed: String,
    pub measurement_window: OfferMeasurementWindow,
    pub entry_count: usize,
    pub entries: Vec<OfferEntry>,
    pub digest_sha256: String,
}

/// Values read from the frozen run plan and scenario contract, not inferred
/// from the offer ledger itself. This binds a ledger to an expected workload.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OfferLedgerExpectation {
    pub run_id: String,
    pub scenario_id: String,
    pub workload_manifest_digest_sha256: String,
    pub generator_digest_sha256: String,
    pub seed: String,
    pub expected_entry_count: usize,
    pub measurement_window: OfferMeasurementWindow,
}

/// A value that can only be created by successful verification. Its fields
/// are private so downstream QOPS code cannot substitute an arbitrary ID list
/// or caller-provided duration.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VerifiedOfferLedger {
    run_id: String,
    scenario_id: String,
    operation_ids: Vec<String>,
    measurement_window_ns: u64,
    entry_count: usize,
    digest_sha256: String,
    generator_digest_sha256: String,
    workload_manifest_digest_sha256: String,
    raw_artifact_digest_sha256: String,
}

impl VerifiedOfferLedger {
    pub fn run_id(&self) -> &str {
        &self.run_id
    }

    pub fn scenario_id(&self) -> &str {
        &self.scenario_id
    }

    pub fn operation_ids(&self) -> &[String] {
        &self.operation_ids
    }

    pub fn measurement_window_ns(&self) -> u64 {
        self.measurement_window_ns
    }

    pub fn entry_count(&self) -> usize {
        self.entry_count
    }

    pub fn digest_sha256(&self) -> &str {
        &self.digest_sha256
    }

    pub fn generator_digest_sha256(&self) -> &str {
        &self.generator_digest_sha256
    }

    pub fn workload_manifest_digest_sha256(&self) -> &str {
        &self.workload_manifest_digest_sha256
    }

    pub fn raw_artifact_digest_sha256(&self) -> &str {
        &self.raw_artifact_digest_sha256
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum OfferLedgerError {
    UnsupportedSchemaVersion,
    EmptyRunId,
    EmptyScenarioId,
    EmptySeed,
    EmptyWindowId,
    EmptyClockSource,
    EmptyMonotonicClockId,
    EmptyMonotonicClockEpoch,
    InvalidDigest(&'static str),
    RunIdMismatch,
    ScenarioIdMismatch,
    WorkloadManifestMismatch,
    GeneratorDigestMismatch,
    SeedMismatch,
    MeasurementWindowMismatch,
    EmptyExpectedOfferSet,
    EntryCountMismatch { declared: usize, actual: usize },
    ExpectedEntryCountMismatch { expected: usize, actual: usize },
    InvalidClockFrequency,
    InvalidMeasurementWindow,
    WindowDurationOutOfRange,
    NonContiguousSequence { index: usize, actual: u64 },
    EmptyOperationId(usize),
    DeterministicOperationIdMismatch(usize),
    DuplicateOperationId(String),
    DuplicateIdempotencyKey(String),
    EmptyIdempotencyKey(usize),
    OfferOutsideMeasurementWindow(usize),
    OfferScheduleReversed(usize),
    LedgerDigestMismatch,
    Serialization(String),
    NonCanonicalArtifact,
}

impl fmt::Display for OfferLedgerError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{self:?}")
    }
}

impl std::error::Error for OfferLedgerError {}

/// SHA-256 rendered as lowercase hexadecimal.
pub fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut output = String::with_capacity(64);
    for byte in digest {
        use fmt::Write as _;
        // Writing to a String is infallible.
        let _ = write!(&mut output, "{byte:02x}");
    }
    output
}

fn is_sha256_digest(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn update_bytes(hasher: &mut Sha256, value: &[u8]) {
    hasher.update((value.len() as u64).to_be_bytes());
    hasher.update(value);
}

fn update_string(hasher: &mut Sha256, value: &str) {
    update_bytes(hasher, value.as_bytes());
}

fn update_u64(hasher: &mut Sha256, value: u64) {
    hasher.update(value.to_be_bytes());
}

fn update_option_string(hasher: &mut Sha256, value: Option<&str>) {
    match value {
        Some(value) => {
            hasher.update([1_u8]);
            update_string(hasher, value);
        }
        None => hasher.update([0_u8]),
    }
}

/// Deterministic operation ID for one offer.
///
/// The ID is bound to the frozen run/scenario/workload/generator/seed tuple,
/// the zero-based sequence and the request digest. It is not an authorization
/// token and does not prove that the scheduler sent the request.
pub fn operation_id_for_offer(
    run_id: &str,
    scenario_id: &str,
    workload_manifest_digest_sha256: &str,
    generator_digest_sha256: &str,
    seed: &str,
    sequence: u64,
    request_digest_sha256: &str,
) -> String {
    let mut hasher = Sha256::new();
    update_bytes(&mut hasher, b"MYCELIX-MCRB-OFFER-ID");
    update_string(&mut hasher, OFFER_LEDGER_SCHEMA_VERSION);
    update_string(&mut hasher, run_id);
    update_string(&mut hasher, scenario_id);
    update_string(&mut hasher, workload_manifest_digest_sha256);
    update_string(&mut hasher, generator_digest_sha256);
    update_string(&mut hasher, seed);
    update_u64(&mut hasher, sequence);
    update_string(&mut hasher, request_digest_sha256);

    let digest = hasher.finalize();
    let mut output = String::from("mcrb-op-");
    for byte in digest {
        use fmt::Write as _;
        let _ = write!(&mut output, "{byte:02x}");
    }
    output
}

/// Create the canonical digest for a ledger's declared content.
pub fn compute_ledger_digest(ledger: &OfferLedger) -> String {
    let mut hasher = Sha256::new();
    update_bytes(&mut hasher, b"MYCELIX-MCRB-OFFER-LEDGER");
    update_string(&mut hasher, &ledger.schema_version);
    update_string(&mut hasher, &ledger.run_id);
    update_string(&mut hasher, &ledger.scenario_id);
    update_string(&mut hasher, &ledger.workload_manifest_digest_sha256);
    update_string(&mut hasher, &ledger.generator_digest_sha256);
    update_string(&mut hasher, &ledger.seed);

    update_string(&mut hasher, &ledger.measurement_window.window_id);
    update_string(&mut hasher, &ledger.measurement_window.clock_source);
    update_string(&mut hasher, &ledger.measurement_window.monotonic_clock_id);
    update_string(&mut hasher, &ledger.measurement_window.monotonic_clock_epoch);
    update_u64(&mut hasher, ledger.measurement_window.start_ticks);
    update_u64(&mut hasher, ledger.measurement_window.end_ticks);
    update_u64(&mut hasher, ledger.measurement_window.ticks_per_second);
    update_u64(&mut hasher, ledger.entry_count as u64);

    update_u64(&mut hasher, ledger.entries.len() as u64);
    for entry in &ledger.entries {
        update_u64(&mut hasher, entry.sequence);
        update_string(&mut hasher, &entry.operation_id);
        update_string(&mut hasher, &entry.request_digest_sha256);
        update_option_string(&mut hasher, entry.idempotency_key.as_deref());
        update_u64(&mut hasher, entry.scheduled_at_ticks);
    }

    let digest = hasher.finalize();
    let mut output = String::with_capacity(64);
    for byte in digest {
        use fmt::Write as _;
        let _ = write!(&mut output, "{byte:02x}");
    }
    output
}

impl OfferLedger {
    /// Seal a newly generated ledger. Verification against a frozen
    /// expectation is still required before the ledger can feed QOPS.
    pub fn seal(mut self) -> Self {
        self.entry_count = self.entries.len();
        self.digest_sha256 = compute_ledger_digest(&self);
        self
    }
}

fn derive_window_ns(window: &OfferMeasurementWindow) -> Result<u64, OfferLedgerError> {
    let ticks = window
        .end_ticks
        .checked_sub(window.start_ticks)
        .ok_or(OfferLedgerError::InvalidMeasurementWindow)?;
    if ticks == 0 || window.ticks_per_second == 0 {
        return Err(OfferLedgerError::InvalidMeasurementWindow);
    }

    let nanoseconds = u128::from(ticks)
        .checked_mul(1_000_000_000)
        .ok_or(OfferLedgerError::WindowDurationOutOfRange)?
        / u128::from(window.ticks_per_second);

    if nanoseconds == 0 || nanoseconds > u128::from(u64::MAX) {
        return Err(OfferLedgerError::WindowDurationOutOfRange);
    }

    Ok(nanoseconds as u64)
}


/// Serialize the ledger using the single accepted compact JSON representation.
/// The internal ledger digest does not depend on JSON ordering or whitespace.
pub fn serialize_offer_ledger_json(
    ledger: &OfferLedger,
) -> Result<Vec<u8>, OfferLedgerError> {
    serde_json::to_vec(ledger).map_err(|error| OfferLedgerError::Serialization(error.to_string()))
}

/// Parse and verify an exact canonical JSON artifact.
///
/// Non-canonical whitespace/key ordering is rejected so the receipt artifact
/// digest and the typed ledger refer to the same byte sequence. Unknown and
/// duplicate struct fields are rejected by the deny_unknown_fields Serde
/// representation.
pub fn verify_offer_ledger_json(
    raw_json: &[u8],
    expected: &OfferLedgerExpectation,
) -> Result<VerifiedOfferLedger, OfferLedgerError> {
    let ledger: OfferLedger = serde_json::from_slice(raw_json)
        .map_err(|error| OfferLedgerError::Serialization(error.to_string()))?;
    let canonical = serialize_offer_ledger_json(&ledger)?;
    if canonical != raw_json {
        return Err(OfferLedgerError::NonCanonicalArtifact);
    }
    verify_offer_ledger(&ledger, expected)
}

/// Verify a content-addressed offer ledger against frozen run expectations.
///
/// The expected count and run contract must be loaded from the pinned scenario
/// manifest / runner plan. Passing values copied from the ledger itself defeats
/// the independent-denominator guarantee.
pub fn verify_offer_ledger(
    ledger: &OfferLedger,
    expected: &OfferLedgerExpectation,
) -> Result<VerifiedOfferLedger, OfferLedgerError> {
    if ledger.schema_version != OFFER_LEDGER_SCHEMA_VERSION {
        return Err(OfferLedgerError::UnsupportedSchemaVersion);
    }
    if ledger.run_id.trim().is_empty() || expected.run_id.trim().is_empty() {
        return Err(OfferLedgerError::EmptyRunId);
    }
    if ledger.scenario_id.trim().is_empty() || expected.scenario_id.trim().is_empty() {
        return Err(OfferLedgerError::EmptyScenarioId);
    }
    if ledger.seed.trim().is_empty() || expected.seed.trim().is_empty() {
        return Err(OfferLedgerError::EmptySeed);
    }
    if ledger.measurement_window.window_id.trim().is_empty() {
        return Err(OfferLedgerError::EmptyWindowId);
    }
    if ledger.measurement_window.clock_source.trim().is_empty() {
        return Err(OfferLedgerError::EmptyClockSource);
    }
    if ledger.measurement_window.monotonic_clock_id.trim().is_empty() {
        return Err(OfferLedgerError::EmptyMonotonicClockId);
    }
    if ledger.measurement_window.monotonic_clock_epoch.trim().is_empty() {
        return Err(OfferLedgerError::EmptyMonotonicClockEpoch);
    }

    for (value, field) in [
        (
            ledger.workload_manifest_digest_sha256.as_str(),
            "workload manifest digest",
        ),
        (ledger.generator_digest_sha256.as_str(), "generator digest"),
        (ledger.digest_sha256.as_str(), "offer ledger digest"),
    ] {
        if !is_sha256_digest(value) {
            return Err(OfferLedgerError::InvalidDigest(field));
        }
    }

    if ledger.run_id != expected.run_id {
        return Err(OfferLedgerError::RunIdMismatch);
    }
    if ledger.scenario_id != expected.scenario_id {
        return Err(OfferLedgerError::ScenarioIdMismatch);
    }
    if ledger.workload_manifest_digest_sha256 != expected.workload_manifest_digest_sha256
        || !is_sha256_digest(&expected.workload_manifest_digest_sha256)
    {
        return Err(OfferLedgerError::WorkloadManifestMismatch);
    }
    if ledger.generator_digest_sha256 != expected.generator_digest_sha256
        || !is_sha256_digest(&expected.generator_digest_sha256)
    {
        return Err(OfferLedgerError::GeneratorDigestMismatch);
    }
    if ledger.seed != expected.seed {
        return Err(OfferLedgerError::SeedMismatch);
    }
    if ledger.measurement_window != expected.measurement_window {
        return Err(OfferLedgerError::MeasurementWindowMismatch);
    }
    if ledger.measurement_window.ticks_per_second == 0 {
        return Err(OfferLedgerError::InvalidClockFrequency);
    }

    if ledger.entries.is_empty() || expected.expected_entry_count == 0 {
        return Err(OfferLedgerError::EmptyExpectedOfferSet);
    }
    if ledger.entry_count != ledger.entries.len() {
        return Err(OfferLedgerError::EntryCountMismatch {
            declared: ledger.entry_count,
            actual: ledger.entries.len(),
        });
    }
    if ledger.entries.len() != expected.expected_entry_count {
        return Err(OfferLedgerError::ExpectedEntryCountMismatch {
            expected: expected.expected_entry_count,
            actual: ledger.entries.len(),
        });
    }

    let window_ns = derive_window_ns(&ledger.measurement_window)?;
    let mut operation_ids = Vec::with_capacity(ledger.entries.len());
    let mut seen_ids = BTreeSet::new();
    let mut seen_idempotency_keys = BTreeSet::new();
    let mut prior_tick: Option<u64> = None;

    for (index, entry) in ledger.entries.iter().enumerate() {
        let expected_sequence = index as u64;
        if entry.sequence != expected_sequence {
            return Err(OfferLedgerError::NonContiguousSequence {
                index,
                actual: entry.sequence,
            });
        }
        if entry.operation_id.trim().is_empty() {
            return Err(OfferLedgerError::EmptyOperationId(index));
        }
        if !is_sha256_digest(&entry.request_digest_sha256) {
            return Err(OfferLedgerError::InvalidDigest("request digest"));
        }

        let deterministic_id = operation_id_for_offer(
            &ledger.run_id,
            &ledger.scenario_id,
            &ledger.workload_manifest_digest_sha256,
            &ledger.generator_digest_sha256,
            &ledger.seed,
            entry.sequence,
            &entry.request_digest_sha256,
        );
        if entry.operation_id != deterministic_id {
            return Err(OfferLedgerError::DeterministicOperationIdMismatch(index));
        }
        if !seen_ids.insert(entry.operation_id.clone()) {
            return Err(OfferLedgerError::DuplicateOperationId(
                entry.operation_id.clone(),
            ));
        }

        if let Some(key) = entry.idempotency_key.as_deref() {
            if key.trim().is_empty() {
                return Err(OfferLedgerError::EmptyIdempotencyKey(index));
            }
            if !seen_idempotency_keys.insert(key.to_string()) {
                return Err(OfferLedgerError::DuplicateIdempotencyKey(
                    key.to_string(),
                ));
            }
        }

        if entry.scheduled_at_ticks < ledger.measurement_window.start_ticks
            || entry.scheduled_at_ticks > ledger.measurement_window.end_ticks
        {
            return Err(OfferLedgerError::OfferOutsideMeasurementWindow(index));
        }
        if prior_tick.is_some_and(|prior| entry.scheduled_at_ticks < prior) {
            return Err(OfferLedgerError::OfferScheduleReversed(index));
        }
        prior_tick = Some(entry.scheduled_at_ticks);
        operation_ids.push(entry.operation_id.clone());
    }

    let computed_digest = compute_ledger_digest(ledger);
    if computed_digest != ledger.digest_sha256 {
        return Err(OfferLedgerError::LedgerDigestMismatch);
    }

    let canonical_artifact = serialize_offer_ledger_json(ledger)?;
    let raw_artifact_digest_sha256 = sha256_hex(&canonical_artifact);

    Ok(VerifiedOfferLedger {
        run_id: ledger.run_id.clone(),
        scenario_id: ledger.scenario_id.clone(),
        operation_ids,
        measurement_window_ns: window_ns,
        entry_count: ledger.entry_count,
        digest_sha256: computed_digest,
        generator_digest_sha256: ledger.generator_digest_sha256.clone(),
        workload_manifest_digest_sha256: ledger.workload_manifest_digest_sha256.clone(),
        raw_artifact_digest_sha256,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const RUN_ID: &str = "run-2026-11-19-001";
    const SCENARIO_ID: &str = "MCRB-03";
    const WORKLOAD_DIGEST: &str =
        "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
    const GENERATOR_DIGEST: &str =
        "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";
    const SEED: &str = "seed-42";

    fn window() -> OfferMeasurementWindow {
        OfferMeasurementWindow {
            window_id: "window-1".to_string(),
            clock_source: "clock_gettime(CLOCK_MONOTONIC)".to_string(),
            monotonic_clock_id: "host-a-monotonic".to_string(),
            monotonic_clock_epoch: "boot-epoch-a".to_string(),
            start_ticks: 1_000,
            end_ticks: 1_000_001_000,
            ticks_per_second: 1_000_000_000,
        }
    }

    fn ledger_with_count(count: usize) -> OfferLedger {
        let measurement_window = window();
        let entries = (0..count)
            .map(|sequence| {
                let request_digest_sha256 = sha256_hex(format!("request-{sequence}").as_bytes());
                OfferEntry {
                    sequence: sequence as u64,
                    operation_id: operation_id_for_offer(
                        RUN_ID,
                        SCENARIO_ID,
                        WORKLOAD_DIGEST,
                        GENERATOR_DIGEST,
                        SEED,
                        sequence as u64,
                        &request_digest_sha256,
                    ),
                    request_digest_sha256,
                    idempotency_key: Some(format!("effect-{sequence}")),
                    scheduled_at_ticks: measurement_window.start_ticks + sequence as u64,
                }
            })
            .collect();

        OfferLedger {
            schema_version: OFFER_LEDGER_SCHEMA_VERSION.to_string(),
            run_id: RUN_ID.to_string(),
            scenario_id: SCENARIO_ID.to_string(),
            workload_manifest_digest_sha256: WORKLOAD_DIGEST.to_string(),
            generator_digest_sha256: GENERATOR_DIGEST.to_string(),
            seed: SEED.to_string(),
            measurement_window,
            entry_count: 0,
            entries,
            digest_sha256: String::new(),
        }
        .seal()
    }

    fn expectation(count: usize) -> OfferLedgerExpectation {
        OfferLedgerExpectation {
            run_id: RUN_ID.to_string(),
            scenario_id: SCENARIO_ID.to_string(),
            workload_manifest_digest_sha256: WORKLOAD_DIGEST.to_string(),
            generator_digest_sha256: GENERATOR_DIGEST.to_string(),
            seed: SEED.to_string(),
            expected_entry_count: count,
            measurement_window: window(),
        }
    }

    #[test]
    fn sha256_matches_known_vector() {
        assert_eq!(
            sha256_hex(b"hello world"),
            "b94d27b9934d3e08a52e52d7da7dabfac484efe37a5380ee9088f7ace2efcde9"
        );
    }

    #[test]
    fn valid_ledger_derives_operation_set_and_window_from_raw_ticks() {
        let ledger = ledger_with_count(5);
        let verified = verify_offer_ledger(&ledger, &expectation(5)).unwrap();
        assert_eq!(verified.entry_count(), 5);
        assert_eq!(verified.operation_ids().len(), 5);
        assert_eq!(verified.measurement_window_ns(), 1_000_000_000);
        assert_eq!(verified.digest_sha256(), ledger.digest_sha256);
        assert_eq!(verified.run_id(), RUN_ID);
        assert_eq!(verified.scenario_id(), SCENARIO_ID);
    }

    #[test]
    fn truncated_ledger_cannot_shrink_frozen_denominator() {
        let ledger = ledger_with_count(4);
        let error = verify_offer_ledger(&ledger, &expectation(5)).unwrap_err();
        assert_eq!(
            error,
            OfferLedgerError::ExpectedEntryCountMismatch {
                expected: 5,
                actual: 4,
            }
        );
    }

    #[test]
    fn reordered_or_missing_sequence_is_rejected_even_with_resealed_digest() {
        let mut ledger = ledger_with_count(5);
        ledger.entries.swap(2, 3);
        ledger = ledger.seal();
        let error = verify_offer_ledger(&ledger, &expectation(5)).unwrap_err();
        assert!(matches!(error, OfferLedgerError::NonContiguousSequence { .. }));
    }

    #[test]
    fn request_mutation_invalidates_the_digest() {
        let mut ledger = ledger_with_count(5);
        ledger.entries[0].request_digest_sha256 =
            "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc".to_string();
        let error = verify_offer_ledger(&ledger, &expectation(5)).unwrap_err();
        assert!(matches!(
            error,
            OfferLedgerError::DeterministicOperationIdMismatch(0)
        ));
    }

    #[test]
    fn changed_digest_without_resealing_is_rejected() {
        let mut ledger = ledger_with_count(5);
        ledger.entries[0].scheduled_at_ticks += 1;
        let error = verify_offer_ledger(&ledger, &expectation(5)).unwrap_err();
        assert_eq!(error, OfferLedgerError::LedgerDigestMismatch);
    }

    #[test]
    fn duplicate_idempotency_key_is_rejected() {
        let mut ledger = ledger_with_count(5);
        ledger.entries[1].idempotency_key = ledger.entries[0].idempotency_key.clone();
        ledger = ledger.seal();
        let error = verify_offer_ledger(&ledger, &expectation(5)).unwrap_err();
        assert!(matches!(error, OfferLedgerError::DuplicateIdempotencyKey(_)));
    }

    #[test]
    fn stale_generator_or_scenario_is_rejected() {
        let ledger = ledger_with_count(5);
        let mut wrong = expectation(5);
        wrong.generator_digest_sha256 =
            "dddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddddd".to_string();
        assert_eq!(
            verify_offer_ledger(&ledger, &wrong).unwrap_err(),
            OfferLedgerError::GeneratorDigestMismatch
        );

        wrong = expectation(5);
        wrong.scenario_id = "MCRB-02".to_string();
        assert_eq!(
            verify_offer_ledger(&ledger, &wrong).unwrap_err(),
            OfferLedgerError::ScenarioIdMismatch
        );
    }

    #[test]
    fn zero_clock_frequency_is_rejected_before_duration_derivation() {
        let mut ledger = ledger_with_count(5);
        ledger.measurement_window.ticks_per_second = 0;
        ledger = ledger.seal();
        let mut expected = expectation(5);
        expected.measurement_window.ticks_per_second = 0;
        assert_eq!(
            verify_offer_ledger(&ledger, &expected).unwrap_err(),
            OfferLedgerError::InvalidClockFrequency
        );
    }

    #[test]
    fn schedule_outside_measurement_window_is_rejected() {
        let mut ledger = ledger_with_count(5);
        ledger.entries[0].scheduled_at_ticks = ledger.measurement_window.start_ticks - 1;
        ledger = ledger.seal();
        assert_eq!(
            verify_offer_ledger(&ledger, &expectation(5)).unwrap_err(),
            OfferLedgerError::OfferOutsideMeasurementWindow(0)
        );
    }

    #[test]
    fn reversed_schedule_is_rejected() {
        let mut ledger = ledger_with_count(5);
        ledger.entries[1].scheduled_at_ticks = ledger.entries[0].scheduled_at_ticks - 1;
        ledger = ledger.seal();
        assert_eq!(
            verify_offer_ledger(&ledger, &expectation(5)).unwrap_err(),
            OfferLedgerError::OfferScheduleReversed(1)
        );
    }

    #[test]
    fn ledger_digest_is_domain_separated_and_deterministic() {
        let first = ledger_with_count(5);
        let second = ledger_with_count(5);
        assert_eq!(first.digest_sha256, second.digest_sha256);
        assert_ne!(
            first.digest_sha256,
            sha256_hex(first.digest_sha256.as_bytes())
        );
    }

    #[test]
    fn caller_cannot_supply_a_verified_ledger_with_public_fields() {
        // This test documents the Rust API invariant: the verifier result has
        // private fields. Construction remains available only through
        // verify_offer_ledger, which checks the frozen expectation.
        let verified = verify_offer_ledger(&ledger_with_count(5), &expectation(5)).unwrap();
        assert_eq!(verified.operation_ids().len(), verified.entry_count());
    }
}
