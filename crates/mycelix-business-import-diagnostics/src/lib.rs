// Copyright (C) 2024-2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Non-mutating import diagnostics for Mycelix Business pilots.
//!
//! The diagnostic path binds source-file identity, denominators, parser rejections, connector
//! lineage, source-event replay protection, and privacy-minimal normalized-observation commitments
//! before field qualification. It never admits records to durable witness state.

use std::collections::{BTreeMap, BTreeSet};
use std::io::Read;

use csv::{ReaderBuilder, StringRecord, WriterBuilder};
use mycelix_business_adapter_delimited::{
    AdapterError, DelimitedIngressAdapter, MAX_BATCH_INPUT_BYTES,
};
use mycelix_business_core::{Digest32, ReferenceId};
use mycelix_business_field_qualification::{
    DataQualityEvidence, FieldQualificationDecision, FieldQualificationEvidence,
};
use mycelix_business_ingress::IngressQualificationBinding;
use mycelix_business_pilot_hospitality::{
    HospitalityForecastPilotRegistration, HospitalityPilotEvidence, HospitalityPilotEvidenceError,
    evaluate_hospitality_forecast_pilot, sales_input_ref,
};
use mycelix_business_shadow::MetricObservation;
use sha2::{Digest, Sha256};

pub const IMPORT_DIAGNOSTICS_ARE_READ_ONLY: bool = true;
pub const MAX_DIAGNOSTIC_INPUT_BYTES: u64 = MAX_BATCH_INPUT_BYTES;

fn zero_digest(value: &Digest32) -> bool {
    value == &Digest32([0; 32])
}

fn hash_str(hasher: &mut Sha256, value: &str) {
    hasher.update((value.len() as u64).to_be_bytes());
    hasher.update(value.as_bytes());
}

fn finish_digest(hasher: Sha256) -> Digest32 {
    Digest32(hasher.finalize().into())
}

fn digest_bytes(label: &str, bytes: &[u8]) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, label);
    hasher.update((bytes.len() as u64).to_be_bytes());
    hasher.update(bytes);
    finish_digest(hasher)
}

pub fn upstream_export_completeness_unverified_ref() -> ReferenceId {
    ReferenceId::new("limitation:upstream-export-completeness-unverified:v1")
        .expect("static limitation id is canonical")
}

/// Privacy-minimal key used to detect the same provider event across multiple export files.
pub fn source_event_key_commitment(
    source_system: &ReferenceId,
    source_event_id: &ReferenceId,
) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:source-event-key:v1");
    hash_str(&mut hasher, source_system.as_str());
    hash_str(&mut hasher, source_event_id.as_str());
    finish_digest(hasher)
}

/// Exact commitment to a normalized observation without copying the provider payload.
///
/// A forecast actual can prove membership in an extraction campaign by reproducing this digest.
pub fn normalized_observation_commitment(observation: &MetricObservation) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:normalized-observation:v1");
    hash_str(&mut hasher, observation.observation.as_ref_id().as_str());
    hash_str(&mut hasher, observation.source_system.as_str());
    hash_str(&mut hasher, observation.source_event_id.as_str());
    hasher.update(observation.source_payload_digest.0);
    hasher.update(observation.mapping_digest.0);
    hash_str(&mut hasher, observation.metric.as_str());
    hash_str(&mut hasher, observation.scope.as_ref_id().as_str());
    hasher.update(observation.value.mantissa.to_be_bytes());
    hasher.update(observation.value.scale.to_be_bytes());
    hash_str(&mut hasher, observation.value.unit.as_str());
    hasher.update(observation.observed_at_unix_ms.to_be_bytes());
    finish_digest(hasher)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ImportRejectionClass {
    CsvStructure,
    EmptyField,
    InvalidReference,
    InvalidTimestamp,
    TimestampOutOfRange,
    InvalidNumber,
    InvalidGrouping,
    PrecisionLoss,
    NegativeNotAllowed,
    ArithmeticOverflow,
    InvalidScope,
    DuplicateSourceEvent,
    AdapterConfiguration,
    IngressValidation,
    Other,
}

impl ImportRejectionClass {
    fn tag(self) -> u8 {
        match self {
            Self::CsvStructure => 1,
            Self::EmptyField => 2,
            Self::InvalidReference => 3,
            Self::InvalidTimestamp => 4,
            Self::TimestampOutOfRange => 5,
            Self::InvalidNumber => 6,
            Self::InvalidGrouping => 7,
            Self::PrecisionLoss => 8,
            Self::NegativeNotAllowed => 9,
            Self::ArithmeticOverflow => 10,
            Self::InvalidScope => 11,
            Self::DuplicateSourceEvent => 12,
            Self::AdapterConfiguration => 13,
            Self::IngressValidation => 14,
            Self::Other => 15,
        }
    }
}

fn classify_adapter_error(error: &AdapterError) -> ImportRejectionClass {
    match error {
        AdapterError::Csv(_) => ImportRejectionClass::CsvStructure,
        AdapterError::EmptyField { .. } => ImportRejectionClass::EmptyField,
        AdapterError::InvalidReference { .. } => ImportRejectionClass::InvalidReference,
        AdapterError::InvalidTimestamp { .. } => ImportRejectionClass::InvalidTimestamp,
        AdapterError::TimestampOutOfRange { .. } => ImportRejectionClass::TimestampOutOfRange,
        AdapterError::InvalidNumber { .. } => ImportRejectionClass::InvalidNumber,
        AdapterError::InvalidGrouping { .. } => ImportRejectionClass::InvalidGrouping,
        AdapterError::PrecisionLoss { .. } => ImportRejectionClass::PrecisionLoss,
        AdapterError::NegativeNotAllowed { .. } => ImportRejectionClass::NegativeNotAllowed,
        AdapterError::ArithmeticOverflow { .. } => ImportRejectionClass::ArithmeticOverflow,
        AdapterError::InvalidScope { .. } => ImportRejectionClass::InvalidScope,
        AdapterError::Config(_) => ImportRejectionClass::AdapterConfiguration,
        AdapterError::Ingress(_) => ImportRejectionClass::IngressValidation,
        AdapterError::Io(_)
        | AdapterError::InputTooLarge { .. }
        | AdapterError::HeaderMismatch
        | AdapterError::TooManyRecords { .. }
        | AdapterError::EmptyFile => ImportRejectionClass::Other,
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ImportDiagnosticManifest {
    pub connector: IngressQualificationBinding,
    pub supported_inputs: BTreeSet<ReferenceId>,
    pub source_file_digest: Digest32,
    pub discovered_rows: u64,
    pub accepted_rows: u64,
    pub rejected_rows: u64,
    pub future_timestamp_rows: u64,
    pub earliest_observed_at_unix_ms: Option<u64>,
    pub latest_observed_at_unix_ms: Option<u64>,
    pub maximum_observed_ingest_delay_ms: u64,
    pub maximum_batch_records: u32,
    pub recommended_batches: u64,
    pub rejection_counts: BTreeMap<ImportRejectionClass, u64>,
    pub accepted_event_keys: BTreeSet<Digest32>,
    pub accepted_observation_commitments: BTreeSet<Digest32>,
    pub manifest_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ManifestError {
    ZeroDigest,
    ZeroDiscoveredRows,
    CountMismatch,
    RejectionCountMismatch,
    EventCommitmentCountMismatch,
    ObservationCommitmentsMissing,
    ZeroCommitment,
    InvalidObservedWindow,
    ZeroBatchLimit,
    RecommendedBatchMismatch,
    DigestMismatch,
}

impl ImportDiagnosticManifest {
    pub fn validate(&self) -> Result<(), ManifestError> {
        if zero_digest(&self.source_file_digest) || zero_digest(&self.manifest_digest) {
            return Err(ManifestError::ZeroDigest);
        }
        if self.discovered_rows == 0 {
            return Err(ManifestError::ZeroDiscoveredRows);
        }
        if self.accepted_rows.saturating_add(self.rejected_rows) != self.discovered_rows {
            return Err(ManifestError::CountMismatch);
        }
        if self.accepted_event_keys.len() as u64 != self.accepted_rows {
            return Err(ManifestError::EventCommitmentCountMismatch);
        }
        if self.accepted_rows > 0 && self.accepted_observation_commitments.is_empty() {
            return Err(ManifestError::ObservationCommitmentsMissing);
        }
        if self.accepted_event_keys.iter().any(zero_digest)
            || self.accepted_observation_commitments.iter().any(zero_digest)
        {
            return Err(ManifestError::ZeroCommitment);
        }
        let rejection_sum = self
            .rejection_counts
            .values()
            .copied()
            .fold(0_u64, u64::saturating_add);
        if rejection_sum != self.rejected_rows {
            return Err(ManifestError::RejectionCountMismatch);
        }
        match (
            self.earliest_observed_at_unix_ms,
            self.latest_observed_at_unix_ms,
        ) {
            (Some(first), Some(last)) if first > last => {
                return Err(ManifestError::InvalidObservedWindow);
            }
            (Some(_), None) | (None, Some(_)) => return Err(ManifestError::InvalidObservedWindow),
            _ => {}
        }
        if self.maximum_batch_records == 0 {
            return Err(ManifestError::ZeroBatchLimit);
        }
        let expected_batches = if self.accepted_rows == 0 {
            0
        } else {
            self.accepted_rows
                .div_ceil(u64::from(self.maximum_batch_records))
        };
        if self.recommended_batches != expected_batches {
            return Err(ManifestError::RecommendedBatchMismatch);
        }
        if self.manifest_digest != compute_manifest_digest(self) {
            return Err(ManifestError::DigestMismatch);
        }
        Ok(())
    }
}

#[derive(Debug)]
pub enum DiagnosticError {
    Io(std::io::Error),
    Csv(csv::Error),
    AdapterConfiguration,
    InputTooLarge { maximum_bytes: u64 },
    HeaderMismatch,
    EmptyFile,
    ObservationCommitmentCollision,
    Manifest(ManifestError),
}

pub fn diagnose_delimited_import<R: Read>(
    adapter: &DelimitedIngressAdapter,
    reader: R,
    ingested_at_unix_ms: u64,
) -> Result<ImportDiagnosticManifest, DiagnosticError> {
    let descriptor = adapter
        .descriptor()
        .map_err(|_| DiagnosticError::AdapterConfiguration)?;
    let bytes = read_bounded(reader)?;
    let source_file_digest = digest_bytes("mycelix:import-source-file:v1", &bytes);

    let mut csv = ReaderBuilder::new()
        .delimiter(adapter.config.delimiter)
        .has_headers(true)
        .flexible(false)
        .trim(csv::Trim::None)
        .from_reader(bytes.as_slice());
    let actual_headers = csv.headers().map_err(DiagnosticError::Csv)?.clone();
    let expected_headers = StringRecord::from(adapter.config.expected_headers.clone());
    if actual_headers != expected_headers {
        return Err(DiagnosticError::HeaderMismatch);
    }

    let mut discovered_rows = 0_u64;
    let mut accepted_rows = 0_u64;
    let mut rejected_rows = 0_u64;
    let mut future_timestamp_rows = 0_u64;
    let mut first_observed = None;
    let mut last_observed = None;
    let mut maximum_delay = 0_u64;
    let mut rejection_counts = BTreeMap::new();
    let mut accepted_event_ids = BTreeSet::new();
    let mut accepted_event_keys = BTreeSet::new();
    let mut accepted_observation_commitments = BTreeSet::new();

    for result in csv.records() {
        discovered_rows = discovered_rows.saturating_add(1);
        let record = match result {
            Ok(record) => record,
            Err(_) => {
                record_rejection(
                    &mut rejected_rows,
                    &mut rejection_counts,
                    ImportRejectionClass::CsvStructure,
                );
                continue;
            }
        };

        let one_row =
            serialize_single_record(adapter.config.delimiter, &actual_headers, &record)?;
        match adapter.parse_batch(one_row.as_slice(), ingested_at_unix_ms) {
            Ok(batch) => {
                let Some(parsed) = batch.records.first() else {
                    record_rejection(
                        &mut rejected_rows,
                        &mut rejection_counts,
                        ImportRejectionClass::IngressValidation,
                    );
                    continue;
                };
                let event_id = parsed.event.witness.source_event_id.clone();
                if !accepted_event_ids.insert(event_id.clone()) {
                    record_rejection(
                        &mut rejected_rows,
                        &mut rejection_counts,
                        ImportRejectionClass::DuplicateSourceEvent,
                    );
                    continue;
                }
                let event_key = source_event_key_commitment(
                    &parsed.event.witness.source_system,
                    &event_id,
                );
                if !accepted_event_keys.insert(event_key) {
                    return Err(DiagnosticError::ObservationCommitmentCollision);
                }
                let mut row_observations = BTreeSet::new();
                for normalized in &parsed.normalized {
                    let commitment = normalized_observation_commitment(&normalized.observation);
                    if !row_observations.insert(commitment)
                        || accepted_observation_commitments.contains(&commitment)
                    {
                        return Err(DiagnosticError::ObservationCommitmentCollision);
                    }
                }
                accepted_observation_commitments.extend(row_observations);

                let observed = parsed.event.witness.observed_at_unix_ms;
                if observed > ingested_at_unix_ms {
                    future_timestamp_rows = future_timestamp_rows.saturating_add(1);
                } else {
                    maximum_delay = maximum_delay.max(ingested_at_unix_ms - observed);
                }
                first_observed = Some(first_observed.map_or(observed, |value: u64| value.min(observed)));
                last_observed = Some(last_observed.map_or(observed, |value: u64| value.max(observed)));
                accepted_rows = accepted_rows.saturating_add(1);
            }
            Err(error) => {
                record_rejection(
                    &mut rejected_rows,
                    &mut rejection_counts,
                    classify_adapter_error(&error),
                );
            }
        }
    }

    if discovered_rows == 0 {
        return Err(DiagnosticError::EmptyFile);
    }

    let recommended_batches = if accepted_rows == 0 {
        0
    } else {
        accepted_rows.div_ceil(u64::from(descriptor.maximum_batch_records))
    };
    let mut manifest = ImportDiagnosticManifest {
        connector: IngressQualificationBinding::from(&descriptor),
        supported_inputs: descriptor.supported_inputs,
        source_file_digest,
        discovered_rows,
        accepted_rows,
        rejected_rows,
        future_timestamp_rows,
        earliest_observed_at_unix_ms: first_observed,
        latest_observed_at_unix_ms: last_observed,
        maximum_observed_ingest_delay_ms: maximum_delay,
        maximum_batch_records: descriptor.maximum_batch_records,
        recommended_batches,
        rejection_counts,
        accepted_event_keys,
        accepted_observation_commitments,
        manifest_digest: Digest32([0; 32]),
    };
    manifest.manifest_digest = compute_manifest_digest(&manifest);
    manifest.validate().map_err(DiagnosticError::Manifest)?;
    Ok(manifest)
}

fn read_bounded<R: Read>(reader: R) -> Result<Vec<u8>, DiagnosticError> {
    let mut limited = reader.take(MAX_DIAGNOSTIC_INPUT_BYTES.saturating_add(1));
    let mut bytes = Vec::new();
    limited
        .read_to_end(&mut bytes)
        .map_err(DiagnosticError::Io)?;
    if bytes.len() as u64 > MAX_DIAGNOSTIC_INPUT_BYTES {
        return Err(DiagnosticError::InputTooLarge {
            maximum_bytes: MAX_DIAGNOSTIC_INPUT_BYTES,
        });
    }
    Ok(bytes)
}

fn serialize_single_record(
    delimiter: u8,
    headers: &StringRecord,
    record: &StringRecord,
) -> Result<Vec<u8>, DiagnosticError> {
    let mut bytes = Vec::new();
    {
        let mut writer = WriterBuilder::new().delimiter(delimiter).from_writer(&mut bytes);
        writer.write_record(headers).map_err(DiagnosticError::Csv)?;
        writer.write_record(record).map_err(DiagnosticError::Csv)?;
        writer.flush().map_err(DiagnosticError::Io)?;
    }
    Ok(bytes)
}

fn record_rejection(
    rejected_rows: &mut u64,
    rejection_counts: &mut BTreeMap<ImportRejectionClass, u64>,
    class: ImportRejectionClass,
) {
    *rejected_rows = rejected_rows.saturating_add(1);
    *rejection_counts.entry(class).or_insert(0) += 1;
}

fn compute_manifest_digest(manifest: &ImportDiagnosticManifest) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:import-diagnostic-manifest:v2");
    hash_str(&mut hasher, manifest.connector.source_system.as_str());
    hash_str(&mut hasher, manifest.connector.adapter_semantic_id.as_str());
    hasher.update(manifest.connector.adapter_digest.0);
    hasher.update(manifest.connector.mapping_digest.0);
    hasher.update(manifest.connector.source_schema_digest.0);
    for input in &manifest.supported_inputs {
        hash_str(&mut hasher, input.as_str());
    }
    hasher.update(manifest.source_file_digest.0);
    hasher.update(manifest.discovered_rows.to_be_bytes());
    hasher.update(manifest.accepted_rows.to_be_bytes());
    hasher.update(manifest.rejected_rows.to_be_bytes());
    hasher.update(manifest.future_timestamp_rows.to_be_bytes());
    for value in [
        manifest.earliest_observed_at_unix_ms,
        manifest.latest_observed_at_unix_ms,
    ] {
        match value {
            Some(value) => {
                hasher.update([1]);
                hasher.update(value.to_be_bytes());
            }
            None => hasher.update([0]),
        }
    }
    hasher.update(manifest.maximum_observed_ingest_delay_ms.to_be_bytes());
    hasher.update(manifest.maximum_batch_records.to_be_bytes());
    hasher.update(manifest.recommended_batches.to_be_bytes());
    for (class, count) in &manifest.rejection_counts {
        hasher.update([class.tag()]);
        hasher.update(count.to_be_bytes());
    }
    for commitment in &manifest.accepted_event_keys {
        hasher.update(commitment.0);
    }
    for commitment in &manifest.accepted_observation_commitments {
        hasher.update(commitment.0);
    }
    finish_digest(hasher)
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExtractionCampaignManifest {
    pub connector: IngressQualificationBinding,
    pub supported_inputs: BTreeSet<ReferenceId>,
    pub files: Vec<ImportDiagnosticManifest>,
    pub discovered_rows: u64,
    pub accepted_rows: u64,
    pub rejected_rows: u64,
    pub future_timestamp_rows: u64,
    pub maximum_observed_ingest_delay_ms: u64,
    pub accepted_event_keys: BTreeSet<Digest32>,
    pub accepted_observation_commitments: BTreeSet<Digest32>,
    pub campaign_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CampaignError {
    Empty,
    InvalidFile { index: usize, error: ManifestError },
    ConnectorMismatch,
    SupportedInputsMismatch,
    DuplicateSourceFile,
    DuplicateSourceEventAcrossFiles,
    DuplicateObservationAcrossFiles,
    EventCommitmentCountMismatch,
    Overflow,
    DigestMismatch,
}

impl ExtractionCampaignManifest {
    pub fn from_files(mut files: Vec<ImportDiagnosticManifest>) -> Result<Self, CampaignError> {
        if files.is_empty() {
            return Err(CampaignError::Empty);
        }
        for (index, file) in files.iter().enumerate() {
            file.validate()
                .map_err(|error| CampaignError::InvalidFile { index, error })?;
        }
        files.sort_by_key(|file| file.source_file_digest);
        let connector = files[0].connector.clone();
        let supported_inputs = files[0].supported_inputs.clone();
        let mut file_digests = BTreeSet::new();
        let mut event_keys = BTreeSet::new();
        let mut observations = BTreeSet::new();
        let mut discovered_rows = 0_u64;
        let mut accepted_rows = 0_u64;
        let mut rejected_rows = 0_u64;
        let mut future_timestamp_rows = 0_u64;
        let mut maximum_delay = 0_u64;
        for file in &files {
            if file.connector != connector {
                return Err(CampaignError::ConnectorMismatch);
            }
            if file.supported_inputs != supported_inputs {
                return Err(CampaignError::SupportedInputsMismatch);
            }
            if !file_digests.insert(file.source_file_digest) {
                return Err(CampaignError::DuplicateSourceFile);
            }
            for event_key in &file.accepted_event_keys {
                if !event_keys.insert(*event_key) {
                    return Err(CampaignError::DuplicateSourceEventAcrossFiles);
                }
            }
            for observation in &file.accepted_observation_commitments {
                if !observations.insert(*observation) {
                    return Err(CampaignError::DuplicateObservationAcrossFiles);
                }
            }
            discovered_rows = discovered_rows
                .checked_add(file.discovered_rows)
                .ok_or(CampaignError::Overflow)?;
            accepted_rows = accepted_rows
                .checked_add(file.accepted_rows)
                .ok_or(CampaignError::Overflow)?;
            rejected_rows = rejected_rows
                .checked_add(file.rejected_rows)
                .ok_or(CampaignError::Overflow)?;
            future_timestamp_rows = future_timestamp_rows
                .checked_add(file.future_timestamp_rows)
                .ok_or(CampaignError::Overflow)?;
            maximum_delay = maximum_delay.max(file.maximum_observed_ingest_delay_ms);
        }
        if event_keys.len() as u64 != accepted_rows {
            return Err(CampaignError::EventCommitmentCountMismatch);
        }
        let mut campaign = Self {
            connector,
            supported_inputs,
            files,
            discovered_rows,
            accepted_rows,
            rejected_rows,
            future_timestamp_rows,
            maximum_observed_ingest_delay_ms: maximum_delay,
            accepted_event_keys: event_keys,
            accepted_observation_commitments: observations,
            campaign_digest: Digest32([0; 32]),
        };
        campaign.campaign_digest = compute_campaign_digest(&campaign);
        Ok(campaign)
    }

    pub fn validate(&self) -> Result<(), CampaignError> {
        let rebuilt = Self::from_files(self.files.clone())?;
        if &rebuilt != self {
            return Err(CampaignError::DigestMismatch);
        }
        Ok(())
    }

    pub fn contains_observation(&self, observation: &MetricObservation) -> bool {
        self.accepted_observation_commitments
            .contains(&normalized_observation_commitment(observation))
    }
}

fn compute_campaign_digest(campaign: &ExtractionCampaignManifest) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:extraction-campaign:v2");
    hash_str(&mut hasher, campaign.connector.source_system.as_str());
    hash_str(&mut hasher, campaign.connector.adapter_semantic_id.as_str());
    hasher.update(campaign.connector.adapter_digest.0);
    hasher.update(campaign.connector.mapping_digest.0);
    hasher.update(campaign.connector.source_schema_digest.0);
    for input in &campaign.supported_inputs {
        hash_str(&mut hasher, input.as_str());
    }
    for file in &campaign.files {
        hasher.update(file.manifest_digest.0);
    }
    hasher.update(campaign.discovered_rows.to_be_bytes());
    hasher.update(campaign.accepted_rows.to_be_bytes());
    hasher.update(campaign.rejected_rows.to_be_bytes());
    hasher.update(campaign.future_timestamp_rows.to_be_bytes());
    hasher.update(campaign.maximum_observed_ingest_delay_ms.to_be_bytes());
    for commitment in &campaign.accepted_event_keys {
        hasher.update(commitment.0);
    }
    for commitment in &campaign.accepted_observation_commitments {
        hasher.update(commitment.0);
    }
    finish_digest(hasher)
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExtractionBoundHospitalityEvidence {
    pub campaign: ExtractionCampaignManifest,
    pub pilot_evidence: HospitalityPilotEvidence,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ExtractionBoundPilotError {
    Campaign(CampaignError),
    ConnectorMismatch,
    SalesInputNotMapped,
    FutureSourceTimestamp { rows: u64 },
    MissingSalesQualityEvidence,
    DenominatorMismatch { manifest: u64, evidence: u64 },
    RejectedRowsNotCountedMissing { rejected: u64, missing: u64 },
    IngestDelayMismatch { manifest: u64, evidence: u64 },
    UpstreamCompletenessLimitationMissing,
    Pilot(HospitalityPilotEvidenceError),
}

/// Evaluate a hospitality field gate only after the field-quality denominator is bound to the
/// non-mutating extraction campaign. Rejected parser rows cannot silently disappear.
pub fn evaluate_extraction_bound_hospitality_pilot(
    registration: &HospitalityForecastPilotRegistration,
    evidence: &ExtractionBoundHospitalityEvidence,
) -> Result<FieldQualificationDecision, ExtractionBoundPilotError> {
    evidence
        .campaign
        .validate()
        .map_err(ExtractionBoundPilotError::Campaign)?;
    if evidence.campaign.connector != registration.ingress_connector
        || evidence.pilot_evidence.ingress_connector != registration.ingress_connector
    {
        return Err(ExtractionBoundPilotError::ConnectorMismatch);
    }
    let sales_input = sales_input_ref();
    if !evidence.campaign.supported_inputs.contains(&sales_input) {
        return Err(ExtractionBoundPilotError::SalesInputNotMapped);
    }
    if evidence.campaign.future_timestamp_rows != 0 {
        return Err(ExtractionBoundPilotError::FutureSourceTimestamp {
            rows: evidence.campaign.future_timestamp_rows,
        });
    }

    let Some(quality) = sales_quality(&evidence.pilot_evidence.field_evidence, &sales_input) else {
        return Err(ExtractionBoundPilotError::MissingSalesQualityEvidence);
    };
    if quality.expected_records != evidence.campaign.discovered_rows {
        return Err(ExtractionBoundPilotError::DenominatorMismatch {
            manifest: evidence.campaign.discovered_rows,
            evidence: quality.expected_records,
        });
    }
    if quality.missing_records < evidence.campaign.rejected_rows {
        return Err(ExtractionBoundPilotError::RejectedRowsNotCountedMissing {
            rejected: evidence.campaign.rejected_rows,
            missing: quality.missing_records,
        });
    }
    if quality.maximum_observed_ingest_delay_ms
        != evidence.campaign.maximum_observed_ingest_delay_ms
    {
        return Err(ExtractionBoundPilotError::IngestDelayMismatch {
            manifest: evidence.campaign.maximum_observed_ingest_delay_ms,
            evidence: quality.maximum_observed_ingest_delay_ms,
        });
    }
    if !evidence
        .pilot_evidence
        .field_evidence
        .known_limitations
        .contains(&upstream_export_completeness_unverified_ref())
    {
        return Err(ExtractionBoundPilotError::UpstreamCompletenessLimitationMissing);
    }

    evaluate_hospitality_forecast_pilot(registration, &evidence.pilot_evidence)
        .map_err(ExtractionBoundPilotError::Pilot)
}

fn sales_quality<'a>(
    evidence: &'a FieldQualificationEvidence,
    sales_input: &ReferenceId,
) -> Option<&'a DataQualityEvidence> {
    evidence
        .data_quality
        .iter()
        .find(|quality| &quality.input == sales_input)
}

#[cfg(test)]
mod tests {
    use mycelix_business_adapter_delimited::{
        DecimalPolicy, DelimitedAdapterConfig, OutputMapping, ScopeMapping, TimestampEncoding,
        ValueMapping,
    };
    use mycelix_business_core::ScopeRef;
    use mycelix_business_field_qualification::{DataQualityEvidence, SliceEvidence};
    use mycelix_business_pilot_hospitality::{
        DAY_MS, HOUR_MS, HospitalityForecastPilotConfig, HospitalityPilotPolicy,
        standard_daypart_slices_v1,
    };

    use super::*;

    fn id(value: &str) -> ReferenceId {
        ReferenceId::new(value).unwrap()
    }

    fn adapter() -> DelimitedIngressAdapter {
        DelimitedIngressAdapter::new(DelimitedAdapterConfig {
            adapter_semantic_id: id("adapter:delimited:hospitality-sales:v1"),
            source_system: id("source:test-pos-export"),
            adapter_digest: Digest32::repeat(1),
            source_schema: id("schema:test-pos:v1"),
            delimiter: b',',
            expected_headers: vec![
                "event_id".into(),
                "occurred_at".into(),
                "location".into(),
                "quantity".into(),
            ],
            source_event_id_column: "event_id".into(),
            observed_at_column: "occurred_at".into(),
            timestamp_encoding: TimestampEncoding::UnixMilliseconds,
            scope: ScopeMapping::Column {
                column: "location".into(),
                prefix: "scope:location:".into(),
            },
            outputs: vec![OutputMapping {
                input: sales_input_ref(),
                metric: id("metric:hospitality:item-demand"),
                value: ValueMapping::Column {
                    column: "quantity".into(),
                    decimal: DecimalPolicy::default(),
                },
                unit: id("unit:count"),
                scale: 0,
            }],
            maximum_batch_records: 2,
        })
        .unwrap()
    }

    fn valid_csv() -> &'static [u8] {
        b"event_id,occurred_at,location,quantity\nevent:1,1000,a,1\nevent:2,1100,a,2\n"
    }

    #[test]
    fn diagnostics_keep_rejected_rows_in_denominator() {
        let csv = b"event_id,occurred_at,location,quantity\nevent:1,1000,a,1\nevent:2,1100,a,oops\nevent:1,1200,a,3\n";
        let manifest = diagnose_delimited_import(&adapter(), csv.as_slice(), 2_000).unwrap();
        assert!(IMPORT_DIAGNOSTICS_ARE_READ_ONLY);
        assert_eq!(manifest.discovered_rows, 3);
        assert_eq!(manifest.accepted_rows, 1);
        assert_eq!(manifest.rejected_rows, 2);
        assert_eq!(manifest.accepted_event_keys.len(), 1);
        assert_eq!(manifest.accepted_observation_commitments.len(), 1);
        assert_eq!(
            manifest.rejection_counts[&ImportRejectionClass::InvalidNumber],
            1
        );
        assert_eq!(
            manifest.rejection_counts[&ImportRejectionClass::DuplicateSourceEvent],
            1
        );
    }

    #[test]
    fn exact_normalized_observation_is_committed() {
        let adapter = adapter();
        let manifest = diagnose_delimited_import(&adapter, valid_csv(), 2_000).unwrap();
        let batch = adapter.parse_batch(valid_csv(), 2_000).unwrap();
        for record in &batch.records {
            for output in &record.normalized {
                assert!(manifest
                    .accepted_observation_commitments
                    .contains(&normalized_observation_commitment(&output.observation)));
            }
        }
    }

    #[test]
    fn campaign_rejects_same_source_event_across_files() {
        let first = diagnose_delimited_import(
            &adapter(),
            b"event_id,occurred_at,location,quantity\nevent:1,1000,a,1\n".as_slice(),
            2_000,
        )
        .unwrap();
        let second = diagnose_delimited_import(
            &adapter(),
            b"event_id,occurred_at,location,quantity\nevent:1,1200,a,2\n".as_slice(),
            2_000,
        )
        .unwrap();
        assert_eq!(
            ExtractionCampaignManifest::from_files(vec![first, second]),
            Err(CampaignError::DuplicateSourceEventAcrossFiles)
        );
    }

    fn pilot_registration(
        connector: IngressQualificationBinding,
    ) -> HospitalityForecastPilotRegistration {
        HospitalityForecastPilotRegistration::build(HospitalityForecastPilotConfig {
            plan_id: id("pilot:test:v1"),
            scope: ScopeRef(id("scope:location:a")),
            timezone: id("timezone:iana:Africa/Johannesburg"),
            candidate_model_lineage: id("model:test:v1"),
            connector,
            registered_at_unix_ms: DAY_MS,
            evaluation_start_unix_ms: 2 * DAY_MS,
            evaluation_end_unix_ms: 4 * DAY_MS,
            policy: HospitalityPilotPolicy {
                minimum_preregistration_lead_ms: HOUR_MS,
                minimum_evaluation_duration_ms: DAY_MS,
                minimum_total_forecast_cases: 1,
                minimum_cases_per_slice: 1,
                maximum_abstention_bps: 10_000,
                maximum_missing_bps: 10_000,
                maximum_conflicting_bps: 10_000,
                maximum_stale_bps: 10_000,
                maximum_ingest_delay_ms: 10_000,
            },
            slices: Some(standard_daypart_slices_v1(
                id("timezone:iana:Africa/Johannesburg"),
                1,
            )),
        })
        .unwrap()
    }

    fn field_evidence(
        registration: &HospitalityForecastPilotRegistration,
        expected_records: u64,
        missing_records: u64,
        maximum_delay: u64,
        include_limitation: bool,
    ) -> FieldQualificationEvidence {
        let mut limitations = BTreeSet::new();
        if include_limitation {
            limitations.insert(upstream_export_completeness_unverified_ref());
        }
        FieldQualificationEvidence {
            plan_digest: registration.field_plan.plan_digest,
            profile: registration.field_plan.profile.clone(),
            capability: registration.field_plan.capability.clone(),
            scope: registration.field_plan.scope.clone(),
            shadow_protocol_digest: registration.field_plan.shadow_protocol_digest,
            connectors: registration.field_plan.connectors.clone(),
            data_quality: vec![DataQualityEvidence {
                input: sales_input_ref(),
                expected_records,
                missing_records,
                conflicting_records: 0,
                stale_records: 0,
                maximum_observed_ingest_delay_ms: maximum_delay,
                evidence_digest: Digest32::repeat(12),
            }],
            slices: registration
                .slices
                .iter()
                .enumerate()
                .map(|(index, slice)| SliceEvidence {
                    slice: slice.slice.clone(),
                    criterion_digest: slice.criterion_digest,
                    cases: 1,
                    shadow_gate_passed: true,
                    evidence_digest: Digest32::repeat((20 + index) as u8),
                })
                .collect(),
            known_limitations: limitations,
            evidence_digest: Digest32::repeat(13),
        }
    }

    fn pilot_evidence(
        registration: &HospitalityForecastPilotRegistration,
        field_evidence: FieldQualificationEvidence,
    ) -> HospitalityPilotEvidence {
        HospitalityPilotEvidence {
            registration_digest: registration.registration_digest,
            ingress_connector: registration.ingress_connector.clone(),
            field_evidence,
        }
    }

    #[test]
    fn extraction_bound_pilot_refuses_free_form_denominator() {
        let manifest = diagnose_delimited_import(&adapter(), valid_csv(), 2_000).unwrap();
        let campaign = ExtractionCampaignManifest::from_files(vec![manifest]).unwrap();
        let registration = pilot_registration(campaign.connector.clone());
        let field = field_evidence(
            &registration,
            campaign.discovered_rows + 1,
            0,
            campaign.maximum_observed_ingest_delay_ms,
            true,
        );
        let evidence = ExtractionBoundHospitalityEvidence {
            campaign: campaign.clone(),
            pilot_evidence: pilot_evidence(&registration, field),
        };
        assert_eq!(
            evaluate_extraction_bound_hospitality_pilot(&registration, &evidence),
            Err(ExtractionBoundPilotError::DenominatorMismatch {
                manifest: campaign.discovered_rows,
                evidence: campaign.discovered_rows + 1,
            })
        );
    }

    #[test]
    fn complete_extraction_binding_still_only_passes_shadow_field_gate() {
        let manifest = diagnose_delimited_import(&adapter(), valid_csv(), 2_000).unwrap();
        let campaign = ExtractionCampaignManifest::from_files(vec![manifest]).unwrap();
        let registration = pilot_registration(campaign.connector.clone());
        let field = field_evidence(
            &registration,
            campaign.discovered_rows,
            campaign.rejected_rows,
            campaign.maximum_observed_ingest_delay_ms,
            true,
        );
        let evidence = ExtractionBoundHospitalityEvidence {
            campaign,
            pilot_evidence: pilot_evidence(&registration, field),
        };
        assert_eq!(
            evaluate_extraction_bound_hospitality_pilot(&registration, &evidence),
            Ok(FieldQualificationDecision::PassShadowFieldGate)
        );
    }
}
