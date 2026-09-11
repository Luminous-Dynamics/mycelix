// Copyright (C) 2024-2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Campaign-wide integrity evidence for read-only Mycelix Business imports.
//!
//! Per-file diagnostics are necessary but not sufficient: the same provider event can otherwise
//! appear once in multiple valid exports and be counted repeatedly. This crate replays the exact
//! campaign and proves source-event uniqueness across all files without persisting event IDs.

use std::collections::{BTreeMap, BTreeSet};

use csv::{ReaderBuilder, StringRecord, WriterBuilder};
use mycelix_business_adapter_delimited::DelimitedIngressAdapter;
use mycelix_business_core::{Digest32, ReferenceId};
use mycelix_business_import_diagnostics::{
    CampaignError, DiagnosticError, ExtractionCampaignManifest, ImportDiagnosticManifest,
    diagnose_delimited_import,
};
use mycelix_business_import_membership::CampaignSourceFile;
use mycelix_business_ingress::IngressQualificationBinding;
use sha2::{Digest, Sha256};

pub const CAMPAIGN_INTEGRITY_IS_READ_ONLY: bool = true;

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

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CampaignIntegrityEvidence {
    pub campaign_digest: Digest32,
    pub connector: IngressQualificationBinding,
    pub source_file_digests: Vec<Digest32>,
    pub unique_source_events: u64,
    pub source_event_set_digest: Digest32,
    pub earliest_observed_at_unix_ms: Option<u64>,
    pub latest_observed_at_unix_ms: Option<u64>,
    pub evidence_digest: Digest32,
}

#[derive(Debug)]
pub enum IntegrityError {
    Campaign(CampaignError),
    AdapterConfiguration,
    ConnectorMismatch,
    SourceFileCountMismatch { expected: usize, actual: usize },
    Diagnostic { index: usize, error: DiagnosticError },
    UnexpectedSourceFile { index: usize },
    ManifestMismatch { index: usize, source_file_digest: Digest32 },
    MissingCampaignFile,
    Csv { index: usize },
    Serialization { index: usize },
    DuplicateCampaignSourceEvent { source_event_id: ReferenceId },
    AcceptedCountMismatch { manifest_total: u64, unique_events: u64 },
    CoverageMismatch,
    ArithmeticOverflow,
    InvalidEvidence,
}

impl CampaignIntegrityEvidence {
    pub fn validate_against(
        &self,
        campaign: &ExtractionCampaignManifest,
    ) -> Result<(), IntegrityError> {
        campaign.validate().map_err(IntegrityError::Campaign)?;
        if zero_digest(&self.evidence_digest)
            || zero_digest(&self.source_event_set_digest)
            || self.campaign_digest != campaign.campaign_digest
            || self.connector != campaign.connector
        {
            return Err(IntegrityError::InvalidEvidence);
        }
        let expected_files = campaign
            .files
            .iter()
            .map(|file| file.source_file_digest)
            .collect::<Vec<_>>();
        if self.source_file_digests != expected_files {
            return Err(IntegrityError::InvalidEvidence);
        }
        if self.unique_source_events != campaign.accepted_rows {
            return Err(IntegrityError::AcceptedCountMismatch {
                manifest_total: campaign.accepted_rows,
                unique_events: self.unique_source_events,
            });
        }
        let (expected_first, expected_last) = manifest_coverage(&campaign.files);
        if self.earliest_observed_at_unix_ms != expected_first
            || self.latest_observed_at_unix_ms != expected_last
        {
            return Err(IntegrityError::CoverageMismatch);
        }
        if self.evidence_digest != integrity_evidence_digest(self) {
            return Err(IntegrityError::InvalidEvidence);
        }
        Ok(())
    }
}

/// Replay the exact registered campaign and prove that an accepted provider event appears at most
/// once across the entire campaign. Duplicate rows within one file retain the diagnostic scanner's
/// first-accepted / later-rejected semantics.
pub fn verify_campaign_integrity(
    adapter: &DelimitedIngressAdapter,
    campaign: &ExtractionCampaignManifest,
    files: &[CampaignSourceFile],
) -> Result<CampaignIntegrityEvidence, IntegrityError> {
    campaign.validate().map_err(IntegrityError::Campaign)?;
    let descriptor = adapter
        .descriptor()
        .map_err(|_| IntegrityError::AdapterConfiguration)?;
    let connector = IngressQualificationBinding::from(&descriptor);
    if connector != campaign.connector {
        return Err(IntegrityError::ConnectorMismatch);
    }
    if files.len() != campaign.files.len() {
        return Err(IntegrityError::SourceFileCountMismatch {
            expected: campaign.files.len(),
            actual: files.len(),
        });
    }

    let mut expected_manifests = campaign
        .files
        .iter()
        .map(|manifest| (manifest.source_file_digest, manifest))
        .collect::<BTreeMap<_, _>>();
    let mut global_events = BTreeSet::new();
    let mut source_file_digests = Vec::with_capacity(files.len());
    let mut earliest = None;
    let mut latest = None;

    for (file_index, file) in files.iter().enumerate() {
        let manifest = diagnose_delimited_import(
            adapter,
            file.bytes.as_slice(),
            file.ingested_at_unix_ms,
        )
        .map_err(|error| IntegrityError::Diagnostic {
            index: file_index,
            error,
        })?;
        let Some(expected) = expected_manifests.remove(&manifest.source_file_digest) else {
            return Err(IntegrityError::UnexpectedSourceFile { index: file_index });
        };
        if &manifest != expected {
            return Err(IntegrityError::ManifestMismatch {
                index: file_index,
                source_file_digest: manifest.source_file_digest,
            });
        }
        source_file_digests.push(manifest.source_file_digest);
        scan_file_events(
            adapter,
            file,
            file_index,
            &mut global_events,
            &mut earliest,
            &mut latest,
        )?;
    }
    if !expected_manifests.is_empty() {
        return Err(IntegrityError::MissingCampaignFile);
    }

    let unique_source_events =
        u64::try_from(global_events.len()).map_err(|_| IntegrityError::ArithmeticOverflow)?;
    if unique_source_events != campaign.accepted_rows {
        return Err(IntegrityError::AcceptedCountMismatch {
            manifest_total: campaign.accepted_rows,
            unique_events: unique_source_events,
        });
    }
    let expected_coverage = manifest_coverage(&campaign.files);
    if (earliest, latest) != expected_coverage {
        return Err(IntegrityError::CoverageMismatch);
    }

    source_file_digests.sort();
    let source_event_set_digest = source_event_set_digest(&connector.source_system, &global_events);
    let mut evidence = CampaignIntegrityEvidence {
        campaign_digest: campaign.campaign_digest,
        connector,
        source_file_digests,
        unique_source_events,
        source_event_set_digest,
        earliest_observed_at_unix_ms: earliest,
        latest_observed_at_unix_ms: latest,
        evidence_digest: Digest32([0; 32]),
    };
    evidence.evidence_digest = integrity_evidence_digest(&evidence);
    evidence.validate_against(campaign)?;
    Ok(evidence)
}

fn scan_file_events(
    adapter: &DelimitedIngressAdapter,
    file: &CampaignSourceFile,
    file_index: usize,
    global_events: &mut BTreeSet<ReferenceId>,
    earliest: &mut Option<u64>,
    latest: &mut Option<u64>,
) -> Result<(), IntegrityError> {
    let mut csv = ReaderBuilder::new()
        .delimiter(adapter.config.delimiter)
        .has_headers(true)
        .flexible(false)
        .trim(csv::Trim::None)
        .from_reader(file.bytes.as_slice());
    let headers = csv
        .headers()
        .map_err(|_| IntegrityError::Csv { index: file_index })?
        .clone();
    let expected = StringRecord::from(adapter.config.expected_headers.clone());
    if headers != expected {
        return Err(IntegrityError::Csv { index: file_index });
    }

    let mut local_events = BTreeSet::new();
    for row in csv.records() {
        let Ok(record) = row else {
            continue;
        };
        let one_row = serialize_single_record(adapter.config.delimiter, &headers, &record, file_index)?;
        let Ok(batch) = adapter.parse_batch(one_row.as_slice(), file.ingested_at_unix_ms) else {
            continue;
        };
        let Some(parsed) = batch.records.first() else {
            continue;
        };
        let event = parsed.event.witness.source_event_id.clone();
        if !local_events.insert(event.clone()) {
            // Same-file repeats are rejected by diagnostics after the first accepted occurrence.
            continue;
        }
        if !global_events.insert(event.clone()) {
            return Err(IntegrityError::DuplicateCampaignSourceEvent {
                source_event_id: event,
            });
        }
        let observed = parsed.event.witness.observed_at_unix_ms;
        *earliest = Some(earliest.map_or(observed, |value| value.min(observed)));
        *latest = Some(latest.map_or(observed, |value| value.max(observed)));
    }
    Ok(())
}

fn serialize_single_record(
    delimiter: u8,
    headers: &StringRecord,
    record: &StringRecord,
    file_index: usize,
) -> Result<Vec<u8>, IntegrityError> {
    let mut bytes = Vec::new();
    {
        let mut writer = WriterBuilder::new().delimiter(delimiter).from_writer(&mut bytes);
        writer
            .write_record(headers)
            .map_err(|_| IntegrityError::Serialization { index: file_index })?;
        writer
            .write_record(record)
            .map_err(|_| IntegrityError::Serialization { index: file_index })?;
        writer
            .flush()
            .map_err(|_| IntegrityError::Serialization { index: file_index })?;
    }
    Ok(bytes)
}

fn manifest_coverage(files: &[ImportDiagnosticManifest]) -> (Option<u64>, Option<u64>) {
    let earliest = files
        .iter()
        .filter_map(|file| file.earliest_observed_at_unix_ms)
        .min();
    let latest = files
        .iter()
        .filter_map(|file| file.latest_observed_at_unix_ms)
        .max();
    (earliest, latest)
}

fn source_event_set_digest(
    source_system: &ReferenceId,
    events: &BTreeSet<ReferenceId>,
) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:campaign-source-event-set:v1");
    hash_str(&mut hasher, source_system.as_str());
    hasher.update((events.len() as u64).to_be_bytes());
    for event in events {
        hash_str(&mut hasher, event.as_str());
    }
    finish_digest(hasher)
}

fn integrity_evidence_digest(evidence: &CampaignIntegrityEvidence) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:campaign-integrity-evidence:v1");
    hasher.update(evidence.campaign_digest.0);
    hash_str(&mut hasher, evidence.connector.source_system.as_str());
    hash_str(&mut hasher, evidence.connector.adapter_semantic_id.as_str());
    hasher.update(evidence.connector.adapter_digest.0);
    hasher.update(evidence.connector.mapping_digest.0);
    hasher.update(evidence.connector.source_schema_digest.0);
    for digest in &evidence.source_file_digests {
        hasher.update(digest.0);
    }
    hasher.update(evidence.unique_source_events.to_be_bytes());
    hasher.update(evidence.source_event_set_digest.0);
    for value in [
        evidence.earliest_observed_at_unix_ms,
        evidence.latest_observed_at_unix_ms,
    ] {
        match value {
            Some(value) => {
                hasher.update([1]);
                hasher.update(value.to_be_bytes());
            }
            None => hasher.update([0]),
        }
    }
    finish_digest(hasher)
}

#[cfg(test)]
mod tests {
    use mycelix_business_adapter_delimited::{
        DecimalPolicy, DelimitedAdapterConfig, OutputMapping, ScopeMapping, TimestampEncoding,
        ValueMapping,
    };
    use mycelix_business_core::Digest32;
    use mycelix_business_import_diagnostics::{
        ExtractionCampaignManifest, diagnose_delimited_import,
    };

    use super::*;

    fn id(value: &str) -> ReferenceId {
        ReferenceId::new(value).unwrap()
    }

    fn adapter() -> DelimitedIngressAdapter {
        DelimitedIngressAdapter::new(DelimitedAdapterConfig {
            adapter_semantic_id: id("adapter:campaign-test:v1"),
            source_system: id("source:test-pos"),
            adapter_digest: Digest32::repeat(1),
            source_schema: id("schema:campaign-test:v1"),
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
                input: id("input:hospitality:sales-transactions:v1"),
                metric: id("metric:hospitality:item-demand"),
                value: ValueMapping::Column {
                    column: "quantity".into(),
                    decimal: DecimalPolicy::default(),
                },
                unit: id("unit:count"),
                scale: 0,
            }],
            maximum_batch_records: 100,
        })
        .unwrap()
    }

    fn file(bytes: &[u8], ingested_at_unix_ms: u64) -> CampaignSourceFile {
        CampaignSourceFile {
            bytes: bytes.to_vec(),
            ingested_at_unix_ms,
        }
    }

    fn campaign(
        adapter: &DelimitedIngressAdapter,
        files: &[CampaignSourceFile],
    ) -> ExtractionCampaignManifest {
        let manifests = files
            .iter()
            .map(|file| {
                diagnose_delimited_import(
                    adapter,
                    file.bytes.as_slice(),
                    file.ingested_at_unix_ms,
                )
                .unwrap()
            })
            .collect::<Vec<_>>();
        ExtractionCampaignManifest::from_files(manifests).unwrap()
    }

    #[test]
    fn globally_unique_events_produce_digest_bound_evidence() {
        let adapter = adapter();
        let files = vec![
            file(
                b"event_id,occurred_at,location,quantity\nevent:1,1000,a,1\nevent:2,1100,a,2\n",
                2_000,
            ),
            file(
                b"event_id,occurred_at,location,quantity\nevent:3,1200,a,3\nevent:4,1300,a,4\n",
                2_000,
            ),
        ];
        let campaign = campaign(&adapter, &files);
        let evidence = verify_campaign_integrity(&adapter, &campaign, &files).unwrap();
        assert!(CAMPAIGN_INTEGRITY_IS_READ_ONLY);
        assert_eq!(evidence.unique_source_events, 4);
        assert_eq!(evidence.validate_against(&campaign), Ok(()));
    }

    #[test]
    fn event_repeated_across_two_valid_files_fails_closed() {
        let adapter = adapter();
        let files = vec![
            file(
                b"event_id,occurred_at,location,quantity\nevent:1,1000,a,1\nevent:2,1100,a,2\n",
                2_000,
            ),
            file(
                b"event_id,occurred_at,location,quantity\nevent:2,1100,a,2\nevent:3,1200,a,3\n",
                2_000,
            ),
        ];
        let campaign = campaign(&adapter, &files);
        assert!(matches!(
            verify_campaign_integrity(&adapter, &campaign, &files),
            Err(IntegrityError::DuplicateCampaignSourceEvent { .. })
        ));
    }

    #[test]
    fn same_file_duplicate_retains_diagnostic_first_acceptance_semantics() {
        let adapter = adapter();
        let files = vec![file(
            b"event_id,occurred_at,location,quantity\nevent:1,1000,a,1\nevent:1,1000,a,1\nevent:2,1100,a,2\n",
            2_000,
        )];
        let campaign = campaign(&adapter, &files);
        assert_eq!(campaign.accepted_rows, 2);
        let evidence = verify_campaign_integrity(&adapter, &campaign, &files).unwrap();
        assert_eq!(evidence.unique_source_events, 2);
    }
}
