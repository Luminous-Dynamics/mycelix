// Copyright (C) 2024-2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Canonical bounded replay of accepted external observations for Business qualification.
//!
//! New qualification layers should consume this replay rather than implement another row parser.
//! The replay exactly revalidates registered file manifests, preserves diagnostic rejection and
//! duplicate semantics, and returns only normalized witness-derived observations.

use std::collections::{BTreeMap, BTreeSet};

use csv::{ReaderBuilder, StringRecord, WriterBuilder};
use mycelix_business_adapter_delimited::DelimitedIngressAdapter;
use mycelix_business_core::{Digest32, ReferenceId};
use mycelix_business_import_diagnostics::{
    CampaignError, DiagnosticError, ExtractionCampaignManifest, diagnose_delimited_import,
};
use mycelix_business_import_membership::CampaignSourceFile;
use mycelix_business_ingress::{IngressQualificationBinding, NormalizedInput};
use mycelix_business_shadow::{MetricObservation, ScaledValue};
use sha2::{Digest, Sha256};

pub const CAMPAIGN_REPLAY_IS_READ_ONLY: bool = true;
pub const MAX_REPLAY_FILES: usize = 4_096;
pub const MAX_REPLAY_NORMALIZED_OBSERVATIONS: usize = 2_000_000;

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
pub struct ReplayedObservation {
    pub input: ReferenceId,
    pub observation: MetricObservation,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CampaignReplayEvidence {
    pub campaign_digest: Digest32,
    pub connector: IngressQualificationBinding,
    pub source_file_digests: Vec<Digest32>,
    pub accepted_source_events: u64,
    pub normalized_observations: u64,
    pub observation_set_digest: Digest32,
    pub evidence_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CampaignReplay {
    pub evidence: CampaignReplayEvidence,
    pub observations: Vec<ReplayedObservation>,
}

#[derive(Debug)]
pub enum ReplayError {
    Campaign(CampaignError),
    AdapterConfiguration,
    ConnectorMismatch,
    NoFiles,
    TooManyFiles { actual: usize, maximum: usize },
    SourceFileCountMismatch { expected: usize, actual: usize },
    Diagnostic { index: usize, error: DiagnosticError },
    UnexpectedSourceFile { index: usize },
    ManifestMismatch { index: usize, source_file_digest: Digest32 },
    MissingCampaignFile,
    Csv { index: usize },
    Serialization { index: usize },
    DuplicateCampaignSourceEvent { source_event_id: ReferenceId },
    AcceptedCountMismatch { expected: u64, actual: u64 },
    TooManyNormalizedObservations { maximum: usize },
    ArithmeticOverflow,
    InvalidEvidence,
}

impl CampaignReplayEvidence {
    pub fn validate_against(
        &self,
        campaign: &ExtractionCampaignManifest,
    ) -> Result<(), ReplayError> {
        campaign.validate().map_err(ReplayError::Campaign)?;
        if zero_digest(&self.observation_set_digest)
            || zero_digest(&self.evidence_digest)
            || self.campaign_digest != campaign.campaign_digest
            || self.connector != campaign.connector
            || self.accepted_source_events != campaign.accepted_rows
        {
            return Err(ReplayError::InvalidEvidence);
        }
        let expected_files = campaign
            .files
            .iter()
            .map(|file| file.source_file_digest)
            .collect::<Vec<_>>();
        if self.source_file_digests != expected_files {
            return Err(ReplayError::InvalidEvidence);
        }
        if self.evidence_digest != replay_evidence_digest(self) {
            return Err(ReplayError::InvalidEvidence);
        }
        Ok(())
    }
}

pub fn replay_campaign(
    adapter: &DelimitedIngressAdapter,
    campaign: &ExtractionCampaignManifest,
    files: &[CampaignSourceFile],
) -> Result<CampaignReplay, ReplayError> {
    campaign.validate().map_err(ReplayError::Campaign)?;
    if files.is_empty() {
        return Err(ReplayError::NoFiles);
    }
    if files.len() > MAX_REPLAY_FILES {
        return Err(ReplayError::TooManyFiles {
            actual: files.len(),
            maximum: MAX_REPLAY_FILES,
        });
    }
    if files.len() != campaign.files.len() {
        return Err(ReplayError::SourceFileCountMismatch {
            expected: campaign.files.len(),
            actual: files.len(),
        });
    }
    let descriptor = adapter
        .descriptor()
        .map_err(|_| ReplayError::AdapterConfiguration)?;
    let connector = IngressQualificationBinding::from(&descriptor);
    if connector != campaign.connector {
        return Err(ReplayError::ConnectorMismatch);
    }

    let mut expected_manifests = campaign
        .files
        .iter()
        .map(|manifest| (manifest.source_file_digest, manifest))
        .collect::<BTreeMap<_, _>>();
    let mut global_source_events = BTreeSet::new();
    let mut observations = Vec::new();
    let mut source_file_digests = Vec::with_capacity(files.len());

    for (file_index, file) in files.iter().enumerate() {
        let manifest = diagnose_delimited_import(
            adapter,
            file.bytes.as_slice(),
            file.ingested_at_unix_ms,
        )
        .map_err(|error| ReplayError::Diagnostic {
            index: file_index,
            error,
        })?;
        let Some(expected) = expected_manifests.remove(&manifest.source_file_digest) else {
            return Err(ReplayError::UnexpectedSourceFile { index: file_index });
        };
        if &manifest != expected {
            return Err(ReplayError::ManifestMismatch {
                index: file_index,
                source_file_digest: manifest.source_file_digest,
            });
        }
        source_file_digests.push(manifest.source_file_digest);
        replay_file(
            adapter,
            file,
            file_index,
            &mut global_source_events,
            &mut observations,
        )?;
    }
    if !expected_manifests.is_empty() {
        return Err(ReplayError::MissingCampaignFile);
    }

    let accepted_source_events = u64::try_from(global_source_events.len())
        .map_err(|_| ReplayError::ArithmeticOverflow)?;
    if accepted_source_events != campaign.accepted_rows {
        return Err(ReplayError::AcceptedCountMismatch {
            expected: campaign.accepted_rows,
            actual: accepted_source_events,
        });
    }
    let normalized_observations =
        u64::try_from(observations.len()).map_err(|_| ReplayError::ArithmeticOverflow)?;
    source_file_digests.sort();
    observations.sort_by(|left, right| {
        left.input
            .cmp(&right.input)
            .then_with(|| left.observation.observation.cmp(&right.observation.observation))
    });
    let observation_set_digest = observation_set_digest(&observations);
    let mut evidence = CampaignReplayEvidence {
        campaign_digest: campaign.campaign_digest,
        connector,
        source_file_digests,
        accepted_source_events,
        normalized_observations,
        observation_set_digest,
        evidence_digest: Digest32([0; 32]),
    };
    evidence.evidence_digest = replay_evidence_digest(&evidence);
    evidence.validate_against(campaign)?;
    Ok(CampaignReplay {
        evidence,
        observations,
    })
}

fn replay_file(
    adapter: &DelimitedIngressAdapter,
    file: &CampaignSourceFile,
    file_index: usize,
    global_source_events: &mut BTreeSet<ReferenceId>,
    observations: &mut Vec<ReplayedObservation>,
) -> Result<(), ReplayError> {
    let mut csv = ReaderBuilder::new()
        .delimiter(adapter.config.delimiter)
        .has_headers(true)
        .flexible(false)
        .trim(csv::Trim::None)
        .from_reader(file.bytes.as_slice());
    let headers = csv
        .headers()
        .map_err(|_| ReplayError::Csv { index: file_index })?
        .clone();
    let expected = StringRecord::from(adapter.config.expected_headers.clone());
    if headers != expected {
        return Err(ReplayError::Csv { index: file_index });
    }

    let mut local_source_events = BTreeSet::new();
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
        let source_event = parsed.event.witness.source_event_id.clone();
        if !local_source_events.insert(source_event.clone()) {
            // Mirror diagnostics: first accepted occurrence counts, later same-file repeats do not.
            continue;
        }
        if !global_source_events.insert(source_event.clone()) {
            return Err(ReplayError::DuplicateCampaignSourceEvent {
                source_event_id: source_event,
            });
        }
        for normalized in &parsed.normalized {
            push_normalized(observations, normalized)?;
        }
    }
    Ok(())
}

fn push_normalized(
    observations: &mut Vec<ReplayedObservation>,
    normalized: &NormalizedInput,
) -> Result<(), ReplayError> {
    if observations.len() >= MAX_REPLAY_NORMALIZED_OBSERVATIONS {
        return Err(ReplayError::TooManyNormalizedObservations {
            maximum: MAX_REPLAY_NORMALIZED_OBSERVATIONS,
        });
    }
    observations.push(ReplayedObservation {
        input: normalized.input.clone(),
        observation: normalized.observation.clone(),
    });
    Ok(())
}

fn serialize_single_record(
    delimiter: u8,
    headers: &StringRecord,
    record: &StringRecord,
    file_index: usize,
) -> Result<Vec<u8>, ReplayError> {
    let mut bytes = Vec::new();
    {
        let mut writer = WriterBuilder::new().delimiter(delimiter).from_writer(&mut bytes);
        writer
            .write_record(headers)
            .map_err(|_| ReplayError::Serialization { index: file_index })?;
        writer
            .write_record(record)
            .map_err(|_| ReplayError::Serialization { index: file_index })?;
        writer
            .flush()
            .map_err(|_| ReplayError::Serialization { index: file_index })?;
    }
    Ok(bytes)
}

fn observation_set_digest(observations: &[ReplayedObservation]) -> Digest32 {
    let mut digests = observations
        .iter()
        .map(replayed_observation_digest)
        .collect::<Vec<_>>();
    digests.sort_unstable();
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:campaign-replay-observation-set:v1");
    hasher.update((digests.len() as u64).to_be_bytes());
    for digest in digests {
        hasher.update(digest.0);
    }
    finish_digest(hasher)
}

fn replayed_observation_digest(value: &ReplayedObservation) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:campaign-replayed-observation:v1");
    hash_str(&mut hasher, value.input.as_str());
    hash_metric_observation(&mut hasher, &value.observation);
    finish_digest(hasher)
}

fn hash_metric_observation(hasher: &mut Sha256, observation: &MetricObservation) {
    hash_str(hasher, observation.observation.as_ref_id().as_str());
    hash_str(hasher, observation.source_system.as_str());
    hash_str(hasher, observation.source_event_id.as_str());
    hasher.update(observation.source_payload_digest.0);
    hasher.update(observation.mapping_digest.0);
    hash_str(hasher, observation.metric.as_str());
    hash_str(hasher, observation.scope.as_ref_id().as_str());
    hash_scaled_value(hasher, &observation.value);
    hasher.update(observation.observed_at_unix_ms.to_be_bytes());
}

fn hash_scaled_value(hasher: &mut Sha256, value: &ScaledValue) {
    hasher.update(value.mantissa.to_be_bytes());
    hasher.update(value.scale.to_be_bytes());
    hash_str(hasher, value.unit.as_str());
}

fn replay_evidence_digest(evidence: &CampaignReplayEvidence) -> Digest32 {
    let mut hasher = Sha256::new();
    hash_str(&mut hasher, "mycelix:campaign-replay-evidence:v1");
    hasher.update(evidence.campaign_digest.0);
    hash_str(&mut hasher, evidence.connector.source_system.as_str());
    hash_str(&mut hasher, evidence.connector.adapter_semantic_id.as_str());
    hasher.update(evidence.connector.adapter_digest.0);
    hasher.update(evidence.connector.mapping_digest.0);
    hasher.update(evidence.connector.source_schema_digest.0);
    for digest in &evidence.source_file_digests {
        hasher.update(digest.0);
    }
    hasher.update(evidence.accepted_source_events.to_be_bytes());
    hasher.update(evidence.normalized_observations.to_be_bytes());
    hasher.update(evidence.observation_set_digest.0);
    finish_digest(hasher)
}

#[cfg(test)]
mod tests {
    use mycelix_business_adapter_delimited::{
        DecimalPolicy, DelimitedAdapterConfig, OutputMapping, ScopeMapping, TimestampEncoding,
        ValueMapping,
    };
    use mycelix_business_core::ScopeRef;
    use mycelix_business_import_diagnostics::diagnose_delimited_import;

    use super::*;

    fn id(value: &str) -> ReferenceId {
        ReferenceId::new(value).unwrap()
    }

    fn adapter() -> DelimitedIngressAdapter {
        DelimitedIngressAdapter::new(DelimitedAdapterConfig {
            adapter_semantic_id: id("adapter:replay-test:v1"),
            source_system: id("source:test-pos"),
            adapter_digest: Digest32::repeat(1),
            source_schema: id("schema:replay-test:v1"),
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

    fn source(bytes: &[u8]) -> CampaignSourceFile {
        CampaignSourceFile {
            bytes: bytes.to_vec(),
            ingested_at_unix_ms: 10_000,
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
    fn replay_returns_only_accepted_normalized_observations() {
        let adapter = adapter();
        let files = vec![source(
            b"event_id,occurred_at,location,quantity\nevent:1,1000,a,2\nevent:2,1100,a,oops\nevent:1,1200,a,3\nevent:3,1300,a,4\n",
        )];
        let campaign = campaign(&adapter, &files);
        let replay = replay_campaign(&adapter, &campaign, &files).unwrap();
        assert!(CAMPAIGN_REPLAY_IS_READ_ONLY);
        assert_eq!(replay.evidence.accepted_source_events, 2);
        assert_eq!(replay.observations.len(), 2);
        assert_eq!(replay.observations[0].observation.value.mantissa, 2);
        assert_eq!(replay.observations[1].observation.value.mantissa, 4);
        assert!(replay.evidence.validate_against(&campaign).is_ok());
    }

    #[test]
    fn cross_file_duplicate_fails_closed() {
        let adapter = adapter();
        let files = vec![
            source(b"event_id,occurred_at,location,quantity\nevent:1,1000,a,2\n"),
            source(b"event_id,occurred_at,location,quantity\nevent:1,1100,a,3\n"),
        ];
        let campaign = campaign(&adapter, &files);
        assert!(matches!(
            replay_campaign(&adapter, &campaign, &files),
            Err(ReplayError::DuplicateCampaignSourceEvent { .. })
        ));
    }

    #[test]
    fn replay_keeps_input_identity_for_downstream_projection() {
        let adapter = adapter();
        let files = vec![source(
            b"event_id,occurred_at,location,quantity\nevent:1,1000,a,2\n",
        )];
        let campaign = campaign(&adapter, &files);
        let replay = replay_campaign(&adapter, &campaign, &files).unwrap();
        assert_eq!(
            replay.observations[0].input,
            id("input:hospitality:sales-transactions:v1")
        );
        assert_eq!(
            replay.observations[0].observation.scope,
            ScopeRef(id("scope:location:a"))
        );
    }
}
