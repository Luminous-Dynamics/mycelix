// Copyright (C) 2024-2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Read-only external witness ingress for the Mycelix Business Fabric.
//!
//! This crate has no outbound mutation or credential types. Accepted records remain witness
//! evidence and do not become authoritative business truth merely by ingestion.

use std::collections::{BTreeMap, BTreeSet};

use mycelix_business_core::{Digest32, ReferenceId};
use mycelix_business_shadow::{MetricObservation, SourceWitness, WitnessError, WitnessRegistry};

pub const INGRESS_IS_READ_ONLY: bool = true;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExternalAdapterDescriptor {
    pub adapter_semantic_id: ReferenceId,
    pub source_system: ReferenceId,
    pub adapter_digest: Digest32,
    pub source_schema: ReferenceId,
    pub source_schema_digest: Digest32,
    pub mapping_digest: Digest32,
    pub supported_inputs: BTreeSet<ReferenceId>,
    pub maximum_batch_records: u32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AdapterDescriptorError {
    ZeroAdapterDigest,
    ZeroSourceSchemaDigest,
    ZeroMappingDigest,
    NoSupportedInputs,
    ZeroMaximumBatchRecords,
}

fn zero_digest(value: &Digest32) -> bool {
    value == &Digest32([0; 32])
}

impl ExternalAdapterDescriptor {
    pub fn validate(&self) -> Result<(), AdapterDescriptorError> {
        if zero_digest(&self.adapter_digest) {
            return Err(AdapterDescriptorError::ZeroAdapterDigest);
        }
        if zero_digest(&self.source_schema_digest) {
            return Err(AdapterDescriptorError::ZeroSourceSchemaDigest);
        }
        if zero_digest(&self.mapping_digest) {
            return Err(AdapterDescriptorError::ZeroMappingDigest);
        }
        if self.supported_inputs.is_empty() {
            return Err(AdapterDescriptorError::NoSupportedInputs);
        }
        if self.maximum_batch_records == 0 {
            return Err(AdapterDescriptorError::ZeroMaximumBatchRecords);
        }
        Ok(())
    }
}

/// External-event metadata without raw payload bytes or credentials.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExternalEventEnvelope {
    pub witness: SourceWitness,
    pub source_schema: ReferenceId,
    pub source_schema_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NormalizedInput {
    pub input: ReferenceId,
    pub observation: MetricObservation,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IngressRecord {
    pub event: ExternalEventEnvelope,
    pub mapping_digest: Digest32,
    pub normalized: Vec<NormalizedInput>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum IngressRecordError {
    InvalidDescriptor(AdapterDescriptorError),
    InvalidWitness(WitnessError),
    SourceSystemMismatch,
    SourceSchemaMismatch,
    SourceSchemaDigestMismatch,
    MappingDigestMismatch,
    NoNormalizedOutputs,
    UnsupportedInput { input: ReferenceId },
    ObservationSourceSystemMismatch { input: ReferenceId },
    ObservationSourceEventMismatch { input: ReferenceId },
    ObservationPayloadDigestMismatch { input: ReferenceId },
    ObservationMappingDigestMismatch { input: ReferenceId },
    DuplicateInputObservation { input: ReferenceId },
}

impl IngressRecord {
    pub fn validate_against(
        &self,
        descriptor: &ExternalAdapterDescriptor,
    ) -> Result<(), IngressRecordError> {
        descriptor
            .validate()
            .map_err(IngressRecordError::InvalidDescriptor)?;
        self.event
            .witness
            .validate()
            .map_err(IngressRecordError::InvalidWitness)?;

        if self.event.witness.source_system != descriptor.source_system {
            return Err(IngressRecordError::SourceSystemMismatch);
        }
        if self.event.source_schema != descriptor.source_schema {
            return Err(IngressRecordError::SourceSchemaMismatch);
        }
        if self.event.source_schema_digest != descriptor.source_schema_digest {
            return Err(IngressRecordError::SourceSchemaDigestMismatch);
        }
        if self.mapping_digest != descriptor.mapping_digest {
            return Err(IngressRecordError::MappingDigestMismatch);
        }
        if self.normalized.is_empty() {
            return Err(IngressRecordError::NoNormalizedOutputs);
        }

        let mut seen = BTreeSet::new();
        for output in &self.normalized {
            if !descriptor.supported_inputs.contains(&output.input) {
                return Err(IngressRecordError::UnsupportedInput {
                    input: output.input.clone(),
                });
            }
            output
                .observation
                .validate()
                .map_err(|_| IngressRecordError::ObservationPayloadDigestMismatch {
                    input: output.input.clone(),
                })?;
            if output.observation.source_system != self.event.witness.source_system {
                return Err(IngressRecordError::ObservationSourceSystemMismatch {
                    input: output.input.clone(),
                });
            }
            if output.observation.source_event_id != self.event.witness.source_event_id {
                return Err(IngressRecordError::ObservationSourceEventMismatch {
                    input: output.input.clone(),
                });
            }
            if output.observation.source_payload_digest != self.event.witness.payload_digest {
                return Err(IngressRecordError::ObservationPayloadDigestMismatch {
                    input: output.input.clone(),
                });
            }
            if output.observation.mapping_digest != self.mapping_digest {
                return Err(IngressRecordError::ObservationMappingDigestMismatch {
                    input: output.input.clone(),
                });
            }
            let identity = (output.input.clone(), output.observation.observation.clone());
            if !seen.insert(identity) {
                return Err(IngressRecordError::DuplicateInputObservation {
                    input: output.input.clone(),
                });
            }
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IngressBatch {
    pub adapter_digest: Digest32,
    pub records: Vec<IngressRecord>,
    pub batch_digest: Digest32,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum IngressBatchError {
    InvalidDescriptor(AdapterDescriptorError),
    AdapterDigestMismatch,
    ZeroBatchDigest,
    EmptyBatch,
    BatchTooLarge { actual: usize, maximum: u32 },
    InvalidRecord { index: usize, error: IngressRecordError },
    DuplicateSourceEvent { source_event_id: ReferenceId },
    WitnessConflict { index: usize, error: WitnessError },
}

impl IngressBatch {
    pub fn validate_and_register(
        &self,
        descriptor: &ExternalAdapterDescriptor,
        registry: &mut WitnessRegistry,
    ) -> Result<(), IngressBatchError> {
        descriptor
            .validate()
            .map_err(IngressBatchError::InvalidDescriptor)?;
        if self.adapter_digest != descriptor.adapter_digest {
            return Err(IngressBatchError::AdapterDigestMismatch);
        }
        if zero_digest(&self.batch_digest) {
            return Err(IngressBatchError::ZeroBatchDigest);
        }
        if self.records.is_empty() {
            return Err(IngressBatchError::EmptyBatch);
        }
        if self.records.len() > descriptor.maximum_batch_records as usize {
            return Err(IngressBatchError::BatchTooLarge {
                actual: self.records.len(),
                maximum: descriptor.maximum_batch_records,
            });
        }

        let mut batch_events = BTreeSet::new();
        for (index, record) in self.records.iter().enumerate() {
            record
                .validate_against(descriptor)
                .map_err(|error| IngressBatchError::InvalidRecord { index, error })?;
            if !batch_events.insert(record.event.witness.source_event_id.clone()) {
                return Err(IngressBatchError::DuplicateSourceEvent {
                    source_event_id: record.event.witness.source_event_id.clone(),
                });
            }
        }

        // Do not partially mutate the caller's registry if a later witness conflicts.
        let mut staged = registry.clone();
        for (index, record) in self.records.iter().enumerate() {
            staged
                .accept(&record.event.witness)
                .map_err(|error| IngressBatchError::WitnessConflict { index, error })?;
        }
        *registry = staged;
        Ok(())
    }
}

/// Evidence-only summary suitable for field-qualification connector binding.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IngressQualificationBinding {
    pub source_system: ReferenceId,
    pub adapter_semantic_id: ReferenceId,
    pub adapter_digest: Digest32,
    pub mapping_digest: Digest32,
    pub source_schema_digest: Digest32,
}

impl From<&ExternalAdapterDescriptor> for IngressQualificationBinding {
    fn from(value: &ExternalAdapterDescriptor) -> Self {
        Self {
            source_system: value.source_system.clone(),
            adapter_semantic_id: value.adapter_semantic_id.clone(),
            adapter_digest: value.adapter_digest,
            mapping_digest: value.mapping_digest,
            source_schema_digest: value.source_schema_digest,
        }
    }
}

/// Count normalized outputs per declared input without interpreting them as authoritative truth.
pub fn normalized_input_counts(batch: &IngressBatch) -> BTreeMap<ReferenceId, u64> {
    let mut result = BTreeMap::new();
    for record in &batch.records {
        for output in &record.normalized {
            *result.entry(output.input.clone()).or_insert(0) += 1;
        }
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_business_core::{ObservationRef, ScopeRef};
    use mycelix_business_shadow::ScaledValue;

    fn id(value: &str) -> ReferenceId {
        ReferenceId::new(value).unwrap()
    }

    fn descriptor() -> ExternalAdapterDescriptor {
        ExternalAdapterDescriptor {
            adapter_semantic_id: id("adapter:pos:readonly:v1"),
            source_system: id("source:pos"),
            adapter_digest: Digest32::repeat(1),
            source_schema: id("schema:pos:event:v1"),
            source_schema_digest: Digest32::repeat(2),
            mapping_digest: Digest32::repeat(3),
            supported_inputs: BTreeSet::from([id("input:hospitality:sales-transactions:v1")]),
            maximum_batch_records: 100,
        }
    }

    fn record(event_id: &str, payload: u8) -> IngressRecord {
        let witness = SourceWitness {
            source_system: id("source:pos"),
            source_event_id: id(event_id),
            payload_digest: Digest32::repeat(payload),
            observed_at_unix_ms: 1_000,
            ingested_at_unix_ms: 1_010,
        };
        IngressRecord {
            event: ExternalEventEnvelope {
                witness: witness.clone(),
                source_schema: id("schema:pos:event:v1"),
                source_schema_digest: Digest32::repeat(2),
            },
            mapping_digest: Digest32::repeat(3),
            normalized: vec![NormalizedInput {
                input: id("input:hospitality:sales-transactions:v1"),
                observation: MetricObservation {
                    observation: ObservationRef(id(&format!("observation:{event_id}"))),
                    source_system: witness.source_system,
                    source_event_id: witness.source_event_id,
                    source_payload_digest: witness.payload_digest,
                    mapping_digest: Digest32::repeat(3),
                    metric: id("metric:hospitality:item-demand"),
                    scope: ScopeRef(id("scope:location:a")),
                    value: ScaledValue {
                        mantissa: 1,
                        scale: 0,
                        unit: id("unit:count"),
                    },
                    observed_at_unix_ms: 1_000,
                },
            }],
        }
    }

    #[test]
    fn record_binds_source_schema_payload_and_mapping() {
        assert_eq!(record("event:1", 4).validate_against(&descriptor()), Ok(()));
    }

    #[test]
    fn unsupported_input_fails_closed() {
        let mut value = record("event:1", 4);
        value.normalized[0].input = id("input:undeclared");
        assert!(matches!(
            value.validate_against(&descriptor()),
            Err(IngressRecordError::UnsupportedInput { .. })
        ));
    }

    #[test]
    fn observation_cannot_detach_from_source_payload() {
        let mut value = record("event:1", 4);
        value.normalized[0].observation.source_payload_digest = Digest32::repeat(9);
        assert!(matches!(
            value.validate_against(&descriptor()),
            Err(IngressRecordError::ObservationPayloadDigestMismatch { .. })
        ));
    }

    #[test]
    fn duplicate_event_in_batch_fails_closed() {
        let batch = IngressBatch {
            adapter_digest: Digest32::repeat(1),
            records: vec![record("event:1", 4), record("event:1", 4)],
            batch_digest: Digest32::repeat(8),
        };
        assert!(matches!(
            batch.validate_and_register(&descriptor(), &mut WitnessRegistry::default()),
            Err(IngressBatchError::DuplicateSourceEvent { .. })
        ));
    }

    #[test]
    fn conflicting_historical_witness_does_not_partially_commit_batch() {
        let mut registry = WitnessRegistry::default();
        let first = IngressBatch {
            adapter_digest: Digest32::repeat(1),
            records: vec![record("event:old", 4)],
            batch_digest: Digest32::repeat(7),
        };
        first.validate_and_register(&descriptor(), &mut registry).unwrap();
        let before = registry.clone();

        let conflict = IngressBatch {
            adapter_digest: Digest32::repeat(1),
            records: vec![record("event:new", 5), record("event:old", 9)],
            batch_digest: Digest32::repeat(8),
        };
        assert!(matches!(
            conflict.validate_and_register(&descriptor(), &mut registry),
            Err(IngressBatchError::WitnessConflict { .. })
        ));
        assert_eq!(registry, before);
    }

    #[test]
    fn ingress_surface_is_structurally_read_only() {
        assert!(INGRESS_IS_READ_ONLY);
        let binding = IngressQualificationBinding::from(&descriptor());
        assert_eq!(binding.source_system, id("source:pos"));
        assert_eq!(binding.mapping_digest, Digest32::repeat(3));
    }
}
