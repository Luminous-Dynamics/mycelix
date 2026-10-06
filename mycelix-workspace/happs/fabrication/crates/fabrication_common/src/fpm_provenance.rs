//! Deterministic provenance-disjointness qualification for FPM.
//!
//! This module qualifies the supplied lineage graph, not real-world independence.
//! It rejects missing lineage, cycles, duplicate nodes, mismatched observation
//! bindings, and shared acquisition ancestry between required participants.

use crate::fpm_qualification::source_observation_binding_digest;
use crate::fpm_registration::RegistrationEnvelope;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};

pub const FPM_PROVENANCE_QUALIFICATION_SCHEMA_VERSION: &str =
    "fpm.registration.provenance-disjointness.v1";
pub const FPM_PROVENANCE_PROFILE_ID: &str =
    "fpm.registration.provenance-disjointness";
pub const FPM_PROVENANCE_PROFILE_VERSION: &str = "1";
const SHA256_HEX_LEN: usize = 64;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct AcquisitionLineageWitness {
    /// Stable identifier for this lineage node within the supplied provenance graph.
    pub node_id: String,
    /// Exact source/modality participant represented by this lineage witness.
    pub source_id: String,
    pub modality: String,
    /// Exact source-observation binding digest for this participant.
    pub source_observation_digest: String,
    /// Root acquisition commitment for this lineage component.
    pub acquisition_root_digest: String,
    /// Direct parent node identifiers.
    pub parent_node_ids: Vec<String>,
}

impl AcquisitionLineageWitness {
    pub fn digest(&self) -> String {
        let mut bytes = Vec::new();
        append_field(&mut bytes, b"fpm.acquisition-lineage-node.v2");
        append_field(&mut bytes, self.node_id.as_bytes());
        append_field(&mut bytes, self.source_id.as_bytes());
        append_field(&mut bytes, self.modality.as_bytes());
        append_field(&mut bytes, self.source_observation_digest.as_bytes());
        append_field(&mut bytes, self.acquisition_root_digest.as_bytes());
        for parent in &self.parent_node_ids {
            append_field(&mut bytes, parent.as_bytes());
        }
        hex_digest(&bytes)
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ProvenanceQualificationInput {
    pub registration_envelope_digest: String,
    pub envelope: RegistrationEnvelope,
    pub lineage: Vec<AcquisitionLineageWitness>,
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProvenanceQualificationStatus {
    QualifiedForProfile,
    InsufficientEvidence,
    InvalidEvidence,
    ConflictingProvenance,
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum ProvenanceQualificationReason {
    EnvelopeDigestMismatch,
    MissingParticipantWitness,
    DuplicateParticipantWitness,
    DuplicateLineageNode,
    InvalidDigestEncoding,
    LineageDigestMismatch,
    ObservationBindingMismatch,
    MissingParent,
    LineageCycle,
    SharedAcquisitionRoot,
    SharedAncestry,
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ProvenanceQualification {
    pub schema_version: String,
    pub registration_envelope_digest: String,
    pub lineage_manifest_digest: String,
    pub profile_id: String,
    pub profile_version: String,
    pub status: ProvenanceQualificationStatus,
    pub reasons: Vec<ProvenanceQualificationReason>,
}

pub fn qualify_provenance(
    input: &ProvenanceQualificationInput,
) -> ProvenanceQualification {
    let mut reasons = BTreeSet::new();

    if !is_canonical_digest(&input.registration_envelope_digest)
        || input
            .envelope
            .digest()
            .map(|digest| digest != input.registration_envelope_digest)
            .unwrap_or(true)
    {
        reasons.insert(ProvenanceQualificationReason::EnvelopeDigestMismatch);
    }

    let expected_participants = participant_keys(&input.envelope);
    let mut by_participant = BTreeMap::new();
    let mut by_node_id = BTreeMap::new();
    let mut by_digest = BTreeMap::new();

    for witness in &input.lineage {
        if witness.node_id.trim().is_empty()
            || !is_canonical_digest(&witness.source_observation_digest)
            || !is_canonical_digest(&witness.acquisition_root_digest)
        {
            reasons.insert(ProvenanceQualificationReason::InvalidDigestEncoding);
        }

        let node_digest = witness.digest();
        if by_digest.insert(node_digest.clone(), witness.clone()).is_some() {
            reasons.insert(ProvenanceQualificationReason::DuplicateLineageNode);
        }
        if by_node_id.insert(witness.node_id.clone(), node_digest.clone()).is_some() {
            reasons.insert(ProvenanceQualificationReason::DuplicateLineageNode);
        }

        let participant = (witness.source_id.clone(), witness.modality.clone());
        if by_participant.insert(participant, node_digest).is_some() {
            reasons.insert(ProvenanceQualificationReason::DuplicateParticipantWitness);
        }
    }

    for participant in &expected_participants {
        if !by_participant.contains_key(participant) {
            reasons.insert(ProvenanceQualificationReason::MissingParticipantWitness);
            continue;
        }

        let node_digest = by_participant[participant].clone();
        let witness = &by_digest[&node_digest];

        let expected_observation = participant_observation_digest(&input.envelope, participant);
        if expected_observation != Some(witness.source_observation_digest.clone()) {
            reasons.insert(ProvenanceQualificationReason::ObservationBindingMismatch);
        }

        if witness.digest() != node_digest {
            reasons.insert(ProvenanceQualificationReason::LineageDigestMismatch);
        }
    }

    for witness in &input.lineage {
        for parent_id in &witness.parent_node_ids {
            if !by_node_id.contains_key(parent_id) {
                reasons.insert(ProvenanceQualificationReason::MissingParent);
            }
        }
    }

    let roots = expected_participants
        .iter()
        .filter_map(|participant| by_participant.get(participant))
        .filter_map(|node_digest| by_digest.get(node_digest))
        .map(|witness| witness.acquisition_root_digest.clone())
        .collect::<Vec<_>>();

    if roots.len() != roots.iter().collect::<BTreeSet<_>>().len() {
        reasons.insert(ProvenanceQualificationReason::SharedAcquisitionRoot);
    }

    let participant_nodes = expected_participants
        .iter()
        .filter_map(|participant| by_participant.get(participant))
        .cloned()
        .collect::<Vec<_>>();

    for left in 0..participant_nodes.len() {
        for right in (left + 1)..participant_nodes.len() {
            let a = lineage_ancestors(&participant_nodes[left], &by_node_id, &by_digest, &mut BTreeSet::new());
            let b = lineage_ancestors(&participant_nodes[right], &by_node_id, &by_digest, &mut BTreeSet::new());
            if a.intersection(&b).next().is_some() {
                reasons.insert(ProvenanceQualificationReason::SharedAncestry);
            }
        }
    }

    for node_id in by_node_id.keys() {
        if graph_has_cycle(node_id, &by_node_id, &by_digest) {
            reasons.insert(ProvenanceQualificationReason::LineageCycle);
            break;
        }
    }

    let lineage_manifest_digest = lineage_manifest_digest(&input.lineage);
    let status = if reasons.is_empty() {
        ProvenanceQualificationStatus::QualifiedForProfile
    } else if reasons.iter().any(|reason| {
        matches!(
            reason,
            ProvenanceQualificationReason::EnvelopeDigestMismatch
                | ProvenanceQualificationReason::DuplicateLineageNode
                | ProvenanceQualificationReason::DuplicateParticipantWitness
                | ProvenanceQualificationReason::InvalidDigestEncoding
                | ProvenanceQualificationReason::LineageDigestMismatch
                | ProvenanceQualificationReason::ObservationBindingMismatch
                | ProvenanceQualificationReason::LineageCycle
        )
    }) {
        ProvenanceQualificationStatus::InvalidEvidence
    } else if reasons.contains(&ProvenanceQualificationReason::SharedAcquisitionRoot)
        || reasons.contains(&ProvenanceQualificationReason::SharedAncestry)
    {
        ProvenanceQualificationStatus::ConflictingProvenance
    } else {
        ProvenanceQualificationStatus::InsufficientEvidence
    };

    ProvenanceQualification {
        schema_version: FPM_PROVENANCE_QUALIFICATION_SCHEMA_VERSION.into(),
        registration_envelope_digest: input.registration_envelope_digest.clone(),
        lineage_manifest_digest,
        profile_id: FPM_PROVENANCE_PROFILE_ID.into(),
        profile_version: FPM_PROVENANCE_PROFILE_VERSION.into(),
        status,
        reasons: reasons.into_iter().collect(),
    }
}

fn participant_keys(envelope: &RegistrationEnvelope) -> Vec<(String, String)> {
    std::iter::once(&envelope.reference)
        .chain(envelope.related.iter())
        .map(|participant| (participant.source_id.clone(), participant.modality.clone()))
        .collect()
}

fn participant_observation_digest(
    envelope: &RegistrationEnvelope,
    participant: &(String, String),
) -> Option<String> {
    std::iter::once(&envelope.reference)
        .chain(envelope.related.iter())
        .find(|candidate| candidate.source_id == participant.0 && candidate.modality == participant.1)
        .map(source_observation_binding_digest)
}

fn lineage_ancestors(
    node_id: &str,
    by_node_id: &BTreeMap<String, String>,
    by_digest: &BTreeMap<String, AcquisitionLineageWitness>,
    visiting: &mut BTreeSet<String>,
) -> BTreeSet<String> {
    let mut ancestors = BTreeSet::new();
    if !visiting.insert(node_id.to_string()) {
        return ancestors;
    }

    let Some(node_digest) = by_node_id.get(node_id) else {
        visiting.remove(node_id);
        return ancestors;
    };
    let Some(witness) = by_digest.get(node_digest) else {
        visiting.remove(node_id);
        return ancestors;
    };

    for parent_id in &witness.parent_node_ids {
        ancestors.insert(parent_id.clone());
        ancestors.extend(lineage_ancestors(parent_id, by_node_id, by_digest, visiting));
    }

    visiting.remove(node_id);
    ancestors
}

fn graph_has_cycle(
    node_id: &str,
    by_node_id: &BTreeMap<String, String>,
    by_digest: &BTreeMap<String, AcquisitionLineageWitness>,
) -> bool {
    fn visit(
        node_id: &str,
        by_node_id: &BTreeMap<String, String>,
        by_digest: &BTreeMap<String, AcquisitionLineageWitness>,
        active: &mut BTreeSet<String>,
        done: &mut BTreeSet<String>,
    ) -> bool {
        if active.contains(node_id) {
            return true;
        }
        if done.contains(node_id) {
            return false;
        }

        let Some(node_digest) = by_node_id.get(node_id) else {
            return false;
        };
        let Some(witness) = by_digest.get(node_digest) else {
            return false;
        };

        active.insert(node_id.to_string());
        for parent_id in &witness.parent_node_ids {
            if visit(parent_id, by_node_id, by_digest, active, done) {
                return true;
            }
        }
        active.remove(node_id);
        done.insert(node_id.to_string());
        false
    }

    visit(node_id, by_node_id, by_digest, &mut BTreeSet::new(), &mut BTreeSet::new())
}

fn lineage_manifest_digest(lineage: &[AcquisitionLineageWitness]) -> String {
    let mut entries = lineage
        .iter()
        .map(AcquisitionLineageWitness::digest)
        .collect::<Vec<_>>();
    entries.sort_unstable();

    let mut bytes = Vec::new();
    append_field(&mut bytes, b"fpm.provenance-lineage-manifest.v1");
    for digest in entries {
        append_field(&mut bytes, digest.as_bytes());
    }
    hex_digest(&bytes)
}

fn is_canonical_digest(value: &str) -> bool {
    value.len() == SHA256_HEX_LEN
        && value.bytes().all(|byte| matches!(byte, b'0'..=b'9' | b'a'..=b'f'))
}

fn append_field(buffer: &mut Vec<u8>, field: &[u8]) {
    buffer.extend_from_slice(&(field.len() as u64).to_be_bytes());
    buffer.extend_from_slice(field);
}

fn hex_digest(bytes: &[u8]) -> String {
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    hasher.finalize().iter().map(|byte| format!("{byte:02x}")).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fpm_registration::{AlignmentMethod, ModalityObservationRef, FPM_REGISTRATION_SCHEMA_VERSION};

    fn sample(source: &str, modality: &str, sequence: u64, data: &[u8]) -> ModalityObservationRef {
        ModalityObservationRef {
            source_id: source.into(),
            modality: modality.into(),
            clock_domain: "ptp-domain-1".into(),
            source_sequence: sequence,
            correlation_domain: "printer-frame-domain-1".into(),
            correlation_id: format!("frame-{sequence}"),
            source_timestamp_micros: Some(1_000_000),
            calibration_profile_digest: digest('a'),
            process_context_digest: digest('b'),
            source_data_digest: hex_digest(data),
        }
    }

    fn digest(ch: char) -> String {
        std::iter::repeat(ch).take(SHA256_HEX_LEN).collect()
    }

    fn witness(
        participant: &ModalityObservationRef,
        root: &str,
        parents: Vec<String>,
    ) -> AcquisitionLineageWitness {
        AcquisitionLineageWitness {
            source_id: participant.source_id.clone(),
            modality: participant.modality.clone(),
            source_observation_digest: source_observation_binding_digest(participant),
            acquisition_root_digest: root.into(),
            node_id: format!("{}-node", participant.source_id),
        parent_node_ids: parents,
        }
    }

    fn qualified_input() -> ProvenanceQualificationInput {
        let reference = sample("thermal-1", "thermal", 10, b"thermal");
        let related = sample("vibration-1", "vibration", 10, b"vibration");
        let envelope = RegistrationEnvelope {
            schema_version: FPM_REGISTRATION_SCHEMA_VERSION.into(),
            reference: reference.clone(),
            related: vec![related.clone()],
            alignment_method: Some(AlignmentMethod::ExactCorrelationId),
        };
        let root_a = hex_digest(b"root-a");
        let root_b = hex_digest(b"root-b");
        let lineage = vec![
            witness(&reference, &root_a, vec![]),
            witness(&related, &root_b, vec![]),
        ];
        ProvenanceQualificationInput {
            registration_envelope_digest: envelope.digest().unwrap(),
            envelope,
            lineage,
        }
    }

    #[test]
    fn disjoint_lineage_is_qualified() {
        assert_eq!(
            qualify_provenance(&qualified_input()).status,
            ProvenanceQualificationStatus::QualifiedForProfile
        );
    }

    #[test]
    fn shared_root_is_conflicting_not_independent() {
        let mut input = qualified_input();
        input.lineage[1].acquisition_root_digest = input.lineage[0].acquisition_root_digest.clone();
        assert_eq!(
            qualify_provenance(&input).status,
            ProvenanceQualificationStatus::ConflictingProvenance
        );
    }

    #[test]
    fn shared_ancestor_is_conflicting() {
        let mut input = qualified_input();
        let shared = witness(
            &input.envelope.reference,
            &hex_digest(b"shared-root"),
            vec![],
        );
        let shared_digest = shared.digest();
        input.lineage.push(shared);
        input.lineage[0].parent_digests = vec![shared_digest.clone()];
        input.lineage[1].parent_digests = vec![shared_digest];
        assert_eq!(
            qualify_provenance(&input).status,
            ProvenanceQualificationStatus::ConflictingProvenance
        );
    }

    #[test]
    fn missing_parent_is_insufficient() {
        let mut input = qualified_input();
        input.lineage[0].parent_digests = vec![digest('e')];
        assert_eq!(
            qualify_provenance(&input).status,
            ProvenanceQualificationStatus::InsufficientEvidence
        );
        assert!(qualify_provenance(&input)
            .reasons
            .contains(&ProvenanceQualificationReason::MissingParent));
    }

    #[test]
    fn cycle_is_invalid() {
        let mut input = qualified_input();
        input.lineage[0].node_id = "node-a".into();
        input.lineage[1].node_id = "node-b".into();
        input.lineage[0].parent_node_ids = vec!["node-b".into()];
        input.lineage[1].parent_node_ids = vec!["node-a".into()];
        assert_eq!(
            qualify_provenance(&input).status,
            ProvenanceQualificationStatus::InvalidEvidence
        );
        assert!(qualify_provenance(&input)
            .reasons
            .contains(&ProvenanceQualificationReason::LineageCycle));
    }

    #[test]
    fn observation_binding_mismatch_is_invalid() {
        let mut input = qualified_input();
        input.lineage[0].source_observation_digest = digest('f');
        assert_eq!(
            qualify_provenance(&input).status,
            ProvenanceQualificationStatus::InvalidEvidence
        );
        assert!(qualify_provenance(&input)
            .reasons
            .contains(&ProvenanceQualificationReason::ObservationBindingMismatch));
    }

    #[test]
    fn replay_with_new_source_id_without_new_observation_binding_is_invalid() {
        let mut input = qualified_input();
        input.lineage[0].source_id = "thermal-replay".into();
        assert_eq!(
            qualify_provenance(&input).status,
            ProvenanceQualificationStatus::InvalidEvidence
        );
    }
}
