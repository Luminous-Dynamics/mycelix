// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Authority-neutral privacy leakage observation evidence for Mycelix PEC.
//!
//! This crate records what was measured under an exact experiment. It does not
//! translate a negative result into a privacy theorem.

use privacy_computation_core::{BackendIdentity, QualificationState, SemanticAuthority};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum LeakageObservationDimension {
    Identity,
    ParticipantCount,
    InputSize,
    OutputSize,
    SetSize,
    IntersectionSize,
    QueryIndex,
    AccessPattern,
    Timing,
    MessageSize,
    AbortBehavior,
    DropoutBehavior,
    CrossSessionLinkability,
    IdentifierReuse,
    TranscriptCorrelation,
    ResultValue,
    AuxiliaryInformationSensitivity,
}

#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ObserverRole {
    LocalClient,
    RemotePeer,
    Coordinator,
    Verifier,
    Issuer,
    DatabaseServer,
    NetworkObserver,
    ColludingPartySet { profile: String },
    CompromisedEndpoint,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ObservationDisposition {
    NotMeasured,
    NoDifferenceDetectedUnderExperiment,
    StatisticalSignalObserved,
    DeterministicallyRevealed,
    DerivedFromProtocolDefinition,
    Inconclusive,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LeakageObservation {
    pub dimension: LeakageObservationDimension,
    pub observer: ObserverRole,
    pub disposition: ObservationDisposition,
    /// Human/machine-readable metric identity, such as `welch-t-v1` or
    /// `classifier-auc-v1`. This is an identity only, not validation.
    pub metric_profile: String,
    /// Optional textual summary kept deliberately non-authoritative. Raw or
    /// structured measurements should live behind the evidence reference.
    pub summary: Option<String>,
    pub evidence_digest: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LeakageExperimentIdentity {
    pub experiment_profile: String,
    pub subject_backend: BackendIdentity,
    pub subject_qualification: QualificationState,
    pub subject_revision: String,
    pub workload_digest: String,
    pub execution_capsule_digest: String,
    pub measurement_method_digest: String,
    pub observer_profile: String,
}

impl LeakageExperimentIdentity {
    pub fn is_structurally_complete(&self) -> bool {
        !self.experiment_profile.trim().is_empty()
            && !self.subject_backend.backend.trim().is_empty()
            && !self.subject_backend.version.trim().is_empty()
            && !self.subject_backend.profile.trim().is_empty()
            && !self.subject_revision.trim().is_empty()
            && !self.workload_digest.trim().is_empty()
            && !self.execution_capsule_digest.trim().is_empty()
            && !self.measurement_method_digest.trim().is_empty()
            && !self.observer_profile.trim().is_empty()
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LeakageReceipt {
    pub experiment: LeakageExperimentIdentity,
    pub observations: Vec<LeakageObservation>,
    pub raw_evidence_digest: Option<String>,
    pub nonclaims: Vec<String>,
    pub authority: SemanticAuthority,
}

impl LeakageReceipt {
    pub fn new(
        experiment: LeakageExperimentIdentity,
        observations: Vec<LeakageObservation>,
        raw_evidence_digest: Option<String>,
        nonclaims: Vec<String>,
    ) -> Self {
        Self {
            experiment,
            observations,
            raw_evidence_digest,
            nonclaims,
            authority: SemanticAuthority::StructuralOnly,
        }
    }

    pub const fn privacy_established(&self) -> bool {
        false
    }

    pub const fn leakage_exhaustively_measured(&self) -> bool {
        false
    }

    pub const fn cryptographic_theorem_proven(&self) -> bool {
        false
    }

    pub const fn production_admission_granted(&self) -> bool {
        false
    }

    pub const fn application_authority_granted(&self) -> bool {
        false
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum LeakageEvidenceFailure {
    IncompleteExperimentIdentity,
    NoObservations,
    MissingMetricProfile,
    MissingEvidenceDigest,
    EmptyCollusionProfile,
}

pub fn validate_receipt(receipt: &LeakageReceipt) -> Result<(), LeakageEvidenceFailure> {
    if !receipt.experiment.is_structurally_complete() {
        return Err(LeakageEvidenceFailure::IncompleteExperimentIdentity);
    }

    if receipt.observations.is_empty() {
        return Err(LeakageEvidenceFailure::NoObservations);
    }

    for observation in &receipt.observations {
        if observation.metric_profile.trim().is_empty() {
            return Err(LeakageEvidenceFailure::MissingMetricProfile);
        }
        if observation.evidence_digest.trim().is_empty() {
            return Err(LeakageEvidenceFailure::MissingEvidenceDigest);
        }
        if let ObserverRole::ColludingPartySet { profile } = &observation.observer {
            if profile.trim().is_empty() {
                return Err(LeakageEvidenceFailure::EmptyCollusionProfile);
            }
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use privacy_computation_core::PrivacyPrimitive;

    fn experiment() -> LeakageExperimentIdentity {
        LeakageExperimentIdentity {
            experiment_profile: "timing-v1".into(),
            subject_backend: BackendIdentity {
                primitive: PrivacyPrimitive::PrivateInformationRetrieval,
                backend: "synthetic-pir".into(),
                version: "0.0.1".into(),
                profile: "single-server-demo".into(),
            },
            subject_qualification: QualificationState::Experimental,
            subject_revision: "git:deadbeef".into(),
            workload_digest: "sha256:workload".into(),
            execution_capsule_digest: "sha256:capsule".into(),
            measurement_method_digest: "sha256:method".into(),
            observer_profile: "server-view-v1".into(),
        }
    }

    fn observation(disposition: ObservationDisposition) -> LeakageObservation {
        LeakageObservation {
            dimension: LeakageObservationDimension::Timing,
            observer: ObserverRole::DatabaseServer,
            disposition,
            metric_profile: "welch-t-v1".into(),
            summary: None,
            evidence_digest: "sha256:measurement".into(),
        }
    }

    #[test]
    fn negative_experiment_result_does_not_mint_privacy() {
        let receipt = LeakageReceipt::new(
            experiment(),
            vec![observation(
                ObservationDisposition::NoDifferenceDetectedUnderExperiment,
            )],
            Some("sha256:raw".into()),
            vec!["timing leakage absence not established".into()],
        );
        assert_eq!(validate_receipt(&receipt), Ok(()));
        assert_eq!(receipt.authority, SemanticAuthority::StructuralOnly);
        assert!(!receipt.privacy_established());
        assert!(!receipt.leakage_exhaustively_measured());
        assert!(!receipt.cryptographic_theorem_proven());
        assert!(!receipt.production_admission_granted());
        assert!(!receipt.application_authority_granted());
    }

    #[test]
    fn not_measured_is_distinct_from_negative_result() {
        assert_ne!(
            ObservationDisposition::NotMeasured,
            ObservationDisposition::NoDifferenceDetectedUnderExperiment
        );
    }

    #[test]
    fn observer_identity_is_theorem_bearing() {
        let mut server = observation(ObservationDisposition::Inconclusive);
        let mut network = server.clone();
        network.observer = ObserverRole::NetworkObserver;
        assert_ne!(server.observer, network.observer);

        server.observer = ObserverRole::ColludingPartySet {
            profile: "server-plus-issuer".into(),
        };
        assert_ne!(server.observer, network.observer);
    }

    #[test]
    fn empty_collusion_profile_fails_closed() {
        let mut obs = observation(ObservationDisposition::StatisticalSignalObserved);
        obs.observer = ObserverRole::ColludingPartySet {
            profile: String::new(),
        };
        let receipt = LeakageReceipt::new(experiment(), vec![obs], None, vec![]);
        assert_eq!(
            validate_receipt(&receipt),
            Err(LeakageEvidenceFailure::EmptyCollusionProfile)
        );
    }

    #[test]
    fn incomplete_experiment_identity_fails_closed() {
        let mut exp = experiment();
        exp.workload_digest.clear();
        let receipt = LeakageReceipt::new(
            exp,
            vec![observation(ObservationDisposition::Inconclusive)],
            None,
            vec![],
        );
        assert_eq!(
            validate_receipt(&receipt),
            Err(LeakageEvidenceFailure::IncompleteExperimentIdentity)
        );
    }

    #[test]
    fn wire_names_are_stable() {
        assert_eq!(
            serde_json::to_string(&LeakageObservationDimension::CrossSessionLinkability).unwrap(),
            "\"CrossSessionLinkability\""
        );
        assert_eq!(
            serde_json::to_string(&ObservationDisposition::NoDifferenceDetectedUnderExperiment)
                .unwrap(),
            "\"NoDifferenceDetectedUnderExperiment\""
        );
    }
}
