// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Native provenance composition for one pinned Holochain target and two exact
//! coordinator-code observations inside one qualified Linux conductor-process fence.
//!
//! This crate deliberately stops before release matching, subject/attempt authority
//! and effects. Its positive output is historical observation provenance only and has
//! no future validity lease. A later admission orchestrator may consume the retained
//! short-lived target/code evidence immediately through #290/#298, but must never
//! interpret this completed process fence as future conductor authority.

use holo_hash::{AgentPubKey, DnaHash};
use holochain_client::CellId;
use mycelix_authority_coordinator_conductor_process::{
    ConductorProcessError, QualifiedConductorProcessFence, TrustedConductorProcessStore,
};
use mycelix_authority_coordinator_deployment::{
    CoordinatorCodeIdentity, CoordinatorDeploymentError, ObservedCoordinatorDeployment,
    CONDUCTOR_ADMIN_SOURCE_PROFILE,
};
use mycelix_authority_coordinator_native_attestor::{
    observe_local_coordinator_deployment, LocalAdminEndpoint, NativeAttestorError,
};
use mycelix_authority_coordinator_target_cell::{
    QualifiedTargetCellSelection, TargetCellError, TrustedTargetCellStore,
};
use serde::Serialize;
use std::fmt;

pub const PROTOCOL_VERSION: &str =
    "mycelix-authority-coordinator-native-observation-composer-v0.1";
pub const OBSERVATION_IDENTITY_PROFILE: &str =
    "mycelix-authority-coordinator-native-observation-identity-v1-blake3-framed";
pub const COMPOSITION_PROFILE: &str =
    "mycelix-authority-coordinator-native-observation-composition-v1-blake3-framed";

const DOMAIN_OBSERVATION: &[u8] =
    b"mycelix/authority/coordinator-native-observation-identity/v1";
const DOMAIN_COMPOSITION: &[u8] =
    b"mycelix/authority/coordinator-native-observation-composition/v1";

/// Non-deserializable historical evidence that the exact pinned target was live and
/// two exact coordinator deployments were observed while the same pinned Linux
/// listener/process snapshot remained stable.
///
/// There is intentionally no `valid_until_ms` on this type. The retained target and
/// observations each keep their own short evidence horizons for immediate downstream
/// #290/#298 composition; the process fence itself is historical only.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedProcessBoundCoordinatorObservations {
    target: QualifiedTargetCellSelection,
    pre_observation: ObservedCoordinatorDeployment,
    post_observation: ObservedCoordinatorDeployment,
    process_fence: QualifiedConductorProcessFence,
    pre_observation_digest: [u8; 32],
    post_observation_digest: [u8; 32],
    composition_digest: [u8; 32],
    composition_profile: String,
}

impl QualifiedProcessBoundCoordinatorObservations {
    pub fn target(&self) -> &QualifiedTargetCellSelection {
        &self.target
    }

    pub fn pre_observation(&self) -> &ObservedCoordinatorDeployment {
        &self.pre_observation
    }

    pub fn post_observation(&self) -> &ObservedCoordinatorDeployment {
        &self.post_observation
    }

    pub fn process_fence(&self) -> &QualifiedConductorProcessFence {
        &self.process_fence
    }

    pub fn pre_observation_digest(&self) -> [u8; 32] {
        self.pre_observation_digest
    }

    pub fn post_observation_digest(&self) -> [u8; 32] {
        self.post_observation_digest
    }

    pub fn composition_digest(&self) -> [u8; 32] {
        self.composition_digest
    }

    pub fn composition_profile(&self) -> &str {
        &self.composition_profile
    }
}

/// Own one complete target/process/code provenance interval.
///
/// The caller supplies only the two already-provisioned local stores. It supplies no
/// target CellId, Admin endpoint, PID/process snapshot, coordinator observation,
/// observation timestamp or evidence horizon.
pub async fn observe_process_bound_coordinator_state(
    process_store: &TrustedConductorProcessStore,
    target_store: &TrustedTargetCellStore,
) -> Result<QualifiedProcessBoundCoordinatorObservations, NativeObservationComposerError> {
    let guard = process_store
        .begin_fence()
        .map_err(NativeObservationComposerError::ProcessFence)?;
    let endpoint = guard.admin_endpoint();

    // Read only the state-owned immutable target binding identity. The target store
    // has no in-band rebind operation in v0.1; qualification below still proves the
    // exact pinned cell is live now and advances its own trusted clock state.
    let target_state = target_store
        .state_summary()
        .map_err(NativeObservationComposerError::Target)?;
    if target_state.admin_endpoint != endpoint {
        return Err(NativeObservationComposerError::EndpointMismatch);
    }

    let target = target_store
        .qualify_live_target()
        .await
        .map_err(NativeObservationComposerError::Target)?;
    if target.binding_digest() != target_state.binding_digest
        || target.selection().dna_hash_raw_39 != target_state.dna_hash_raw_39
        || target.selection().agent_pub_key_raw_39 != target_state.agent_pub_key_raw_39
    {
        return Err(NativeObservationComposerError::TargetBindingChanged);
    }

    let cell_id = cell_id_from_target(&target)?;
    let local_endpoint = LocalAdminEndpoint::new(endpoint)
        .map_err(NativeObservationComposerError::NativeAttestor)?;

    let pre_observation = observe_local_coordinator_deployment(local_endpoint, cell_id.clone())
        .await
        .map_err(NativeObservationComposerError::NativeAttestor)?;
    let post_observation = observe_local_coordinator_deployment(local_endpoint, cell_id)
        .await
        .map_err(NativeObservationComposerError::NativeAttestor)?;

    // Closing the process fence re-resolves the exact listener/process/executable,
    // persists that historical interval, and only then returns positive process
    // evidence. Nothing below may turn that historical evidence into a future lease.
    let process_fence = guard
        .finish()
        .map_err(NativeObservationComposerError::ProcessFence)?;

    validate_process_bound_timeline(
        &target,
        &pre_observation,
        &post_observation,
        &process_fence,
    )?;

    let pre_observation_digest = observation_digest(&pre_observation)?;
    let post_observation_digest = observation_digest(&post_observation)?;
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_COMPOSITION);
    frame(&mut h, PROTOCOL_VERSION.as_bytes());
    frame(&mut h, COMPOSITION_PROFILE.as_bytes());
    frame(&mut h, &target.qualification_digest());
    frame(&mut h, target.qualification_profile().as_bytes());
    frame(&mut h, &pre_observation_digest);
    frame(&mut h, OBSERVATION_IDENTITY_PROFILE.as_bytes());
    frame(&mut h, &post_observation_digest);
    frame(&mut h, OBSERVATION_IDENTITY_PROFILE.as_bytes());
    frame(&mut h, &process_fence.qualification_digest());
    frame(&mut h, process_fence.qualification_profile().as_bytes());
    frame(&mut h, &process_fence.started_at_ms().to_le_bytes());
    frame(&mut h, &process_fence.ended_at_ms().to_le_bytes());
    let composition_digest = *h.finalize().as_bytes();

    Ok(QualifiedProcessBoundCoordinatorObservations {
        target,
        pre_observation,
        post_observation,
        process_fence,
        pre_observation_digest,
        post_observation_digest,
        composition_digest,
        composition_profile: COMPOSITION_PROFILE.into(),
    })
}

fn cell_id_from_target(
    target: &QualifiedTargetCellSelection,
) -> Result<CellId, NativeObservationComposerError> {
    let selection = target.selection();
    let dna_hash = DnaHash::try_from_raw_39(selection.dna_hash_raw_39.clone())
        .map_err(|error| NativeObservationComposerError::InvalidTargetHash(error.to_string()))?;
    let agent = AgentPubKey::try_from_raw_39(selection.agent_pub_key_raw_39.clone())
        .map_err(|error| NativeObservationComposerError::InvalidTargetHash(error.to_string()))?;
    Ok(CellId::new(dna_hash, agent))
}

fn validate_process_bound_timeline(
    target: &QualifiedTargetCellSelection,
    pre: &ObservedCoordinatorDeployment,
    post: &ObservedCoordinatorDeployment,
    process: &QualifiedConductorProcessFence,
) -> Result<(), NativeObservationComposerError> {
    let selection = target.selection();
    for observed in [pre, post] {
        if observed.dna_hash_raw_39 != selection.dna_hash_raw_39
            || observed.agent_pub_key_raw_39 != selection.agent_pub_key_raw_39
        {
            return Err(NativeObservationComposerError::ObservedCellMismatch);
        }
        if observed.source_profile != CONDUCTOR_ADMIN_SOURCE_PROFILE {
            return Err(NativeObservationComposerError::WrongObservationSourceProfile);
        }
    }

    if target.observed_at_ms() < process.started_at_ms()
        || pre.observed_at_ms < target.observed_at_ms()
        || post.observed_at_ms <= pre.observed_at_ms
        || process.ended_at_ms() < post.observed_at_ms
    {
        return Err(NativeObservationComposerError::InvalidObservationOrdering);
    }

    // All short-lived target/code evidence must still cover the instant at which the
    // historical process fence closes. This makes immediate #290/#298 composition
    // possible without widening any underlying lease.
    let fence_end = process.ended_at_ms();
    if target.valid_until_ms() <= fence_end
        || pre.valid_until_ms <= fence_end
        || post.valid_until_ms <= fence_end
    {
        return Err(NativeObservationComposerError::EvidenceExpiredBeforeFenceClose);
    }

    // #298 requires the post observation to occur inside the pre-composition window.
    // Release currentness may shorten that window further later; this native layer can
    // at least prove the pre observation's own horizon was not already exceeded.
    if post.observed_at_ms > pre.valid_until_ms {
        return Err(NativeObservationComposerError::PostOutsidePreObservationWindow);
    }

    pre.validate(fence_end)
        .map_err(NativeObservationComposerError::DeploymentEvidence)?;
    post.validate(fence_end)
        .map_err(NativeObservationComposerError::DeploymentEvidence)?;
    Ok(())
}

fn observation_digest(
    observed: &ObservedCoordinatorDeployment,
) -> Result<[u8; 32], NativeObservationComposerError> {
    observed
        .validate(observed.observed_at_ms)
        .map_err(NativeObservationComposerError::DeploymentEvidence)?;
    let mut coordinators: Vec<&CoordinatorCodeIdentity> = observed.coordinators.iter().collect();
    coordinators.sort_by(|left, right| left.zome_name.cmp(&right.zome_name));

    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_OBSERVATION);
    frame(&mut h, OBSERVATION_IDENTITY_PROFILE.as_bytes());
    frame(&mut h, observed.protocol_version.as_bytes());
    frame(&mut h, &observed.dna_hash_raw_39);
    frame(&mut h, &observed.agent_pub_key_raw_39);
    frame(&mut h, observed.source_profile.as_bytes());
    frame(&mut h, observed.source_ref.as_bytes());
    frame(&mut h, &observed.observed_at_ms.to_le_bytes());
    frame(&mut h, &observed.valid_until_ms.to_le_bytes());
    for coordinator in coordinators {
        frame(&mut h, coordinator.zome_name.as_bytes());
        frame(&mut h, &coordinator.wasm_hash_raw_39);
    }
    Ok(*h.finalize().as_bytes())
}

fn frame(h: &mut blake3::Hasher, bytes: &[u8]) {
    h.update(&(bytes.len() as u64).to_le_bytes());
    h.update(bytes);
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum NativeObservationComposerError {
    ProcessFence(ConductorProcessError),
    Target(TargetCellError),
    NativeAttestor(NativeAttestorError),
    DeploymentEvidence(CoordinatorDeploymentError),
    InvalidTargetHash(String),
    EndpointMismatch,
    TargetBindingChanged,
    ObservedCellMismatch,
    WrongObservationSourceProfile,
    InvalidObservationOrdering,
    EvidenceExpiredBeforeFenceClose,
    PostOutsidePreObservationWindow,
}

impl fmt::Display for NativeObservationComposerError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::ProcessFence(error) => write!(f, "conductor process fence denied: {error}"),
            Self::Target(error) => write!(f, "target-cell qualification denied: {error}"),
            Self::NativeAttestor(error) => write!(f, "native coordinator observation denied: {error}"),
            Self::DeploymentEvidence(error) => write!(f, "coordinator observation evidence denied: {error}"),
            Self::InvalidTargetHash(error) => write!(f, "pinned target CellId bytes are invalid: {error}"),
            Self::EndpointMismatch => write!(f, "target and process stores pin different Admin endpoints"),
            Self::TargetBindingChanged => write!(f, "target binding changed across state summary and live qualification"),
            Self::ObservedCellMismatch => write!(f, "native coordinator observation does not name the exact qualified target CellId"),
            Self::WrongObservationSourceProfile => write!(f, "native coordinator observation has the wrong source profile"),
            Self::InvalidObservationOrdering => write!(f, "target/pre/post observations are not causally ordered inside the process fence"),
            Self::EvidenceExpiredBeforeFenceClose => write!(f, "target/code evidence expired before the process fence closed"),
            Self::PostOutsidePreObservationWindow => write!(f, "post coordinator observation occurred outside the pre-observation reuse window"),
        }
    }
}

impl std::error::Error for NativeObservationComposerError {}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_authority_coordinator_deployment::{
        PROTOCOL_VERSION as DEPLOYMENT_PROTOCOL_VERSION, MAX_OBSERVATION_REUSE_MS,
    };

    fn h(byte: u8) -> Vec<u8> {
        vec![byte; 39]
    }

    fn c(name: &str, byte: u8) -> CoordinatorCodeIdentity {
        CoordinatorCodeIdentity {
            zome_name: name.into(),
            wasm_hash_raw_39: h(byte),
        }
    }

    fn observed(at: u64, source: &str) -> ObservedCoordinatorDeployment {
        ObservedCoordinatorDeployment {
            protocol_version: DEPLOYMENT_PROTOCOL_VERSION.into(),
            dna_hash_raw_39: h(9),
            agent_pub_key_raw_39: h(8),
            coordinators: vec![c("authority_current_freshness_verifier", 1)],
            source_profile: CONDUCTOR_ADMIN_SOURCE_PROFILE.into(),
            source_ref: source.into(),
            observed_at_ms: at,
            valid_until_ms: at + MAX_OBSERVATION_REUSE_MS,
        }
    }

    #[test]
    fn observation_identity_commits_code_source_and_time() {
        let a = observed(100, "admin:a");
        let mut b = a.clone();
        assert_eq!(observation_digest(&a).unwrap(), observation_digest(&b).unwrap());
        b.coordinators[0].wasm_hash_raw_39 = h(2);
        assert_ne!(observation_digest(&a).unwrap(), observation_digest(&b).unwrap());
        b = a.clone();
        b.source_ref = "admin:b".into();
        assert_ne!(observation_digest(&a).unwrap(), observation_digest(&b).unwrap());
        b = a.clone();
        b.observed_at_ms += 1;
        b.valid_until_ms += 1;
        assert_ne!(observation_digest(&a).unwrap(), observation_digest(&b).unwrap());
    }

    #[test]
    fn post_must_be_strictly_later_and_inside_pre_window() {
        let pre = observed(100, "admin:pre");
        let same_time = observed(100, "admin:post");
        assert!(same_time.observed_at_ms <= pre.observed_at_ms);

        let too_late = observed(pre.valid_until_ms + 1, "admin:post");
        assert!(too_late.observed_at_ms > pre.valid_until_ms);
    }

    #[test]
    fn positive_type_exposes_no_future_process_lease_field() {
        let source = include_str!("lib.rs");
        let positive = source
            .split("pub struct QualifiedProcessBoundCoordinatorObservations {")
            .nth(1)
            .unwrap()
            .split("\n}")
            .next()
            .unwrap();
        assert!(!positive.contains("valid_until_ms"));
    }
}
