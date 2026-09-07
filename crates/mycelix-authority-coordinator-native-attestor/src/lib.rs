// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Native observation boundary for exact Holochain coordinator deployment state.
//!
//! This crate deliberately lives outside zome/WASM authority. Its live API creates
//! its own loopback-only Holochain admin connection, queries the exact target
//! `CellId`, extracts the complete installed coordinator set and exact raw WasmHash
//! identities, then returns the deserializable observation shape defined by
//! `mycelix-authority-coordinator-deployment`.
//!
//! It does NOT authenticate an approved release requirement, run the pure matcher,
//! prove conductor-process integrity or make coordinator updates atomic with a
//! later effect. Those remain separate trust boundaries.

use holochain_client::{AdminWebsocket, CellId};
use mycelix_authority_coordinator_deployment::{
    CoordinatorCodeIdentity, CoordinatorDeploymentError, ObservedCoordinatorDeployment,
    CONDUCTOR_ADMIN_SOURCE_PROFILE, MAX_OBSERVATION_REUSE_MS, PROTOCOL_VERSION,
};
use std::fmt;
use std::net::SocketAddr;
use std::time::{SystemTime, UNIX_EPOCH};

const ATTESTOR_ORIGIN: &str = "mycelix-authority-coordinator-native-attestor-v0.1";

/// Loopback-only native Holochain admin endpoint.
///
/// This prevents the attestor from silently widening its trust boundary to a
/// network-reachable admin interface. It is not proof that the local process on
/// this socket is an uncompromised Holochain conductor.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LocalAdminEndpoint(SocketAddr);

impl LocalAdminEndpoint {
    pub fn new(socket_addr: SocketAddr) -> Result<Self, NativeAttestorError> {
        if socket_addr.port() == 0 || !socket_addr.ip().is_loopback() {
            return Err(NativeAttestorError::InvalidAdminEndpoint);
        }
        Ok(Self(socket_addr))
    }

    pub fn socket_addr(self) -> SocketAddr {
        self.0
    }
}

/// Query the exact target cell through a loopback Holochain Admin websocket and
/// return a complete, bounded coordinator-deployment observation.
///
/// The caller does not provide an observation timestamp or validity horizon. The
/// adapter samples host time only after the conductor response and coordinator
/// hash extraction complete, then applies the pure contract's fixed reuse cap.
pub async fn observe_local_coordinator_deployment(
    endpoint: LocalAdminEndpoint,
    cell_id: CellId,
) -> Result<ObservedCoordinatorDeployment, NativeAttestorError> {
    let admin = AdminWebsocket::connect(
        endpoint.socket_addr(),
        Some(ATTESTOR_ORIGIN.to_string()),
    )
    .await
    .map_err(|error| NativeAttestorError::Conductor(error.to_string()))?;

    let dna_def = admin
        .get_dna_definition(cell_id.clone())
        .await
        .map_err(|error| NativeAttestorError::Conductor(error.to_string()))?;

    let mut coordinators = Vec::with_capacity(dna_def.coordinator_zomes.len());
    for (zome_name, _) in &dna_def.coordinator_zomes {
        let wasm_hash = dna_def
            .get_wasm_zome_hash(zome_name)
            .map_err(|error| NativeAttestorError::CoordinatorDefinition(error.to_string()))?;
        coordinators.push(CoordinatorCodeIdentity {
            zome_name: zome_name.to_string(),
            wasm_hash_raw_39: wasm_hash.get_raw_39().to_vec(),
        });
    }
    coordinators.sort_by(|left, right| left.zome_name.cmp(&right.zome_name));

    // Observation time is deliberately sampled after the external response and
    // local extraction complete, so the resulting lease never predates evidence
    // production.
    let observed_at_ms = system_time_ms()?;
    build_observation(
        cell_id.dna_hash().get_raw_39().to_vec(),
        cell_id.agent_pubkey().get_raw_39().to_vec(),
        coordinators,
        source_ref(endpoint, &cell_id),
        observed_at_ms,
    )
}

fn build_observation(
    dna_hash_raw_39: Vec<u8>,
    agent_pub_key_raw_39: Vec<u8>,
    mut coordinators: Vec<CoordinatorCodeIdentity>,
    source_ref: String,
    observed_at_ms: u64,
) -> Result<ObservedCoordinatorDeployment, NativeAttestorError> {
    coordinators.sort_by(|left, right| left.zome_name.cmp(&right.zome_name));
    let valid_until_ms = observed_at_ms
        .checked_add(MAX_OBSERVATION_REUSE_MS)
        .ok_or(NativeAttestorError::ClockOverflow)?;
    let observation = ObservedCoordinatorDeployment {
        protocol_version: PROTOCOL_VERSION.into(),
        dna_hash_raw_39,
        agent_pub_key_raw_39,
        coordinators,
        source_profile: CONDUCTOR_ADMIN_SOURCE_PROFILE.into(),
        source_ref,
        observed_at_ms,
        valid_until_ms,
    };
    observation
        .validate(observed_at_ms)
        .map_err(NativeAttestorError::ObservationDenied)?;
    Ok(observation)
}

fn source_ref(endpoint: LocalAdminEndpoint, cell_id: &CellId) -> String {
    format!(
        "admin-websocket:loopback:get-dna-definition:{}:{}",
        endpoint.socket_addr(),
        cell_id
    )
}

fn system_time_ms() -> Result<u64, NativeAttestorError> {
    let duration = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_err(|_| NativeAttestorError::ClockBeforeUnixEpoch)?;
    u64::try_from(duration.as_millis()).map_err(|_| NativeAttestorError::ClockOverflow)
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum NativeAttestorError {
    InvalidAdminEndpoint,
    Conductor(String),
    CoordinatorDefinition(String),
    ClockBeforeUnixEpoch,
    ClockOverflow,
    ObservationDenied(CoordinatorDeploymentError),
}

impl fmt::Display for NativeAttestorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidAdminEndpoint => {
                write!(f, "admin attestation endpoint must be loopback with a non-zero port")
            }
            Self::Conductor(error) => write!(f, "Holochain admin request failed: {error}"),
            Self::CoordinatorDefinition(error) => {
                write!(f, "installed coordinator definition is not a WASM zome: {error}")
            }
            Self::ClockBeforeUnixEpoch => write!(f, "system clock is before the Unix epoch"),
            Self::ClockOverflow => write!(f, "native attestation clock overflow"),
            Self::ObservationDenied(error) => {
                write!(f, "coordinator deployment observation denied: {error}")
            }
        }
    }
}

impl std::error::Error for NativeAttestorError {}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::{Ipv4Addr, Ipv6Addr};

    fn h(byte: u8) -> Vec<u8> {
        vec![byte; 39]
    }

    fn c(name: &str, byte: u8) -> CoordinatorCodeIdentity {
        CoordinatorCodeIdentity {
            zome_name: name.into(),
            wasm_hash_raw_39: h(byte),
        }
    }

    #[test]
    fn admin_endpoint_is_loopback_only() {
        assert!(LocalAdminEndpoint::new(SocketAddr::from((Ipv4Addr::LOCALHOST, 30_000))).is_ok());
        assert!(LocalAdminEndpoint::new(SocketAddr::from((Ipv6Addr::LOCALHOST, 30_000))).is_ok());
        assert_eq!(
            LocalAdminEndpoint::new(SocketAddr::from(([10, 0, 0, 1], 30_000))).unwrap_err(),
            NativeAttestorError::InvalidAdminEndpoint
        );
        assert_eq!(
            LocalAdminEndpoint::new(SocketAddr::from((Ipv4Addr::LOCALHOST, 0))).unwrap_err(),
            NativeAttestorError::InvalidAdminEndpoint
        );
    }

    #[test]
    fn observation_builder_owns_exact_reuse_horizon_and_sorting() {
        let observation = build_observation(
            h(9),
            h(8),
            vec![c("zeta", 2), c("alpha", 1)],
            "admin-websocket:loopback:test".into(),
            100,
        )
        .unwrap();
        assert_eq!(observation.observed_at_ms, 100);
        assert_eq!(observation.valid_until_ms, 100 + MAX_OBSERVATION_REUSE_MS);
        assert_eq!(observation.source_profile, CONDUCTOR_ADMIN_SOURCE_PROFILE);
        assert_eq!(observation.coordinators[0].zome_name, "alpha");
        assert_eq!(observation.coordinators[1].zome_name, "zeta");
    }

    #[test]
    fn malformed_complete_set_is_not_sanitized() {
        let error = build_observation(
            h(9),
            h(8),
            vec![c("same", 1), c("same", 1)],
            "admin-websocket:loopback:test".into(),
            100,
        )
        .unwrap_err();
        assert!(matches!(
            error,
            NativeAttestorError::ObservationDenied(
                CoordinatorDeploymentError::DuplicateCoordinator(_)
            )
        ));
    }

    #[test]
    fn observation_clock_overflow_denies() {
        assert_eq!(
            build_observation(
                h(9),
                h(8),
                vec![c("one", 1)],
                "admin-websocket:loopback:test".into(),
                u64::MAX,
            )
            .unwrap_err(),
            NativeAttestorError::ClockOverflow
        );
    }
}
