// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pure exact-match contract for Holochain coordinator deployment code.
//!
//! This crate validates deployment evidence shape and exact expected-vs-observed
//! code equality for one exact Holochain cell. It does not establish where an
//! observed snapshot came from. A live authority/effect boundary must obtain
//! observations from an independently trusted native conductor/AdminWebsocket
//! adapter; caller/zome self-report is insufficient.

use serde::{Deserialize, Serialize};
use std::fmt;

pub const PROTOCOL_VERSION: &str = "mycelix-authority-coordinator-deployment-v0.1";
pub const REQUIREMENT_PROFILE: &str =
    "mycelix-authority-coordinator-requirement-v1-blake3-framed";
pub const MATCH_PROFILE: &str =
    "mycelix-authority-coordinator-deployment-match-v1-blake3-framed";
pub const CONDUCTOR_ADMIN_SOURCE_PROFILE: &str =
    "holochain-admin-get-dna-definition-cell-coordinator-wasm-v1";
/// Short local cache/reuse ceiling for one conductor observation. This is not a
/// natural code-expiry theorem: coordinator code can still be updated during the
/// window, so effect admission must re-observe and handle update races explicitly.
pub const MAX_OBSERVATION_REUSE_MS: u64 = 5_000;

const DOMAIN_REQUIREMENT: &[u8] = b"mycelix/authority/coordinator-requirement/v1";
const DOMAIN_MATCH: &[u8] = b"mycelix/authority/coordinator-deployment-match/v1";
const HOLO_HASH_RAW_LEN: usize = 39;
const MAX_ZOME_NAME_BYTES: usize = 128;
const MAX_SOURCE_REF_BYTES: usize = 2048;
const MAX_COORDINATORS: usize = 128;

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CoordinatorCodeIdentity {
    pub zome_name: String,
    /// Exact raw 39-byte Holochain `WasmHash` representation supplied by the
    /// native conductor-side adapter. No alternate textual hash identity is used.
    pub wasm_hash_raw_39: Vec<u8>,
}

impl CoordinatorCodeIdentity {
    fn validate(&self) -> Result<(), CoordinatorDeploymentError> {
        validate_name(&self.zome_name)?;
        validate_holo_hash(&self.wasm_hash_raw_39, "coordinator wasm hash")?;
        Ok(())
    }
}

/// Expected deployment for one exact target `CellId` plus its complete approved
/// coordinator set.
///
/// Production composition must source the target cell identity independently from
/// the authenticated release policy that supplies the approved coordinator set.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RequiredCoordinatorDeployment {
    pub protocol_version: String,
    pub dna_hash_raw_39: Vec<u8>,
    pub agent_pub_key_raw_39: Vec<u8>,
    pub coordinators: Vec<CoordinatorCodeIdentity>,
}

impl RequiredCoordinatorDeployment {
    pub fn validate(&self) -> Result<(), CoordinatorDeploymentError> {
        require_protocol(&self.protocol_version)?;
        validate_holo_hash(&self.dna_hash_raw_39, "DNA hash")?;
        validate_holo_hash(&self.agent_pub_key_raw_39, "cell agent public key")?;
        validate_set(&self.coordinators)?;
        Ok(())
    }

    pub fn requirement_digest(&self) -> Result<[u8; 32], CoordinatorDeploymentError> {
        self.validate()?;
        let canonical = canonical_coordinators(&self.coordinators);
        let mut hasher = blake3::Hasher::new();
        hasher.update(DOMAIN_REQUIREMENT);
        frame(&mut hasher, PROTOCOL_VERSION.as_bytes());
        frame(&mut hasher, REQUIREMENT_PROFILE.as_bytes());
        frame(&mut hasher, &self.dna_hash_raw_39);
        frame(&mut hasher, &self.agent_pub_key_raw_39);
        for coordinator in canonical {
            frame(&mut hasher, coordinator.zome_name.as_bytes());
            frame(&mut hasher, &coordinator.wasm_hash_raw_39);
        }
        Ok(*hasher.finalize().as_bytes())
    }
}

/// Deserializable observation from a native conductor-side adapter for one exact
/// Holochain `CellId` (`DnaHash` + agent public key).
///
/// Validation proves evidence shape only. It does NOT prove the bytes came from
/// the Holochain admin API; live consumers must establish that provenance through
/// their direct native adapter boundary.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ObservedCoordinatorDeployment {
    pub protocol_version: String,
    pub dna_hash_raw_39: Vec<u8>,
    pub agent_pub_key_raw_39: Vec<u8>,
    pub coordinators: Vec<CoordinatorCodeIdentity>,
    pub source_profile: String,
    pub source_ref: String,
    pub observed_at_ms: u64,
    pub valid_until_ms: u64,
}

impl ObservedCoordinatorDeployment {
    pub fn validate(&self, now_ms: u64) -> Result<(), CoordinatorDeploymentError> {
        require_protocol(&self.protocol_version)?;
        validate_holo_hash(&self.dna_hash_raw_39, "DNA hash")?;
        validate_holo_hash(&self.agent_pub_key_raw_39, "cell agent public key")?;
        validate_set(&self.coordinators)?;
        if self.source_profile != CONDUCTOR_ADMIN_SOURCE_PROFILE {
            return Err(CoordinatorDeploymentError::WrongSourceProfile);
        }
        validate_ref(&self.source_ref)?;
        if now_ms == 0
            || self.observed_at_ms == 0
            || self.observed_at_ms > now_ms
            || self.valid_until_ms <= now_ms
            || self.valid_until_ms <= self.observed_at_ms
        {
            return Err(CoordinatorDeploymentError::InvalidObservationWindow);
        }
        let width = self
            .valid_until_ms
            .checked_sub(self.observed_at_ms)
            .ok_or(CoordinatorDeploymentError::InvalidObservationWindow)?;
        if width > MAX_OBSERVATION_REUSE_MS {
            return Err(CoordinatorDeploymentError::ObservationLeaseTooWide);
        }
        Ok(())
    }
}

/// Non-deserializable exact-set match over one exact target cell and one live
/// conductor observation. This still does not prove observation or release-policy
/// provenance.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct MatchedCoordinatorDeployment {
    requirement_digest: [u8; 32],
    requirement_profile: String,
    dna_hash_raw_39: Vec<u8>,
    agent_pub_key_raw_39: Vec<u8>,
    source_profile: String,
    source_ref: String,
    observed_at_ms: u64,
    valid_until_ms: u64,
    matched_coordinator_count: u32,
    match_digest: [u8; 32],
    match_profile: String,
}

impl MatchedCoordinatorDeployment {
    pub fn requirement_digest(&self) -> [u8; 32] {
        self.requirement_digest
    }

    pub fn requirement_profile(&self) -> &str {
        &self.requirement_profile
    }

    pub fn dna_hash_raw_39(&self) -> &[u8] {
        &self.dna_hash_raw_39
    }

    pub fn agent_pub_key_raw_39(&self) -> &[u8] {
        &self.agent_pub_key_raw_39
    }

    pub fn source_profile(&self) -> &str {
        &self.source_profile
    }

    pub fn source_ref(&self) -> &str {
        &self.source_ref
    }

    pub fn observed_at_ms(&self) -> u64 {
        self.observed_at_ms
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    pub fn matched_coordinator_count(&self) -> u32 {
        self.matched_coordinator_count
    }

    pub fn match_digest(&self) -> [u8; 32] {
        self.match_digest
    }

    pub fn match_profile(&self) -> &str {
        &self.match_profile
    }
}

/// Require the complete installed coordinator set for the exact target `CellId`
/// to equal the independently approved set by exact zome name and exact `WasmHash`.
///
/// An extra installed coordinator is executable code and therefore denies just as
/// a missing or substituted coordinator does. This contract intentionally does not
/// implement a "required subset" mode for live authority/effect admission.
pub fn match_required_coordinator_deployment(
    required: &RequiredCoordinatorDeployment,
    observed: &ObservedCoordinatorDeployment,
    now_ms: u64,
) -> Result<MatchedCoordinatorDeployment, CoordinatorDeploymentError> {
    required.validate()?;
    observed.validate(now_ms)?;
    if required.dna_hash_raw_39 != observed.dna_hash_raw_39 {
        return Err(CoordinatorDeploymentError::DnaMismatch);
    }
    if required.agent_pub_key_raw_39 != observed.agent_pub_key_raw_39 {
        return Err(CoordinatorDeploymentError::CellAgentMismatch);
    }

    let required_set = canonical_coordinators(&required.coordinators);
    let observed_set = canonical_coordinators(&observed.coordinators);

    for expected in &required_set {
        let actual = observed_set
            .iter()
            .find(|candidate| candidate.zome_name == expected.zome_name)
            .ok_or_else(|| {
                CoordinatorDeploymentError::MissingCoordinator(expected.zome_name.clone())
            })?;
        if actual.wasm_hash_raw_39 != expected.wasm_hash_raw_39 {
            return Err(CoordinatorDeploymentError::WasmHashMismatch(
                expected.zome_name.clone(),
            ));
        }
    }

    for actual in &observed_set {
        if !required_set
            .iter()
            .any(|expected| expected.zome_name == actual.zome_name)
        {
            return Err(CoordinatorDeploymentError::UnexpectedCoordinator(
                actual.zome_name.clone(),
            ));
        }
    }

    let requirement_digest = required.requirement_digest()?;
    let count = u32::try_from(required_set.len())
        .map_err(|_| CoordinatorDeploymentError::TooManyCoordinators)?;
    let mut hasher = blake3::Hasher::new();
    hasher.update(DOMAIN_MATCH);
    frame(&mut hasher, PROTOCOL_VERSION.as_bytes());
    frame(&mut hasher, MATCH_PROFILE.as_bytes());
    frame(&mut hasher, REQUIREMENT_PROFILE.as_bytes());
    frame(&mut hasher, &requirement_digest);
    frame(&mut hasher, &observed.dna_hash_raw_39);
    frame(&mut hasher, &observed.agent_pub_key_raw_39);
    frame(&mut hasher, observed.source_profile.as_bytes());
    frame(&mut hasher, observed.source_ref.as_bytes());
    frame(&mut hasher, &observed.observed_at_ms.to_le_bytes());
    frame(&mut hasher, &observed.valid_until_ms.to_le_bytes());
    frame(&mut hasher, &count.to_le_bytes());
    for coordinator in required_set {
        frame(&mut hasher, coordinator.zome_name.as_bytes());
        frame(&mut hasher, &coordinator.wasm_hash_raw_39);
    }
    let match_digest = *hasher.finalize().as_bytes();

    Ok(MatchedCoordinatorDeployment {
        requirement_digest,
        requirement_profile: REQUIREMENT_PROFILE.into(),
        dna_hash_raw_39: observed.dna_hash_raw_39.clone(),
        agent_pub_key_raw_39: observed.agent_pub_key_raw_39.clone(),
        source_profile: observed.source_profile.clone(),
        source_ref: observed.source_ref.clone(),
        observed_at_ms: observed.observed_at_ms,
        valid_until_ms: observed.valid_until_ms,
        matched_coordinator_count: count,
        match_digest,
        match_profile: MATCH_PROFILE.into(),
    })
}

fn require_protocol(value: &str) -> Result<(), CoordinatorDeploymentError> {
    if value == PROTOCOL_VERSION {
        Ok(())
    } else {
        Err(CoordinatorDeploymentError::WrongProtocol)
    }
}

fn validate_holo_hash(value: &[u8], field: &'static str) -> Result<(), CoordinatorDeploymentError> {
    if value.len() != HOLO_HASH_RAW_LEN || value.iter().all(|byte| *byte == 0) {
        Err(CoordinatorDeploymentError::InvalidHash(field))
    } else {
        Ok(())
    }
}

fn validate_name(value: &str) -> Result<(), CoordinatorDeploymentError> {
    let bytes = value.as_bytes();
    if bytes.is_empty()
        || bytes.len() > MAX_ZOME_NAME_BYTES
        || bytes.iter().any(|byte| byte.is_ascii_whitespace())
    {
        Err(CoordinatorDeploymentError::InvalidZomeName)
    } else {
        Ok(())
    }
}

fn validate_ref(value: &str) -> Result<(), CoordinatorDeploymentError> {
    if value.trim().is_empty() || value.len() > MAX_SOURCE_REF_BYTES {
        Err(CoordinatorDeploymentError::InvalidSourceRef)
    } else {
        Ok(())
    }
}

fn validate_set(values: &[CoordinatorCodeIdentity]) -> Result<(), CoordinatorDeploymentError> {
    if values.is_empty() || values.len() > MAX_COORDINATORS {
        return Err(CoordinatorDeploymentError::InvalidCoordinatorSet);
    }
    for value in values {
        value.validate()?;
    }
    let canonical = canonical_coordinators(values);
    for pair in canonical.windows(2) {
        if pair[0].zome_name == pair[1].zome_name {
            return Err(CoordinatorDeploymentError::DuplicateCoordinator(
                pair[0].zome_name.clone(),
            ));
        }
    }
    Ok(())
}

fn canonical_coordinators(values: &[CoordinatorCodeIdentity]) -> Vec<CoordinatorCodeIdentity> {
    let mut canonical = values.to_vec();
    canonical.sort_by(|left, right| left.zome_name.cmp(&right.zome_name));
    canonical
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum CoordinatorDeploymentError {
    WrongProtocol,
    InvalidHash(&'static str),
    InvalidZomeName,
    InvalidSourceRef,
    InvalidCoordinatorSet,
    TooManyCoordinators,
    DuplicateCoordinator(String),
    WrongSourceProfile,
    InvalidObservationWindow,
    ObservationLeaseTooWide,
    DnaMismatch,
    CellAgentMismatch,
    MissingCoordinator(String),
    UnexpectedCoordinator(String),
    WasmHashMismatch(String),
}

impl fmt::Display for CoordinatorDeploymentError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocol => write!(f, "wrong coordinator-deployment protocol"),
            Self::InvalidHash(field) => write!(f, "invalid {field}"),
            Self::InvalidZomeName => write!(f, "invalid coordinator zome name"),
            Self::InvalidSourceRef => write!(f, "invalid conductor observation source reference"),
            Self::InvalidCoordinatorSet => {
                write!(f, "invalid coordinator requirement/observation set")
            }
            Self::TooManyCoordinators => write!(f, "too many coordinators"),
            Self::DuplicateCoordinator(name) => write!(f, "duplicate coordinator {name}"),
            Self::WrongSourceProfile => write!(f, "wrong coordinator observation source profile"),
            Self::InvalidObservationWindow => {
                write!(f, "coordinator observation is stale, future-dated or inverted")
            }
            Self::ObservationLeaseTooWide => write!(
                f,
                "coordinator observation exceeds the bounded local reuse window"
            ),
            Self::DnaMismatch => write!(f, "coordinator observation belongs to another DNA"),
            Self::CellAgentMismatch => {
                write!(f, "coordinator observation belongs to another cell agent")
            }
            Self::MissingCoordinator(name) => {
                write!(f, "approved coordinator {name} is not installed")
            }
            Self::UnexpectedCoordinator(name) => {
                write!(f, "unapproved coordinator {name} is installed")
            }
            Self::WasmHashMismatch(name) => {
                write!(f, "installed coordinator {name} has the wrong WASM hash")
            }
        }
    }
}

impl std::error::Error for CoordinatorDeploymentError {}

#[cfg(test)]
mod tests {
    use super::*;

    fn h(byte: u8) -> Vec<u8> {
        vec![byte; HOLO_HASH_RAW_LEN]
    }

    fn c(name: &str, byte: u8) -> CoordinatorCodeIdentity {
        CoordinatorCodeIdentity {
            zome_name: name.into(),
            wasm_hash_raw_39: h(byte),
        }
    }

    fn required() -> RequiredCoordinatorDeployment {
        RequiredCoordinatorDeployment {
            protocol_version: PROTOCOL_VERSION.into(),
            dna_hash_raw_39: h(9),
            agent_pub_key_raw_39: h(8),
            coordinators: vec![
                c("authority_current_freshness_verifier", 1),
                c("constitution_currentness_verifier", 2),
            ],
        }
    }

    fn observed() -> ObservedCoordinatorDeployment {
        ObservedCoordinatorDeployment {
            protocol_version: PROTOCOL_VERSION.into(),
            dna_hash_raw_39: h(9),
            agent_pub_key_raw_39: h(8),
            coordinators: vec![
                c("constitution_currentness_verifier", 2),
                c("authority_current_freshness_verifier", 1),
            ],
            source_profile: CONDUCTOR_ADMIN_SOURCE_PROFILE.into(),
            source_ref: "admin-websocket:get-dna-definition:cell:test".into(),
            observed_at_ms: 100,
            valid_until_ms: 5_100,
        }
    }

    #[test]
    fn exact_cell_and_approved_set_match_order_independently() {
        let a = match_required_coordinator_deployment(&required(), &observed(), 150).unwrap();
        let mut reversed = required();
        reversed.coordinators.reverse();
        let b = match_required_coordinator_deployment(&reversed, &observed(), 150).unwrap();
        assert_eq!(a.requirement_digest(), b.requirement_digest());
        assert_eq!(a.match_digest(), b.match_digest());
        assert_eq!(a.matched_coordinator_count(), 2);
        assert_eq!(a.agent_pub_key_raw_39(), h(8));
    }

    #[test]
    fn substituted_wasm_hash_denies() {
        let mut value = observed();
        value.coordinators[1].wasm_hash_raw_39 = h(99);
        assert!(matches!(
            match_required_coordinator_deployment(&required(), &value, 150),
            Err(CoordinatorDeploymentError::WasmHashMismatch(_))
        ));
    }

    #[test]
    fn missing_approved_coordinator_denies() {
        let mut value = observed();
        value
            .coordinators
            .retain(|coordinator| coordinator.zome_name != "constitution_currentness_verifier");
        assert!(matches!(
            match_required_coordinator_deployment(&required(), &value, 150),
            Err(CoordinatorDeploymentError::MissingCoordinator(_))
        ));
    }

    #[test]
    fn unexpected_extra_coordinator_denies() {
        let mut value = observed();
        value.coordinators.push(c("unapproved_helper", 7));
        assert!(matches!(
            match_required_coordinator_deployment(&required(), &value, 150),
            Err(CoordinatorDeploymentError::UnexpectedCoordinator(_))
        ));
    }

    #[test]
    fn duplicate_name_denies_even_if_hashes_agree() {
        let mut value = observed();
        value
            .coordinators
            .push(c("constitution_currentness_verifier", 2));
        assert!(matches!(
            match_required_coordinator_deployment(&required(), &value, 150),
            Err(CoordinatorDeploymentError::DuplicateCoordinator(_))
        ));
    }

    #[test]
    fn wrong_dna_or_cell_agent_denies() {
        let mut wrong_dna = observed();
        wrong_dna.dna_hash_raw_39 = h(7);
        assert_eq!(
            match_required_coordinator_deployment(&required(), &wrong_dna, 150).unwrap_err(),
            CoordinatorDeploymentError::DnaMismatch
        );

        let mut wrong_agent = observed();
        wrong_agent.agent_pub_key_raw_39 = h(6);
        assert_eq!(
            match_required_coordinator_deployment(&required(), &wrong_agent, 150).unwrap_err(),
            CoordinatorDeploymentError::CellAgentMismatch
        );
    }

    #[test]
    fn stale_or_over_wide_observation_denies() {
        let mut stale = observed();
        stale.valid_until_ms = 150;
        assert_eq!(
            match_required_coordinator_deployment(&required(), &stale, 150).unwrap_err(),
            CoordinatorDeploymentError::InvalidObservationWindow
        );

        let mut wide = observed();
        wide.valid_until_ms = wide.observed_at_ms + MAX_OBSERVATION_REUSE_MS + 1;
        assert_eq!(
            match_required_coordinator_deployment(&required(), &wide, 150).unwrap_err(),
            CoordinatorDeploymentError::ObservationLeaseTooWide
        );
    }

    #[test]
    fn caller_controlled_source_profile_denies() {
        let mut value = observed();
        value.source_profile = "zome-self-report".into();
        assert_eq!(
            match_required_coordinator_deployment(&required(), &value, 150).unwrap_err(),
            CoordinatorDeploymentError::WrongSourceProfile
        );
    }

    #[test]
    fn serde_round_trip_still_requires_direct_provenance_outside_matcher() {
        let value = observed();
        let json = serde_json::to_string(&value).unwrap();
        let decoded: ObservedCoordinatorDeployment = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded, value);
        match_required_coordinator_deployment(&required(), &decoded, 150).unwrap();
    }
}
