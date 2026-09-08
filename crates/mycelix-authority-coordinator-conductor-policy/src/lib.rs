// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Pure independently pinned deployment policy for the Holochain conductor process
//! that may participate in coordinator admission.
//!
//! This crate answers only: "what exact conductor deployment is allowed?"
//! It does not inspect `/proc`, open sockets, query Holochain, verify coordinator
//! code, select a release, or authorize an effect.

use serde::{Deserialize, Serialize};
use std::fmt;
use std::net::SocketAddr;
use std::path::Path;

pub const PROTOCOL_VERSION: &str = "mycelix-authority-coordinator-conductor-policy-v0.1";
pub const POLICY_PROFILE: &str =
    "mycelix-authority-coordinator-conductor-policy-linux-systemd-nix-v1-blake3-framed";
pub const PIN_PROTOCOL_VERSION: &str =
    "mycelix-authority-coordinator-conductor-policy-pin-v0.1";
pub const PIN_PROFILE: &str =
    "mycelix-authority-coordinator-conductor-policy-pin-v1-blake3-framed";
pub const QUALIFICATION_PROFILE: &str =
    "mycelix-authority-coordinator-conductor-policy-qualification-v1-blake3-framed";
pub const TARGET_BINDING_PROFILE: &str =
    "mycelix-authority-coordinator-target-cell-binding-v1-blake3-framed";
pub const EXECUTABLE_DIGEST_PROFILE: &str = "blake3-file-v1";
pub const CONFIG_DIGEST_PROFILE: &str = "blake3-file-v1";
pub const LAUNCH_ARGV_DIGEST_PROFILE: &str = "blake3-nul-separated-argv-v1";

const DOMAIN_POLICY: &[u8] = b"mycelix/authority/coordinator-conductor-policy/v1";
const DOMAIN_QUALIFICATION: &[u8] =
    b"mycelix/authority/coordinator-conductor-policy-qualification/v1";
const MAX_TEXT_BYTES: usize = 4096;
const NIX_STORE_PREFIX: &str = "/nix/store/";

/// Candidate Linux/systemd/Nix deployment identity.
///
/// This is intentionally deserializable policy data. Positive authority arises only
/// after exact equality with an independently delivered pin.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ConductorDeploymentPolicy {
    pub protocol_version: String,
    pub policy_id: String,
    pub policy_version: u64,
    pub target_binding_digest: [u8; 32],
    pub target_binding_profile: String,
    pub admin_endpoint: SocketAddr,
    pub executable_path: String,
    pub executable_digest: [u8; 32],
    pub executable_digest_profile: String,
    pub config_path: String,
    pub config_digest: [u8; 32],
    pub config_digest_profile: String,
    pub launch_argv_digest: [u8; 32],
    pub launch_argv_digest_profile: String,
    pub service_uid: u32,
    pub service_gid: u32,
    pub systemd_unit: String,
    pub provisioning_ref: String,
    pub valid_from_ms: u64,
    pub valid_until_ms: u64,
}

impl ConductorDeploymentPolicy {
    pub fn validate(&self) -> Result<(), ConductorPolicyError> {
        if self.protocol_version != PROTOCOL_VERSION {
            return Err(ConductorPolicyError::WrongProtocol);
        }
        validate_text(&self.policy_id, "policy id")?;
        if self.policy_version == 0 {
            return Err(ConductorPolicyError::InvalidPolicyVersion);
        }
        validate_digest(&self.target_binding_digest, "target binding digest")?;
        if self.target_binding_profile != TARGET_BINDING_PROFILE {
            return Err(ConductorPolicyError::WrongTargetBindingProfile);
        }
        if self.admin_endpoint.port() == 0 || !self.admin_endpoint.ip().is_loopback() {
            return Err(ConductorPolicyError::InvalidAdminEndpoint);
        }

        validate_nix_store_file_path(&self.executable_path, "executable path")?;
        validate_digest(&self.executable_digest, "executable digest")?;
        if self.executable_digest_profile != EXECUTABLE_DIGEST_PROFILE {
            return Err(ConductorPolicyError::WrongExecutableDigestProfile);
        }

        validate_nix_store_file_path(&self.config_path, "config path")?;
        validate_digest(&self.config_digest, "config digest")?;
        if self.config_digest_profile != CONFIG_DIGEST_PROFILE {
            return Err(ConductorPolicyError::WrongConfigDigestProfile);
        }

        validate_digest(&self.launch_argv_digest, "launch argv digest")?;
        if self.launch_argv_digest_profile != LAUNCH_ARGV_DIGEST_PROFILE {
            return Err(ConductorPolicyError::WrongLaunchArgvDigestProfile);
        }
        if self.service_uid == 0 || self.service_gid == 0 {
            return Err(ConductorPolicyError::RootServiceIdentityForbidden);
        }
        validate_systemd_unit(&self.systemd_unit)?;
        validate_text(&self.provisioning_ref, "provisioning reference")?;
        if self.valid_from_ms == 0 || self.valid_until_ms <= self.valid_from_ms {
            return Err(ConductorPolicyError::InvalidValidityWindow);
        }
        Ok(())
    }

    pub fn policy_digest(&self) -> Result<[u8; 32], ConductorPolicyError> {
        self.validate()?;
        let mut h = blake3::Hasher::new();
        h.update(DOMAIN_POLICY);
        frame(&mut h, PROTOCOL_VERSION.as_bytes());
        frame(&mut h, POLICY_PROFILE.as_bytes());
        frame(&mut h, self.policy_id.as_bytes());
        frame(&mut h, &self.policy_version.to_le_bytes());
        frame(&mut h, &self.target_binding_digest);
        frame(&mut h, self.target_binding_profile.as_bytes());
        frame(&mut h, self.admin_endpoint.to_string().as_bytes());
        frame(&mut h, self.executable_path.as_bytes());
        frame(&mut h, &self.executable_digest);
        frame(&mut h, self.executable_digest_profile.as_bytes());
        frame(&mut h, self.config_path.as_bytes());
        frame(&mut h, &self.config_digest);
        frame(&mut h, self.config_digest_profile.as_bytes());
        frame(&mut h, &self.launch_argv_digest);
        frame(&mut h, self.launch_argv_digest_profile.as_bytes());
        frame(&mut h, &self.service_uid.to_le_bytes());
        frame(&mut h, &self.service_gid.to_le_bytes());
        frame(&mut h, self.systemd_unit.as_bytes());
        frame(&mut h, self.provisioning_ref.as_bytes());
        frame(&mut h, &self.valid_from_ms.to_le_bytes());
        frame(&mut h, &self.valid_until_ms.to_le_bytes());
        Ok(*h.finalize().as_bytes())
    }
}

/// Independently delivered exact policy fingerprint.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ConductorDeploymentPolicyPin {
    pub protocol_version: String,
    pub pin_profile: String,
    pub policy_digest: [u8; 32],
    pub policy_profile: String,
}

impl ConductorDeploymentPolicyPin {
    pub fn validate(&self) -> Result<(), ConductorPolicyError> {
        if self.protocol_version != PIN_PROTOCOL_VERSION || self.pin_profile != PIN_PROFILE {
            return Err(ConductorPolicyError::WrongPinProtocol);
        }
        validate_digest(&self.policy_digest, "policy pin digest")?;
        if self.policy_profile != POLICY_PROFILE {
            return Err(ConductorPolicyError::WrongPolicyProfile);
        }
        Ok(())
    }
}

/// Non-deserializable positive allowed-conductor policy.
#[derive(Clone, Debug, PartialEq, Eq, Serialize)]
pub struct QualifiedConductorDeploymentPolicy {
    policy: ConductorDeploymentPolicy,
    policy_digest: [u8; 32],
    policy_profile: String,
    qualification_digest: [u8; 32],
    qualification_profile: String,
    qualified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedConductorDeploymentPolicy {
    pub fn policy(&self) -> &ConductorDeploymentPolicy {
        &self.policy
    }

    pub fn policy_digest(&self) -> [u8; 32] {
        self.policy_digest
    }

    pub fn policy_profile(&self) -> &str {
        &self.policy_profile
    }

    pub fn qualification_digest(&self) -> [u8; 32] {
        self.qualification_digest
    }

    pub fn qualification_profile(&self) -> &str {
        &self.qualification_profile
    }

    pub fn qualified_at_ms(&self) -> u64 {
        self.qualified_at_ms
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }
}

/// Qualify one exact candidate deployment policy against an independently delivered pin.
///
/// This establishes policy identity only. It does not observe or authenticate a live
/// process. The clock is supplied by the caller because this is a pure theorem; a live
/// native adapter must own the clock when consuming it.
pub fn qualify_conductor_deployment_policy(
    policy: ConductorDeploymentPolicy,
    pin: &ConductorDeploymentPolicyPin,
    now_ms: u64,
) -> Result<QualifiedConductorDeploymentPolicy, ConductorPolicyError> {
    policy.validate()?;
    pin.validate()?;
    if now_ms < policy.valid_from_ms || now_ms >= policy.valid_until_ms {
        return Err(ConductorPolicyError::PolicyNotLive);
    }
    let policy_digest = policy.policy_digest()?;
    if pin.policy_digest != policy_digest || pin.policy_profile != POLICY_PROFILE {
        return Err(ConductorPolicyError::PinMismatch);
    }

    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_QUALIFICATION);
    frame(&mut h, PROTOCOL_VERSION.as_bytes());
    frame(&mut h, QUALIFICATION_PROFILE.as_bytes());
    frame(&mut h, &policy_digest);
    frame(&mut h, POLICY_PROFILE.as_bytes());
    let qualification_digest = *h.finalize().as_bytes();
    let valid_until_ms = policy.valid_until_ms;

    Ok(QualifiedConductorDeploymentPolicy {
        policy,
        policy_digest,
        policy_profile: POLICY_PROFILE.into(),
        qualification_digest,
        qualification_profile: QUALIFICATION_PROFILE.into(),
        qualified_at_ms: now_ms,
        valid_until_ms,
    })
}

fn validate_nix_store_file_path(value: &str, field: &'static str) -> Result<(), ConductorPolicyError> {
    validate_text(value, field)?;
    let path = Path::new(value);
    if !path.is_absolute()
        || !value.starts_with(NIX_STORE_PREFIX)
        || value.ends_with('/')
        || path.components().any(|component| matches!(component, std::path::Component::ParentDir))
    {
        return Err(ConductorPolicyError::InvalidNixStorePath(field));
    }
    Ok(())
}

fn validate_systemd_unit(value: &str) -> Result<(), ConductorPolicyError> {
    validate_text(value, "systemd unit")?;
    if !value.ends_with(".service")
        || value.contains('/')
        || value.chars().any(|c| c.is_control() || c.is_whitespace())
    {
        return Err(ConductorPolicyError::InvalidSystemdUnit);
    }
    Ok(())
}

fn validate_text(value: &str, field: &'static str) -> Result<(), ConductorPolicyError> {
    if value.trim().is_empty()
        || value.len() > MAX_TEXT_BYTES
        || value.chars().any(|c| c.is_control())
    {
        return Err(ConductorPolicyError::InvalidText(field));
    }
    Ok(())
}

fn validate_digest(value: &[u8; 32], field: &'static str) -> Result<(), ConductorPolicyError> {
    if value.iter().all(|byte| *byte == 0) {
        Err(ConductorPolicyError::InvalidDigest(field))
    } else {
        Ok(())
    }
}

fn frame(hasher: &mut blake3::Hasher, bytes: &[u8]) {
    hasher.update(&(bytes.len() as u64).to_le_bytes());
    hasher.update(bytes);
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ConductorPolicyError {
    WrongProtocol,
    WrongPinProtocol,
    WrongPolicyProfile,
    WrongTargetBindingProfile,
    WrongExecutableDigestProfile,
    WrongConfigDigestProfile,
    WrongLaunchArgvDigestProfile,
    InvalidPolicyVersion,
    InvalidAdminEndpoint,
    InvalidNixStorePath(&'static str),
    RootServiceIdentityForbidden,
    InvalidSystemdUnit,
    InvalidValidityWindow,
    InvalidText(&'static str),
    InvalidDigest(&'static str),
    PolicyNotLive,
    PinMismatch,
}

impl fmt::Display for ConductorPolicyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongProtocol => write!(f, "wrong conductor deployment policy protocol"),
            Self::WrongPinProtocol => write!(f, "wrong conductor deployment policy pin protocol/profile"),
            Self::WrongPolicyProfile => write!(f, "wrong conductor deployment policy profile"),
            Self::WrongTargetBindingProfile => write!(f, "wrong target binding profile"),
            Self::WrongExecutableDigestProfile => write!(f, "wrong executable digest profile"),
            Self::WrongConfigDigestProfile => write!(f, "wrong config digest profile"),
            Self::WrongLaunchArgvDigestProfile => write!(f, "wrong launch argv digest profile"),
            Self::InvalidPolicyVersion => write!(f, "invalid conductor deployment policy version"),
            Self::InvalidAdminEndpoint => write!(f, "conductor admin endpoint must be loopback with non-zero port"),
            Self::InvalidNixStorePath(field) => write!(f, "invalid Nix store {field}"),
            Self::RootServiceIdentityForbidden => write!(f, "root UID/GID conductor service identity is forbidden"),
            Self::InvalidSystemdUnit => write!(f, "invalid systemd service unit"),
            Self::InvalidValidityWindow => write!(f, "invalid conductor deployment policy validity window"),
            Self::InvalidText(field) => write!(f, "invalid {field}"),
            Self::InvalidDigest(field) => write!(f, "invalid {field}"),
            Self::PolicyNotLive => write!(f, "conductor deployment policy is not live"),
            Self::PinMismatch => write!(f, "candidate conductor deployment policy does not match pinned identity"),
        }
    }
}

impl std::error::Error for ConductorPolicyError {}

#[cfg(test)]
mod tests {
    use super::*;
    use std::net::{Ipv4Addr, SocketAddr};

    fn d(byte: u8) -> [u8; 32] {
        [byte; 32]
    }

    fn policy() -> ConductorDeploymentPolicy {
        ConductorDeploymentPolicy {
            protocol_version: PROTOCOL_VERSION.into(),
            policy_id: "mycelix-holochain-conductor".into(),
            policy_version: 1,
            target_binding_digest: d(1),
            target_binding_profile: TARGET_BINDING_PROFILE.into(),
            admin_endpoint: SocketAddr::from((Ipv4Addr::LOCALHOST, 8888)),
            executable_path: "/nix/store/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa-holochain/bin/holochain".into(),
            executable_digest: d(2),
            executable_digest_profile: EXECUTABLE_DIGEST_PROFILE.into(),
            config_path: "/nix/store/bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb-holochain-config/conductor-config.yaml".into(),
            config_digest: d(3),
            config_digest_profile: CONFIG_DIGEST_PROFILE.into(),
            launch_argv_digest: d(4),
            launch_argv_digest_profile: LAUNCH_ARGV_DIGEST_PROFILE.into(),
            service_uid: 991,
            service_gid: 991,
            systemd_unit: "mycelix-holochain.service".into(),
            provisioning_ref: "provisioning:conductor:1".into(),
            valid_from_ms: 100,
            valid_until_ms: 1_000,
        }
    }

    fn pin(policy: &ConductorDeploymentPolicy) -> ConductorDeploymentPolicyPin {
        ConductorDeploymentPolicyPin {
            protocol_version: PIN_PROTOCOL_VERSION.into(),
            pin_profile: PIN_PROFILE.into(),
            policy_digest: policy.policy_digest().unwrap(),
            policy_profile: POLICY_PROFILE.into(),
        }
    }

    #[test]
    fn exact_pinned_policy_qualifies() {
        let candidate = policy();
        let qualified = qualify_conductor_deployment_policy(candidate.clone(), &pin(&candidate), 200).unwrap();
        assert_eq!(qualified.policy_digest(), candidate.policy_digest().unwrap());
        assert_eq!(qualified.valid_until_ms(), 1_000);
    }

    #[test]
    fn changed_executable_or_target_binding_breaks_pin() {
        let original = policy();
        let root_pin = pin(&original);

        let mut changed_executable = original.clone();
        changed_executable.executable_digest = d(9);
        assert_eq!(
            qualify_conductor_deployment_policy(changed_executable, &root_pin, 200).unwrap_err(),
            ConductorPolicyError::PinMismatch
        );

        let mut changed_target = original;
        changed_target.target_binding_digest = d(8);
        assert_eq!(
            qualify_conductor_deployment_policy(changed_target, &root_pin, 200).unwrap_err(),
            ConductorPolicyError::PinMismatch
        );
    }

    #[test]
    fn non_loopback_or_root_service_denies() {
        let mut candidate = policy();
        candidate.admin_endpoint = SocketAddr::from(([10, 0, 0, 1], 8888));
        assert_eq!(candidate.validate().unwrap_err(), ConductorPolicyError::InvalidAdminEndpoint);

        let mut root_service = policy();
        root_service.service_uid = 0;
        assert_eq!(root_service.validate().unwrap_err(), ConductorPolicyError::RootServiceIdentityForbidden);
    }

    #[test]
    fn mutable_non_nix_paths_deny() {
        let mut candidate = policy();
        candidate.executable_path = "/usr/bin/holochain".into();
        assert!(matches!(
            candidate.validate().unwrap_err(),
            ConductorPolicyError::InvalidNixStorePath("executable path")
        ));

        let mut candidate = policy();
        candidate.config_path = "/etc/mycelix/conductor.yaml".into();
        assert!(matches!(
            candidate.validate().unwrap_err(),
            ConductorPolicyError::InvalidNixStorePath("config path")
        ));
    }

    #[test]
    fn policy_expiry_denies() {
        let candidate = policy();
        assert_eq!(
            qualify_conductor_deployment_policy(candidate.clone(), &pin(&candidate), 1_000).unwrap_err(),
            ConductorPolicyError::PolicyNotLive
        );
    }
}
