//! Provisioned filesystem identity binding for integration runtime stores.
//!
//! This crate does not decide which store is trusted. It verifies that the path
//! still names the exact regular file identity and security attributes supplied
//! by an enclosing deployment/configuration authority. Recreating a binding after
//! restart is safe only when the expectation comes from that independent trusted
//! provisioning source, not by accepting whatever file currently exists.

#[cfg(not(unix))]
compile_error!("integration runtime store binding v0.1 requires Unix file identity semantics");

use mycelix_institutional_core::Digest32;
use std::fs;
use std::os::unix::fs::MetadataExt;
use std::path::{Path, PathBuf};
use thiserror::Error;

pub const RUNTIME_STORE_BINDING_PROFILE: &str =
    "mycelix-integration-runtime-store-binding-v1-blake3-framed";
const DOMAIN_BINDING: &[u8] = b"mycelix/integration/runtime-store-binding/v1";
const MAX_PROVISIONING_REF_BYTES: usize = 2048;
const PERMISSION_MASK: u32 = 0o7777;

/// Exact deployment-owned expectation for the SQLite main-file security
/// boundary. These values must come from independent provisioning evidence.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RuntimeStoreProvisioningExpectation {
    pub device: u64,
    pub inode: u64,
    pub uid: u32,
    pub gid: u32,
    /// Unix permission/special bits, normalized to `0o7777`.
    pub mode: u32,
}

impl RuntimeStoreProvisioningExpectation {
    pub fn validate(self) -> Result<(), RuntimeStoreBindingError> {
        if self.inode == 0 || self.mode & !PERMISSION_MASK != 0 {
            return Err(RuntimeStoreBindingError::InvalidExpectedIdentity);
        }
        // SQLite must remain owner-readable/writable and must not be writable by
        // group/other identities. Exact mode is frozen after this check.
        if self.mode & 0o600 != 0o600 || self.mode & 0o022 != 0 {
            return Err(RuntimeStoreBindingError::UnsafeFileMode);
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct QualifiedRuntimeStoreBinding {
    canonical_path: PathBuf,
    expectation: RuntimeStoreProvisioningExpectation,
    provisioning_ref: String,
    binding_digest: Digest32,
}

impl QualifiedRuntimeStoreBinding {
    pub fn path(&self) -> &Path {
        &self.canonical_path
    }

    pub fn expected_device(&self) -> u64 {
        self.expectation.device
    }

    pub fn expected_inode(&self) -> u64 {
        self.expectation.inode
    }

    pub fn expected_uid(&self) -> u32 {
        self.expectation.uid
    }

    pub fn expected_gid(&self) -> u32 {
        self.expectation.gid
    }

    pub fn expected_mode(&self) -> u32 {
        self.expectation.mode
    }

    pub fn provisioning_ref(&self) -> &str {
        &self.provisioning_ref
    }

    pub fn binding_digest(&self) -> Digest32 {
        self.binding_digest
    }

    /// Recheck both object identity and security attributes. `chmod`/`chown`
    /// drift invalidates the binding even when device/inode remain unchanged.
    pub fn revalidate(&self) -> Result<(), RuntimeStoreBindingError> {
        require_exact_identity_and_security(&self.canonical_path, self.expectation)
    }

    pub const fn provisioning_origin_verified_here(&self) -> bool {
        false
    }

    pub const fn current_file_identity_verified_here(&self) -> bool {
        true
    }

    pub const fn current_security_attributes_verified_here(&self) -> bool {
        true
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

pub fn qualify_runtime_store_binding(
    path: impl AsRef<Path>,
    expectation: RuntimeStoreProvisioningExpectation,
    provisioning_ref: impl Into<String>,
) -> Result<QualifiedRuntimeStoreBinding, RuntimeStoreBindingError> {
    expectation.validate()?;
    let provisioning_ref = provisioning_ref.into();
    validate_provisioning_ref(&provisioning_ref)?;

    let path = path.as_ref();
    let link = fs::symlink_metadata(path).map_err(RuntimeStoreBindingError::Io)?;
    if link.file_type().is_symlink() {
        return Err(RuntimeStoreBindingError::SymlinkPath);
    }
    let canonical_path = fs::canonicalize(path).map_err(RuntimeStoreBindingError::Io)?;
    require_exact_identity_and_security(&canonical_path, expectation)?;

    let binding_digest = binding_digest(&canonical_path, expectation, &provisioning_ref);
    Ok(QualifiedRuntimeStoreBinding {
        canonical_path,
        expectation,
        provisioning_ref,
        binding_digest,
    })
}

fn require_exact_identity_and_security(
    path: &Path,
    expectation: RuntimeStoreProvisioningExpectation,
) -> Result<(), RuntimeStoreBindingError> {
    let metadata = fs::metadata(path).map_err(RuntimeStoreBindingError::Io)?;
    if !metadata.is_file()
        || metadata.dev() != expectation.device
        || metadata.ino() != expectation.inode
    {
        return Err(RuntimeStoreBindingError::StoreIdentityChanged);
    }
    let mode = metadata.mode() & PERMISSION_MASK;
    if metadata.uid() != expectation.uid
        || metadata.gid() != expectation.gid
        || mode != expectation.mode
    {
        return Err(RuntimeStoreBindingError::StoreSecurityAttributesChanged);
    }
    // Defense in depth if an invalid expectation somehow crossed the constructor.
    if mode & 0o600 != 0o600 || mode & 0o022 != 0 {
        return Err(RuntimeStoreBindingError::UnsafeFileMode);
    }
    Ok(())
}

fn binding_digest(
    path: &Path,
    expectation: RuntimeStoreProvisioningExpectation,
    provisioning_ref: &str,
) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_BINDING);
    frame(&mut h, RUNTIME_STORE_BINDING_PROFILE.as_bytes());
    frame(&mut h, path.as_os_str().as_encoded_bytes());
    frame(&mut h, &expectation.device.to_le_bytes());
    frame(&mut h, &expectation.inode.to_le_bytes());
    frame(&mut h, &expectation.uid.to_le_bytes());
    frame(&mut h, &expectation.gid.to_le_bytes());
    frame(&mut h, &expectation.mode.to_le_bytes());
    frame(&mut h, provisioning_ref.as_bytes());
    Digest32(*h.finalize().as_bytes())
}

fn validate_provisioning_ref(value: &str) -> Result<(), RuntimeStoreBindingError> {
    if value.trim().is_empty()
        || value.len() > MAX_PROVISIONING_REF_BYTES
        || value.chars().any(char::is_control)
    {
        Err(RuntimeStoreBindingError::InvalidProvisioningRef)
    } else {
        Ok(())
    }
}

fn frame(h: &mut blake3::Hasher, bytes: &[u8]) {
    h.update(&(bytes.len() as u64).to_le_bytes());
    h.update(bytes);
}

#[derive(Debug, Error)]
pub enum RuntimeStoreBindingError {
    #[error("runtime store provisioning reference is invalid")]
    InvalidProvisioningRef,
    #[error("runtime store expected filesystem identity/security attributes are invalid")]
    InvalidExpectedIdentity,
    #[error("runtime store path is a symlink")]
    SymlinkPath,
    #[error("runtime store identity changed")]
    StoreIdentityChanged,
    #[error("runtime store ownership or mode changed")]
    StoreSecurityAttributesChanged,
    #[error("runtime store mode must be owner-readable/writable and not group/world writable")]
    UnsafeFileMode,
    #[error("runtime store filesystem error: {0}")]
    Io(std::io::Error),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn provisioning_reference_rejects_empty_and_control_data() {
        assert!(validate_provisioning_ref("").is_err());
        assert!(validate_provisioning_ref("bad\nref").is_err());
        assert!(validate_provisioning_ref("nix/store-binding/v1").is_ok());
    }

    #[test]
    fn provisioning_expectation_rejects_unsafe_modes() {
        let base = RuntimeStoreProvisioningExpectation {
            device: 1,
            inode: 2,
            uid: 1000,
            gid: 1000,
            mode: 0o600,
        };
        assert!(base.validate().is_ok());
        assert!(
            RuntimeStoreProvisioningExpectation {
                mode: 0o640,
                ..base
            }
            .validate()
            .is_ok()
        );
        assert!(
            RuntimeStoreProvisioningExpectation {
                mode: 0o660,
                ..base
            }
            .validate()
            .is_err()
        );
        assert!(
            RuntimeStoreProvisioningExpectation {
                mode: 0o400,
                ..base
            }
            .validate()
            .is_err()
        );
    }
}
