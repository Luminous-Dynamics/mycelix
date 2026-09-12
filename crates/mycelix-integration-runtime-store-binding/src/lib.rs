//! Provisioned filesystem identity binding for integration runtime stores.
//!
//! This crate does not decide which store is trusted. It verifies that the path
//! still names the exact regular file identity provisioned by an enclosing
//! deployment/configuration authority. Recreating a binding after restart is safe
//! only when the expected device/inode values come from that independent trusted
//! provisioning source, not by simply accepting whatever file currently exists.

#[cfg(not(unix))]
compile_error!("integration runtime store binding v0.1 requires Unix file identity semantics");

use mycelix_institutional_core::Digest32;
use std::fs;
use std::os::unix::fs::{MetadataExt, PermissionsExt};
use std::path::{Path, PathBuf};
use thiserror::Error;

pub const RUNTIME_STORE_BINDING_PROFILE: &str =
    "mycelix-integration-runtime-store-binding-v1-blake3-framed";
const DOMAIN_BINDING: &[u8] = b"mycelix/integration/runtime-store-binding/v1";
const MAX_PROVISIONING_REF_BYTES: usize = 2048;

#[derive(Clone, Debug)]
pub struct QualifiedRuntimeStoreBinding {
    canonical_path: PathBuf,
    expected_device: u64,
    expected_inode: u64,
    provisioning_ref: String,
    binding_digest: Digest32,
}

impl QualifiedRuntimeStoreBinding {
    pub fn path(&self) -> &Path {
        &self.canonical_path
    }

    pub fn expected_device(&self) -> u64 {
        self.expected_device
    }

    pub fn expected_inode(&self) -> u64 {
        self.expected_inode
    }

    pub fn provisioning_ref(&self) -> &str {
        &self.provisioning_ref
    }

    pub fn binding_digest(&self) -> Digest32 {
        self.binding_digest
    }

    /// Recheck that the provisioned path still names the exact regular file.
    pub fn revalidate(&self) -> Result<(), RuntimeStoreBindingError> {
        require_exact_identity(
            &self.canonical_path,
            self.expected_device,
            self.expected_inode,
        )
    }

    pub const fn provisioning_origin_verified_here(&self) -> bool {
        false
    }

    pub const fn current_file_identity_verified_here(&self) -> bool {
        true
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

pub fn qualify_runtime_store_binding(
    path: impl AsRef<Path>,
    expected_device: u64,
    expected_inode: u64,
    provisioning_ref: impl Into<String>,
) -> Result<QualifiedRuntimeStoreBinding, RuntimeStoreBindingError> {
    let provisioning_ref = provisioning_ref.into();
    validate_provisioning_ref(&provisioning_ref)?;
    if expected_inode == 0 {
        return Err(RuntimeStoreBindingError::InvalidExpectedIdentity);
    }

    let path = path.as_ref();
    let link = fs::symlink_metadata(path).map_err(RuntimeStoreBindingError::Io)?;
    if link.file_type().is_symlink() {
        return Err(RuntimeStoreBindingError::SymlinkPath);
    }
    let canonical_path = fs::canonicalize(path).map_err(RuntimeStoreBindingError::Io)?;
    require_exact_identity(&canonical_path, expected_device, expected_inode)?;

    let metadata = fs::metadata(&canonical_path).map_err(RuntimeStoreBindingError::Io)?;
    // v0.1 requires the SQLite main file not be group/world writable. This is
    // intentionally weaker than requiring 0600 so existing owner-readable
    // deployments can migrate without silently accepting writable-by-others.
    if metadata.permissions().mode() & 0o022 != 0 {
        return Err(RuntimeStoreBindingError::UnsafeFileMode);
    }

    let binding_digest = binding_digest(
        &canonical_path,
        expected_device,
        expected_inode,
        &provisioning_ref,
    );
    Ok(QualifiedRuntimeStoreBinding {
        canonical_path,
        expected_device,
        expected_inode,
        provisioning_ref,
        binding_digest,
    })
}

fn require_exact_identity(
    path: &Path,
    expected_device: u64,
    expected_inode: u64,
) -> Result<(), RuntimeStoreBindingError> {
    let metadata = fs::metadata(path).map_err(RuntimeStoreBindingError::Io)?;
    if !metadata.is_file()
        || metadata.dev() != expected_device
        || metadata.ino() != expected_inode
    {
        return Err(RuntimeStoreBindingError::StoreIdentityChanged);
    }
    Ok(())
}

fn binding_digest(path: &Path, device: u64, inode: u64, provisioning_ref: &str) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_BINDING);
    frame(&mut h, RUNTIME_STORE_BINDING_PROFILE.as_bytes());
    frame(&mut h, path.as_os_str().as_encoded_bytes());
    frame(&mut h, &device.to_le_bytes());
    frame(&mut h, &inode.to_le_bytes());
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
    #[error("runtime store expected filesystem identity is invalid")]
    InvalidExpectedIdentity,
    #[error("runtime store path is a symlink")]
    SymlinkPath,
    #[error("runtime store identity changed")]
    StoreIdentityChanged,
    #[error("runtime store file is group/world writable")]
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
}
