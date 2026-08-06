// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Offline cutover of validated file-backed authority journals into PostgreSQL.

use super::credential_registry::{read_public_key_trust_file, read_signing_key};
use mycelix_desci_core::{
    AuthorityWriteLease, FileAuthorityWriteLeaseProvider, PostgresAuthorityConfig,
    PostgresAuthorityFencingConfig, PostgresAuthorityStore, Result, ScientificCredentialGovernance,
    ScientificCredentialRegistry, SignedAuthorityWriteLease,
};
use std::collections::BTreeSet;
use std::fs::OpenOptions;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::Arc;

#[allow(clippy::too_many_arguments)]
pub async fn import_file_authority(
    database_url: String,
    registry_path: PathBuf,
    governance_path: PathBuf,
    bootstrap_trust_file: PathBuf,
    acceptance_signing_key_file: PathBuf,
    acceptance_trust_file: Option<PathBuf>,
    outbox_signing_key_file: PathBuf,
    deployment_id: String,
    write_lease_file: PathBuf,
    write_lease_trust_file: PathBuf,
    auto_migrate: bool,
) -> Result<()> {
    let bootstrap_trust =
        read_public_key_trust_file(&bootstrap_trust_file, "credential bootstrap trust", false)?;
    let acceptance_key = Arc::new(read_signing_key(
        &acceptance_signing_key_file,
        "credential acceptance service signing key",
    )?);
    let mut acceptance_trust = acceptance_trust_file
        .as_deref()
        .map(|path| read_public_key_trust_file(path, "credential acceptance trust", true))
        .transpose()?
        .unwrap_or_else(BTreeSet::new);
    acceptance_trust.insert(acceptance_key.verifying_key().to_bytes());
    let outbox_key = Arc::new(read_signing_key(
        &outbox_signing_key_file,
        "authority outbox delivery signing key",
    )?);

    let registry = Arc::new(
        ScientificCredentialRegistry::open_file(
            registry_path,
            bootstrap_trust,
            acceptance_trust.clone(),
            Some(acceptance_key.clone()),
        )
        .await?,
    );
    if registry
        .projection()
        .await
        .owns_signing_key(&outbox_key.verifying_key().to_bytes())
    {
        return Err(mycelix_desci_core::Error::VerificationFailed(
            "authority outbox signing key must not be registered as a scientific actor key"
                .to_string(),
        ));
    }
    let governance = ScientificCredentialGovernance::open_file(
        registry.clone(),
        governance_path,
        acceptance_trust.clone(),
        Some(acceptance_key),
    )
    .await?;
    let credential_records = registry.events().await;
    let governance_records = governance.events().await;
    if credential_records.is_empty() || governance_records.is_empty() {
        return Err(mycelix_desci_core::Error::Validation(
            "PostgreSQL cutover requires initialized credential and governance journals"
                .to_string(),
        ));
    }

    let mut config = PostgresAuthorityConfig::from_database_url(database_url);
    config.run_migrations = auto_migrate;
    let write_lease_trust = read_public_key_trust_file(
        &write_lease_trust_file,
        "authority write-lease trust",
        false,
    )?;
    let registry_projection = registry.projection().await;
    if write_lease_trust
        .iter()
        .any(|key| registry_projection.owns_signing_key(key))
    {
        return Err(mycelix_desci_core::Error::VerificationFailed(
            "authority write-lease keys must not be registered as scientific actor keys"
                .to_string(),
        ));
    }
    let fencing = PostgresAuthorityFencingConfig::new(
        deployment_id,
        write_lease_trust,
        Arc::new(FileAuthorityWriteLeaseProvider::new(write_lease_file)?),
    );
    let store =
        PostgresAuthorityStore::connect_with_fencing(config, acceptance_trust, outbox_key, fencing)
            .await?;
    store
        .import_credential_authority_history(&credential_records, &governance_records)
        .await?;

    println!("Imported credential records: {}", credential_records.len());
    println!("Imported governance records: {}", governance_records.len());
    println!("PostgreSQL authority cutover completed atomically");
    Ok(())
}

pub fn sign_write_lease(
    lease_file: PathBuf,
    signing_key_file: PathBuf,
    output: PathBuf,
) -> Result<()> {
    reject_existing_output(&output, "signed authority write lease")?;
    let metadata = std::fs::symlink_metadata(&lease_file)
        .map_err(|error| mycelix_desci_core::Error::Storage(error.to_string()))?;
    if metadata.file_type().is_symlink() || !metadata.is_file() {
        return Err(mycelix_desci_core::Error::Validation(format!(
            "authority write lease input must be a regular non-symbolic-link file: {}",
            lease_file.display()
        )));
    }
    if metadata.len() == 0 || metadata.len() > 1024 * 1024 {
        return Err(mycelix_desci_core::Error::Validation(
            "authority write lease input must contain 1-1048576 bytes".to_string(),
        ));
    }
    let bytes = std::fs::read(&lease_file)
        .map_err(|error| mycelix_desci_core::Error::Storage(error.to_string()))?;
    let lease: AuthorityWriteLease = serde_json::from_slice(&bytes)?;
    lease.validate()?;
    let key = read_signing_key(&signing_key_file, "authority write-lease signing key")?;
    let signed = SignedAuthorityWriteLease::sign(lease, &key)?;
    write_json_create_new(&output, &signed)?;
    println!("Signed authority write lease: {}", output.display());
    println!("Lease id: {}", signed.lease.lease_id);
    println!("Generation: {}", signed.lease.generation);
    println!("Expires at: {}", signed.lease.expires_at.to_rfc3339());
    println!(
        "Signer public key: {}",
        hex::encode(signed.signer_public_key)
    );
    Ok(())
}

fn reject_existing_output(path: &Path, label: &str) -> Result<()> {
    match std::fs::symlink_metadata(path) {
        Ok(_) => Err(mycelix_desci_core::Error::Validation(format!(
            "{label} output already exists: {}",
            path.display()
        ))),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => Ok(()),
        Err(error) => Err(mycelix_desci_core::Error::Storage(error.to_string())),
    }
}

fn write_json_create_new<T: serde::Serialize>(path: &Path, value: &T) -> Result<()> {
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent)
            .map_err(|error| mycelix_desci_core::Error::Storage(error.to_string()))?;
    }
    let bytes = serde_json::to_vec_pretty(value)?;
    let mut file = OpenOptions::new()
        .create_new(true)
        .write(true)
        .open(path)
        .map_err(|error| mycelix_desci_core::Error::Storage(error.to_string()))?;
    if let Err(error) = file
        .write_all(&bytes)
        .and_then(|_| file.write_all(b"\n"))
        .and_then(|_| file.sync_all())
    {
        let _ = std::fs::remove_file(path);
        return Err(mycelix_desci_core::Error::Storage(error.to_string()));
    }
    Ok(())
}
