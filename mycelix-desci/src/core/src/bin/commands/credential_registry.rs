// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Offline bootstrap for the append-only scientific credential registry.

use chrono::{Duration, Utc};
use ed25519_dalek::SigningKey;
use mycelix_desci_core::{
    ActorId, AuthorizedActorKey, OrganizationId, ResolvedScientificActor, Result,
    ScientificCredentialEnvelope, ScientificCredentialRegistry, ScientificRole,
    SignedScientificCredentialEvent,
};
use std::collections::BTreeSet;
use std::fs::{File, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::sync::Arc;

pub async fn initialize(
    registry_path: PathBuf,
    actor_value: String,
    signing_key_file: PathBuf,
    acceptance_signing_key_file: PathBuf,
    bootstrap_trust_output: PathBuf,
    organization: Option<String>,
) -> Result<()> {
    reject_existing_output(&registry_path, "credential registry")?;
    reject_existing_output(&bootstrap_trust_output, "bootstrap trust file")?;

    let signing_key = read_signing_key(&signing_key_file, "credential bootstrap signing key")?;
    let acceptance_key = Arc::new(read_signing_key(
        &acceptance_signing_key_file,
        "credential acceptance service signing key",
    )?);
    if signing_key.verifying_key().to_bytes() == acceptance_key.verifying_key().to_bytes() {
        return Err(mycelix_desci_core::Error::Crypto(
            "credential actor and acceptance service must use distinct signing keys".to_string(),
        ));
    }
    let actor = ActorId::new(actor_value)?;
    let now = Utc::now();
    let organizations = organization
        .map(OrganizationId::new)
        .transpose()?
        .into_iter()
        .collect::<BTreeSet<_>>();
    let administrator = ResolvedScientificActor {
        actor: actor.clone(),
        authorized_keys: vec![AuthorizedActorKey {
            public_key: signing_key.verifying_key().to_bytes(),
            valid_from: now.clone() - Duration::seconds(1),
            valid_until: None,
            revoked_at: None,
        }],
        organizations,
        roles: BTreeSet::from([ScientificRole::RegistryAdmin]),
        authority_revision: None,
    };
    let envelope = ScientificCredentialEnvelope::genesis(actor, now, administrator)?;
    let event = SignedScientificCredentialEvent::sign(envelope, &signing_key)?;
    let trusted_key = signing_key.verifying_key().to_bytes();
    let trusted_keys = BTreeSet::from([trusted_key]);

    write_bootstrap_trust(&bootstrap_trust_output, trusted_key)?;
    let result = async {
        let registry = ScientificCredentialRegistry::open_file(
            &registry_path,
            trusted_keys,
            BTreeSet::from([acceptance_key.verifying_key().to_bytes()]),
            Some(acceptance_key.clone()),
        )
        .await?;
        let receipt = registry.append(0, event).await?;
        println!("Credential registry: {}", registry_path.display());
        println!("Bootstrap trust: {}", bootstrap_trust_output.display());
        println!("Registry revision: {}", receipt.registry_revision);
        println!(
            "Initial administrator public key: {}",
            hex::encode(trusted_key)
        );
        Ok(())
    }
    .await;

    if result.is_err() {
        let _ = std::fs::remove_file(&registry_path);
        let _ = std::fs::remove_file(&bootstrap_trust_output);
    }
    result
}

fn reject_existing_output(path: &Path, label: &str) -> Result<()> {
    if path.exists() {
        return Err(mycelix_desci_core::Error::Validation(format!(
            "{label} already exists and will not be overwritten: {}",
            path.display()
        )));
    }
    if let Some(parent) = path.parent() {
        if !parent.as_os_str().is_empty() {
            std::fs::create_dir_all(parent)?;
        }
    }
    Ok(())
}

fn write_bootstrap_trust(path: &Path, public_key: [u8; 32]) -> Result<()> {
    let encoded = serde_json::to_vec_pretty(&vec![hex::encode(public_key)])?;
    let mut file = OpenOptions::new().write(true).create_new(true).open(path)?;
    file.write_all(&encoded)?;
    file.write_all(b"\n")?;
    file.sync_all()?;
    if let Some(parent) = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        File::open(parent)?.sync_all()?;
    }
    Ok(())
}

pub(super) fn read_signing_key(path: &Path, label: &str) -> Result<SigningKey> {
    let metadata = std::fs::symlink_metadata(path)?;
    if metadata.file_type().is_symlink() || !metadata.is_file() {
        return Err(mycelix_desci_core::Error::Crypto(format!(
            "{label} must be a regular non-symbolic-link file"
        )));
    }
    if metadata.len() > 4096 {
        return Err(mycelix_desci_core::Error::Crypto(format!(
            "{label} file exceeds 4096 bytes"
        )));
    }
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        if metadata.permissions().mode() & 0o077 != 0 {
            return Err(mycelix_desci_core::Error::Crypto(format!(
                "{label} requires mode 0600 or stricter"
            )));
        }
    }
    let bytes = std::fs::read(path)?;
    let key_bytes = if bytes.len() == 32 {
        bytes
    } else {
        let text = std::str::from_utf8(&bytes).map_err(|_| {
            mycelix_desci_core::Error::Crypto(format!(
                "{label} must be 32 raw bytes or 64 hexadecimal characters"
            ))
        })?;
        hex::decode(text.trim()).map_err(|error| {
            mycelix_desci_core::Error::Crypto(format!("invalid hexadecimal {label}: {error}"))
        })?
    };
    let key_bytes: [u8; 32] = key_bytes.try_into().map_err(|_| {
        mycelix_desci_core::Error::Crypto(format!("{label} must decode to exactly 32 bytes"))
    })?;
    Ok(SigningKey::from_bytes(&key_bytes))
}

pub async fn register_actor(
    registry_path: PathBuf,
    bootstrap_trust_file: PathBuf,
    administrator_value: String,
    administrator_signing_key_file: PathBuf,
    acceptance_signing_key_file: PathBuf,
    acceptance_trust_file: Option<PathBuf>,
    actor_value: String,
    actor_public_key_hex: String,
    role_values: Vec<String>,
    organization_values: Vec<String>,
) -> Result<()> {
    let trust =
        read_public_key_trust_file(&bootstrap_trust_file, "credential bootstrap trust", false)?;
    let acceptance_key = Arc::new(read_signing_key(
        &acceptance_signing_key_file,
        "credential acceptance service signing key",
    )?);
    let mut acceptance_trust = acceptance_trust_file
        .as_deref()
        .map(|path| read_public_key_trust_file(path, "credential acceptance trust", true))
        .transpose()?
        .unwrap_or_default();
    acceptance_trust.insert(acceptance_key.verifying_key().to_bytes());
    let registry = ScientificCredentialRegistry::open_file(
        &registry_path,
        trust,
        acceptance_trust,
        Some(acceptance_key.clone()),
    )
    .await?;
    let events = registry.events().await;
    let previous = events.last().ok_or_else(|| {
        mycelix_desci_core::Error::Validation(
            "credential registry must be initialized before actors are registered".to_string(),
        )
    })?;
    let administrator = ActorId::new(administrator_value)?;
    let administrator_key = read_signing_key(
        &administrator_signing_key_file,
        "registry administrator signing key",
    )?;
    if administrator_key.verifying_key().to_bytes() == acceptance_key.verifying_key().to_bytes() {
        return Err(mycelix_desci_core::Error::Crypto(
            "registry administrator and acceptance service must use distinct signing keys"
                .to_string(),
        ));
    }
    let now = Utc::now();
    let administrator_profile = registry
        .projection()
        .await
        .resolve(&administrator)
        .ok_or_else(|| {
            mycelix_desci_core::Error::VerificationFailed(
                "administrator is absent from the scientific credential registry".to_string(),
            )
        })?;
    if !administrator_profile.has_role(ScientificRole::RegistryAdmin)
        || !administrator_profile
            .authorizes_key(&administrator_key.verifying_key().to_bytes(), &now)
    {
        return Err(mycelix_desci_core::Error::VerificationFailed(
            "supplied administrator key is not active with registry_admin authority".to_string(),
        ));
    }

    let actor = ActorId::new(actor_value)?;
    let public_key = decode_public_key(&actor_public_key_hex)?;
    let roles = role_values
        .iter()
        .map(|value| parse_role(value))
        .collect::<Result<BTreeSet<_>>>()?;
    if roles.is_empty() {
        return Err(mycelix_desci_core::Error::Validation(
            "new scientific actors require at least one role".to_string(),
        ));
    }
    let organizations = organization_values
        .into_iter()
        .map(OrganizationId::new)
        .collect::<Result<BTreeSet<_>>>()?;
    let profile = ResolvedScientificActor {
        actor: actor.clone(),
        authorized_keys: vec![AuthorizedActorKey {
            public_key,
            valid_from: now.clone() - Duration::seconds(1),
            valid_until: None,
            revoked_at: None,
        }],
        organizations,
        roles,
        authority_revision: None,
    };
    let envelope = ScientificCredentialEnvelope::successor(
        previous,
        administrator,
        now,
        format!("register-actor:{}", actor.as_str()),
        mycelix_desci_core::ScientificCredentialPayload::ActorRegistered { profile },
    )?;
    let event = SignedScientificCredentialEvent::sign(envelope, &administrator_key)?;
    let receipt = registry.append(events.len() as u64, event).await?;
    println!("Registered scientific actor: {actor}");
    println!("Registry revision: {}", receipt.registry_revision);
    Ok(())
}

pub(super) fn read_public_key_trust_file(
    path: &Path,
    label: &str,
    allow_empty: bool,
) -> Result<BTreeSet<[u8; 32]>> {
    let metadata = std::fs::symlink_metadata(path)?;
    if metadata.file_type().is_symlink() || !metadata.is_file() {
        return Err(mycelix_desci_core::Error::Crypto(format!(
            "{label} must be a regular non-symbolic-link file"
        )));
    }
    if metadata.len() > 1024 * 1024 {
        return Err(mycelix_desci_core::Error::Crypto(format!(
            "{label} file exceeds 1 MiB"
        )));
    }
    let encoded: Vec<String> = serde_json::from_slice(&std::fs::read(path)?)?;
    if encoded.is_empty() && !allow_empty {
        return Err(mycelix_desci_core::Error::Crypto(format!(
            "{label} cannot be empty"
        )));
    }
    let mut keys = BTreeSet::new();
    for value in encoded {
        let key = decode_public_key(&value)?;
        if !keys.insert(key) {
            return Err(mycelix_desci_core::Error::Crypto(format!(
                "{label} contains a duplicate public key"
            )));
        }
    }
    Ok(keys)
}

pub(super) fn decode_public_key(value: &str) -> Result<[u8; 32]> {
    let decoded = hex::decode(value.trim()).map_err(|error| {
        mycelix_desci_core::Error::Crypto(format!(
            "invalid hexadecimal scientific actor public key: {error}"
        ))
    })?;
    decoded.try_into().map_err(|_| {
        mycelix_desci_core::Error::Crypto(
            "scientific actor public keys must decode to exactly 32 bytes".to_string(),
        )
    })
}

fn parse_role(value: &str) -> Result<ScientificRole> {
    match value.trim().to_ascii_lowercase().as_str() {
        "contributor" => Ok(ScientificRole::Contributor),
        "reviewer" => Ok(ScientificRole::Reviewer),
        "editor" => Ok(ScientificRole::Editor),
        "institution" => Ok(ScientificRole::Institution),
        "service" => Ok(ScientificRole::Service),
        "migration_service" | "migration-service" => Ok(ScientificRole::MigrationService),
        "registry_admin" | "registry-admin" => Ok(ScientificRole::RegistryAdmin),
        other => Err(mycelix_desci_core::Error::Validation(format!(
            "unknown scientific role '{other}'"
        ))),
    }
}
