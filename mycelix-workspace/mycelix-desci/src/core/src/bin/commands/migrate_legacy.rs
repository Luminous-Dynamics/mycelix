// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root

use ed25519_dalek::SigningKey;
use mycelix_desci_core::{
    ActorId, DefaultScientificAuthorizationPolicy, DesciClaim, FileScientificAuthorityAuditStore,
    FileScientificEventLog, GovernedScientificEventLog, LegacyMigrationContext,
    LegacyMigrationReport, OrganizationId, Result, ScientificAuthorityAuditStore,
    ScientificCredentialRegistry, ScientificRole, migrate_legacy_claim,
};
use std::collections::BTreeSet;
use std::path::{Path, PathBuf};
use std::sync::Arc;

const MAX_LEGACY_SOURCE_BYTES: u64 = 16 * 1024 * 1024;

pub async fn execute(
    input: PathBuf,
    event_log: PathBuf,
    actor_value: String,
    signing_key_file: PathBuf,
    credential_registry: PathBuf,
    credential_bootstrap_trust_file: PathBuf,
    authority_audit: PathBuf,
    receipt_signing_key_file: PathBuf,
    receipt_trust_file: Option<PathBuf>,
    organization: Option<String>,
    source_system: String,
    report_path: Option<PathBuf>,
) -> Result<()> {
    let actor = ActorId::new(actor_value)?;
    let organization = organization.map(OrganizationId::new).transpose()?;
    let signing_key = read_signing_key(&signing_key_file, "migration actor signing key")?;
    let receipt_signing_key = Arc::new(read_signing_key(
        &receipt_signing_key_file,
        "authority receipt signing key",
    )?);
    if signing_key.verifying_key().to_bytes() == receipt_signing_key.verifying_key().to_bytes() {
        return Err(mycelix_desci_core::Error::Crypto(
            "migration actor and authority receipt service must use distinct signing keys"
                .to_string(),
        ));
    }
    let mut trusted_receipt_keys = receipt_trust_file
        .as_deref()
        .map(read_receipt_trust_file)
        .transpose()?
        .unwrap_or_default();
    trusted_receipt_keys.insert(receipt_signing_key.verifying_key().to_bytes());
    let now = chrono::Utc::now();
    let bootstrap_trust_keys = read_public_key_trust_file(
        &credential_bootstrap_trust_file,
        "credential bootstrap trust",
        false,
    )?;
    let credential_registry = ScientificCredentialRegistry::open_file(
        &credential_registry,
        bootstrap_trust_keys,
        trusted_receipt_keys.clone(),
        Some(receipt_signing_key.clone()),
    )
    .await?;
    let profile = credential_registry
        .projection()
        .await
        .resolve(&actor)
        .ok_or_else(|| {
            mycelix_desci_core::Error::VerificationFailed(
                "migration actor is absent from the scientific credential registry".to_string(),
            )
        })?;
    if !profile.has_role(ScientificRole::MigrationService) {
        return Err(mycelix_desci_core::Error::VerificationFailed(
            "migration actor lacks the migration_service role".to_string(),
        ));
    }
    if !profile.authorizes_key(&signing_key.verifying_key().to_bytes(), &now) {
        return Err(mycelix_desci_core::Error::VerificationFailed(
            "migration signing key is not active in the scientific credential registry".to_string(),
        ));
    }
    if let Some(organization) = &organization {
        if !profile.organizations.contains(organization) {
            return Err(mycelix_desci_core::Error::VerificationFailed(
                "migration actor is not a member of the requested acting organization".to_string(),
            ));
        }
    }
    let raw_log = FileScientificEventLog::open(&event_log).await?;
    let audit = Arc::new(
        FileScientificAuthorityAuditStore::open(&authority_audit, trusted_receipt_keys).await?,
    );
    audit.reconcile(&raw_log).await?;
    let log = GovernedScientificEventLog::new(
        raw_log,
        credential_registry,
        DefaultScientificAuthorizationPolicy,
        audit.clone(),
        Some(receipt_signing_key),
    );

    let files = legacy_claim_files(&input)?;
    if files.is_empty() {
        return Err(mycelix_desci_core::Error::NotFound(format!(
            "no legacy JSON claim files found at {}",
            input.display()
        )));
    }

    let mut reports: Vec<LegacyMigrationReport> = Vec::with_capacity(files.len());
    for path in files {
        let source_bytes = read_legacy_source(&path)?;
        let claim: DesciClaim = serde_json::from_slice(&source_bytes)?;
        let source_locator = public_source_locator(&input, &path);
        let mut context = LegacyMigrationContext::new(actor.clone(), source_system.clone())
            .with_source_locator(source_locator);
        if let Some(organization) = &organization {
            context = context.with_organization(organization.clone());
        }
        let report =
            migrate_legacy_claim(&log, &claim, &source_bytes, &context, &signing_key).await?;
        println!(
            "{:?}: legacy claim {} -> canonical stream {} ({})",
            report.status,
            report.legacy_claim_id,
            report.canonical_claim_id,
            report.source_record_hash
        );
        reports.push(report);
    }

    let authority_summary = audit.reconcile(&log).await?;
    if authority_summary.pending_receipts != 0 || authority_summary.unattested_events != 0 {
        return Err(mycelix_desci_core::Error::Storage(format!(
            "migration authority audit is incomplete: {} pending, {} legacy-unattested, {} unsafe-unattested",
            authority_summary.pending_receipts,
            authority_summary.legacy_unattested_events,
            authority_summary.unsafe_unattested_events
        )));
    }

    let report_json = serde_json::to_string_pretty(&reports)?;
    if let Some(path) = report_path {
        if let Some(parent) = path.parent() {
            if !parent.as_os_str().is_empty() {
                std::fs::create_dir_all(parent)?;
            }
        }
        std::fs::write(&path, report_json.as_bytes())?;
        println!("Migration report: {}", path.display());
    } else {
        println!("{report_json}");
    }
    Ok(())
}

fn legacy_claim_files(input: &Path) -> Result<Vec<PathBuf>> {
    if input.is_file() {
        return Ok(vec![input.to_path_buf()]);
    }
    if !input.is_dir() {
        return Err(mycelix_desci_core::Error::NotFound(format!(
            "legacy input path does not exist: {}",
            input.display()
        )));
    }
    let mut files = std::fs::read_dir(input)?
        .filter_map(|entry| entry.ok().map(|entry| entry.path()))
        .filter(|path| path.extension().and_then(|value| value.to_str()) == Some("json"))
        .collect::<Vec<_>>();
    files.sort();
    Ok(files)
}

fn read_legacy_source(path: &Path) -> Result<Vec<u8>> {
    let metadata = std::fs::symlink_metadata(path)?;
    if metadata.file_type().is_symlink() {
        return Err(mycelix_desci_core::Error::Validation(format!(
            "legacy source must not be a symbolic link: {}",
            path.display()
        )));
    }
    if !metadata.is_file() {
        return Err(mycelix_desci_core::Error::Validation(format!(
            "legacy source must be a regular file: {}",
            path.display()
        )));
    }
    if metadata.len() > MAX_LEGACY_SOURCE_BYTES {
        return Err(mycelix_desci_core::Error::Validation(format!(
            "legacy source exceeds {} bytes: {}",
            MAX_LEGACY_SOURCE_BYTES,
            path.display()
        )));
    }
    Ok(std::fs::read(path)?)
}

fn public_source_locator(input: &Path, path: &Path) -> String {
    if input.is_dir() {
        path.strip_prefix(input)
            .ok()
            .filter(|relative| !relative.as_os_str().is_empty())
            .unwrap_or(path)
            .to_string_lossy()
            .replace('\\', "/")
    } else {
        path.file_name()
            .unwrap_or(path.as_os_str())
            .to_string_lossy()
            .to_string()
    }
}

fn read_signing_key(path: &Path, label: &str) -> Result<SigningKey> {
    let metadata = std::fs::symlink_metadata(path)?;
    if metadata.file_type().is_symlink() {
        return Err(mycelix_desci_core::Error::Crypto(format!(
            "{label} file must not be a symbolic link"
        )));
    }
    if !metadata.is_file() {
        return Err(mycelix_desci_core::Error::Crypto(format!(
            "{label} path must be a regular file"
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
                "{label} permissions are too broad; require mode 0600 or stricter"
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

fn read_receipt_trust_file(path: &Path) -> Result<BTreeSet<[u8; 32]>> {
    read_public_key_trust_file(path, "authority receipt", true)
}

fn read_public_key_trust_file(
    path: &Path,
    label: &str,
    allow_empty: bool,
) -> Result<BTreeSet<[u8; 32]>> {
    let metadata = std::fs::symlink_metadata(path)?;
    if metadata.file_type().is_symlink() || !metadata.is_file() {
        return Err(mycelix_desci_core::Error::Crypto(format!(
            "{label} file must be a regular non-symbolic-link file"
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
            "{label} file cannot be empty"
        )));
    }
    let mut keys = BTreeSet::new();
    for value in encoded {
        let decoded = hex::decode(value.trim()).map_err(|error| {
            mycelix_desci_core::Error::Crypto(format!("invalid {label} public key: {error}"))
        })?;
        let key: [u8; 32] = decoded.try_into().map_err(|_| {
            mycelix_desci_core::Error::Crypto(format!(
                "{label} public keys must decode to 32 bytes"
            ))
        })?;
        if !keys.insert(key) {
            return Err(mycelix_desci_core::Error::Crypto(format!(
                "duplicate {label} public key"
            )));
        }
    }
    Ok(keys)
}
