// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Offline threshold-governance operations for the scientific credential registry.

use super::credential_registry::{read_public_key_trust_file, read_signing_key};
use chrono::{Duration, Utc};
use ed25519_dalek::SigningKey;
use mycelix_desci_core::{
    ActorId, AuthorizedCredentialAcceptanceKey, CredentialGovernanceAction,
    CredentialGovernanceEnvelope, CredentialGovernancePayload, CredentialGovernancePolicy,
    CredentialGovernanceProposalId, CredentialTransparencyCheckpoint, OrganizationId, Result,
    ScientificCredentialGovernance, ScientificCredentialRegistry, SignedCredentialGovernanceEvent,
    SignedCredentialTransparencyWitness,
};
use serde::de::DeserializeOwned;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::Arc;
use uuid::Uuid;

const MAX_OFFLINE_INPUT_BYTES: u64 = 16 * 1024 * 1024;

struct OpenGovernance {
    governance: ScientificCredentialGovernance,
    administrator: ActorId,
    administrator_key: SigningKey,
}

async fn open_governance_journal(
    registry_path: &Path,
    bootstrap_trust_file: &Path,
    acceptance_signing_key_file: &Path,
    acceptance_trust_file: Option<&Path>,
    governance_path: &Path,
) -> Result<(ScientificCredentialGovernance, Arc<SigningKey>)> {
    let bootstrap_trust =
        read_public_key_trust_file(bootstrap_trust_file, "credential bootstrap trust", false)?;
    let acceptance_key = Arc::new(read_signing_key(
        acceptance_signing_key_file,
        "credential authority acceptance signing key",
    )?);
    let mut acceptance_trust = acceptance_trust_file
        .map(|path| read_public_key_trust_file(path, "credential authority acceptance trust", true))
        .transpose()?
        .unwrap_or_default();
    acceptance_trust.insert(acceptance_key.verifying_key().to_bytes());
    let registry = Arc::new(
        ScientificCredentialRegistry::open_file(
            registry_path,
            bootstrap_trust,
            acceptance_trust.clone(),
            Some(acceptance_key.clone()),
        )
        .await?,
    );
    let governance = ScientificCredentialGovernance::open_file(
        registry,
        governance_path,
        acceptance_trust,
        Some(acceptance_key.clone()),
    )
    .await?;
    Ok((governance, acceptance_key))
}

#[allow(clippy::too_many_arguments)]
async fn open_governance(
    registry_path: &Path,
    bootstrap_trust_file: &Path,
    acceptance_signing_key_file: &Path,
    acceptance_trust_file: Option<&Path>,
    governance_path: &Path,
    administrator_value: &str,
    administrator_signing_key_file: &Path,
) -> Result<OpenGovernance> {
    let (governance, acceptance_key) = open_governance_journal(
        registry_path,
        bootstrap_trust_file,
        acceptance_signing_key_file,
        acceptance_trust_file,
        governance_path,
    )
    .await?;
    let administrator = ActorId::new(administrator_value.to_string())?;
    let administrator_key = read_signing_key(
        administrator_signing_key_file,
        "credential governance administrator signing key",
    )?;
    if administrator_key.verifying_key().to_bytes() == acceptance_key.verifying_key().to_bytes() {
        return Err(mycelix_desci_core::Error::Crypto(
            "governance administrator and authority acceptance service must use distinct keys"
                .to_string(),
        ));
    }
    Ok(OpenGovernance {
        governance,
        administrator,
        administrator_key,
    })
}

#[allow(clippy::too_many_arguments)]
pub async fn initialize(
    registry_path: PathBuf,
    bootstrap_trust_file: PathBuf,
    acceptance_signing_key_file: PathBuf,
    acceptance_trust_file: Option<PathBuf>,
    governance_path: PathBuf,
    administrator_value: String,
    administrator_signing_key_file: PathBuf,
    approval_threshold: u16,
    activation_delay_seconds: u64,
    proposal_ttl_seconds: u64,
) -> Result<()> {
    if governance_path.exists() {
        return Err(mycelix_desci_core::Error::Validation(format!(
            "credential governance journal already exists and will not be overwritten: {}",
            governance_path.display()
        )));
    }
    let opened = open_governance(
        &registry_path,
        &bootstrap_trust_file,
        &acceptance_signing_key_file,
        acceptance_trust_file.as_deref(),
        &governance_path,
        &administrator_value,
        &administrator_signing_key_file,
    )
    .await?;
    let policy = CredentialGovernancePolicy {
        approval_threshold,
        activation_delay_seconds,
        proposal_ttl_seconds,
        proposer_counts_as_approval: true,
        emergency_cancellation_enabled: true,
    };
    policy.validate()?;
    let now = Utc::now();
    let acceptance_key = read_signing_key(
        &acceptance_signing_key_file,
        "credential authority acceptance signing key",
    )?;
    let envelope = CredentialGovernanceEnvelope::new(
        0,
        None,
        opened.administrator,
        now.clone(),
        format!("governance-policy-init:{}", Uuid::new_v4()),
        CredentialGovernancePayload::PolicyInitialized {
            policy,
            initial_acceptance_service_keys: vec![AuthorizedCredentialAcceptanceKey {
                public_key: acceptance_key.verifying_key().to_bytes(),
                valid_from: now - Duration::seconds(1),
                valid_until: None,
                revoked_at: None,
            }],
        },
    )?;
    let event = SignedCredentialGovernanceEvent::sign(envelope, &opened.administrator_key)?;
    let receipt = opened.governance.append(0, event).await?;
    println!(
        "Credential governance journal: {}",
        governance_path.display()
    );
    println!("Governance revision: {}", receipt.governance_revision);
    println!("Approval threshold: {approval_threshold}");
    println!("Activation delay: {activation_delay_seconds} seconds");
    Ok(())
}

#[allow(clippy::too_many_arguments)]
pub async fn propose(
    registry_path: PathBuf,
    bootstrap_trust_file: PathBuf,
    acceptance_signing_key_file: PathBuf,
    acceptance_trust_file: Option<PathBuf>,
    governance_path: PathBuf,
    administrator_value: String,
    administrator_signing_key_file: PathBuf,
    action_file: PathBuf,
    reason: String,
) -> Result<()> {
    let opened = open_governance(
        &registry_path,
        &bootstrap_trust_file,
        &acceptance_signing_key_file,
        acceptance_trust_file.as_deref(),
        &governance_path,
        &administrator_value,
        &administrator_signing_key_file,
    )
    .await?;
    let action: CredentialGovernanceAction = read_json_regular(&action_file, "governance action")?;
    action.validate()?;
    let projection = opened.governance.projection().await;
    let policy = projection.policy.clone().ok_or_else(|| {
        mycelix_desci_core::Error::Validation(
            "credential governance must be initialized before proposals are opened".to_string(),
        )
    })?;
    let registry_head = opened
        .governance
        .credential_registry_head()
        .await?
        .ok_or_else(|| {
            mycelix_desci_core::Error::Validation(
                "credential registry must be initialized before governance proposals".to_string(),
            )
        })?;
    let proposal_id = CredentialGovernanceProposalId::new();
    let now = Utc::now();
    let payload = CredentialGovernancePayload::ProposalOpened {
        proposal_id,
        base_registry_head: registry_head,
        action,
        activation_not_before: now.clone()
            + Duration::seconds(policy.activation_delay_seconds as i64),
        expires_at: now.clone() + Duration::seconds(policy.proposal_ttl_seconds as i64),
        reason,
    };
    let receipt =
        append_signed(&opened, payload, format!("proposal-open:{}", proposal_id.0)).await?;
    let approval = opened
        .governance
        .approval_status(proposal_id, Utc::now())
        .await?;
    println!("Proposal ID: {}", proposal_id.0);
    println!("Risk tier: {:?}", approval.risk_tier);
    println!(
        "Required approvals: {} across {} distinct organizations",
        approval.required_approvals, approval.required_distinct_organizations
    );
    println!("Governance revision: {}", receipt.governance_revision);
    Ok(())
}

#[allow(clippy::too_many_arguments)]
pub async fn approve(
    registry_path: PathBuf,
    bootstrap_trust_file: PathBuf,
    acceptance_signing_key_file: PathBuf,
    acceptance_trust_file: Option<PathBuf>,
    governance_path: PathBuf,
    administrator_value: String,
    administrator_signing_key_file: PathBuf,
    proposal_id: Uuid,
) -> Result<()> {
    let opened = open_governance(
        &registry_path,
        &bootstrap_trust_file,
        &acceptance_signing_key_file,
        acceptance_trust_file.as_deref(),
        &governance_path,
        &administrator_value,
        &administrator_signing_key_file,
    )
    .await?;
    let receipt = append_signed(
        &opened,
        CredentialGovernancePayload::ProposalApproved {
            proposal_id: CredentialGovernanceProposalId(proposal_id),
        },
        format!(
            "proposal-approve:{proposal_id}:{}",
            opened.administrator.as_str()
        ),
    )
    .await?;
    println!("Approved proposal: {proposal_id}");
    println!("Governance revision: {}", receipt.governance_revision);
    Ok(())
}

#[allow(clippy::too_many_arguments)]
pub async fn cancel(
    registry_path: PathBuf,
    bootstrap_trust_file: PathBuf,
    acceptance_signing_key_file: PathBuf,
    acceptance_trust_file: Option<PathBuf>,
    governance_path: PathBuf,
    administrator_value: String,
    administrator_signing_key_file: PathBuf,
    proposal_id: Uuid,
    reason: String,
) -> Result<()> {
    let opened = open_governance(
        &registry_path,
        &bootstrap_trust_file,
        &acceptance_signing_key_file,
        acceptance_trust_file.as_deref(),
        &governance_path,
        &administrator_value,
        &administrator_signing_key_file,
    )
    .await?;
    let receipt = append_signed(
        &opened,
        CredentialGovernancePayload::ProposalCancelled {
            proposal_id: CredentialGovernanceProposalId(proposal_id),
            reason,
        },
        format!("proposal-cancel:{proposal_id}:{}", Uuid::new_v4()),
    )
    .await?;
    println!("Cancelled proposal: {proposal_id}");
    println!("Governance revision: {}", receipt.governance_revision);
    Ok(())
}

#[allow(clippy::too_many_arguments)]
pub async fn execute(
    registry_path: PathBuf,
    bootstrap_trust_file: PathBuf,
    acceptance_signing_key_file: PathBuf,
    acceptance_trust_file: Option<PathBuf>,
    governance_path: PathBuf,
    administrator_value: String,
    administrator_signing_key_file: PathBuf,
    proposal_id: Uuid,
) -> Result<()> {
    let opened = open_governance(
        &registry_path,
        &bootstrap_trust_file,
        &acceptance_signing_key_file,
        acceptance_trust_file.as_deref(),
        &governance_path,
        &administrator_value,
        &administrator_signing_key_file,
    )
    .await?;
    let (sequence, previous_hash) = governance_head(&opened.governance).await?;
    let envelope = CredentialGovernanceEnvelope::new(
        sequence,
        previous_hash,
        opened.administrator,
        Utc::now(),
        format!("proposal-execute:{proposal_id}:{}", Uuid::new_v4()),
        CredentialGovernancePayload::ProposalExecuted {
            proposal_id: CredentialGovernanceProposalId(proposal_id),
        },
    )?;
    let event = SignedCredentialGovernanceEvent::sign(envelope, &opened.administrator_key)?;
    let receipt = opened.governance.execute(sequence, event).await?;
    println!("Executed proposal: {proposal_id}");
    println!("Governance revision: {}", receipt.governance_revision);
    if let Some(credential) = receipt.credential_receipt {
        println!(
            "Credential registry revision: {}",
            credential.registry_revision
        );
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
pub async fn checkpoint(
    registry_path: PathBuf,
    bootstrap_trust_file: PathBuf,
    acceptance_signing_key_file: PathBuf,
    acceptance_trust_file: Option<PathBuf>,
    governance_path: PathBuf,
    administrator_value: String,
    administrator_signing_key_file: PathBuf,
    output: Option<PathBuf>,
) -> Result<()> {
    let opened = open_governance(
        &registry_path,
        &bootstrap_trust_file,
        &acceptance_signing_key_file,
        acceptance_trust_file.as_deref(),
        &governance_path,
        &administrator_value,
        &administrator_signing_key_file,
    )
    .await?;
    let checkpoint = opened.governance.checkpoint_candidate().await?;
    let (sequence, previous_hash) = governance_head(&opened.governance).await?;
    let envelope = CredentialGovernanceEnvelope::new(
        sequence,
        previous_hash,
        opened.administrator.clone(),
        checkpoint.issued_at.clone(),
        format!("transparency-checkpoint:{}", Uuid::new_v4()),
        CredentialGovernancePayload::TransparencyCheckpointPublished {
            checkpoint: checkpoint.clone(),
        },
    )?;
    let event = SignedCredentialGovernanceEvent::sign(envelope, &opened.administrator_key)?;
    let receipt = opened.governance.append(sequence, event).await?;
    if let Some(path) = output {
        write_json_new(&path, &checkpoint)?;
        println!("Checkpoint export: {}", path.display());
    } else {
        println!("{}", serde_json::to_string_pretty(&checkpoint)?);
    }
    println!("Governance revision: {}", receipt.governance_revision);
    Ok(())
}

#[allow(clippy::too_many_arguments)]
pub async fn witness_checkpoint(
    registry_path: PathBuf,
    bootstrap_trust_file: PathBuf,
    acceptance_signing_key_file: PathBuf,
    acceptance_trust_file: Option<PathBuf>,
    governance_path: PathBuf,
    witness_actor_value: String,
    witness_organization_value: String,
    witness_signing_key_file: PathBuf,
    checkpoint_file: PathBuf,
) -> Result<()> {
    let (governance, acceptance_key) = open_governance_journal(
        &registry_path,
        &bootstrap_trust_file,
        &acceptance_signing_key_file,
        acceptance_trust_file.as_deref(),
        &governance_path,
    )
    .await?;
    let witness_key = read_signing_key(
        &witness_signing_key_file,
        "transparency witness signing key",
    )?;
    if witness_key.verifying_key().to_bytes() == acceptance_key.verifying_key().to_bytes() {
        return Err(mycelix_desci_core::Error::Crypto(
            "transparency witness and acceptance service must use distinct keys".to_string(),
        ));
    }
    let checkpoint: CredentialTransparencyCheckpoint =
        read_json_regular(&checkpoint_file, "transparency checkpoint")?;
    let checkpoint_hash = checkpoint.checkpoint_hash();
    let projection = governance.projection().await;
    if !projection
        .checkpoints
        .iter()
        .any(|published| published.checkpoint_hash() == checkpoint_hash)
    {
        return Err(mycelix_desci_core::Error::NotFound(
            "checkpoint file is not present in the governance journal".to_string(),
        ));
    }
    let witness_actor = ActorId::new(witness_actor_value)?;
    let witness_organization = OrganizationId::new(witness_organization_value)?;
    let now = Utc::now();
    let witness = SignedCredentialTransparencyWitness::sign(
        checkpoint_hash,
        witness_actor.clone(),
        witness_organization,
        now.clone(),
        &witness_key,
    )?;
    let (sequence, previous_hash) = governance_head(&governance).await?;
    let envelope = CredentialGovernanceEnvelope::new(
        sequence,
        previous_hash,
        witness_actor,
        now,
        format!(
            "transparency-witness:{}:{}",
            checkpoint_hash,
            Uuid::new_v4()
        ),
        CredentialGovernancePayload::TransparencyCheckpointWitnessed { witness },
    )?;
    let event = SignedCredentialGovernanceEvent::sign(envelope, &witness_key)?;
    let receipt = governance.append(sequence, event).await?;
    println!("Witnessed checkpoint: {checkpoint_hash}");
    println!("Governance revision: {}", receipt.governance_revision);
    Ok(())
}

async fn append_signed(
    opened: &OpenGovernance,
    payload: CredentialGovernancePayload,
    idempotency_key: String,
) -> Result<mycelix_desci_core::CredentialGovernanceAppendReceipt> {
    let (sequence, previous_hash) = governance_head(&opened.governance).await?;
    let envelope = CredentialGovernanceEnvelope::new(
        sequence,
        previous_hash,
        opened.administrator.clone(),
        Utc::now(),
        idempotency_key,
        payload,
    )?;
    let event = SignedCredentialGovernanceEvent::sign(envelope, &opened.administrator_key)?;
    opened.governance.append(sequence, event).await
}

async fn governance_head(
    governance: &ScientificCredentialGovernance,
) -> Result<(u64, Option<mycelix_desci_core::ContentHash>)> {
    let projection = governance.projection().await;
    Ok((projection.event_count, projection.head_hash))
}

fn read_json_regular<T: DeserializeOwned>(path: &Path, label: &str) -> Result<T> {
    let metadata = fs::symlink_metadata(path)?;
    if metadata.file_type().is_symlink() || !metadata.is_file() {
        return Err(mycelix_desci_core::Error::Validation(format!(
            "{label} must be a regular non-symbolic-link file"
        )));
    }
    if metadata.len() > MAX_OFFLINE_INPUT_BYTES {
        return Err(mycelix_desci_core::Error::Validation(format!(
            "{label} exceeds {MAX_OFFLINE_INPUT_BYTES} bytes"
        )));
    }
    Ok(serde_json::from_slice(&fs::read(path)?)?)
}

fn write_json_new<T: serde::Serialize>(path: &Path, value: &T) -> Result<()> {
    if let Some(parent) = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        fs::create_dir_all(parent)?;
    }
    let bytes = serde_json::to_vec_pretty(value)?;
    let mut options = fs::OpenOptions::new();
    options.write(true).create_new(true);
    let mut file = options.open(path)?;
    use std::io::Write;
    file.write_all(&bytes)?;
    file.write_all(b"\n")?;
    file.sync_all()?;
    if let Some(parent) = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        fs::File::open(parent)?.sync_all()?;
    }
    Ok(())
}
