// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Explicit migration of mutable legacy claims into canonical event streams.
//!
//! Migration is deliberately conservative: it preserves the complete source
//! record by content hash and locator, but it does not translate legacy tiers,
//! verification counts, aggregate trust values, or reproducibility scores into
//! canonical evidence. The imported projection therefore begins as unassessed
//! history and can only mature through new signed scientific events.

use crate::claims::DesciClaim;
use crate::scientific_events::{
    ActorId, AppendReceipt, AtomicClaim, ClaimId, ClaimOrigin, ClaimProjection, ContentHash,
    LegacyImportMetadata, OrganizationId, ResearchObject, ResearchObjectId, ResearchObjectType,
    ScientificEventEnvelope, ScientificEventLog, ScientificEventPayload, SignedScientificEvent,
};
use crate::{Error, Result};
use ed25519_dalek::SigningKey;
use serde::{Deserialize, Serialize};
use uuid::Uuid;

const LEGACY_OBJECT_NAMESPACE: &[u8] = b"MYCELIX-DESCI-LEGACY-RESEARCH-OBJECT\0";

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum LegacyMigrationStatus {
    Imported,
    AlreadyImported,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct LegacyMigrationReport {
    pub legacy_claim_id: Uuid,
    pub canonical_claim_id: ClaimId,
    pub source_record_hash: ContentHash,
    pub status: LegacyMigrationStatus,
    pub receipt: Option<AppendReceipt>,
    pub warnings: Vec<String>,
}

#[derive(Debug, Clone)]
pub struct LegacyMigrationContext {
    pub actor: ActorId,
    pub acting_organization: Option<OrganizationId>,
    pub source_system: String,
    pub source_locator: Option<String>,
}

impl LegacyMigrationContext {
    pub fn new(actor: ActorId, source_system: impl Into<String>) -> Self {
        Self {
            actor,
            acting_organization: None,
            source_system: source_system.into(),
            source_locator: None,
        }
    }

    pub fn with_organization(mut self, organization: OrganizationId) -> Self {
        self.acting_organization = Some(organization);
        self
    }

    pub fn with_source_locator(mut self, locator: impl Into<String>) -> Self {
        self.source_locator = Some(locator.into());
        self
    }
}

/// Import one legacy record. Repeating the same import is idempotent when the
/// existing stream has the same source-record hash; a conflicting source record
/// for the same claim identifier fails closed.
pub async fn migrate_legacy_claim<L: ScientificEventLog + ?Sized>(
    log: &L,
    claim: &DesciClaim,
    source_bytes: &[u8],
    context: &LegacyMigrationContext,
    signing_key: &SigningKey,
) -> Result<LegacyMigrationReport> {
    let canonical_claim_id = ClaimId(claim.id);
    let source_record_hash = ContentHash::digest(source_bytes);

    if log.head(canonical_claim_id).await?.is_some() {
        let projection = ClaimProjection::rebuild(&log.stream(canonical_claim_id).await?)?;
        return match projection.origin {
            ClaimOrigin::LegacyImport { metadata }
                if metadata.source_record_hash == source_record_hash =>
            {
                Ok(LegacyMigrationReport {
                    legacy_claim_id: claim.id,
                    canonical_claim_id,
                    source_record_hash,
                    status: LegacyMigrationStatus::AlreadyImported,
                    receipt: None,
                    warnings: migration_warnings(claim),
                })
            }
            _ => Err(Error::Storage(format!(
                "claim stream {} already exists with a different canonical origin or source hash",
                canonical_claim_id
            ))),
        };
    }

    let research_object_id = deterministic_research_object_id(claim.id);
    let research_object = ResearchObject {
        id: research_object_id,
        title: legacy_title(claim),
        object_type: ResearchObjectType::Other("legacy_claim_record".to_string()),
        persistent_identifier: claim.content.storage_ref.clone(),
    };
    let atomic_claim = AtomicClaim {
        id: canonical_claim_id,
        research_object_id,
        statement: legacy_statement(claim),
        scope: legacy_scope(claim),
    };
    let import = LegacyImportMetadata {
        source_system: context.source_system.clone(),
        source_record_hash,
        source_locator: context.source_locator.clone(),
        legacy_creator: (!claim.creator.trim().is_empty()).then(|| claim.creator.clone()),
        legacy_tier: format!("{:?}", claim.epistemic_tier),
        legacy_verification_count: claim.verifications.len(),
        legacy_provenance_count: claim.provenance.len(),
        omitted_fields: vec![
            "epistemic_fingerprint".to_string(),
            "epistemic_position".to_string(),
            "legacy_verification_material".to_string(),
            "legacy_provenance_bodies".to_string(),
            "reproducibility_score".to_string(),
            "mutable_updated_at".to_string(),
        ],
    };
    let payload = ScientificEventPayload::LegacyClaimImported {
        research_object,
        claim: atomic_claim,
        import,
    };
    let mut envelope =
        ScientificEventEnvelope::genesis(context.actor.clone(), claim.created_at.clone(), payload)?
            .with_idempotency_key(format!("legacy-import:{}", claim.id))?;
    if let Some(organization) = &context.acting_organization {
        envelope = envelope.with_acting_organization(organization.clone());
    }
    let event = SignedScientificEvent::sign(envelope, signing_key)?;
    let receipt = log.append(0, event).await?;

    Ok(LegacyMigrationReport {
        legacy_claim_id: claim.id,
        canonical_claim_id,
        source_record_hash,
        status: LegacyMigrationStatus::Imported,
        receipt: Some(receipt),
        warnings: migration_warnings(claim),
    })
}

/// Serialize a legacy claim exactly as the migration source when no original
/// file bytes are available. Operators should prefer the original stored bytes
/// so the source hash remains independently reproducible.
pub fn canonical_legacy_source_bytes(claim: &DesciClaim) -> Result<Vec<u8>> {
    serde_json::to_vec(claim).map_err(Error::from)
}

fn deterministic_research_object_id(claim_id: Uuid) -> ResearchObjectId {
    let mut hasher = blake3::Hasher::new();
    hasher.update(LEGACY_OBJECT_NAMESPACE);
    hasher.update(claim_id.as_bytes());
    let digest = hasher.finalize();
    let mut bytes = [0_u8; 16];
    bytes.copy_from_slice(&digest.as_bytes()[..16]);
    // RFC 4122-compatible deterministic identifier (version-5-shaped, but
    // BLAKE3-derived and domain separated rather than SHA-1 UUIDv5).
    bytes[6] = (bytes[6] & 0x0f) | 0x50;
    bytes[8] = (bytes[8] & 0x3f) | 0x80;
    ResearchObjectId(Uuid::from_bytes(bytes))
}

fn legacy_statement(claim: &DesciClaim) -> String {
    let description = claim.content.description.trim();
    if description.is_empty() {
        format!("Legacy claim {} (description unavailable)", claim.id)
    } else {
        description.to_string()
    }
}

fn legacy_title(claim: &DesciClaim) -> String {
    let description = claim.content.description.trim();
    if description.is_empty() {
        return format!("Legacy claim {}", claim.id);
    }
    let mut title: String = description.chars().take(120).collect();
    if description.chars().count() > 120 {
        title.push('…');
    }
    title
}

fn legacy_scope(claim: &DesciClaim) -> Option<String> {
    let category = claim.content.category.trim();
    let keywords = claim
        .content
        .keywords
        .iter()
        .map(|keyword| keyword.trim())
        .filter(|keyword| !keyword.is_empty())
        .collect::<Vec<_>>();
    match (category.is_empty(), keywords.is_empty()) {
        (true, true) => None,
        (false, true) => Some(format!("legacy category: {category}")),
        (true, false) => Some(format!("legacy keywords: {}", keywords.join(", "))),
        (false, false) => Some(format!(
            "legacy category: {category}; legacy keywords: {}",
            keywords.join(", ")
        )),
    }
}

fn migration_warnings(claim: &DesciClaim) -> Vec<String> {
    let mut warnings = vec![
        "legacy epistemic tier was preserved as metadata only".to_string(),
        "legacy verification material was not converted into attestations".to_string(),
        "legacy aggregate fingerprint and trust scores were not imported as evidence".to_string(),
    ];
    if claim.content.description.trim().is_empty() {
        warnings.push(
            "legacy description was empty; migration inserted a neutral placeholder statement"
                .to_string(),
        );
    }
    if claim.content.reproducibility_score.is_some() {
        warnings.push(
            "legacy reproducibility score was omitted; submit a protocol-bound reproduction attestation"
                .to_string(),
        );
    }
    if !claim.provenance.is_empty() {
        warnings.push(
            "legacy provenance bodies remain available only through the content-addressed source record"
                .to_string(),
        );
    }
    warnings
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::claims::{ClaimContent, EpistemicTier, Verification};
    use crate::scientific_authority_audit::MemoryScientificAuthorityAuditStore;
    use crate::scientific_events::MemoryScientificEventLog;
    use crate::scientific_governance::{
        AuthorizedActorKey, DefaultScientificAuthorizationPolicy, GovernedScientificEventLog,
        MemoryScientificIdentityResolver, ResolvedScientificActor, ScientificRole,
    };
    use chrono::{Duration, Utc};
    use std::collections::BTreeSet;
    use std::sync::Arc;

    #[tokio::test]
    async fn migration_is_idempotent_and_does_not_launder_tier_or_verifications() {
        let key = SigningKey::from_bytes(&[42; 32]);
        let actor = ActorId::new("did:key:migration-service").unwrap();
        let now = Utc::now();
        let resolver = MemoryScientificIdentityResolver::new();
        resolver
            .register(ResolvedScientificActor {
                actor: actor.clone(),
                authorized_keys: vec![AuthorizedActorKey {
                    public_key: key.verifying_key().to_bytes(),
                    valid_from: now - Duration::days(1),
                    valid_until: None,
                    revoked_at: None,
                }],
                organizations: BTreeSet::new(),
                roles: BTreeSet::from([ScientificRole::MigrationService]),
                authority_revision: None,
            })
            .await
            .unwrap();
        let receipt_key = Arc::new(SigningKey::from_bytes(&[43; 32]));
        let audit = Arc::new(MemoryScientificAuthorityAuditStore::new(BTreeSet::from([
            receipt_key.verifying_key().to_bytes(),
        ])));
        let log = GovernedScientificEventLog::new(
            MemoryScientificEventLog::new(),
            resolver,
            DefaultScientificAuthorizationPolicy,
            audit,
            Some(receipt_key),
        );
        let mut claim = DesciClaim::new(
            EpistemicTier::E4,
            ClaimContent {
                dataset_hash: "legacy-hash".to_string(),
                description: "Historical result".to_string(),
                category: "biology".to_string(),
                keywords: vec!["replication".to_string()],
                storage_ref: None,
                reproducibility_score: Some(1.0),
                license: None,
            },
            "legacy-author".to_string(),
        );
        claim.verifications.push(Verification {
            verifier: "legacy-reviewer".to_string(),
            timestamp: now.clone(),
            signature: vec![1, 2, 3],
            notes: None,
        });
        let bytes = canonical_legacy_source_bytes(&claim).unwrap();
        let context = LegacyMigrationContext::new(actor, "mycelix-desci-legacy-json");

        let first = migrate_legacy_claim(&log, &claim, &bytes, &context, &key)
            .await
            .unwrap();
        assert_eq!(first.status, LegacyMigrationStatus::Imported);
        let second = migrate_legacy_claim(&log, &claim, &bytes, &context, &key)
            .await
            .unwrap();
        assert_eq!(second.status, LegacyMigrationStatus::AlreadyImported);

        let projection =
            ClaimProjection::rebuild(&log.stream(ClaimId(claim.id)).await.unwrap()).unwrap();
        assert!(projection.origin.is_legacy_unassessed());
        assert_eq!(projection.evidence_profile.review_count, 0);
        assert_eq!(
            projection.maturity(),
            crate::scientific_events::EvidenceMaturity::Proposed
        );
        assert!(
            projection
                .assessment()
                .reasons
                .iter()
                .any(|reason| reason.contains("unassessed history"))
        );
    }
}
