#![deny(unsafe_code)]

use blake3::Hasher;
use constitutional_authority::{AuthorityPrincipal, Branch, ConstitutionalCapability, ConstitutionalPower};
use constitutional_payments_provider_journal::{ProviderJournalEntry, ProviderJournalRecord};
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;
use thiserror::Error;

pub const PROVIDER_AUTHOR_GRANT_SCHEMA_VERSION: u16 = 1;
pub const GRANT_COMMITMENT_PREFIX: &str = "payments-provider-author-grant-v1:";
const GRANT_DOMAIN: &[u8] = b"MYCELIX-PAYMENTS-PROVIDER-AUTHOR-GRANT\0V1\0";
const MAX_ID_LEN: usize = 512;

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum ProviderAuthorityError {
    #[error("{0}")]
    Violation(String),
}

pub type ProviderAuthorityResult<T> = Result<T, ProviderAuthorityError>;

fn violation(message: impl Into<String>) -> ProviderAuthorityError {
    ProviderAuthorityError::Violation(message.into())
}

fn require_opaque(label: &str, value: &str) -> ProviderAuthorityResult<()> {
    if value.trim().is_empty() || value.len() > MAX_ID_LEN {
        return Err(violation(format!("{label} must be non-empty and <= {MAX_ID_LEN} bytes")));
    }
    Ok(())
}

fn push_str(hasher: &mut Hasher, value: &str) {
    hasher.update(&(value.len() as u64).to_be_bytes());
    hasher.update(value.as_bytes());
}

#[derive(Serialize, Deserialize, Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum ProviderAuthorRole {
    OrchestratorIntent,
    OrchestratorDispatch,
    OperationIndex,
    ProviderObservation,
}

pub fn required_role_for_record(record: &ProviderJournalRecord) -> ProviderAuthorRole {
    match &record.entry {
        ProviderJournalEntry::OperationIntent { .. } => ProviderAuthorRole::OrchestratorIntent,
        ProviderJournalEntry::DispatchAttempt { .. } => ProviderAuthorRole::OrchestratorDispatch,
        ProviderJournalEntry::Observation { .. } => ProviderAuthorRole::ProviderObservation,
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct PublicFundsAuthorityAnchor {
    pub capability_id: String,
    pub holder_id: String,
    pub jurisdiction: String,
    pub capability_commitment: String,
    pub valid_from_us: i64,
    pub expires_at_us: Option<i64>,
}

impl PublicFundsAuthorityAnchor {
    pub fn from_root_capability(
        capability: &ConstitutionalCapability,
        capability_commitment: impl Into<String>,
    ) -> ProviderAuthorityResult<Self> {
        capability
            .validate()
            .map_err(|e| violation(format!("invalid constitutional capability: {e:?}")))?;
        if capability.holder != AuthorityPrincipal::Branch(Branch::Stewardship) {
            return Err(violation("public-funds execution anchor must be held by Stewardship"));
        }
        if capability.power != ConstitutionalPower::ExecuteAppropriation {
            return Err(violation("public-funds execution anchor must carry ExecuteAppropriation"));
        }
        let capability_commitment = capability_commitment.into();
        require_opaque("capability commitment", &capability_commitment)?;
        Ok(Self {
            capability_id: capability.id.clone(),
            holder_id: capability.holder_id.clone(),
            jurisdiction: capability.jurisdiction.clone(),
            capability_commitment,
            valid_from_us: capability.valid_from_us,
            expires_at_us: capability.expires_at_us,
        })
    }

    pub fn valid_at(&self, action_time_us: i64) -> bool {
        if action_time_us < self.valid_from_us {
            return false;
        }
        self.expires_at_us
            .map(|expiry| action_time_us < expiry)
            .unwrap_or(true)
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ThresholdCommitteeDescriptor {
    pub committee_id: String,
    pub epoch: u64,
    pub threshold: u32,
    pub member_count: u32,
    pub committee_commitment: String,
    pub public_key_commitment: String,
    pub scope_commitment: String,
}

impl ThresholdCommitteeDescriptor {
    pub fn validate(&self) -> ProviderAuthorityResult<()> {
        require_opaque("committee id", &self.committee_id)?;
        require_opaque("committee commitment", &self.committee_commitment)?;
        require_opaque("public key commitment", &self.public_key_commitment)?;
        require_opaque("scope commitment", &self.scope_commitment)?;
        if self.epoch == 0 {
            return Err(violation("committee epoch must be non-zero"));
        }
        if self.threshold == 0 || self.member_count == 0 || self.threshold > self.member_count {
            return Err(violation("invalid threshold committee cardinality"));
        }
        Ok(())
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ProviderAuthorMode {
    DirectAuthor { author_id: String },
    ThresholdCommittee(ThresholdCommitteeDescriptor),
}

impl ProviderAuthorMode {
    fn validate(&self) -> ProviderAuthorityResult<()> {
        match self {
            Self::DirectAuthor { author_id } => require_opaque("author id", author_id),
            Self::ThresholdCommittee(descriptor) => descriptor.validate(),
        }
    }

    fn commit_into(&self, h: &mut Hasher) {
        match self {
            Self::DirectAuthor { author_id } => {
                h.update(&[0]);
                push_str(h, author_id);
            }
            Self::ThresholdCommittee(d) => {
                h.update(&[1]);
                push_str(h, &d.committee_id);
                h.update(&d.epoch.to_be_bytes());
                h.update(&d.threshold.to_be_bytes());
                h.update(&d.member_count.to_be_bytes());
                push_str(h, &d.committee_commitment);
                push_str(h, &d.public_key_commitment);
                push_str(h, &d.scope_commitment);
            }
        }
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ProviderAuthorGrant {
    pub schema_version: u16,
    pub grant_id: String,
    pub epoch: u64,
    pub provider_profile_id: String,
    pub provider_profile_commitment: String,
    pub jurisdiction: String,
    pub upstream_authority: PublicFundsAuthorityAnchor,
    pub roles: Vec<ProviderAuthorRole>,
    pub mode: ProviderAuthorMode,
    pub valid_from_us: i64,
    pub expires_at_us: Option<i64>,
    pub predecessor_grant_commitment: Option<String>,
    pub grant_commitment: String,
}

impl ProviderAuthorGrant {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        grant_id: impl Into<String>,
        epoch: u64,
        provider_profile_id: impl Into<String>,
        provider_profile_commitment: impl Into<String>,
        jurisdiction: impl Into<String>,
        upstream_authority: PublicFundsAuthorityAnchor,
        roles: impl IntoIterator<Item = ProviderAuthorRole>,
        mode: ProviderAuthorMode,
        valid_from_us: i64,
        expires_at_us: Option<i64>,
        predecessor_grant_commitment: Option<String>,
    ) -> ProviderAuthorityResult<Self> {
        let mut grant = Self {
            schema_version: PROVIDER_AUTHOR_GRANT_SCHEMA_VERSION,
            grant_id: grant_id.into(),
            epoch,
            provider_profile_id: provider_profile_id.into(),
            provider_profile_commitment: provider_profile_commitment.into(),
            jurisdiction: jurisdiction.into(),
            upstream_authority,
            roles: roles.into_iter().collect(),
            mode,
            valid_from_us,
            expires_at_us,
            predecessor_grant_commitment,
            grant_commitment: String::new(),
        };
        grant.normalize_roles();
        grant.validate_without_commitment()?;
        grant.grant_commitment = grant.compute_commitment();
        Ok(grant)
    }

    fn normalize_roles(&mut self) {
        let set: BTreeSet<_> = self.roles.iter().copied().collect();
        self.roles = set.into_iter().collect();
    }

    fn validate_without_commitment(&self) -> ProviderAuthorityResult<()> {
        if self.schema_version != PROVIDER_AUTHOR_GRANT_SCHEMA_VERSION {
            return Err(violation("provider author grant schema version drift"));
        }
        require_opaque("grant id", &self.grant_id)?;
        require_opaque("provider profile id", &self.provider_profile_id)?;
        require_opaque("provider profile commitment", &self.provider_profile_commitment)?;
        require_opaque("jurisdiction", &self.jurisdiction)?;
        if self.epoch == 0 {
            return Err(violation("grant epoch must be non-zero"));
        }
        if self.roles.is_empty() {
            return Err(violation("provider author grant must authorize at least one record role"));
        }
        let canonical: Vec<_> = self.roles.iter().copied().collect::<BTreeSet<_>>().into_iter().collect();
        if canonical != self.roles {
            return Err(violation("provider author roles must be unique and canonically ordered"));
        }
        self.mode.validate()?;
        if self.jurisdiction != self.upstream_authority.jurisdiction {
            return Err(violation("grant jurisdiction must equal upstream authority jurisdiction"));
        }
        if self.valid_from_us < self.upstream_authority.valid_from_us {
            return Err(violation("grant cannot begin before upstream constitutional authority"));
        }
        if let Some(expiry) = self.expires_at_us {
            if expiry <= self.valid_from_us {
                return Err(violation("grant expiry must be after grant start"));
            }
            if let Some(parent_expiry) = self.upstream_authority.expires_at_us {
                if expiry > parent_expiry {
                    return Err(violation("grant cannot outlive upstream constitutional authority"));
                }
            }
        } else if self.upstream_authority.expires_at_us.is_some() {
            return Err(violation("grant must not omit expiry when upstream authority expires"));
        }
        if let Some(predecessor) = &self.predecessor_grant_commitment {
            require_opaque("predecessor grant commitment", predecessor)?;
        }
        Ok(())
    }

    pub fn validate(&self) -> ProviderAuthorityResult<()> {
        self.validate_without_commitment()?;
        if self.grant_commitment != self.compute_commitment() {
            return Err(violation("provider author grant commitment mismatch"));
        }
        Ok(())
    }

    pub fn permits(&self, role: ProviderAuthorRole) -> bool {
        self.roles.binary_search(&role).is_ok()
    }

    fn compute_commitment(&self) -> String {
        let mut h = Hasher::new();
        h.update(GRANT_DOMAIN);
        h.update(&self.schema_version.to_be_bytes());
        push_str(&mut h, &self.grant_id);
        h.update(&self.epoch.to_be_bytes());
        push_str(&mut h, &self.provider_profile_id);
        push_str(&mut h, &self.provider_profile_commitment);
        push_str(&mut h, &self.jurisdiction);
        push_str(&mut h, &self.upstream_authority.capability_id);
        push_str(&mut h, &self.upstream_authority.holder_id);
        push_str(&mut h, &self.upstream_authority.jurisdiction);
        push_str(&mut h, &self.upstream_authority.capability_commitment);
        h.update(&self.upstream_authority.valid_from_us.to_be_bytes());
        match self.upstream_authority.expires_at_us {
            Some(v) => { h.update(&[1]); h.update(&v.to_be_bytes()); }
            None => { h.update(&[0]); }
        }
        h.update(&(self.roles.len() as u64).to_be_bytes());
        for role in &self.roles {
            h.update(&[*role as u8]);
        }
        self.mode.commit_into(&mut h);
        h.update(&self.valid_from_us.to_be_bytes());
        match self.expires_at_us {
            Some(v) => { h.update(&[1]); h.update(&v.to_be_bytes()); }
            None => { h.update(&[0]); }
        }
        match &self.predecessor_grant_commitment {
            Some(v) => { h.update(&[1]); push_str(&mut h, v); }
            None => { h.update(&[0]); }
        }
        format!("{GRANT_COMMITMENT_PREFIX}{}", h.finalize().to_hex())
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ProviderGrantStatus {
    Active,
    Revoked { revocation_id: String, effective_at_us: i64 },
    Superseded { successor_grant_commitment: String, effective_at_us: i64 },
}

impl ProviderGrantStatus {
    pub fn permits_time(&self, action_time_us: i64) -> ProviderAuthorityResult<bool> {
        match self {
            Self::Active => Ok(true),
            Self::Revoked { revocation_id, effective_at_us } => {
                require_opaque("revocation id", revocation_id)?;
                Ok(action_time_us < *effective_at_us)
            }
            Self::Superseded { successor_grant_commitment, effective_at_us } => {
                require_opaque("successor grant commitment", successor_grant_commitment)?;
                Ok(action_time_us < *effective_at_us)
            }
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RecordAuthorProof {
    Direct { action_author_id: String },
    ThresholdModelVerified {
        committee_id: String,
        committee_epoch: u64,
        signed_record_commitment: String,
        threshold_signature_commitment: String,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HistoricalAuthorEvidence {
    pub grant_commitment: String,
    pub grant_epoch: u64,
    pub action_time_us: i64,
    pub role: ProviderAuthorRole,
    pub record_commitment: String,
    pub proof: RecordAuthorProof,
}

pub fn authorize_record(
    grant: &ProviderAuthorGrant,
    status: &ProviderGrantStatus,
    expected_provider_profile_id: &str,
    expected_provider_profile_commitment: &str,
    expected_jurisdiction: &str,
    evidence: &HistoricalAuthorEvidence,
) -> ProviderAuthorityResult<()> {
    grant.validate()?;
    require_opaque("record commitment", &evidence.record_commitment)?;
    if evidence.grant_commitment != grant.grant_commitment || evidence.grant_epoch != grant.epoch {
        return Err(violation("historical author evidence does not bind exact grant"));
    }
    if grant.provider_profile_id != expected_provider_profile_id
        || grant.provider_profile_commitment != expected_provider_profile_commitment
    {
        return Err(violation("provider profile binding mismatch"));
    }
    if grant.jurisdiction != expected_jurisdiction {
        return Err(violation("provider jurisdiction mismatch"));
    }
    if !grant.permits(evidence.role) {
        return Err(violation("provider author role not granted"));
    }
    if evidence.action_time_us < grant.valid_from_us {
        return Err(violation("provider author grant not yet valid"));
    }
    if let Some(expiry) = grant.expires_at_us {
        if evidence.action_time_us >= expiry {
            return Err(violation("provider author grant expired at candidate action time"));
        }
    }
    if !grant.upstream_authority.valid_at(evidence.action_time_us) {
        return Err(violation("upstream constitutional authority invalid at candidate action time"));
    }
    if !status.permits_time(evidence.action_time_us)? {
        return Err(violation("provider author grant revoked or superseded at candidate action time"));
    }

    match (&grant.mode, &evidence.proof) {
        (ProviderAuthorMode::DirectAuthor { author_id }, RecordAuthorProof::Direct { action_author_id }) => {
            if author_id != action_author_id {
                return Err(violation("direct provider action author mismatch"));
            }
        }
        (
            ProviderAuthorMode::ThresholdCommittee(descriptor),
            RecordAuthorProof::ThresholdModelVerified {
                committee_id,
                committee_epoch,
                signed_record_commitment,
                threshold_signature_commitment,
            },
        ) => {
            descriptor.validate()?;
            require_opaque("threshold signature commitment", threshold_signature_commitment)?;
            if committee_id != &descriptor.committee_id || *committee_epoch != descriptor.epoch {
                return Err(violation("threshold committee identity/epoch mismatch"));
            }
            if signed_record_commitment != &evidence.record_commitment {
                return Err(violation("threshold signature does not bind exact record commitment"));
            }
        }
        _ => return Err(violation("author proof mode does not match provider author grant mode")),
    }
    Ok(())
}

pub fn validate_rotation(
    predecessor: &ProviderAuthorGrant,
    successor: &ProviderAuthorGrant,
    cutover_us: i64,
) -> ProviderAuthorityResult<()> {
    predecessor.validate()?;
    successor.validate()?;
    if successor.epoch != predecessor.epoch + 1 {
        return Err(violation("provider author rotation epoch must advance by exactly one"));
    }
    if successor.predecessor_grant_commitment.as_deref() != Some(predecessor.grant_commitment.as_str()) {
        return Err(violation("successor does not bind exact predecessor grant commitment"));
    }
    if successor.provider_profile_id != predecessor.provider_profile_id
        || successor.provider_profile_commitment != predecessor.provider_profile_commitment
        || successor.jurisdiction != predecessor.jurisdiction
        || successor.upstream_authority != predecessor.upstream_authority
    {
        return Err(violation("rotation must preserve provider profile, jurisdiction and upstream authority"));
    }
    let predecessor_roles: BTreeSet<_> = predecessor.roles.iter().copied().collect();
    if successor.roles.iter().any(|role| !predecessor_roles.contains(role)) {
        return Err(violation("rotation cannot amplify record roles"));
    }
    if successor.valid_from_us != cutover_us {
        return Err(violation("successor validity must begin at exact cutover"));
    }
    if cutover_us < predecessor.valid_from_us {
        return Err(violation("rotation cutover precedes predecessor validity"));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use constitutional_authority::{CapabilitySource, ConstitutionalCapability};

    fn root_capability() -> ConstitutionalCapability {
        ConstitutionalCapability {
            id: "cap-execute-appropriation".into(),
            holder_id: "stewardship-office".into(),
            holder: AuthorityPrincipal::Branch(Branch::Stewardship),
            power: ConstitutionalPower::ExecuteAppropriation,
            jurisdiction: "municipality:example".into(),
            source: CapabilitySource::Charter { charter_id: "charter-1".into(), version: 1 },
            valid_from_us: 100,
            expires_at_us: Some(10_000),
            delegable: true,
            delegation_depth_remaining: 1,
        }
    }

    fn anchor() -> PublicFundsAuthorityAnchor {
        PublicFundsAuthorityAnchor::from_root_capability(&root_capability(), "cap-commitment-1").unwrap()
    }

    fn direct_grant(epoch: u64, valid_from_us: i64, predecessor: Option<String>) -> ProviderAuthorGrant {
        ProviderAuthorGrant::new(
            format!("grant-{epoch}"),
            epoch,
            "payments-provider-v1",
            "provider-profile-commitment-1",
            "municipality:example",
            anchor(),
            [ProviderAuthorRole::OrchestratorIntent, ProviderAuthorRole::OrchestratorDispatch, ProviderAuthorRole::OperationIndex],
            ProviderAuthorMode::DirectAuthor { author_id: format!("did:mycelix:orchestrator-{epoch}") },
            valid_from_us,
            Some(9_000),
            predecessor,
        ).unwrap()
    }

    fn provider_grant() -> ProviderAuthorGrant {
        ProviderAuthorGrant::new(
            "provider-grant-1",
            1,
            "payments-provider-v1",
            "provider-profile-commitment-1",
            "municipality:example",
            anchor(),
            [ProviderAuthorRole::ProviderObservation],
            ProviderAuthorMode::DirectAuthor { author_id: "did:mycelix:payments-provider".into() },
            200,
            Some(9_000),
            None,
        ).unwrap()
    }

    fn evidence(grant: &ProviderAuthorGrant, role: ProviderAuthorRole, author: &str, time: i64) -> HistoricalAuthorEvidence {
        HistoricalAuthorEvidence {
            grant_commitment: grant.grant_commitment.clone(),
            grant_epoch: grant.epoch,
            action_time_us: time,
            role,
            record_commitment: "journal-record-commitment-1".into(),
            proof: RecordAuthorProof::Direct { action_author_id: author.into() },
        }
    }

    #[test]
    fn automated_provider_does_not_become_constitutional_sovereign() {
        let mut cap = root_capability();
        cap.holder = AuthorityPrincipal::AutomatedAgent;
        assert!(PublicFundsAuthorityAnchor::from_root_capability(&cap, "x").is_err());
    }

    #[test]
    fn observation_requires_provider_attestation_role() {
        let grant = provider_grant();
        let ev = evidence(&grant, ProviderAuthorRole::ProviderObservation, "did:mycelix:payments-provider", 500);
        authorize_record(&grant, &ProviderGrantStatus::Active, "payments-provider-v1", "provider-profile-commitment-1", "municipality:example", &ev).unwrap();
        let wrong = evidence(&grant, ProviderAuthorRole::OrchestratorDispatch, "did:mycelix:payments-provider", 500);
        assert!(authorize_record(&grant, &ProviderGrantStatus::Active, "payments-provider-v1", "provider-profile-commitment-1", "municipality:example", &wrong).is_err());
    }

    #[test]
    fn orchestrator_grant_cannot_author_provider_observation() {
        let grant = direct_grant(1, 200, None);
        let ev = evidence(&grant, ProviderAuthorRole::ProviderObservation, "did:mycelix:orchestrator-1", 500);
        assert!(authorize_record(&grant, &ProviderGrantStatus::Active, "payments-provider-v1", "provider-profile-commitment-1", "municipality:example", &ev).is_err());
    }

    #[test]
    fn revocation_is_time_relative_and_not_retroactive() {
        let grant = provider_grant();
        let status = ProviderGrantStatus::Revoked { revocation_id: "revoke-1".into(), effective_at_us: 1_000 };
        let historical = evidence(&grant, ProviderAuthorRole::ProviderObservation, "did:mycelix:payments-provider", 999);
        authorize_record(&grant, &status, "payments-provider-v1", "provider-profile-commitment-1", "municipality:example", &historical).unwrap();
        let later = evidence(&grant, ProviderAuthorRole::ProviderObservation, "did:mycelix:payments-provider", 1_000);
        assert!(authorize_record(&grant, &status, "payments-provider-v1", "provider-profile-commitment-1", "municipality:example", &later).is_err());
    }

    #[test]
    fn rotation_is_monotone_and_non_amplifying() {
        let first = direct_grant(1, 200, None);
        let second = direct_grant(2, 1_000, Some(first.grant_commitment.clone()));
        validate_rotation(&first, &second, 1_000).unwrap();

        let mut amplified = second.clone();
        amplified.roles.push(ProviderAuthorRole::ProviderObservation);
        amplified.normalize_roles();
        amplified.grant_commitment = amplified.compute_commitment();
        assert!(validate_rotation(&first, &amplified, 1_000).is_err());
    }

    #[test]
    fn supersession_preserves_pre_cutover_history() {
        let first = direct_grant(1, 200, None);
        let second = direct_grant(2, 1_000, Some(first.grant_commitment.clone()));
        validate_rotation(&first, &second, 1_000).unwrap();
        let status = ProviderGrantStatus::Superseded { successor_grant_commitment: second.grant_commitment.clone(), effective_at_us: 1_000 };
        let before = evidence(&first, ProviderAuthorRole::OrchestratorDispatch, "did:mycelix:orchestrator-1", 999);
        authorize_record(&first, &status, "payments-provider-v1", "provider-profile-commitment-1", "municipality:example", &before).unwrap();
        let at = evidence(&first, ProviderAuthorRole::OrchestratorDispatch, "did:mycelix:orchestrator-1", 1_000);
        assert!(authorize_record(&first, &status, "payments-provider-v1", "provider-profile-commitment-1", "municipality:example", &at).is_err());
    }

    #[test]
    fn threshold_proof_binds_exact_record_commitment() {
        let grant = ProviderAuthorGrant::new(
            "threshold-provider-grant",
            1,
            "payments-provider-v1",
            "provider-profile-commitment-1",
            "municipality:example",
            anchor(),
            [ProviderAuthorRole::ProviderObservation],
            ProviderAuthorMode::ThresholdCommittee(ThresholdCommitteeDescriptor {
                committee_id: "committee-payments-1".into(),
                epoch: 7,
                threshold: 3,
                member_count: 5,
                committee_commitment: "committee-commitment-1".into(),
                public_key_commitment: "public-key-commitment-1".into(),
                scope_commitment: "treasury-provider-attestation".into(),
            }),
            200,
            Some(9_000),
            None,
        ).unwrap();
        let mut ev = HistoricalAuthorEvidence {
            grant_commitment: grant.grant_commitment.clone(),
            grant_epoch: grant.epoch,
            action_time_us: 500,
            role: ProviderAuthorRole::ProviderObservation,
            record_commitment: "journal-record-commitment-1".into(),
            proof: RecordAuthorProof::ThresholdModelVerified {
                committee_id: "committee-payments-1".into(),
                committee_epoch: 7,
                signed_record_commitment: "journal-record-commitment-1".into(),
                threshold_signature_commitment: "threshold-signature-1".into(),
            },
        };
        authorize_record(&grant, &ProviderGrantStatus::Active, "payments-provider-v1", "provider-profile-commitment-1", "municipality:example", &ev).unwrap();
        if let RecordAuthorProof::ThresholdModelVerified { signed_record_commitment, .. } = &mut ev.proof {
            *signed_record_commitment = "other-record".into();
        }
        assert!(authorize_record(&grant, &ProviderGrantStatus::Active, "payments-provider-v1", "provider-profile-commitment-1", "municipality:example", &ev).is_err());
    }
}
