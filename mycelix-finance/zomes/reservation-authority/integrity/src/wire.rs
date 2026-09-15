use std::collections::BTreeSet;

use hdi::prelude::*;
use mycelix_bridge_entry_types::did_for_author;
use mycelix_business_core::{
    ActionContractRef, AuthorizedIntentRef, Digest32, ExecutionAttemptRef, ReferenceId,
    ReservationId, SubjectRef,
};
use mycelix_business_decision::DecisionCapsuleRef;
use mycelix_finance_exact::{AssetAmount, AssetId};
use mycelix_finance_reservation_core::{
    reservation_commitment, ReservationDescriptor, ReservationLifecycle, ReservationState,
};
use mycelix_finance_settlement::FinalityProfileRef;

pub const RESERVATION_AUTHORITY_SCHEMA_VERSION: u8 = 1;

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub struct ReservationDescriptorWire {
    pub commitment_profile_revision: u16,
    pub reservation_id: String,
    pub request_id: String,
    pub finance_domain: String,
    pub issuer_authority: String,
    pub issuer_profile: String,
    pub issuer_profile_revision: u64,
    pub issuer_profile_digest: [u8; 32],
    pub finance_policy_sequence: u64,
    pub finance_policy_digest: [u8; 32],
    pub subject: String,
    pub asset: String,
    pub atomic_units: u64,
    pub effect_class: String,
    pub action_contract_semantic_id: String,
    pub action_contract_digest: [u8; 32],
    pub decision_id: String,
    pub decision_digest: [u8; 32],
    pub authorized_intent: String,
    pub intent_digest: [u8; 32],
    pub authority_lease_id: String,
    pub authority_epoch: u64,
    pub fencing_token: u64,
    /// Must already be strictly sorted and duplicate-free on the wire.
    pub aggregate_policy_keys: Vec<String>,
    pub idempotency_key: String,
    pub required_finality_profile_id: String,
    pub required_finality_profile_revision: u64,
    pub required_finality_profile_digest: [u8; 32],
    pub issued_at_unix_ms: u64,
    pub expires_at_unix_ms: u64,
}

impl ReservationDescriptorWire {
    pub fn to_core(&self) -> Result<ReservationDescriptor, String> {
        validate_sorted_unique(&self.aggregate_policy_keys, "aggregate_policy_keys")?;

        let aggregate_policy_keys = self
            .aggregate_policy_keys
            .iter()
            .map(|value| reference(value))
            .collect::<Result<BTreeSet<_>, _>>()?;

        let descriptor = ReservationDescriptor {
            commitment_profile_revision: self.commitment_profile_revision,
            reservation_id: ReservationId(reference(&self.reservation_id)?),
            request_id: reference(&self.request_id)?,
            finance_domain: reference(&self.finance_domain)?,
            issuer_authority: reference(&self.issuer_authority)?,
            issuer_profile: reference(&self.issuer_profile)?,
            issuer_profile_revision: self.issuer_profile_revision,
            issuer_profile_digest: Digest32(self.issuer_profile_digest),
            finance_policy_sequence: self.finance_policy_sequence,
            finance_policy_digest: Digest32(self.finance_policy_digest),
            subject: SubjectRef(reference(&self.subject)?),
            amount: AssetAmount::new(
                self.atomic_units,
                AssetId::new(self.asset.clone())
                    .map_err(|error| format!("invalid asset identifier: {error}"))?,
            ),
            effect_class: reference(&self.effect_class)?,
            action_contract: ActionContractRef {
                semantic_id: reference(&self.action_contract_semantic_id)?,
                digest: Digest32(self.action_contract_digest),
            },
            decision: DecisionCapsuleRef {
                id: reference(&self.decision_id)?,
                digest: Digest32(self.decision_digest),
            },
            authorized_intent: AuthorizedIntentRef(reference(&self.authorized_intent)?),
            intent_digest: Digest32(self.intent_digest),
            authority_lease_id: reference(&self.authority_lease_id)?,
            authority_epoch: self.authority_epoch,
            fencing_token: self.fencing_token,
            aggregate_policy_keys,
            idempotency_key: reference(&self.idempotency_key)?,
            required_finality_profile: FinalityProfileRef {
                id: reference(&self.required_finality_profile_id)?,
                revision: self.required_finality_profile_revision,
                digest: Digest32(self.required_finality_profile_digest),
            },
            issued_at_unix_ms: self.issued_at_unix_ms,
            expires_at_unix_ms: self.expires_at_unix_ms,
        };

        descriptor
            .validate()
            .map_err(|error| format!("invalid reservation descriptor: {error:?}"))?;
        Ok(descriptor)
    }
}

#[derive(Serialize, Deserialize, Debug, Clone, PartialEq, Eq)]
pub enum ReservationLifecycleWire {
    Active,
    Consumed {
        attempt: String,
        idempotency_key: String,
        consumed_at_unix_ms: u64,
    },
    Released {
        evidence: String,
        released_at_unix_ms: u64,
    },
    Revoked {
        evidence: String,
        revoked_at_unix_ms: u64,
    },
    Expired {
        expired_at_unix_ms: u64,
    },
}

impl ReservationLifecycleWire {
    pub fn to_core(&self) -> Result<ReservationLifecycle, String> {
        match self {
            Self::Active => Ok(ReservationLifecycle::Active),
            Self::Consumed {
                attempt,
                idempotency_key,
                consumed_at_unix_ms,
            } => Ok(ReservationLifecycle::Consumed {
                attempt: ExecutionAttemptRef(reference(attempt)?),
                idempotency_key: reference(idempotency_key)?,
                consumed_at_unix_ms: *consumed_at_unix_ms,
            }),
            Self::Released {
                evidence,
                released_at_unix_ms,
            } => Ok(ReservationLifecycle::Released {
                evidence: reference(evidence)?,
                released_at_unix_ms: *released_at_unix_ms,
            }),
            Self::Revoked {
                evidence,
                revoked_at_unix_ms,
            } => Ok(ReservationLifecycle::Revoked {
                evidence: reference(evidence)?,
                revoked_at_unix_ms: *revoked_at_unix_ms,
            }),
            Self::Expired { expired_at_unix_ms } => Ok(ReservationLifecycle::Expired {
                expired_at_unix_ms: *expired_at_unix_ms,
            }),
        }
    }

    pub fn from_core(value: &ReservationLifecycle) -> Self {
        match value {
            ReservationLifecycle::Active => Self::Active,
            ReservationLifecycle::Consumed {
                attempt,
                idempotency_key,
                consumed_at_unix_ms,
            } => Self::Consumed {
                attempt: attempt.0.as_str().to_string(),
                idempotency_key: idempotency_key.as_str().to_string(),
                consumed_at_unix_ms: *consumed_at_unix_ms,
            },
            ReservationLifecycle::Released {
                evidence,
                released_at_unix_ms,
            } => Self::Released {
                evidence: evidence.as_str().to_string(),
                released_at_unix_ms: *released_at_unix_ms,
            },
            ReservationLifecycle::Revoked {
                evidence,
                revoked_at_unix_ms,
            } => Self::Revoked {
                evidence: evidence.as_str().to_string(),
                revoked_at_unix_ms: *revoked_at_unix_ms,
            },
            ReservationLifecycle::Expired { expired_at_unix_ms } => Self::Expired {
                expired_at_unix_ms: *expired_at_unix_ms,
            },
        }
    }
}

#[hdk_entry_helper]
#[derive(Clone, PartialEq, Eq)]
pub struct ReservationAuthorityEntry {
    pub schema_version: u8,
    pub descriptor: ReservationDescriptorWire,
    pub descriptor_commitment: [u8; 32],
    pub state_sequence: u64,
    pub lifecycle: ReservationLifecycleWire,
    pub state_commitment: [u8; 32],
    /// Redundant with the action author by design; integrity binds the two.
    pub issuer_agent: AgentPubKey,
}

impl ReservationAuthorityEntry {
    pub fn verified_semantics(&self) -> Result<(ReservationDescriptor, ReservationState), String> {
        if self.schema_version != RESERVATION_AUTHORITY_SCHEMA_VERSION {
            return Err(format!(
                "unsupported reservation authority schema version {}",
                self.schema_version
            ));
        }

        let descriptor = self.descriptor.to_core()?;
        let descriptor_commitment = reservation_commitment(&descriptor)
            .map_err(|error| format!("cannot compute descriptor commitment: {error:?}"))?;
        if descriptor_commitment.into_bytes() != self.descriptor_commitment {
            return Err("descriptor commitment does not match canonical descriptor".into());
        }

        let lifecycle = self.lifecycle.to_core()?;
        let state = ReservationState::verified_from_claimed(
            &descriptor,
            self.state_sequence,
            lifecycle,
            self.state_commitment,
        )
        .map_err(|error| format!("invalid reservation lifecycle state: {error:?}"))?;

        Ok((descriptor, state))
    }

    pub fn issuer_authority_matches(&self, author: &AgentPubKey) -> Result<(), String> {
        if &self.issuer_agent != author {
            return Err("issuer_agent does not match the Holochain action author".into());
        }
        let expected = did_for_author(author);
        if self.descriptor.issuer_authority != expected {
            return Err(format!(
                "descriptor issuer_authority must equal action author DID {expected}"
            ));
        }
        Ok(())
    }
}

pub fn authority_did_for_agent(agent: &AgentPubKey) -> String {
    did_for_author(agent)
}

fn reference(value: &str) -> Result<ReferenceId, String> {
    ReferenceId::new(value.to_string())
        .map_err(|error| format!("invalid canonical reference {value:?}: {error:?}"))
}

fn validate_sorted_unique(values: &[String], field: &str) -> Result<(), String> {
    if values.windows(2).any(|pair| pair[0] >= pair[1]) {
        return Err(format!(
            "{field} must be strictly sorted and duplicate-free on the wire"
        ));
    }
    Ok(())
}
