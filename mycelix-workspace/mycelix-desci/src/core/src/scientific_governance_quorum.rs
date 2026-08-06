// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Risk-tiered quorum policy for scientific credential governance.

use crate::scientific_credential_governance::CredentialGovernanceAction;
use crate::scientific_credentials::ScientificCredentialPayload;
use crate::scientific_governance::ScientificRole;
use crate::{Error, Result};
use serde::{Deserialize, Serialize};

const MAX_APPROVAL_THRESHOLD: u16 = 1024;
const MAX_DISTINCT_ORGANIZATIONS: u16 = 1024;
const MAX_ACTIVATION_DELAY_SECONDS: u64 = 90 * 24 * 60 * 60;

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum CredentialGovernanceRiskTier {
    Routine,
    Sensitive,
    Critical,
}

impl CredentialGovernanceRiskTier {
    pub const fn code(self) -> u8 {
        match self {
            Self::Routine => 1,
            Self::Sensitive => 2,
            Self::Critical => 3,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CredentialGovernanceApprovalRule {
    pub approval_threshold: u16,
    pub minimum_distinct_organizations: u16,
    pub activation_delay_seconds: u64,
}

impl CredentialGovernanceApprovalRule {
    pub fn validate(&self, label: &str) -> Result<()> {
        if self.approval_threshold < 2 || self.approval_threshold > MAX_APPROVAL_THRESHOLD {
            return Err(Error::Validation(format!(
                "{label} approval threshold must be between 2 and {MAX_APPROVAL_THRESHOLD}"
            )));
        }
        if self.minimum_distinct_organizations > self.approval_threshold
            || self.minimum_distinct_organizations > MAX_DISTINCT_ORGANIZATIONS
        {
            return Err(Error::Validation(format!(
                "{label} organization diversity cannot exceed its approval threshold"
            )));
        }
        if self.activation_delay_seconds == 0
            || self.activation_delay_seconds > MAX_ACTIVATION_DELAY_SECONDS
        {
            return Err(Error::Validation(format!(
                "{label} activation delay must be between 1 and {MAX_ACTIVATION_DELAY_SECONDS} seconds"
            )));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CredentialGovernanceRiskPolicy {
    pub routine: CredentialGovernanceApprovalRule,
    pub sensitive: CredentialGovernanceApprovalRule,
    pub critical: CredentialGovernanceApprovalRule,
}

impl CredentialGovernanceRiskPolicy {
    pub fn validate(&self) -> Result<()> {
        self.routine.validate("routine governance")?;
        self.sensitive.validate("sensitive governance")?;
        self.critical.validate("critical governance")?;
        if self.sensitive.approval_threshold < self.routine.approval_threshold
            || self.critical.approval_threshold < self.sensitive.approval_threshold
        {
            return Err(Error::Validation(
                "governance approval thresholds must be monotonic by risk tier".to_string(),
            ));
        }
        if self.sensitive.minimum_distinct_organizations
            < self.routine.minimum_distinct_organizations
            || self.critical.minimum_distinct_organizations
                < self.sensitive.minimum_distinct_organizations
        {
            return Err(Error::Validation(
                "governance organization diversity must be monotonic by risk tier".to_string(),
            ));
        }
        if self.sensitive.activation_delay_seconds < self.routine.activation_delay_seconds
            || self.critical.activation_delay_seconds < self.sensitive.activation_delay_seconds
        {
            return Err(Error::Validation(
                "governance activation delays must be monotonic by risk tier".to_string(),
            ));
        }
        Ok(())
    }

    pub fn rule(&self, tier: CredentialGovernanceRiskTier) -> &CredentialGovernanceApprovalRule {
        match tier {
            CredentialGovernanceRiskTier::Routine => &self.routine,
            CredentialGovernanceRiskTier::Sensitive => &self.sensitive,
            CredentialGovernanceRiskTier::Critical => &self.critical,
        }
    }
}

impl CredentialGovernanceAction {
    pub fn risk_tier(&self) -> CredentialGovernanceRiskTier {
        match self {
            Self::AppendCredentialEvent { event } => match &event.envelope.payload {
                ScientificCredentialPayload::RegistryInitialized { .. } => {
                    CredentialGovernanceRiskTier::Critical
                }
                ScientificCredentialPayload::ActorRegistered { profile } => {
                    if profile.has_role(ScientificRole::RegistryAdmin) {
                        CredentialGovernanceRiskTier::Critical
                    } else {
                        CredentialGovernanceRiskTier::Routine
                    }
                }
                ScientificCredentialPayload::ActorKeyAuthorized { .. }
                | ScientificCredentialPayload::RoleGranted { .. }
                | ScientificCredentialPayload::RoleRevoked { .. }
                | ScientificCredentialPayload::OrganizationMembershipRevoked { .. } => {
                    match &event.envelope.payload {
                        ScientificCredentialPayload::RoleGranted {
                            role: ScientificRole::RegistryAdmin,
                            ..
                        }
                        | ScientificCredentialPayload::RoleRevoked {
                            role: ScientificRole::RegistryAdmin,
                            ..
                        } => CredentialGovernanceRiskTier::Critical,
                        _ => CredentialGovernanceRiskTier::Sensitive,
                    }
                }
                ScientificCredentialPayload::ActorKeyRevoked { .. }
                | ScientificCredentialPayload::ActorKeyCompromised { .. } => {
                    CredentialGovernanceRiskTier::Critical
                }
                ScientificCredentialPayload::OrganizationMembershipGranted { .. } => {
                    CredentialGovernanceRiskTier::Routine
                }
            },
            Self::AuthorizeAcceptanceServiceKey { .. } => CredentialGovernanceRiskTier::Sensitive,
            Self::RevokeAcceptanceServiceKey { .. }
            | Self::UpdateGovernancePolicy { .. }
            | Self::UpdateGovernanceRiskPolicy { .. }
            | Self::RevokeTransparencyWitness { .. }
            | Self::RecordTransparencyWitnessCompromise { .. } => {
                CredentialGovernanceRiskTier::Critical
            }
            Self::AuthorizeTransparencyWitness { .. } => CredentialGovernanceRiskTier::Sensitive,
            Self::AuthorizeDatabaseEpochPromotion { .. } => CredentialGovernanceRiskTier::Critical,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::scientific_credential_governance::{
        AuthorizedCredentialTransparencyWitness, CredentialGovernancePolicy,
    };
    use crate::{ActorId, OrganizationId};
    use chrono::{Duration, Utc};
    use ed25519_dalek::SigningKey;

    fn rule(approvals: u16, organizations: u16, delay: u64) -> CredentialGovernanceApprovalRule {
        CredentialGovernanceApprovalRule {
            approval_threshold: approvals,
            minimum_distinct_organizations: organizations,
            activation_delay_seconds: delay,
        }
    }

    #[test]
    fn risk_policy_requires_monotonic_strength() {
        let valid = CredentialGovernanceRiskPolicy {
            routine: rule(2, 1, 60),
            sensitive: rule(3, 2, 3_600),
            critical: rule(4, 3, 86_400),
        };
        assert!(valid.validate().is_ok());

        let weakened = CredentialGovernanceRiskPolicy {
            routine: rule(3, 2, 3_600),
            sensitive: rule(2, 1, 60),
            critical: rule(4, 3, 86_400),
        };
        assert!(weakened.validate().is_err());
    }

    #[test]
    fn critical_actions_cannot_fall_into_the_routine_tier() {
        let policy_change = CredentialGovernanceAction::UpdateGovernancePolicy {
            policy: CredentialGovernancePolicy {
                approval_threshold: 2,
                activation_delay_seconds: 60,
                proposal_ttl_seconds: 3_600,
                proposer_counts_as_approval: true,
                emergency_cancellation_enabled: true,
            },
        };
        assert_eq!(
            policy_change.risk_tier(),
            CredentialGovernanceRiskTier::Critical
        );

        let witness_key = SigningKey::from_bytes(&[42; 32]);
        let witness = CredentialGovernanceAction::AuthorizeTransparencyWitness {
            witness: AuthorizedCredentialTransparencyWitness {
                actor: ActorId::new("did:key:witness").unwrap(),
                organization: OrganizationId::new("org:external-witness").unwrap(),
                public_key: witness_key.verifying_key().to_bytes(),
                valid_from: Utc::now() - Duration::seconds(1),
                valid_until: None,
                revoked_at: None,
            },
        };
        assert_eq!(witness.risk_tier(), CredentialGovernanceRiskTier::Sensitive);
    }
}
