//! Chain-neutral external settlement primitives.
//!
//! This module deliberately contains no HDK/HDI, RPC, wallet, or chain-client
//! dependency.  It defines the evidence contract that external settlement
//! adapters must satisfy before an external chain observation can be admitted
//! into Mycelix finance state.
//!
//! Security boundary:
//!
//! ```text
//! chain name        != settlement security
//! transaction hash  != finality
//! bridge success    != monetary authorization
//! trust label       != proof
//! ```

use serde::{Deserialize, Serialize};

#[path = "settlement_receipt.rs"]
pub mod receipt;
pub use receipt::*;
use core::fmt;

/// Broad execution model of a settlement rail.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SettlementExecutionKind {
    Evm,
    Other(String),
}

/// How a rail's state transitions are validated.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum StateValidationClass {
    EthereumCanonical,
    FraudProof,
    ValidityProof,
    ExternalValidatorSet,
    Other(String),
    Unknown,
}

/// Where the rail's transaction/state data is expected to be available.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum DataAvailabilityClass {
    EthereumCalldata,
    EthereumBlobs,
    ExternalDA,
    OperatorControlled,
    Unknown,
}

/// How an external bridge/message path validates instructions.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum BridgeValidationClass {
    TrustlessProof,
    ValidatorAttested,
    MultisigAttested,
    ExternalMessaging,
    None,
    Unknown,
}

/// Upgrade authority is evidence about who can change the settlement surface.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum UpgradeAuthorityClass {
    Immutable,
    Timelocked,
    GovernanceControlled,
    MultisigControlled,
    AdminControlled,
    Unknown,
}

/// Censorship/exit properties are represented separately from state validation.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum CensorshipResistanceClass {
    EthereumForcedExit,
    PermissionlessExit,
    BoundedOperator,
    Unknown,
}

/// Explicit settlement state.  Inclusion is intentionally not finality.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SettlementFinality {
    Observed,
    Submitted,
    Included,
    ProbabilisticallyFinal,
    BridgeFinal,
    EconomicallyFinal,
    Reconciled,
    Disputed,
    Superseded,
}

/// A frozen description of one external settlement environment.
///
/// The profile is deliberately declarative: it does not claim that a rail is
/// "safe."  It records the exact assumptions a verifier is willing to consume.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SettlementRailProfile {
    pub rail_id: String,
    pub chain_id: u64,
    pub execution: SettlementExecutionKind,
    pub settlement_anchor: Option<String>,
    pub state_validation: StateValidationClass,
    pub data_availability: DataAvailabilityClass,
    pub bridge_validation: BridgeValidationClass,
    pub upgrade_authority: UpgradeAuthorityClass,
    pub censorship_resistance: CensorshipResistanceClass,
    pub configured_finality: SettlementFinality,
    pub max_exposure_minor_units: u128,
    pub configuration_digest: String,
    pub effective_from_micros: i64,
    pub effective_until_micros: Option<i64>,
    pub supersedes_profile_id: Option<String>,
}

impl SettlementRailProfile {
    /// Validate the profile's structural invariants.
    pub fn validate(&self) -> Result<(), SettlementValidationError> {
        require_nonempty(&self.rail_id, "rail_id")?;
        require_nonempty(&self.configuration_digest, "configuration_digest")?;

        if self.chain_id == 0 {
            return Err(SettlementValidationError::InvalidField {
                field: "chain_id",
                reason: "must be non-zero",
            });
        }

        if self.effective_from_micros < 0 {
            return Err(SettlementValidationError::InvalidField {
                field: "effective_from_micros",
                reason: "must be non-negative",
            });
        }

        if let Some(until) = self.effective_until_micros {
            if until <= self.effective_from_micros {
                return Err(SettlementValidationError::InvalidField {
                    field: "effective_until_micros",
                    reason: "must be later than effective_from_micros",
                });
            }
        }

        Ok(())
    }

    /// Whether a claim may be admitted under this profile's configured
    /// high-assurance settlement boundary.
    pub fn admits_high_assurance_claim(
        &self,
        claim: &SettlementClaim,
    ) -> Result<(), SettlementValidationError> {
        self.validate()?;
        claim.validate()?;

        if claim.rail_id != self.rail_id {
            return Err(SettlementValidationError::ProfileMismatch);
        }

        if claim.chain_id != self.chain_id {
            return Err(SettlementValidationError::ChainIdMismatch);
        }

        if claim.configuration_digest != self.configuration_digest {
            return Err(SettlementValidationError::ConfigurationMismatch);
        }

        if claim.amount_minor_units == 0 {
            return Err(SettlementValidationError::InvalidField {
                field: "amount_minor_units",
                reason: "must be greater than zero",
            });
        }

        if claim.amount_minor_units > self.max_exposure_minor_units {
            return Err(SettlementValidationError::ExposureExceeded);
        }

        if claim.finality.assurance_rank() < self.configured_finality.assurance_rank() {
            return Err(SettlementValidationError::InsufficientFinality);
        }

        match &claim.finality {
            SettlementFinality::BridgeFinal
            | SettlementFinality::EconomicallyFinal
            | SettlementFinality::Reconciled => {}
            _ => return Err(SettlementValidationError::InsufficientFinality),
        }

        if claim.proof_or_attestation_ref.is_none() {
            return Err(SettlementValidationError::MissingProofOrAttestation);
        }

        if claim.reconciled != matches!(&claim.finality, SettlementFinality::Reconciled) {
            return Err(SettlementValidationError::ReconciliationStateMismatch);
        }

        Ok(())
    }
}

/// One external settlement observation associated with a Mycelix economic event.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SettlementClaim {
    pub settlement_claim_id: String,
    pub mycelix_event_id: String,
    pub source_issuance_or_obligation_ref: String,
    pub rail_id: String,
    pub chain_id: u64,
    pub source_domain: String,
    pub destination_domain: String,
    pub asset_id: String,
    pub token_contract: Option<String>,
    pub amount_minor_units: u128,
    pub unit: String,
    pub transaction_or_message_id: String,
    pub proof_or_attestation_ref: Option<String>,
    pub finality: SettlementFinality,
    pub reconciled: bool,
    pub adapter_version: String,
    pub configuration_digest: String,
    pub observed_at_micros: i64,
    pub known_at_micros: i64,
    pub supersedes_claim_id: Option<String>,
}

impl SettlementFinality {
    /// Monotonic assurance ordering for admissibility checks.
    /// Disputed/superseded states are intentionally not ranked as stronger
    /// finality; they are rejected separately.
    pub fn assurance_rank(&self) -> u8 {
        match self {
            Self::Observed => 0,
            Self::Submitted => 1,
            Self::Included => 2,
            Self::ProbabilisticallyFinal => 3,
            Self::BridgeFinal => 4,
            Self::EconomicallyFinal => 5,
            Self::Reconciled => 6,
            Self::Disputed | Self::Superseded => 0,
        }
    }
}

impl SettlementClaim {
    pub fn validate(&self) -> Result<(), SettlementValidationError> {
        require_nonempty(&self.settlement_claim_id, "settlement_claim_id")?;
        require_nonempty(&self.mycelix_event_id, "mycelix_event_id")?;
        require_nonempty(
            &self.source_issuance_or_obligation_ref,
            "source_issuance_or_obligation_ref",
        )?;
        require_nonempty(&self.rail_id, "rail_id")?;
        require_nonempty(&self.source_domain, "source_domain")?;
        require_nonempty(&self.destination_domain, "destination_domain")?;
        require_nonempty(&self.asset_id, "asset_id")?;
        require_nonempty(&self.unit, "unit")?;
        require_nonempty(
            &self.transaction_or_message_id,
            "transaction_or_message_id",
        )?;
        require_nonempty(&self.adapter_version, "adapter_version")?;
        require_nonempty(&self.configuration_digest, "configuration_digest")?;

        if self.chain_id == 0 {
            return Err(SettlementValidationError::InvalidField {
                field: "chain_id",
                reason: "must be non-zero",
            });
        }

        if self.amount_minor_units == 0 {
            return Err(SettlementValidationError::InvalidField {
                field: "amount_minor_units",
                reason: "must be greater than zero",
            });
        }

        if self.observed_at_micros < 0 || self.known_at_micros < 0 {
            return Err(SettlementValidationError::InvalidField {
                field: "timestamps",
                reason: "must be non-negative",
            });
        }

        if self.known_at_micros < self.observed_at_micros {
            return Err(SettlementValidationError::InvalidField {
                field: "known_at_micros",
                reason: "must not precede observed_at_micros",
            });
        }

        if matches!(
            &self.finality,
            SettlementFinality::BridgeFinal
                | SettlementFinality::EconomicallyFinal
                | SettlementFinality::Reconciled
        ) && self.proof_or_attestation_ref.is_none()
        {
            return Err(SettlementValidationError::MissingProofOrAttestation);
        }

        if matches!(
            &self.finality,
            SettlementFinality::Disputed | SettlementFinality::Superseded
        ) {
            return Err(SettlementValidationError::NonAdmissibleFinality);
        }

        Ok(())
    }
}

/// Validation errors intentionally expose boundary failures rather than
/// collapsing them into a generic "invalid settlement" result.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum SettlementValidationError {
    InvalidField {
        field: &'static str,
        reason: &'static str,
    },
    ProfileMismatch,
    ChainIdMismatch,
    ConfigurationMismatch,
    ExposureExceeded,
    InsufficientFinality,
    MissingProofOrAttestation,
    ReconciliationStateMismatch,
    NonAdmissibleFinality,
}

impl fmt::Display for SettlementValidationError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidField { field, reason } => write!(f, "{field}: {reason}"),
            Self::ProfileMismatch => write!(f, "settlement rail profile mismatch"),
            Self::ChainIdMismatch => write!(f, "settlement chain ID mismatch"),
            Self::ConfigurationMismatch => {
                write!(f, "settlement configuration digest mismatch")
            }
            Self::ExposureExceeded => write!(f, "settlement exposure limit exceeded"),
            Self::InsufficientFinality => write!(f, "settlement finality is insufficient"),
            Self::MissingProofOrAttestation => {
                write!(f, "settlement proof or attestation is required")
            }
            Self::ReconciliationStateMismatch => {
                write!(f, "reconciliation state does not match finality")
            }
            Self::NonAdmissibleFinality => {
                write!(f, "settlement is disputed or superseded")
            }
        }
    }
}

fn require_nonempty(
    value: &str,
    field: &'static str,
) -> Result<(), SettlementValidationError> {
    if value.trim().is_empty() {
        return Err(SettlementValidationError::InvalidField {
            field,
            reason: "must not be empty",
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn profile() -> SettlementRailProfile {
        SettlementRailProfile {
            rail_id: "polygon-pos-mainnet".into(),
            chain_id: 137,
            execution: SettlementExecutionKind::Evm,
            settlement_anchor: Some("ethereum-mainnet".into()),
            state_validation: StateValidationClass::ExternalValidatorSet,
            data_availability: DataAvailabilityClass::OperatorControlled,
            bridge_validation: BridgeValidationClass::ValidatorAttested,
            upgrade_authority: UpgradeAuthorityClass::MultisigControlled,
            censorship_resistance: CensorshipResistanceClass::BoundedOperator,
            configured_finality: SettlementFinality::BridgeFinal,
            max_exposure_minor_units: 1_000_000,
            configuration_digest: "config-v1".into(),
            effective_from_micros: 1,
            effective_until_micros: None,
            supersedes_profile_id: None,
        }
    }

    fn claim() -> SettlementClaim {
        SettlementClaim {
            settlement_claim_id: "claim-1".into(),
            mycelix_event_id: "event-1".into(),
            source_issuance_or_obligation_ref: "issuance-1".into(),
            rail_id: "polygon-pos-mainnet".into(),
            chain_id: 137,
            source_domain: "mycelix".into(),
            destination_domain: "polygon:137".into(),
            asset_id: "SAP".into(),
            token_contract: Some("0x0000000000000000000000000000000000000001".into()),
            amount_minor_units: 100,
            unit: "micro-SAP".into(),
            transaction_or_message_id: "0xtx".into(),
            proof_or_attestation_ref: Some("proof-1".into()),
            finality: SettlementFinality::Reconciled,
            reconciled: true,
            adapter_version: "polygon-adapter-v1".into(),
            configuration_digest: "config-v1".into(),
            observed_at_micros: 10,
            known_at_micros: 20,
            supersedes_claim_id: None,
        }
    }

    #[test]
    fn valid_claim_admits() {
        let p = profile();
        assert!(p.admits_high_assurance_claim(&claim()).is_ok());
    }

    #[test]
    fn wrong_chain_id_fails_closed() {
        let p = profile();
        let mut c = claim();
        c.chain_id = 1;
        assert_eq!(
            p.admits_high_assurance_claim(&c),
            Err(SettlementValidationError::ChainIdMismatch)
        );
    }

    #[test]
    fn wrong_configuration_fails_closed() {
        let p = profile();
        let mut c = claim();
        c.configuration_digest = "config-v2".into();
        assert_eq!(
            p.admits_high_assurance_claim(&c),
            Err(SettlementValidationError::ConfigurationMismatch)
        );
    }

    #[test]
    fn missing_proof_fails_closed() {
        let p = profile();
        let mut c = claim();
        c.proof_or_attestation_ref = None;
        assert_eq!(
            p.admits_high_assurance_claim(&c),
            Err(SettlementValidationError::MissingProofOrAttestation)
        );
    }

    #[test]
    fn insufficient_finality_fails_closed() {
        let p = profile();
        let mut c = claim();
        c.finality = SettlementFinality::Included;
        c.reconciled = false;
        assert_eq!(
            p.admits_high_assurance_claim(&c),
            Err(SettlementValidationError::InsufficientFinality)
        );
    }

    #[test]
    fn disputed_claim_fails_closed() {
        let mut c = claim();
        c.finality = SettlementFinality::Disputed;
        assert_eq!(
            c.validate(),
            Err(SettlementValidationError::NonAdmissibleFinality)
        );
    }

    #[test]
    fn exposure_limit_fails_closed() {
        let p = profile();
        let mut c = claim();
        c.amount_minor_units = p.max_exposure_minor_units + 1;
        assert_eq!(
            p.admits_high_assurance_claim(&c),
            Err(SettlementValidationError::ExposureExceeded)
        );
    }

    #[test]
    fn timestamp_order_is_enforced() {
        let mut c = claim();
        c.known_at_micros = c.observed_at_micros - 1;
        assert!(matches!(
            c.validate(),
            Err(SettlementValidationError::InvalidField {
                field: "known_at_micros",
                ..
            })
        ));
    }

    #[test]
    fn serde_roundtrip_is_stable() {
        let c = claim();
        let encoded = serde_json::to_string(&c).unwrap();
        let decoded: SettlementClaim = serde_json::from_str(&encoded).unwrap();
        assert_eq!(decoded, c);
    }

    #[test]
    fn empty_profile_ids_fail_closed() {
        let mut p = profile();
        p.rail_id.clear();
        assert!(p.validate().is_err());

        let mut p = profile();
        p.configuration_digest.clear();
        assert!(p.validate().is_err());
    }
}
