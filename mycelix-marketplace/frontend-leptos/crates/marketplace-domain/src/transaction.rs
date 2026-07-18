use crate::{ActionHash, AgentPubKey, EpistemicClassification, TimestampMicros};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum TransactionStatus {
    Pending,
    Confirmed,
    Shipped,
    Delivered,
    Completed,
    Disputed,
    Cancelled,
}

impl TransactionStatus {
    pub const fn label(&self) -> &'static str {
        match self {
            Self::Pending => "Pending",
            Self::Confirmed => "Confirmed",
            Self::Shipped => "Shipped",
            Self::Delivered => "Delivered",
            Self::Completed => "Completed",
            Self::Disputed => "Disputed",
            Self::Cancelled => "Cancelled",
        }
    }

    pub const fn is_terminal(&self) -> bool {
        matches!(self, Self::Completed | Self::Disputed | Self::Cancelled)
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Transaction {
    pub buyer: AgentPubKey,
    pub seller: AgentPubKey,
    pub listing_hash: ActionHash,
    pub quantity: u32,
    pub total_price_cents: u64,
    pub status: TransactionStatus,
    pub created_at: TimestampMicros,
    pub updated_at: TimestampMicros,
    pub tracking_info: Option<String>,
    pub epistemic: EpistemicClassification,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TransactionOutput {
    pub transaction_hash: ActionHash,
    pub transaction: Transaction,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct TransactionsResponse {
    pub transactions: Vec<TransactionOutput>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TransactionResolutionState {
    Resolved,
    AutoResolved,
    AuthorizedResolved,
    Conflicted,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TransactionResolutionReason {
    SingleHead,
    CancellationDominatesPreShipment,
    DisputeDominatesLifecycle,
    BilateralAgreement,
    ArbitrationAward,
    ConvergentExplicitAuthorities,
    ConflictingExplicitAuthorities,
    UnsafeConcurrentLifecycle,
}

impl TransactionResolutionReason {
    pub const fn label(&self) -> &'static str {
        match self {
            Self::SingleHead => "Single authored head",
            Self::CancellationDominatesPreShipment => {
                "Cancellation dominates pre-shipment progression"
            }
            Self::DisputeDominatesLifecycle => "Dispute halts ordinary lifecycle progression",
            Self::BilateralAgreement => "Buyer and seller authorized one branch",
            Self::ArbitrationAward => "Arbitration authorized one branch",
            Self::ConvergentExplicitAuthorities => "Independent authorities selected the same branch",
            Self::ConflictingExplicitAuthorities => "Authority records disagree on the selected branch",
            Self::UnsafeConcurrentLifecycle => "Unsafe concurrent lifecycle evidence",
        }
    }

    pub const fn explanation(&self) -> &'static str {
        match self {
            Self::SingleHead => "Only one live transaction revision is visible.",
            Self::CancellationDominatesPreShipment => {
                "One authored cancellation safely dominates only pending or confirmed branches."
            }
            Self::DisputeDominatesLifecycle => {
                "One authored dispute halts ordinary progression without deciding fault or compensation."
            }
            Self::BilateralAgreement => {
                "Buyer and seller independently approved the same existing branch and exact head set."
            }
            Self::ArbitrationAward => {
                "A conflict-bound arbitration result selected the unique branch authored by the winner."
            }
            Self::ConvergentExplicitAuthorities => {
                "Multiple valid authority records independently selected the same current branch."
            }
            Self::ConflictingExplicitAuthorities => {
                "Valid authority records select different branches, so the transaction remains halted."
            }
            Self::UnsafeConcurrentLifecycle => {
                "No narrow safety rule can select an authored head; explicit resolution is required."
            }
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TransactionConflictApproval {
    pub protocol_version: u16,
    pub root_transaction_hash: ActionHash,
    pub head_hashes: Vec<ActionHash>,
    pub selected_head_hash: ActionHash,
    pub approver: AgentPubKey,
    pub rationale: String,
    pub created_at: TimestampMicros,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TransactionConflictApprovalOutput {
    pub approval_hash: ActionHash,
    pub approval: TransactionConflictApproval,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TransactionConflictAuthority {
    Bilateral {
        buyer_approval_hash: ActionHash,
        seller_approval_hash: ActionHash,
    },
    Arbitration {
        dispute_hash: ActionHash,
        result_hash: ActionHash,
    },
}

impl TransactionConflictAuthority {
    pub const fn label(&self) -> &'static str {
        match self {
            Self::Bilateral { .. } => "Bilateral agreement",
            Self::Arbitration { .. } => "Arbitration award",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TransactionConflictResolutionEntry {
    pub protocol_version: u16,
    pub root_transaction_hash: ActionHash,
    pub head_hashes: Vec<ActionHash>,
    pub selected_head_hash: ActionHash,
    pub authority: TransactionConflictAuthority,
    pub summary: String,
    pub created_at: TimestampMicros,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TransactionConflictResolutionOutput {
    pub resolution_hash: ActionHash,
    pub resolution: TransactionConflictResolutionEntry,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AppliedTransactionConflictResolution {
    pub resolution_hash: ActionHash,
    pub protocol_version: u16,
    pub selected_head_hash: ActionHash,
    pub bound_head_hashes: Vec<ActionHash>,
    pub authority: TransactionConflictAuthority,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TransactionResolution {
    pub root_transaction_hash: ActionHash,
    pub policy_version: u16,
    pub state: TransactionResolutionState,
    pub reason: TransactionResolutionReason,
    pub canonical: Option<TransactionOutput>,
    pub heads: Vec<TransactionOutput>,
    pub superseded_heads: Vec<TransactionOutput>,
    pub applied_conflict_resolutions: Vec<AppliedTransactionConflictResolution>,
    pub revision_count: u32,
}

impl TransactionResolution {
    pub fn current(&self) -> Option<&TransactionOutput> {
        match self.state {
            TransactionResolutionState::Resolved
            | TransactionResolutionState::AutoResolved
            | TransactionResolutionState::AuthorizedResolved => self.canonical.as_ref(),
            TransactionResolutionState::Conflicted => None,
        }
    }

    pub const fn is_conflicted(&self) -> bool {
        matches!(self.state, TransactionResolutionState::Conflicted)
    }

    pub const fn is_auto_resolved(&self) -> bool {
        matches!(self.state, TransactionResolutionState::AutoResolved)
    }

    pub const fn is_authorized_resolved(&self) -> bool {
        matches!(self.state, TransactionResolutionState::AuthorizedResolved)
    }
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct TransactionResolutionsResponse {
    pub resolutions: Vec<TransactionResolution>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum TransactionSettlementState {
    Unavailable,
    NotFound,
    Pending,
    Processing,
    Completed,
    Failed,
    Refunded,
    Disputed,
    Invalid,
}

impl TransactionSettlementState {
    pub const fn label(&self) -> &'static str {
        match self {
            Self::Unavailable => "Finance unavailable",
            Self::NotFound => "Not settled",
            Self::Pending => "Pending",
            Self::Processing => "Processing",
            Self::Completed => "Completed",
            Self::Failed => "Failed",
            Self::Refunded => "Refunded",
            Self::Disputed => "Disputed",
            Self::Invalid => "Invalid evidence",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TransactionSettlementResult {
    pub root_transaction_hash: ActionHash,
    pub transaction_revision_hash: ActionHash,
    pub state: TransactionSettlementState,
    pub settled: bool,
    pub idempotency_reference: String,
    pub finance_payment_id: Option<String>,
    pub finance_action_hash: Option<ActionHash>,
    pub error: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ApproveTransactionConflictInput {
    pub transaction_hash: ActionHash,
    pub selected_head_hash: ActionHash,
    pub rationale: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FinalizeBilateralTransactionConflictInput {
    pub transaction_hash: ActionHash,
    pub buyer_approval_hash: ActionHash,
    pub seller_approval_hash: ActionHash,
    pub summary: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ApplyArbitrationTransactionConflictInput {
    pub transaction_hash: ActionHash,
    pub dispute_hash: ActionHash,
    pub result_hash: ActionHash,
    pub summary: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CreateTransactionInput {
    pub seller: AgentPubKey,
    pub listing_hash: ActionHash,
    pub quantity: u32,
    pub total_price_cents: u64,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct MarkShippedInput {
    pub transaction_hash: ActionHash,
    pub tracking_info: Option<String>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DisputeTransactionInput {
    pub transaction_hash: ActionHash,
    pub reason: String,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn action_hash(byte: u8) -> ActionHash {
        ActionHash::new(vec![byte; 39])
    }

    #[test]
    fn transaction_status_matches_integrity_wire_format() {
        assert_eq!(
            serde_json::to_string(&TransactionStatus::Pending).unwrap(),
            "\"pending\""
        );
        assert_eq!(
            serde_json::from_str::<TransactionStatus>("\"completed\"").unwrap(),
            TransactionStatus::Completed
        );
    }

    #[test]
    fn conflict_state_never_exposes_a_canonical_revision() {
        let resolution = TransactionResolution {
            root_transaction_hash: action_hash(1),
            policy_version: 2,
            state: TransactionResolutionState::Conflicted,
            reason: TransactionResolutionReason::UnsafeConcurrentLifecycle,
            canonical: None,
            heads: Vec::new(),
            superseded_heads: Vec::new(),
            applied_conflict_resolutions: Vec::new(),
            revision_count: 2,
        };
        assert!(resolution.is_conflicted());
        assert!(resolution.current().is_none());
    }

    #[test]
    fn auto_resolved_state_exposes_the_existing_canonical_head() {
        let transaction = TransactionOutput {
            transaction_hash: action_hash(2),
            transaction: Transaction {
                buyer: AgentPubKey::new(vec![1; 39]),
                seller: AgentPubKey::new(vec![2; 39]),
                listing_hash: action_hash(3),
                quantity: 1,
                total_price_cents: 100,
                status: TransactionStatus::Cancelled,
                created_at: TimestampMicros(1),
                updated_at: TimestampMicros(2),
                tracking_info: None,
                epistemic: EpistemicClassification {
                    empirical: crate::EmpiricalLevel::E1Testimonial,
                    normative: crate::NormativeLevel::N1Communal,
                    materiality: crate::MaterialityLevel::M1Temporal,
                },
            },
        };
        let resolution = TransactionResolution {
            root_transaction_hash: action_hash(1),
            policy_version: 2,
            state: TransactionResolutionState::AutoResolved,
            reason: TransactionResolutionReason::CancellationDominatesPreShipment,
            canonical: Some(transaction),
            heads: Vec::new(),
            superseded_heads: Vec::new(),
            applied_conflict_resolutions: Vec::new(),
            revision_count: 3,
        };
        assert!(resolution.is_auto_resolved());
        assert_eq!(
            resolution.current().unwrap().transaction.status,
            TransactionStatus::Cancelled
        );
    }

    #[test]
    fn authorized_resolution_exposes_authored_canonical_head_and_evidence() {
        let selected = action_hash(8);
        let transaction = TransactionOutput {
            transaction_hash: selected.clone(),
            transaction: Transaction {
                buyer: AgentPubKey::new(vec![1; 39]),
                seller: AgentPubKey::new(vec![2; 39]),
                listing_hash: action_hash(3),
                quantity: 1,
                total_price_cents: 100,
                status: TransactionStatus::Shipped,
                created_at: TimestampMicros(1),
                updated_at: TimestampMicros(2),
                tracking_info: Some("tracking".into()),
                epistemic: EpistemicClassification {
                    empirical: crate::EmpiricalLevel::E1Testimonial,
                    normative: crate::NormativeLevel::N1Communal,
                    materiality: crate::MaterialityLevel::M1Temporal,
                },
            },
        };
        let resolution = TransactionResolution {
            root_transaction_hash: action_hash(1),
            policy_version: 2,
            state: TransactionResolutionState::AuthorizedResolved,
            reason: TransactionResolutionReason::BilateralAgreement,
            canonical: Some(transaction),
            heads: Vec::new(),
            superseded_heads: Vec::new(),
            applied_conflict_resolutions: vec![AppliedTransactionConflictResolution {
                resolution_hash: action_hash(9),
                protocol_version: 1,
                selected_head_hash: selected,
                bound_head_hashes: vec![action_hash(7), action_hash(8)],
                authority: TransactionConflictAuthority::Bilateral {
                    buyer_approval_hash: action_hash(10),
                    seller_approval_hash: action_hash(11),
                },
            }],
            revision_count: 3,
        };
        assert!(resolution.is_authorized_resolved());
        assert_eq!(
            resolution.current().unwrap().transaction.status,
            TransactionStatus::Shipped
        );
        assert_eq!(resolution.applied_conflict_resolutions.len(), 1);
    }

    #[test]
    fn resolution_state_uses_snake_case_wire_values() {
        assert_eq!(
            serde_json::to_string(&TransactionResolutionState::AutoResolved).unwrap(),
            "\"auto_resolved\""
        );
        assert_eq!(
            serde_json::to_string(
                &TransactionResolutionReason::CancellationDominatesPreShipment
            )
            .unwrap(),
            "\"cancellation_dominates_pre_shipment\""
        );
    }
}
