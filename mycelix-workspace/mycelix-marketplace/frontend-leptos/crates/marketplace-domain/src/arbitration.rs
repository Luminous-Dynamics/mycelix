use crate::{ActionHash, AgentPubKey, TimestampMicros, TransactionOutput};
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum DisputeStatus {
    Filed,
    UnderReview,
    Voting,
    ResolvedBuyer,
    ResolvedSeller,
    Withdrawn,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct Dispute {
    /// Stable transaction identity: the original Transaction Create action.
    pub transaction_hash: ActionHash,
    /// Exact transaction revision that was Disputed when the case was filed.
    pub transaction_revision_hash: ActionHash,
    /// Exact unsafe transaction heads for a conflict-bound arbitration case.
    #[serde(default)]
    pub conflict_heads: Vec<ActionHash>,
    pub filed_by: AgentPubKey,
    pub buyer: AgentPubKey,
    pub seller: AgentPubKey,
    pub reason: String,
    pub evidence_cids: Vec<String>,
    pub status: DisputeStatus,
    pub arbitrators: Vec<AgentPubKey>,
    pub result_hash: Option<ActionHash>,
    pub created_at: TimestampMicros,
    pub updated_at: TimestampMicros,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DisputeOutput {
    /// Current Create/Update action hash. Use the resolution root for stable URLs.
    pub dispute_hash: ActionHash,
    pub dispute: Dispute,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct DisputesResponse {
    pub disputes: Vec<DisputeOutput>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum DisputeResolutionState {
    Resolved,
    Conflicted,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct DisputeResolution {
    pub root_dispute_hash: ActionHash,
    pub state: DisputeResolutionState,
    pub canonical: Option<DisputeOutput>,
    pub heads: Vec<DisputeOutput>,
    pub revision_count: u32,
}

impl DisputeResolution {
    pub fn current(&self) -> Option<&DisputeOutput> {
        match self.state {
            DisputeResolutionState::Resolved => self.canonical.as_ref(),
            DisputeResolutionState::Conflicted => None,
        }
    }

    pub const fn is_conflicted(&self) -> bool {
        matches!(self.state, DisputeResolutionState::Conflicted)
    }
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ArbitrationVote {
    pub dispute_hash: ActionHash,
    pub dispute_revision_hash: ActionHash,
    pub arbitrator: AgentPubKey,
    pub favor_buyer: bool,
    pub reasoning: String,
    /// Guarded v1 accepts only 1.0 until score snapshots are integrity-verifiable.
    pub arbitrator_matl_score: f64,
    pub voted_at: TimestampMicros,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ArbitrationVoteOutput {
    pub vote_hash: ActionHash,
    pub vote: ArbitrationVote,
}

#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ArbitrationVotesResponse {
    pub votes: Vec<ArbitrationVoteOutput>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ArbitrationResult {
    pub dispute_hash: ActionHash,
    pub dispute_revision_hash: ActionHash,
    pub vote_hashes: Vec<ActionHash>,
    pub winner: AgentPubKey,
    pub loser: AgentPubKey,
    /// Equal-weight buyer-vote ratio, retained under the historical wire name.
    pub weighted_vote: f64,
    pub total_votes: u32,
    pub compensation_cents: Option<u64>,
    pub summary: String,
    pub finalized_at: TimestampMicros,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ArbitrationResultOutput {
    pub result_hash: ActionHash,
    pub result: ArbitrationResult,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FileDisputeInput {
    pub transaction_hash: ActionHash,
    pub reason: String,
    pub evidence_cids: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SubmitArbitrationVoteInput {
    pub dispute_hash: ActionHash,
    pub favor_buyer: bool,
    pub reasoning: String,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct OpenDisputeInput {
    pub transaction_hash: ActionHash,
    pub reason: String,
    pub evidence_cids: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct OpenDisputeOutput {
    pub transaction: TransactionOutput,
    pub dispute_hash: ActionHash,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn action_hash(byte: u8) -> ActionHash {
        ActionHash::new(vec![byte; 39])
    }

    #[test]
    fn arbitration_states_match_lowercase_wire_format() {
        assert_eq!(
            serde_json::to_string(&DisputeStatus::UnderReview).unwrap(),
            "\"underreview\""
        );
        assert_eq!(
            serde_json::to_string(&DisputeResolutionState::Conflicted).unwrap(),
            "\"conflicted\""
        );
    }

    #[test]
    fn conflicted_dispute_never_exposes_a_current_revision() {
        let resolution = DisputeResolution {
            root_dispute_hash: action_hash(1),
            state: DisputeResolutionState::Conflicted,
            canonical: None,
            heads: Vec::new(),
            revision_count: 2,
        };
        assert!(resolution.is_conflicted());
        assert!(resolution.current().is_none());
    }
}
