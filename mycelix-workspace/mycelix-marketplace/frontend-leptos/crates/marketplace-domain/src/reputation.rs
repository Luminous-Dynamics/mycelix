use crate::{ActionHash, AgentPubKey, TimestampMicros};
use serde::{Deserialize, Serialize};

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReputationEventKind {
    FulfillmentDelivered,
    ArbitrationWon,
    ArbitrationLost,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReputationEvent {
    pub event_key: String,
    pub subject: AgentPubKey,
    pub counterparty: AgentPubKey,
    pub transaction_hash: ActionHash,
    pub source_hash: ActionHash,
    pub kind: ReputationEventKind,
    pub value_cents: u64,
    pub occurred_at: TimestampMicros,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReputationEventOutput {
    pub event_hash: ActionHash,
    pub event: ReputationEvent,
}

#[derive(Clone, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReputationEventsResponse {
    pub events: Vec<ReputationEventOutput>,
}

#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DerivedReputation {
    pub agent: AgentPubKey,
    pub score: f64,
    pub positive_events: u32,
    pub negative_events: u32,
    pub event_count: u32,
    pub fulfilled_value_cents: u64,
    pub arbitration_value_cents: u64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn event_kind_uses_stable_snake_case_wire_values() {
        assert_eq!(
            serde_json::to_string(&ReputationEventKind::FulfillmentDelivered).unwrap(),
            "\"fulfillment_delivered\""
        );
    }
}
