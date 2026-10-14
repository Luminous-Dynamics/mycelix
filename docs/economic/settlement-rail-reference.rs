//! Bounded executable witness for settlement-rail semantics.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RailState {
    Observed,
    Validated,
    Authorized,
    Executed,
    IncludedOrBooked,
    Confirmed,
    FinalityQualified,
    Reconciled,
    OutcomeObserved,
    Indeterminate,
    Corrected,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FinalityModel {
    Probabilistic,
    ConsensusFinalized,
    ProviderSpecific,
    LedgerPolicySpecific,
    NotASettlementRail,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RailReceipt {
    pub receipt_id: &'static str,
    pub source_event_id: &'static str,
    pub instrument_id: &'static str,
    pub rail_id: &'static str,
    pub state: RailState,
    pub finality: FinalityModel,
    pub authorization_id: Option<&'static str>,
    pub reconciliation_id: Option<&'static str>,
    pub claim_ceiling: &'static str,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Decision { Accepted, Rejected, DuplicateIdempotent }

pub fn can_advance(from: RailState, to: RailState, finality: FinalityModel) -> Decision {
    use RailState::*;
    let allowed = matches!(
        (from,to),
        (Observed,Validated) |
        (Validated,Authorized) |
        (Authorized,Executed) |
        (Executed,IncludedOrBooked) |
        (IncludedOrBooked,Confirmed) |
        (Confirmed,FinalityQualified) |
        (FinalityQualified,Reconciled) |
        (Reconciled,OutcomeObserved) |
        (Indeterminate,FinalityQualified) |
        (OutcomeObserved,Corrected)
    );
    if !allowed { return Decision::Rejected; }
    if to == FinalityQualified && finality == FinalityModel::NotASettlementRail {
        return Decision::Rejected;
    }
    Decision::Accepted
}

pub fn validate_receipt(r: &RailReceipt) -> Decision {
    if r.receipt_id.is_empty() || r.source_event_id.is_empty()
        || r.instrument_id.is_empty() || r.rail_id.is_empty()
        || r.claim_ceiling.is_empty() { return Decision::Rejected; }

    if matches!(r.state, RailState::FinalityQualified | RailState::Reconciled | RailState::OutcomeObserved)
        && r.authorization_id.is_none() { return Decision::Rejected; }

    if r.state == RailState::Reconciled && r.reconciliation_id.is_none() {
        return Decision::Rejected;
    }

    Decision::Accepted
}

pub fn same_logical_settlement(a: &RailReceipt, b: &RailReceipt) -> bool {
    a.source_event_id == b.source_event_id
        && a.instrument_id == b.instrument_id
        && a.rail_id == b.rail_id
        && a.authorization_id == b.authorization_id
}

#[cfg(test)]
mod tests {
    use super::*;

    fn receipt(state: RailState) -> RailReceipt {
        RailReceipt {
            receipt_id: "receipt:1",
            source_event_id: "event:1",
            instrument_id: "eip155:1/erc20:0xTOKEN",
            rail_id: "evm-token-transfer",
            state,
            finality: FinalityModel::ConsensusFinalized,
            authorization_id: Some("auth:1"),
            reconciliation_id: if state == RailState::Reconciled { Some("recon:1") } else { None },
            claim_ceiling: "qualified_rail_observation",
        }
    }

    #[test]
    fn provider_or_confirmation_does_not_skip_finality_profile() {
        assert_eq!(
            can_advance(RailState::Confirmed, RailState::FinalityQualified, FinalityModel::ProviderSpecific),
            Decision::Accepted
        );
    }

    #[test]
    fn accounting_projection_cannot_claim_finality() {
        assert_eq!(
            can_advance(RailState::Confirmed, RailState::FinalityQualified, FinalityModel::NotASettlementRail),
            Decision::Rejected
        );
    }

    #[test]
    fn reconciliation_requires_reconciliation_identity() {
        assert_eq!(validate_receipt(&receipt(RailState::Reconciled)), Decision::Accepted);
        let mut bad = receipt(RailState::Reconciled);
        bad.reconciliation_id = None;
        assert_eq!(validate_receipt(&bad), Decision::Rejected);
    }

    #[test]
    fn logical_duplicate_is_detectable() {
        let a = receipt(RailState::FinalityQualified);
        let b = receipt(RailState::Confirmed);
        assert!(same_logical_settlement(&a, &b));
    }

    #[test]
    fn state_shortcut_is_rejected() {
        assert_eq!(
            can_advance(RailState::Observed, RailState::Reconciled, FinalityModel::ConsensusFinalized),
            Decision::Rejected
        );
    }
}