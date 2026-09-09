use mycelix_integration_core::OutboundStage;

#[test]
fn rejected_is_a_terminal_no_effect_or_denial_stage() {
    assert!(!OutboundStage::Rejected.allows_transition_to(OutboundStage::Reconciled));
    assert!(!OutboundStage::Rejected.allows_transition_to(OutboundStage::Finalized));
    assert!(!OutboundStage::Rejected.allows_transition_to(OutboundStage::DispatchStarted));

    // Effect-bearing or uncertain outcomes still use reconciliation.
    assert!(OutboundStage::Confirmed.allows_transition_to(OutboundStage::Reconciled));
    assert!(OutboundStage::Ambiguous.allows_transition_to(OutboundStage::Reconciled));
}
