use mycelix_integration_core::OutboundStage;

#[test]
fn denial_and_before_commit_rejection_are_distinct_terminal_no_effect_paths() {
    assert!(
        OutboundStage::AuthorityChecked.allows_transition_to(OutboundStage::AuthorityDenied)
    );
    assert!(!OutboundStage::AuthorityDenied.allows_transition_to(OutboundStage::Reconciled));
    assert!(OutboundStage::AuthorityDenied.allows_transition_to(OutboundStage::Finalized));
    assert!(!OutboundStage::AuthorityDenied.allows_transition_to(OutboundStage::OutboxCommitted));

    assert!(
        OutboundStage::DispatchStarted
            .allows_transition_to(OutboundStage::RejectedBeforeCommit)
    );
    assert!(
        !OutboundStage::RejectedBeforeCommit.allows_transition_to(OutboundStage::Reconciled)
    );
    assert!(
        OutboundStage::RejectedBeforeCommit.allows_transition_to(OutboundStage::Finalized)
    );
    assert!(
        !OutboundStage::RejectedBeforeCommit.allows_transition_to(OutboundStage::DispatchStarted)
    );

    // Effect-bearing or uncertain outcomes still require reconciliation.
    assert!(OutboundStage::Confirmed.allows_transition_to(OutboundStage::Reconciled));
    assert!(OutboundStage::Ambiguous.allows_transition_to(OutboundStage::Reconciled));
    assert!(!OutboundStage::Confirmed.allows_transition_to(OutboundStage::Finalized));
    assert!(!OutboundStage::Ambiguous.allows_transition_to(OutboundStage::Finalized));
}
