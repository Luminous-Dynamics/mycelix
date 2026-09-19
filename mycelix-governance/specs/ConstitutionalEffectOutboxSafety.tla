--------------------- MODULE ConstitutionalEffectOutboxSafety ---------------------
EXTENDS ConstitutionalEffectOutbox

UnknownOutcomeBlocksRetry ==
    \A op \in Ops : phase[op] = "UnknownOutcome" =>
        /\ worker = NoOp
        /\ inFlight = NoOp

=============================================================================
