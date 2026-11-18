---------------- MODULE EvidenceAttestationEffectHandoffIdentityV1NegativeControls ----------------
EXTENDS EvidenceAttestationEffectHandoffIdentityV1

CONSTANT Control

NegativeCommit ==
  /\ Control # ""
  /\ attemptCommitted' = TRUE

NegativeNext == NegativeCommit

NegativeSpec ==
  Init /\ [][NegativeNext]_vars

=========================================================================
