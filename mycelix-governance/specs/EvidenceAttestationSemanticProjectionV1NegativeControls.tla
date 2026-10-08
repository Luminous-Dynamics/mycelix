---------------- MODULE EvidenceAttestationSemanticProjectionV1NegativeControls ----------------
EXTENDS EvidenceAttestationSemanticProjectionV1
CONSTANT Control
NegativeCommit ==
  /\ Control # ""
  /\ projectionCommitted' = TRUE
NegativeNext == NegativeCommit
NegativeSpec == Init /\ [][NegativeNext]_vars
=========================================================================
