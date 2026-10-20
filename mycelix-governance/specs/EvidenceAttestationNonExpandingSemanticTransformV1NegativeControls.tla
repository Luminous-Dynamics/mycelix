---------------- MODULE EvidenceAttestationNonExpandingSemanticTransformV1NegativeControls ----------------
EXTENDS EvidenceAttestationNonExpandingSemanticTransformV1

CONSTANT Control

NegativeCommit ==
  /\ Control # ""
  /\ committed' = TRUE

NegativeNext == NegativeCommit

NegativeSpec ==
  Init /\ [][NegativeNext]_vars

=========================================================================
