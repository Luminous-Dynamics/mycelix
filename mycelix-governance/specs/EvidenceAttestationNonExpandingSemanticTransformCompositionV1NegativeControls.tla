---------------- MODULE EvidenceAttestationNonExpandingSemanticTransformCompositionV1NegativeControls ----------------
EXTENDS EvidenceAttestationNonExpandingSemanticTransformCompositionV1

CONSTANT Control

NegativeCommit ==
  /\ Control # ""
  /\ committed' = TRUE

NegativeNext == NegativeCommit

NegativeSpec ==
  Init /\ [][NegativeNext]_vars

=========================================================================
