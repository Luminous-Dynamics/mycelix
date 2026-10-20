---------------- MODULE EvidenceAttestationDecisionEffectBindingV1NegativeControls ----------------
EXTENDS EvidenceAttestationDecisionEffectBindingV1

CONSTANT Control

NegativeNext ==
  /\ Control # ""
  /\ admitted' = TRUE

NegativeSpec ==
  Init /\ [][NegativeNext]_vars

=========================================================================
