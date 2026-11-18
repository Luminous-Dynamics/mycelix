---------------- MODULE EvidenceAttestationClaimCompositionV1NegativeControls ----------------
EXTENDS EvidenceAttestationClaimCompositionV1

CONSTANT Control

BadComposition ==
  /\ Control = "composition-amplification"
  /\ authorizedClaims = Claims
  /\ composedAuthority' = CartesianCapabilities
  /\ UNCHANGED authorizedClaims

NegativeNext == BadComposition

NegativeSpec == Init /\ [][NegativeNext]_vars
=========================================================================
