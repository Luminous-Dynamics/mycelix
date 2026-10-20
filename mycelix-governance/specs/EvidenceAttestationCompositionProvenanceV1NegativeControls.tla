---------------- MODULE EvidenceAttestationCompositionProvenanceV1NegativeControls ----------------
EXTENDS EvidenceAttestationCompositionProvenanceV1

CONSTANT Control

BadContributorSubstitution ==
  /\ Control = "contributor-substitution"
  /\ inputClaims = {Claim1, Claim2}
  /\ composedAuthority' = {Capability1, Capability2}
  /\ contributorClaims' =
       [Capability1 |-> {Claim1},
        Capability2 |-> {Claim3}]
  /\ UNCHANGED inputClaims

NegativeInit ==
  /\ inputClaims = {Claim1, Claim2}
  /\ composedAuthority = ExpectedAuthority
  /\ contributorClaims = ExpectedContributors

NegativeNext == BadContributorSubstitution
NegativeSpec == NegativeInit /\ [][NegativeNext]_vars
=========================================================================
