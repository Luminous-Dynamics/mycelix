---------------- MODULE EvidenceAttestationFullDimensionalCompositionV1NegativeControls ----------------
EXTENDS EvidenceAttestationFullDimensionalCompositionV1

CONSTANT Control

HybridDimensionContributors ==
  [resource |-> {Claim1, Claim3},
   action |-> {Claim2, Claim3},
   audience |-> {Claim3, Claim4},
   expiry |-> {Claim1, Claim4}]

BadComposeHybrid ==
  /\ Control = "hybrid-synthesis"
  /\ composedAuthority' =
       ExpectedAuthority(inputClaims) \cup {HybridCapability}
  /\ contributorClaims' =
       [ExpectedContributors(inputClaims) EXCEPT
          ![HybridCapability] = {Claim1, Claim2, Claim3, Claim4}]
  /\ dimensionContributors' =
       [ExpectedDimensionSources(inputClaims) EXCEPT
          ![HybridCapability] = HybridDimensionContributors]
  /\ UNCHANGED inputClaims

NegativeNext ==
  BadComposeHybrid

NegativeSpec ==
  Init /\ [][NegativeNext]_vars

=========================================================================
