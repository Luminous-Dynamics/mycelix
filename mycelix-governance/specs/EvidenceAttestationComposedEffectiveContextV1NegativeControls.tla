---------------- MODULE EvidenceAttestationComposedEffectiveContextV1NegativeControls ----------------
EXTENDS EvidenceAttestationComposedEffectiveContextV1

CONSTANT Control

BadAuthorizeByDimension ==
  /\ Control = "contextual-laundering"
  /\ composedAuthority' = ExpectedAuthority(inputClaims)
  /\ contributorClaims' =
       [cap \in CapabilityUniverse |->
          ExpectedContributors(inputClaims, cap)]
  /\ resourceContributors' =
       ClaimsForResource(inputClaims, RequestResource)
  /\ actionContributors' =
       ClaimsForAction(inputClaims, RequestAction)
  /\ audienceContributors' =
       ClaimsForAudience(inputClaims, RequestAudience)
  /\ temporalContributors' =
       ClaimsCurrentlyEffective(inputClaims, EvalNow)
  /\ decisionAuthorized' = TRUE
  /\ decisionContributors' =
       resourceContributors'
       \cup actionContributors'
       \cup audienceContributors'
       \cup temporalContributors'
  /\ UNCHANGED inputClaims

NegativeNext ==
  BadAuthorizeByDimension

NegativeSpec ==
  Init /\ [][NegativeNext]_vars

=========================================================================
