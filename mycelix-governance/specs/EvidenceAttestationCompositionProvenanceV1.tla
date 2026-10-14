---------------- MODULE EvidenceAttestationCompositionProvenanceV1 ----------------
EXTENDS Naturals, FiniteSets

CONSTANTS Claim1, Claim2, Claim3,
          R1, R2,
          Read, Write

Claims == {Claim1, Claim2, Claim3}

Capability1 == [resource |-> R1, action |-> Read]
Capability2 == [resource |-> R2, action |-> Write]

ClaimCapabilities ==
  [c \in Claims |->
    CASE
      c = Claim1 -> {Capability1}
      [] c = Claim2 -> {Capability2}
      [] c = Claim3 -> {Capability2}]

ClaimAuthorized ==
  [c \in Claims |-> TRUE]

ClaimGrantBacked ==
  [c \in Claims |-> TRUE]

VARIABLES inputClaims, composedAuthority, contributorClaims

vars == <<inputClaims, composedAuthority, contributorClaims>>

ExpectedAuthority ==
  UNION {ClaimCapabilities[c] : c \in inputClaims}

ExpectedContributors ==
  [cap \in ExpectedAuthority |->
    {c \in inputClaims : cap \in ClaimCapabilities[c]}]

Init ==
  /\ inputClaims = {Claim1, Claim2}
  /\ composedAuthority = {}
  /\ contributorClaims =
       [cap \in {Capability1, Capability2} |-> {}]

ComposeClaims ==
  /\ inputClaims # {}
  /\ composedAuthority' = ExpectedAuthority
  /\ contributorClaims' = ExpectedContributors
  /\ UNCHANGED inputClaims

Next ==
  ComposeClaims

TypeOK ==
  /\ inputClaims \subseteq Claims
  /\ composedAuthority \subseteq {Capability1, Capability2}
  /\ contributorClaims \in
       [{Capability1, Capability2} -> SUBSET Claims]

AllInputClaimsValid ==
  \A c \in inputClaims :
    ClaimAuthorized[c] /\ ClaimGrantBacked[c]

AuthorityComesFromInputClaims ==
  composedAuthority \subseteq ExpectedAuthority

ContributorsAuthorizeTheirCapabilities ==
  \A cap \in composedAuthority :
    \A c \in contributorClaims[cap] :
      cap \in ClaimCapabilities[c] /\
      ClaimAuthorized[c] /\ ClaimGrantBacked[c]

CompositionProvenanceMatchesInputs ==
  contributorClaims = ExpectedContributors

=========================================================================
