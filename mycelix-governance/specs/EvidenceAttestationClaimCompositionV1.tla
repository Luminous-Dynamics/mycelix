---------------- MODULE EvidenceAttestationClaimCompositionV1 ----------------
EXTENDS Naturals, FiniteSets

CONSTANTS R1, R2,
          Read, Write,
          Claim1, Claim2,
          C1, C2

Resources == {R1, R2}
Actions == {Read, Write}
Claims == {Claim1, Claim2}

Capability1 == [resource |-> R1, action |-> Read]
Capability2 == [resource |-> R2, action |-> Write]

ClaimCapabilities ==
  [Claim1 |-> {Capability1},
   Claim2 |-> {Capability2}]

ClaimAuthorized ==
  [Claim1 |-> TRUE,
   Claim2 |-> TRUE]

ClaimGrantBacked ==
  [Claim1 |-> TRUE,
   Claim2 |-> TRUE]

VARIABLES authorizedClaims, composedAuthority

vars == <<authorizedClaims, composedAuthority>>

AuthorizedCapabilities ==
  UNION {ClaimCapabilities[c] : c \in authorizedClaims}

ClaimResourceUnion ==
  {c.resource : c \in AuthorizedCapabilities}

ClaimActionUnion ==
  {c.action : c \in AuthorizedCapabilities}

CartesianCapabilities ==
  { [resource |-> r, action |-> a] :
      r \in ClaimResourceUnion, a \in ClaimActionUnion }

Init ==
  /\ authorizedClaims = Claims
  /\ composedAuthority = {}

ComposeClaims ==
  /\ authorizedClaims # {}
  /\ composedAuthority' = AuthorizedCapabilities
  /\ UNCHANGED authorizedClaims

Next ==
  ComposeClaims

TypeOK ==
  /\ authorizedClaims \subseteq Claims
  /\ composedAuthority \subseteq
       { [resource |-> r, action |-> a] :
           r \in Resources, a \in Actions }

AllAuthorizedClaimsGrantBacked ==
  \A c \in authorizedClaims :
    ClaimAuthorized[c] /\ ClaimGrantBacked[c]

CompositionOnlyUsesAtomicAuthorizedCapabilities ==
  composedAuthority \subseteq AuthorizedCapabilities

=========================================================================
