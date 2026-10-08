---------------- MODULE EvidenceAttestationFullDimensionalCompositionV1 ----------------
EXTENDS Naturals, FiniteSets

CONSTANTS Claim1, Claim2, Claim3, Claim4,
          R1, R2,
          Read, Write,
          AudienceA, AudienceB,
          T1, T2

Claims == {Claim1, Claim2, Claim3, Claim4}
Resources == {R1, R2}
Actions == {Read, Write}
Audiences == {AudienceA, AudienceB}
Times == {T1, T2}

Capability1 == [resource |-> R1, action |-> Read,  audience |-> AudienceA, expiry |-> T1]
Capability2 == [resource |-> R2, action |-> Write, audience |-> AudienceA, expiry |-> T2]
Capability3 == [resource |-> R1, action |-> Write, audience |-> AudienceB, expiry |-> T2]
Capability4 == [resource |-> R2, action |-> Read,  audience |-> AudienceB, expiry |-> T1]

HybridCapability == [resource |-> R1, action |-> Write, audience |-> AudienceB, expiry |-> T1]

ClaimCapabilities(c) ==
  IF c = Claim1 THEN {Capability1}
  ELSE IF c = Claim2 THEN {Capability2}
  ELSE IF c = Claim3 THEN {Capability3}
  ELSE IF c = Claim4 THEN {Capability4}
  ELSE {}

CapabilityUniverse ==
  { [resource |-> r, action |-> a, audience |-> u, expiry |-> t] :
      r \in Resources, a \in Actions, u \in Audiences, t \in Times }

ClaimAuthorized(c) == c \in Claims

ClaimGrantBacked(c) == c \in Claims

ClaimSignerTrusted(c) == c \in Claims

ClaimSubject(c) == "Alice"

ClaimTarget(c) == "Alice"

ExpectedAuthority(input) ==
  UNION {ClaimCapabilities[c] : c \in input}

ExpectedContributors(input) ==
  [cap \in CapabilityUniverse |->
    {c \in input : cap \in ClaimCapabilities[c]}]

ClaimsForResource(input, r) ==
  {c \in input : \E cap \in ClaimCapabilities[c] : cap.resource = r}

ClaimsForAction(input, a) ==
  {c \in input : \E cap \in ClaimCapabilities[c] : cap.action = a}

ClaimsForAudience(input, u) ==
  {c \in input : \E cap \in ClaimCapabilities[c] : cap.audience = u}

ClaimsForExpiry(input, t) ==
  {c \in input : \E cap \in ClaimCapabilities[c] : cap.expiry = t}

ExpectedDimensionSources(input) ==
  [cap \in CapabilityUniverse |->
    [resource |-> ClaimsForResource(input, cap.resource),
     action |-> ClaimsForAction(input, cap.action),
     audience |-> ClaimsForAudience(input, cap.audience),
     expiry |-> ClaimsForExpiry(input, cap.expiry)]]

EmptyContributors ==
  [cap \in CapabilityUniverse |-> {}]

EmptyDimensionSources ==
  [cap \in CapabilityUniverse |->
    [resource |-> {}, action |-> {}, audience |-> {}, expiry |-> {}]]

VARIABLES inputClaims, composedAuthority, contributorClaims, dimensionContributors

vars == <<inputClaims, composedAuthority, contributorClaims, dimensionContributors>>

Init ==
  /\ inputClaims = Claims
  /\ composedAuthority = {}
  /\ contributorClaims = EmptyContributors
  /\ dimensionContributors = EmptyDimensionSources

ComposeAtomic ==
  /\ composedAuthority' = ExpectedAuthority(inputClaims)
  /\ contributorClaims' = ExpectedContributors(inputClaims)
  /\ dimensionContributors' = ExpectedDimensionSources(inputClaims)
  /\ UNCHANGED inputClaims

Next ==
  ComposeAtomic

TypeOK ==
  /\ inputClaims \subseteq Claims
  /\ composedAuthority \subseteq CapabilityUniverse
  /\ contributorClaims \in [CapabilityUniverse -> SUBSET Claims]
  /\ dimensionContributors \in
       [CapabilityUniverse ->
          [resource : SUBSET Claims,
           action : SUBSET Claims,
           audience : SUBSET Claims,
           expiry : SUBSET Claims]]

AllInputClaimsValid ==
  \A c \in inputClaims :
    ClaimAuthorized[c] /\
    ClaimGrantBacked[c] /\
    ClaimSignerTrusted[c] /\
    ClaimSubject[c] = ClaimTarget[c]

DimensionSourcesRemainIndividuallyValid ==
  \A cap \in composedAuthority :
    /\ dimensionContributors[cap].resource =
           ClaimsForResource(inputClaims, cap.resource)
    /\ dimensionContributors[cap].action =
           ClaimsForAction(inputClaims, cap.action)
    /\ dimensionContributors[cap].audience =
           ClaimsForAudience(inputClaims, cap.audience)
    /\ dimensionContributors[cap].expiry =
           ClaimsForExpiry(inputClaims, cap.expiry)

CompositionAtomAndProvenanceExact ==
  \A cap \in composedAuthority :
    /\ cap \in ExpectedAuthority(inputClaims)
    /\ contributorClaims[cap] =
           ExpectedContributors(inputClaims)[cap]
    /\ contributorClaims[cap] # {}

=========================================================================
