---------------- MODULE EvidenceAttestationComposedEffectiveContextV1 ----------------
EXTENDS Naturals, FiniteSets

CONSTANTS Claim1, Claim2, Claim3, Claim4,
          R1, R2,
          Read, Write,
          AudienceA, AudienceB,
          ExpiryShort, ExpiryLong,
          EvalNow,
          RequestResource, RequestAction, RequestAudience

Claims == {Claim1, Claim2, Claim3, Claim4}
Resources == {R1, R2}
Actions == {Read, Write}
Audiences == {AudienceA, AudienceB}
Times == {ExpiryShort, ExpiryLong}

Capability1 == [resource |-> R1, action |-> Read,  audience |-> AudienceA, expiry |-> ExpiryShort]
Capability2 == [resource |-> R2, action |-> Write, audience |-> AudienceA, expiry |-> ExpiryLong]
Capability3 == [resource |-> R1, action |-> Write, audience |-> AudienceB, expiry |-> ExpiryLong]
Capability4 == [resource |-> R2, action |-> Read,  audience |-> AudienceB, expiry |-> ExpiryShort]

ClaimCapabilities ==
  [Claim1 |-> {Capability1},
   Claim2 |-> {Capability2},
   Claim3 |-> {Capability3},
   Claim4 |-> {Capability4}]

CapabilityUniverse ==
  { [resource |-> r, action |-> a, audience |-> u, expiry |-> t] :
      r \in Resources, a \in Actions, u \in Audiences, t \in Times }

ClaimAuthorized ==
  [Claim1 |-> TRUE, Claim2 |-> TRUE, Claim3 |-> TRUE, Claim4 |-> TRUE]

ClaimGrantBacked ==
  [Claim1 |-> TRUE, Claim2 |-> TRUE, Claim3 |-> TRUE, Claim4 |-> TRUE]

ClaimSignerTrusted ==
  [Claim1 |-> TRUE, Claim2 |-> TRUE, Claim3 |-> TRUE, Claim4 |-> TRUE]

ClaimSubject ==
  [Claim1 |-> "Alice", Claim2 |-> "Alice", Claim3 |-> "Alice", Claim4 |-> "Alice"]

ClaimTarget ==
  [Claim1 |-> "Alice", Claim2 |-> "Alice", Claim3 |-> "Alice", Claim4 |-> "Alice"]

ExpectedAuthority(input) ==
  UNION {ClaimCapabilities[c] : c \in input}

ExpectedContributors(input, cap) ==
  {c \in input : cap \in ClaimCapabilities[c]}

ClaimsForResource(input, r) ==
  {c \in input : \E cap \in ClaimCapabilities[c] : cap.resource = r}

ClaimsForAction(input, a) ==
  {c \in input : \E cap \in ClaimCapabilities[c] : cap.action = a}

ClaimsForAudience(input, u) ==
  {c \in input : \E cap \in ClaimCapabilities[c] : cap.audience = u}

ClaimsCurrentlyEffective(input, now) ==
  {c \in input :
    \E cap \in ClaimCapabilities[c] :
      now < cap.expiry}

EffectiveAt(cap) ==
  cap.resource = RequestResource /\
  cap.action = RequestAction /\
  cap.audience = RequestAudience /\
  EvalNow < cap.expiry

ExpectedDecisionContributors(input, authority) ==
  UNION {
    ExpectedContributors(input, cap) :
      cap \in authority /\ EffectiveAt(cap)
  }

EmptyContributors ==
  [cap \in CapabilityUniverse |-> {}]

VARIABLES inputClaims, composedAuthority, contributorClaims,
          resourceContributors, actionContributors,
          audienceContributors, temporalContributors,
          decisionAuthorized, decisionContributors

vars ==
  <<inputClaims, composedAuthority, contributorClaims,
   resourceContributors, actionContributors,
   audienceContributors, temporalContributors,
   decisionAuthorized, decisionContributors>>

Init ==
  /\ inputClaims = Claims
  /\ composedAuthority = {}
  /\ contributorClaims = EmptyContributors
  /\ resourceContributors = {}
  /\ actionContributors = {}
  /\ audienceContributors = {}
  /\ temporalContributors = {}
  /\ decisionAuthorized = FALSE
  /\ decisionContributors = {}

ComposeAndAuthorize ==
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
       ExpectedDecisionContributors(inputClaims, composedAuthority')
  /\ UNCHANGED inputClaims

Next ==
  ComposeAndAuthorize

TypeOK ==
  /\ inputClaims \subseteq Claims
  /\ composedAuthority \subseteq CapabilityUniverse
  /\ contributorClaims \in [CapabilityUniverse -> SUBSET Claims]
  /\ resourceContributors \subseteq Claims
  /\ actionContributors \subseteq Claims
  /\ audienceContributors \subseteq Claims
  /\ temporalContributors \subseteq Claims
  /\ decisionAuthorized \in BOOLEAN
  /\ decisionContributors \subseteq Claims

AllInputClaimsValid ==
  \A c \in inputClaims :
    ClaimAuthorized[c] /\
    ClaimGrantBacked[c] /\
    ClaimSignerTrusted[c] /\
    ClaimSubject[c] = ClaimTarget[c]

CompositionAtomAndProvenanceExact ==
  \A cap \in composedAuthority :
    /\ cap \in ExpectedAuthority(inputClaims)
    /\ contributorClaims[cap] =
         ExpectedContributors(inputClaims, cap)
    /\ contributorClaims[cap] # {}

DecisionDimensionSourcesRemainValid ==
  /\ resourceContributors =
       ClaimsForResource(inputClaims, RequestResource)
  /\ actionContributors =
       ClaimsForAction(inputClaims, RequestAction)
  /\ audienceContributors =
       ClaimsForAudience(inputClaims, RequestAudience)
  /\ temporalContributors =
       ClaimsCurrentlyEffective(inputClaims, EvalNow)
  /\ resourceContributors # {}
  /\ actionContributors # {}
  /\ audienceContributors # {}
  /\ temporalContributors # {}

DecisionRequiresSingleEffectiveAtom ==
  ~decisionAuthorized \/
  \E cap \in composedAuthority :
    /\ EffectiveAt(cap)
    /\ decisionContributors =
         ExpectedContributors(inputClaims, cap)
    /\ decisionContributors # {}

=========================================================================
