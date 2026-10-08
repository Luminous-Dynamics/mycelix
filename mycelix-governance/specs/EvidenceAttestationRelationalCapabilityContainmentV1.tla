---------------- MODULE EvidenceAttestationRelationalCapabilityContainmentV1 ----------------
EXTENDS Naturals, FiniteSets

CONSTANTS ParentCapabilities, ChildCapabilities, DownstreamCapabilities,
          ParentSetId, AuthorizedParentSetId, Control,
          KnownOperations, KnownTargets, KnownAudiences,
          KnownCurrencies, KnownArgumentClasses

VARIABLES committed
vars == <<committed>>
Init == committed = FALSE
Commit == committed' = TRUE
Next == Commit

Wildcard == "*"
NormOp(x) == x
NormTarget(x) == IF x = "acct-alice" THEN "alice" ELSE IF x = "acct-bob" THEN "bob" ELSE x
NormAudience(x) == x
NormCurrency(x) == x
NormArgument(x) == x

TupleOperation(t) == t[1]
TupleTarget(t) == t[2]
TupleAudience(t) == t[3]
TupleCurrency(t) == t[4]
TupleArgumentClass(t) == t[5]
TupleBound(t) == t[6]

CategoryLeq(child,parent,norm) == parent = Wildcard \/ norm(child) = norm(parent)
TupleLeq(child,parent) ==
  /\ CategoryLeq(TupleOperation(child),TupleOperation(parent),NormOp)
  /\ CategoryLeq(TupleTarget(child),TupleTarget(parent),NormTarget)
  /\ CategoryLeq(TupleAudience(child),TupleAudience(parent),NormAudience)
  /\ CategoryLeq(TupleCurrency(child),TupleCurrency(parent),NormCurrency)
  /\ CategoryLeq(TupleArgumentClass(child),TupleArgumentClass(parent),NormArgument)
  /\ TupleBound(child) <= TupleBound(parent)

SetLeq(childSet,parentSet) ==
  \A c \in childSet : \E p \in parentSet : TupleLeq(c,p)

Admits(S,x) == \E c \in S : TupleLeq(x,c)
SemEquivalent(A,B) ==
  \A x \in (ParentCapabilities \cup ChildCapabilities \cup DownstreamCapabilities) :
    Admits(A,x) = Admits(B,x)

RelationalContainmentExact == ~committed \/ SetLeq(ChildCapabilities,ParentCapabilities)
DownstreamRelationalSubset ==
  ~committed \/ SetLeq(DownstreamCapabilities,ChildCapabilities)

CartesianRecombinationRejected ==
  ~committed \/ Control # "cartesian-recombination" \/ SetLeq(ChildCapabilities,ParentCapabilities)
TargetCurrencyCorrelationRejected ==
  ~committed \/ Control # "target-currency-correlation" \/ SetLeq(ChildCapabilities,ParentCapabilities)
OperationArgumentCorrelationRejected ==
  ~committed \/ Control # "operation-argument-correlation" \/ SetLeq(ChildCapabilities,ParentCapabilities)
AudienceTargetCorrelationRejected ==
  ~committed \/ Control # "audience-target-correlation" \/ SetLeq(ChildCapabilities,ParentCapabilities)
WildcardExpansionRejected ==
  ~committed \/ Control # "wildcard-expansion" \/ SetLeq(ChildCapabilities,ParentCapabilities)
CapabilitySetIdentityExact ==
  ~committed \/ Control # "capability-set-identity-substitution" \/ ParentSetId = AuthorizedParentSetId
TupleNormalizationNonEquivalentRejected ==
  ~committed \/ Control # "tuple-normalization-substitution" \/ SetLeq(ChildCapabilities,ParentCapabilities)
ProfileWideningWithSameMarginalsRejected ==
  ~committed \/ Control # "profile-widening-unchanged-marginals" \/ SetLeq(ChildCapabilities,ParentCapabilities)
EffectOutsideRelationRejected ==
  ~committed \/ Control # "effect-outside-relation-inside-marginals" \/ SetLeq(ChildCapabilities,ParentCapabilities)
DownstreamRelationBypassRejected ==
  ~committed \/ Control # "downstream-relational-subset-bypass" \/ DownstreamRelationalSubset

TypeOK ==
  /\ committed \in BOOLEAN
  /\ ParentCapabilities \subseteq KnownOperations \X KnownTargets \X KnownAudiences \X KnownCurrencies \X KnownArgumentClasses \X 0..100
  /\ ChildCapabilities \subseteq KnownOperations \X KnownTargets \X KnownAudiences \X KnownCurrencies \X KnownArgumentClasses \X 0..100
  /\ DownstreamCapabilities \subseteq KnownOperations \X KnownTargets \X KnownAudiences \X KnownCurrencies \X KnownArgumentClasses \X 0..100

SetRelationReflexive == ~committed \/ SetLeq(ParentCapabilities,ParentCapabilities)
SetRelationTransitive == ~committed \/
  \A A \in SUBSET (ParentCapabilities \cup ChildCapabilities \cup DownstreamCapabilities) :
    \A B \in SUBSET (ParentCapabilities \cup ChildCapabilities \cup DownstreamCapabilities) :
      \A C \in SUBSET (ParentCapabilities \cup ChildCapabilities \cup DownstreamCapabilities) :
        (SetLeq(A,B) /\ SetLeq(B,C)) => SetLeq(A,C)
SetRelationAntisymmetricModuloEquivalence == ~committed \/
  \A A \in SUBSET (ParentCapabilities \cup ChildCapabilities \cup DownstreamCapabilities) :
    \A B \in SUBSET (ParentCapabilities \cup ChildCapabilities \cup DownstreamCapabilities) :
      (SetLeq(A,B) /\ SetLeq(B,A)) => SemEquivalent(A,B)

SafetyAggregate == ~committed \/
  /\ TypeOK
  /\ (Control = "canonical" => (ParentSetId = AuthorizedParentSetId /\ RelationalContainmentExact /\ DownstreamRelationalSubset))
  /\ CartesianRecombinationRejected
  /\ TargetCurrencyCorrelationRejected
  /\ OperationArgumentCorrelationRejected
  /\ AudienceTargetCorrelationRejected
  /\ WildcardExpansionRejected
  /\ CapabilitySetIdentityExact
  /\ TupleNormalizationNonEquivalentRejected
  /\ ProfileWideningWithSameMarginalsRejected
  /\ EffectOutsideRelationRejected
  /\ DownstreamRelationBypassRejected

=============================================================================
