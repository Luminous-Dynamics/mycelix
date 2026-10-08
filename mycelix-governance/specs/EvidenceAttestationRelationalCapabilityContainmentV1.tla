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
NormalizeTarget(x) == IF x = "acct-alice" THEN "alice" ELSE IF x = "acct-bob" THEN "bob" ELSE x
TupleOperation(t) == t[1]
TupleTarget(t) == t[2]
TupleAudience(t) == t[3]
TupleCurrency(t) == t[4]
TupleArgumentClass(t) == t[5]
TupleBound(t) == t[6]

CategoryLeq(child,parent,normalize) == parent = Wildcard \/ normalize(child) = normalize(parent)
TupleLeq(child,parent) ==
  /\ CategoryLeq(TupleOperation(child),TupleOperation(parent),NormalizeTarget)
  /\ CategoryLeq(TupleTarget(child),TupleTarget(parent),NormalizeTarget)
  /\ CategoryLeq(TupleAudience(child),TupleAudience(parent),NormalizeTarget)
  /\ CategoryLeq(TupleCurrency(child),TupleCurrency(parent),NormalizeTarget)
  /\ CategoryLeq(TupleArgumentClass(child),TupleArgumentClass(parent),NormalizeTarget)
  /\ TupleBound(child) <= TupleBound(parent)

SetLeq(childSet,parentSet) ==
  \\A c \\in childSet : \\E p \\in parentSet : TupleLeq(c,p)

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
  /\ committed \\in BOOLEAN
  /\ ParentCapabilities \\subseteq KnownOperations \\X KnownTargets \\X KnownAudiences \\X KnownCurrencies \\X KnownArgumentClasses \\X 0..100
  /\ ChildCapabilities \\subseteq KnownOperations \\X KnownTargets \\X KnownAudiences \\X KnownCurrencies \\X KnownArgumentClasses \\X 0..100
  /\ DownstreamCapabilities \\subseteq KnownOperations \\X KnownTargets \\X KnownAudiences \\X KnownCurrencies \\X KnownArgumentClasses \\X 0..100

SetRelationReflexive == ~committed \/ SetLeq(ParentCapabilities,ParentCapabilities)
SetRelationTransitive == ~committed \/ (SetLeq(ChildCapabilities,ParentCapabilities) /\ SetLeq(ParentCapabilities,ParentCapabilities) => SetLeq(ChildCapabilities,ParentCapabilities))
SetRelationAntisymmetric == ~committed \/ ((SetLeq(ChildCapabilities,ParentCapabilities) /\ SetLeq(ParentCapabilities,ChildCapabilities)) => ChildCapabilities = ParentCapabilities)

SafetyAggregate == ~committed \/
  /\ TypeOK
  /\ ParentSetId = AuthorizedParentSetId
  /\ Control = "canonical" => RelationalContainmentExact /\ DownstreamRelationalSubset
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
