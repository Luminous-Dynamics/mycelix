---------------- MODULE EvidenceAttestationRelationalCapabilityContainmentV1NegativeControls ----------------
EXTENDS EvidenceAttestationRelationalCapabilityContainmentV1
NegativeAggregate ==
  /\ TypeOK
  /\ SetRelationReflexive
  /\ SetRelationTransitive
  /\ SetRelationAntisymmetricModuloEquivalence
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
