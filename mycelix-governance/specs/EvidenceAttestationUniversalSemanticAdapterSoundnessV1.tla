---------------- MODULE EvidenceAttestationUniversalSemanticAdapterSoundnessV1 ----------------
EXTENDS Naturals, FiniteSets
CONSTANTS
DomainMin, DomainMax, MonotonicityDomainMax, TransformVariant, MonotonicityDeclared,
AuthorizedAdapterId, AdapterId, AuthorizedImplementationHash, ImplementationHash,
AuthorizedSourceSemanticType, SourceSemanticType, AuthorizedTargetSemanticType, TargetSemanticType,
AuthorizedUnitRule, UnitRule, AuthorizedExternalDependencyState, ExternalDependencyState,
AuthorizedAdapterStatus, AdapterStatus, AuthorizedDomainMax, DeclaredDomainMax,
AuthorizedPreconditionSatisfied, PreconditionSatisfied,
AuthorizedRemovedFields, RemovedFields, ReconstitutedFields,
AuthorizedQuantityRule, QuantityRule
VARIABLES committed
vars == <<committed>>
Init == committed = FALSE
Commit == committed' = TRUE
Next == Commit
InputDomain == DomainMin..DomainMax
MonotonicityDomain == DomainMin..MonotonicityDomainMax
TransformMax(x) ==
  IF TransformVariant = "canonical"
    THEN x \div 2
    ELSE IF x < 14 THEN 7 ELSE 6
TypeOK ==
  /\ committed \in BOOLEAN
  /\ DomainMin >= 0
  /\ DomainMax >= DomainMin
  /\ MonotonicityDomainMax >= DomainMin
  /\ MonotonicityDomainMax <= DomainMax
TransformImplementationExact == ~committed \/ ImplementationHash = AuthorizedImplementationHash
AdapterIdentityExact == ~committed \/ AdapterId = AuthorizedAdapterId
SemanticTypeExact ==
  ~committed \/ (SourceSemanticType = AuthorizedSourceSemanticType /\ TargetSemanticType = AuthorizedTargetSemanticType)
UnitRuleExact ==
  ~committed \/ (UnitRule = AuthorizedUnitRule /\ QuantityRule = AuthorizedQuantityRule)
ExternalDependencyExact == ~committed \/ ExternalDependencyState = AuthorizedExternalDependencyState
AdapterStatusExact == ~committed \/ AdapterStatus = AuthorizedAdapterStatus
AdapterDomainExact ==
  ~committed \/ (DeclaredDomainMax = AuthorizedDomainMax /\ DeclaredDomainMax = DomainMax)
AdapterPreconditionExact == ~committed \/ PreconditionSatisfied = AuthorizedPreconditionSatisfied
AdapterRemovalExact == ~committed \/ RemovedFields = AuthorizedRemovedFields
NoReconstitution == ~committed \/ ReconstitutedFields \cap RemovedFields = {}
NonExpansionUniversal == ~committed \/ \A x \in InputDomain : TransformMax(x) <= x
MonotonicityDomainExact == ~committed \/ MonotonicityDomainMax = DomainMax
MonotoneUniversal ==
  ~committed \/
    \A a \in MonotonicityDomain :
      \A b \in MonotonicityDomain :
        (a <= b) => (TransformMax(a) <= TransformMax(b))
MonotonicityDeclarationExact == ~committed \/ MonotonicityDeclared = TRUE
UniversalTransformSound ==
  ~committed \/
    /\ TypeOK
    /\ TransformImplementationExact
    /\ AdapterIdentityExact
    /\ SemanticTypeExact
    /\ UnitRuleExact
    /\ ExternalDependencyExact
    /\ AdapterStatusExact
    /\ AdapterDomainExact
    /\ AdapterPreconditionExact
    /\ AdapterRemovalExact
    /\ NoReconstitution
    /\ NonExpansionUniversal
    /\ MonotonicityDomainExact
    /\ MonotoneUniversal
    /\ MonotonicityDeclarationExact
=============================================================================