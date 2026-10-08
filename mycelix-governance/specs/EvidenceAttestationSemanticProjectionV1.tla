---------------- MODULE EvidenceAttestationSemanticProjectionV1 ----------------
EXTENDS Naturals, FiniteSets

CONSTANTS
  KnownFields, RequiredFields, AuthorizedFields, DecisionValueFields, EffectFields,
  DefaultedFields, AuthorizedSchema, EffectSchema,
  AuthorizedOperation, AuthorizedTarget, AuthorizedAmount, AuthorizedCurrency,
  ProjectedOperation, ProjectedTarget, ProjectedAmount, ProjectedCurrency,
  ProjectionSourceOperation, ProjectionSourceTarget, ProjectionSourceAmount, ProjectionSourceCurrency

VARIABLES projectionCommitted
vars == <<projectionCommitted>>
Init == projectionCommitted = FALSE
CommitProjection == projectionCommitted' = TRUE
Next == CommitProjection

TypeOK ==
  projectionCommitted \in BOOLEAN

EffectFieldsKnown ==
  ~projectionCommitted \/ EffectFields \subseteq KnownFields

EffectFieldsAuthorized ==
  ~projectionCommitted \/ EffectFields \subseteq AuthorizedFields

RequiredFieldsPresent ==
  ~projectionCommitted \/ RequiredFields \subseteq EffectFields

ProjectionSourcesExact ==
  ~projectionCommitted \/
    /\ ProjectionSourceOperation = "operation"
    /\ ProjectionSourceTarget = "target"
    /\ ProjectionSourceAmount = "amount"
    /\ ProjectionSourceCurrency = "currency"

ProjectedValuesConserved ==
  ~projectionCommitted \/
    /\ (("operation" \in EffectFields) => ProjectedOperation = AuthorizedOperation)
    /\ (("target" \in EffectFields) => ProjectedTarget = AuthorizedTarget)
    /\ (("amount" \in EffectFields) => ProjectedAmount = AuthorizedAmount)
    /\ (("currency" \in EffectFields) => ProjectedCurrency = AuthorizedCurrency)

NoImplicitDefaultAuthority ==
  ~projectionCommitted \/ DefaultedFields \subseteq DecisionValueFields

SchemaProfileExact ==
  ~projectionCommitted \/ EffectSchema = AuthorizedSchema

SemanticProjectionExact ==
  ~projectionCommitted \/
    /\ EffectFieldsKnown
    /\ EffectFieldsAuthorized
    /\ RequiredFieldsPresent
    /\ ProjectionSourcesExact
    /\ ProjectedValuesConserved
    /\ NoImplicitDefaultAuthority
    /\ SchemaProfileExact

=========================================================================
