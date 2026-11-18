---------------- MODULE EvidenceAttestationNonExpandingSemanticTransformV1 ----------------
EXTENDS Naturals, FiniteSets

CONSTANTS
  KnownEffectFields,
  DecisionFields,
  EffectFields,
  ProfileDerivedFields,
  DeclaredDerivationFields,
  DefaultedFields,
  ExternalFields,
  DecisionOperations,
  DecisionTargets,
  DecisionCurrencies,
  DecisionMaxAmount,
  ProfileOperations,
  ProfileTargets,
  ProfileCurrencies,
  ProfileMaxAmount,
  EffectOperation,
  EffectTarget,
  EffectCurrency,
  EffectAmount,
  AuthorizedProfileId,
  ProfileId,
  AuthorizedImplementationHash,
  ImplementationHash,
  AuthorizedSourceSchema,
  SourceSchema,
  AuthorizedTargetSchema,
  TargetSchema,
  AuthorizedConversionFactor,
  ConversionFactor,
  EffectRenderedAmount,
  TransformContextDigest,
  AuthorizedContextDigest,
  DecisionId,
  SourceOperationField,
  SourceTargetField,
  SourceAmountField,
  SourceCurrencyField,
  ProfileDerivationRuleId,
  UsedDerivationRuleId,
  AuthorizedDerivationRuleId

VARIABLES committed
vars == <<committed>>

Init == committed = FALSE
Commit == committed' = TRUE
Next == Commit

TypeOK ==
  committed \in BOOLEAN /\
  EffectAmount >= 0 /\
  ProfileMaxAmount >= 0 /\
  DecisionMaxAmount >= 0 /\
  ConversionFactor > 0 /\
  AuthorizedConversionFactor > 0

ProfileCeilingNarrowed ==
  ~committed \/
    /\ ProfileOperations \subseteq DecisionOperations
    /\ ProfileTargets \subseteq DecisionTargets
    /\ ProfileCurrencies \subseteq DecisionCurrencies
    /\ ProfileMaxAmount <= DecisionMaxAmount

EffectScopeNarrowed ==
  ~committed \/
    /\ {EffectOperation} \subseteq ProfileOperations
    /\ {EffectTarget} \subseteq ProfileTargets
    /\ {EffectCurrency} \subseteq ProfileCurrencies
    /\ EffectAmount <= ProfileMaxAmount

EffectFieldsKnown ==
  ~committed \/ EffectFields \subseteq KnownEffectFields

EffectFieldsBacked ==
  ~committed \/
    (EffectFields \ ProfileDerivedFields) \subseteq DecisionFields

DeclaredDerivationExact ==
  ~committed \/
    /\ ProfileDerivedFields \subseteq DeclaredDerivationFields
    /\ ProfileDerivedFields # {} => UsedDerivationRuleId = AuthorizedDerivationRuleId
    /\ ProfileDerivedFields = {} => UsedDerivationRuleId = "none"

FieldSourcesExact ==
  ~committed \/
    /\ (EffectFieldsBacked \/ ProfileDerivedFields # {})
    /\ "operation" \in EffectFields => EffectOperation = "transfer"
    /\ "target" \in EffectFields => EffectTarget = "acct-alice"
    /\ "currency" \in EffectFields => EffectCurrency = "USD"

ProfileIdentityExact ==
  ~committed \/ ProfileId = AuthorizedProfileId

ImplementationIdentityExact ==
  ~committed \/ ImplementationHash = AuthorizedImplementationHash

SchemaIdentityExact ==
  ~committed \/
    /\ SourceSchema = AuthorizedSourceSchema
    /\ TargetSchema = AuthorizedTargetSchema

ConversionRuleExact ==
  ~committed \/
    /\ ConversionFactor = AuthorizedConversionFactor
    /\ EffectRenderedAmount = EffectAmount * ConversionFactor

NoImplicitDefaults ==
  ~committed \/ DefaultedFields = {}

NoExternalEnrichment ==
  ~committed \/ ExternalFields = {}

DeterministicContextExact ==
  ~committed \/ TransformContextDigest = AuthorizedContextDigest

NonExpandingSemanticTransformExact ==
  ~committed \/
    /\ TypeOK
    /\ ProfileCeilingNarrowed
    /\ EffectScopeNarrowed
    /\ EffectFieldsKnown
    /\ EffectFieldsBacked
    /\ DeclaredDerivationExact
    /\ FieldSourcesExact
    /\ ProfileIdentityExact
    /\ ImplementationIdentityExact
    /\ SchemaIdentityExact
    /\ ConversionRuleExact
    /\ NoImplicitDefaults
    /\ NoExternalEnrichment
    /\ DeterministicContextExact

=========================================================================
