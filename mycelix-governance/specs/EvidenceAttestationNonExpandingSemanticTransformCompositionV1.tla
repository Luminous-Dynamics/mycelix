---------------- MODULE EvidenceAttestationNonExpandingSemanticTransformCompositionV1 ----------------
EXTENDS Naturals, FiniteSets

CONSTANTS
  DecisionOperations,
  DecisionTargets,
  DecisionAudiences,
  DecisionCurrencies,
  DecisionMaxAmount,
  Profile1Operations,
  Profile1Targets,
  Profile1Audiences,
  Profile1Currencies,
  Profile1MaxAmount,
  Effect1Operations,
  Effect1Targets,
  Effect1Audiences,
  Effect1Currencies,
  Effect1MaxAmount,
  Profile2Operations,
  Profile2Targets,
  Profile2Audiences,
  Profile2Currencies,
  Profile2MaxAmount,
  Effect2Operations,
  Effect2Targets,
  Effect2Audiences,
  Effect2Currencies,
  Effect2MaxAmount,
  KnownEffectFields,
  DecisionFields,
  EffectFields,
  AuthorizedDecisionId,
  Step1SourceDecisionId,
  Step2SourceDecisionId,
  AuthorizedEffectId,
  Step1EffectId,
  Step2SourceEffectId,
  Step2EffectId,
  Step2InputOperations,
  Step2InputTargets,
  Step2InputAudiences,
  Step2InputCurrencies,
  Step2InputMaxAmount,
  AuthorizedProfile1Id,
  Profile1Id,
  AuthorizedProfile2Id,
  Profile2Id,
  AuthorizedImplementationHash1,
  ImplementationHash1,
  AuthorizedImplementationHash2,
  ImplementationHash2,
  AuthorizedDerivationRuleId,
  Profile1DerivationRuleId,
  Profile2DerivationRuleId,
  UsedDerivationRule1,
  UsedDerivationRule2,
  AuthorizedSourceSchema,
  AuthorizedTargetSchema,
  Profile1SourceSchema,
  Profile1TargetSchema,
  Profile2SourceSchema,
  Profile2TargetSchema,
  AuthorizedConversionFactor,
  Profile1ConversionFactor,
  Profile2ConversionFactor,
  Step1RenderedAmount,
  Step2RenderedAmount,
  SourceOperationField,
  SourceTargetField,
  SourceAmountField,
  SourceCurrencyField,
  Step1SourceOperationField,
  Step1SourceTargetField,
  Step1SourceAmountField,
  Step1SourceCurrencyField,
  Step2SourceOperationField,
  Step2SourceTargetField,
  Step2SourceAmountField,
  Step2SourceCurrencyField,
  AuthorizedContextDigest,
  Profile1ContextDigest,
  Profile2ContextDigest,
  Step1ExecutionContextDigest,
  Step2ExecutionContextDigest,
  MonotoneDeclared,
  MonoInputAOperations,
  MonoInputATargets,
  MonoInputAAudiences,
  MonoInputACurrencies,
  MonoInputAMaxAmount,
  MonoInputBOperations,
  MonoInputBTargets,
  MonoInputBAudiences,
  MonoInputBCurrencies,
  MonoInputBMaxAmount,
  MonoEffectAOperations,
  MonoEffectATargets,
  MonoEffectAAudiences,
  MonoEffectACurrencies,
  MonoEffectAMaxAmount,
  MonoEffectBOperations,
  MonoEffectBTargets,
  MonoEffectBAudiences,
  MonoEffectBCurrencies,
  MonoEffectBMaxAmount,
  MonoProfileOperations,
  MonoProfileTargets,
  MonoProfileAudiences,
  MonoProfileCurrencies,
  MonoProfileMaxAmount

VARIABLES committed
vars == <<committed>>

Init == committed = FALSE
Commit == committed' = TRUE
Next == Commit

ScopeLeq(aOperations, aTargets, aAudiences, aCurrencies, aMax,
         bOperations, bTargets, bAudiences, bCurrencies, bMax) ==
  /\ aOperations \subseteq bOperations
  /\ aTargets \subseteq bTargets
  /\ aAudiences \subseteq bAudiences
  /\ aCurrencies \subseteq bCurrencies
  /\ aMax <= bMax

CoreScopeLeq(aOperations, aTargets, aCurrencies, aMax,
             bOperations, bTargets, bCurrencies, bMax) ==
  /\ aOperations \subseteq bOperations
  /\ aTargets \subseteq bTargets
  /\ aCurrencies \subseteq bCurrencies
  /\ aMax <= bMax

TypeOK ==
  committed \in BOOLEAN /\
  DecisionMaxAmount >= 0 /\
  Profile1MaxAmount >= 0 /\
  Effect1MaxAmount >= 0 /\
  Profile2MaxAmount >= 0 /\
  Effect2MaxAmount >= 0 /\
  AuthorizedConversionFactor > 0 /\
  Profile1ConversionFactor > 0 /\
  Profile2ConversionFactor > 0 /\
  MonoProfileMaxAmount >= 0 /\
  MonoInputAMaxAmount >= 0 /\
  MonoInputBMaxAmount >= 0 /\
  MonoEffectAMaxAmount >= 0 /\
  MonoEffectBMaxAmount >= 0

RootDecisionIdentityExact ==
  ~committed \/
    /\ Step1SourceDecisionId = AuthorizedDecisionId
    /\ Step2SourceDecisionId = AuthorizedDecisionId
    /\ Step1EffectId = AuthorizedEffectId
    /\ Step2EffectId # Step1EffectId

DerivationRuleIdentityExact ==
  ~committed \/
    /\ Profile1DerivationRuleId = AuthorizedDerivationRuleId
    /\ Profile2DerivationRuleId = AuthorizedDerivationRuleId
    /\ UsedDerivationRule1 = AuthorizedDerivationRuleId
    /\ UsedDerivationRule2 = AuthorizedDerivationRuleId

SourceAmountFieldExact ==
  ~committed \/
    /\ Step1SourceAmountField = SourceAmountField
    /\ Step2SourceAmountField = SourceAmountField

AudienceAttenuationExact ==
  ~committed \/
    /\ Profile1Audiences \subseteq DecisionAudiences
    /\ Effect1Audiences \subseteq Profile1Audiences
    /\ Profile2Audiences \subseteq Effect1Audiences
    /\ Effect2Audiences \subseteq Profile2Audiences

Step1ProfileCoreNarrowed ==
  ~committed \/
    CoreScopeLeq(
      Profile1Operations, Profile1Targets, Profile1Currencies, Profile1MaxAmount,
      DecisionOperations, DecisionTargets, DecisionCurrencies, DecisionMaxAmount)

Step1EffectCoreNarrowed ==
  ~committed \/
    CoreScopeLeq(
      Effect1Operations, Effect1Targets, Effect1Currencies, Effect1MaxAmount,
      Profile1Operations, Profile1Targets, Profile1Currencies, Profile1MaxAmount)

Step2ProfileCoreNarrowed ==
  ~committed \/
    CoreScopeLeq(
      Profile2Operations, Profile2Targets, Profile2Currencies, Profile2MaxAmount,
      Effect1Operations, Effect1Targets, Effect1Currencies, Effect1MaxAmount)

Step2EffectCoreNarrowed ==
  ~committed \/
    CoreScopeLeq(
      Effect2Operations, Effect2Targets, Effect2Currencies, Effect2MaxAmount,
      Profile2Operations, Profile2Targets, Profile2Currencies, Profile2MaxAmount)

ChainLinkExact ==
  ~committed \/
    /\ Step2SourceEffectId = Step1EffectId
    /\ Step2InputOperations = Effect1Operations
    /\ Step2InputTargets = Effect1Targets
    /\ Step2InputAudiences = Effect1Audiences
    /\ Step2InputCurrencies = Effect1Currencies
    /\ Step2InputMaxAmount = Effect1MaxAmount

ProfileIdentityExact ==
  ~committed \/
    /\ Profile1Id = AuthorizedProfile1Id
    /\ Profile2Id = AuthorizedProfile2Id

DownstreamImplementationIdentityExact ==
  ~committed \/ ImplementationHash2 = AuthorizedImplementationHash2

ImplementationIdentityExact ==
  ~committed \/
    /\ ImplementationHash1 = AuthorizedImplementationHash1
    /\ ImplementationHash2 = AuthorizedImplementationHash2

SchemaIdentityExact ==
  ~committed \/
    /\ Profile1SourceSchema = AuthorizedSourceSchema
    /\ Profile1TargetSchema = AuthorizedTargetSchema
    /\ Profile2SourceSchema = Profile1TargetSchema
    /\ Profile2TargetSchema = AuthorizedTargetSchema

ConversionRuleExact ==
  ~committed \/
    /\ Profile1ConversionFactor = AuthorizedConversionFactor
    /\ Profile2ConversionFactor = AuthorizedConversionFactor
    /\ Step1RenderedAmount = Effect1MaxAmount * Profile1ConversionFactor
    /\ Step2RenderedAmount = Effect2MaxAmount * Profile2ConversionFactor

FieldSourcesExact ==
  ~committed \/
    /\ Step1SourceOperationField = SourceOperationField
    /\ Step1SourceTargetField = SourceTargetField
    /\ Step1SourceAmountField = SourceAmountField
    /\ Step1SourceCurrencyField = SourceCurrencyField
    /\ Step2SourceOperationField = SourceOperationField
    /\ Step2SourceTargetField = SourceTargetField
    /\ Step2SourceAmountField = SourceAmountField
    /\ Step2SourceCurrencyField = SourceCurrencyField

SemanticFieldsKnown ==
  ~committed \/ EffectFields \subseteq KnownEffectFields

SemanticFieldsBacked ==
  ~committed \/ EffectFields \subseteq DecisionFields

ContextExact ==
  ~committed \/
    /\ Profile1ContextDigest = AuthorizedContextDigest
    /\ Profile2ContextDigest = AuthorizedContextDigest
    /\ Step1ExecutionContextDigest = AuthorizedContextDigest
    /\ Step2ExecutionContextDigest = AuthorizedContextDigest

NonExpandingStep1 ==
  ~committed \/
    /\ CoreScopeLeq(
         Effect1Operations, Effect1Targets, Effect1Currencies, Effect1MaxAmount,
         DecisionOperations, DecisionTargets, DecisionCurrencies, DecisionMaxAmount)
    /\ Effect1Audiences \subseteq DecisionAudiences

NonExpandingChainClosed ==
  ~committed \/
    /\ ScopeLeq(
         Effect2Operations, Effect2Targets, Effect2Audiences, Effect2Currencies, Effect2MaxAmount,
         DecisionOperations, DecisionTargets, DecisionAudiences, DecisionCurrencies, DecisionMaxAmount)

MonotonicityDeclared ==
  ~committed \/ MonotoneDeclared = TRUE

MonotoneRelationExact ==
  ~committed \/
    ScopeLeq(
      MonoInputAOperations, MonoInputATargets, MonoInputAAudiences, MonoInputACurrencies, MonoInputAMaxAmount,
      MonoInputBOperations, MonoInputBTargets, MonoInputBAudiences, MonoInputBCurrencies, MonoInputBMaxAmount)
    =>
    ScopeLeq(
      MonoEffectAOperations, MonoEffectATargets, MonoEffectAAudiences, MonoEffectACurrencies, MonoEffectAMaxAmount,
      MonoEffectBOperations, MonoEffectBTargets, MonoEffectBAudiences, MonoEffectBCurrencies, MonoEffectBMaxAmount)

MonotoneProbeContractive ==
  ~committed \/
    /\ CoreScopeLeq(MonoProfileOperations, MonoProfileTargets, MonoProfileCurrencies, MonoProfileMaxAmount,
                    MonoInputAOperations, MonoInputATargets, MonoInputACurrencies, MonoInputAMaxAmount)
    /\ CoreScopeLeq(MonoProfileOperations, MonoProfileTargets, MonoProfileCurrencies, MonoProfileMaxAmount,
                    MonoInputBOperations, MonoInputBTargets, MonoInputBCurrencies, MonoInputBMaxAmount)
    /\ CoreScopeLeq(MonoEffectAOperations, MonoEffectATargets, MonoEffectACurrencies, MonoEffectAMaxAmount,
                    MonoInputAOperations, MonoInputATargets, MonoInputACurrencies, MonoInputAMaxAmount)
    /\ CoreScopeLeq(MonoEffectBOperations, MonoEffectBTargets, MonoEffectBCurrencies, MonoEffectBMaxAmount,
                    MonoInputBOperations, MonoInputBTargets, MonoInputBCurrencies, MonoInputBMaxAmount)

NonExpandingSemanticTransformCompositionExact ==
  ~committed \/
    /\ TypeOK
    /\ RootDecisionIdentityExact
    /\ DerivationRuleIdentityExact
    /\ SourceAmountFieldExact
    /\ AudienceAttenuationExact
    /\ Step1ProfileCoreNarrowed
    /\ Step1EffectCoreNarrowed
    /\ Step2ProfileCoreNarrowed
    /\ Step2EffectCoreNarrowed
    /\ ChainLinkExact
    /\ ProfileIdentityExact
    /\ ImplementationIdentityExact
    /\ SchemaIdentityExact
    /\ ConversionRuleExact
    /\ FieldSourcesExact
    /\ SemanticFieldsKnown
    /\ SemanticFieldsBacked
    /\ ContextExact
    /\ NonExpandingStep1
    /\ NonExpandingChainClosed
    /\ MonotonicityDeclared
    /\ MonotoneProbeContractive
    /\ MonotoneRelationExact

=========================================================================
