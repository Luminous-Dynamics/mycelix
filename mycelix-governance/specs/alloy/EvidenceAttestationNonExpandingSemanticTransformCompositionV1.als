module EvidenceAttestationNonExpandingSemanticTransformCompositionV1

enum Bit { On, Off }
abstract sig Operation {}
one sig TransferOp, RefundOp extends Operation {}
abstract sig Target {}
one sig AliceTarget, BobTarget extends Target {}
abstract sig Audience {}
one sig PaymentsAudience, AdminAudience extends Audience {}
abstract sig Currency {}
one sig USD, EUR extends Currency {}
abstract sig Field {}
one sig OperationField, TargetField, AmountField, CurrencyField, DestinationField, RiskScoreField extends Field {}
abstract sig Schema {}
one sig PaymentV1, PaymentV2 extends Schema {}
abstract sig DecisionId {}
one sig CanonicalDecisionId, AlternateDecisionId extends DecisionId {}
abstract sig EffectId {}
one sig CanonicalEffectId, AlternateEffectId extends EffectId {}
abstract sig ProfileId {}
one sig CanonicalProfile1Id, CanonicalProfile2Id, AlternateProfileId extends ProfileId {}
abstract sig ImplementationHash {}
one sig CanonicalImplementationHash1, CanonicalImplementationHash2, AlternateImplementationHash extends ImplementationHash {}
abstract sig DerivationRule {}
one sig NoDerivation, CanonicalDerivation, AlternateDerivation extends DerivationRule {}
abstract sig ContextDigest {}
one sig CanonicalContextDigest, AlternateContextDigest extends ContextDigest {}

sig Scope {
  operations: set Operation,
  targets: set Target,
  audiences: set Audience,
  currencies: set Currency,
  maxAmount: one Int
}

sig Decision {
  id: one DecisionId,
  authority: one Scope,
  fields: set Field,
  sourceSchema: one Schema
}

sig TransformProfile {
  id: one ProfileId,
  implementationHash: one ImplementationHash,
  derivationRule: one DerivationRule,
  ceiling: one Scope,
  sourceSchema: one Schema,
  targetSchema: one Schema,
  conversionFactor: one Int,
  contextDigest: one ContextDigest,
  monotonicityDeclared: one Bit
}

sig TransformStep {
  upstream: lone TransformStep,
  decision: one Decision,
  inputAuthority: one Scope,
  profile: one TransformProfile,
  effect: one Scope,
  sourceDecisionId: one DecisionId,
  sourceEffectId: lone EffectId,
  effectId: one EffectId,
  effectFields: set Field,
  sourceOperation: one Field,
  sourceTarget: one Field,
  sourceAmount: one Field,
  sourceCurrency: one Field,
  renderedAmount: one Int,
  executionContextDigest: one ContextDigest,
  committed: one Bit
}

one sig RootStep extends TransformStep {}
one sig DownstreamStep extends TransformStep {}
one sig RootDecision extends Decision {}
one sig Profile1 extends TransformProfile {}
one sig Profile2 extends TransformProfile {}
one sig MonotoneProfile extends TransformProfile {}
one sig MonotoneRelation {
  inputA: one Scope,
  inputB: one Scope,
  effectA: one Scope,
  effectB: one Scope
}

pred scopeLeq[a,b: Scope] {
  a.operations in b.operations
  a.targets in b.targets
  a.audiences in b.audiences
  a.currencies in b.currencies
  a.maxAmount <= b.maxAmount
}

pred coreScopeLeq[a,b: Scope] {
  a.operations in b.operations
  a.targets in b.targets
  a.currencies in b.currencies
  a.maxAmount <= b.maxAmount
}

pred rootDecisionIdentityExact {
  RootStep.sourceDecisionId = CanonicalDecisionId
  DownstreamStep.sourceDecisionId = CanonicalDecisionId
  RootStep.effectId = CanonicalEffectId
  DownstreamStep.effectId != RootStep.effectId
  RootStep.sourceEffectId = none
}

pred derivationRuleIdentityExact {
  Profile1.derivationRule = NoDerivation
  Profile2.derivationRule = NoDerivation
  RootStep.profile = Profile1
  DownstreamStep.profile = Profile2
}

pred sourceAmountFieldExact {
  RootStep.sourceAmount = AmountField
  DownstreamStep.sourceAmount = AmountField
}

pred audienceAttenuationExact {
  Profile1.ceiling.audiences in RootDecision.authority.audiences
  RootStep.effect.audiences in Profile1.ceiling.audiences
  Profile2.ceiling.audiences in RootStep.effect.audiences
  DownstreamStep.effect.audiences in Profile2.ceiling.audiences
}

pred coreAttenuationExact {
  coreScopeLeq[Profile1.ceiling, RootDecision.authority]
  coreScopeLeq[RootStep.effect, Profile1.ceiling]
  coreScopeLeq[Profile2.ceiling, RootStep.effect]
  coreScopeLeq[DownstreamStep.effect, Profile2.ceiling]
}

pred chainLinkExact {
  DownstreamStep.upstream = RootStep
  DownstreamStep.decision = RootStep.decision
  DownstreamStep.inputAuthority = RootStep.effect
  DownstreamStep.sourceEffectId = RootStep.effectId
  DownstreamStep.effectId != RootStep.effectId
}

pred identityExact {
  Profile1.id = CanonicalProfile1Id
  Profile2.id = CanonicalProfile2Id
  Profile1.implementationHash = CanonicalImplementationHash1
  Profile2.implementationHash = CanonicalImplementationHash2
}

pred schemaConversionContextExact {
  Profile1.sourceSchema = PaymentV1
  Profile1.targetSchema = PaymentV1
  Profile2.sourceSchema = Profile1.targetSchema
  Profile2.targetSchema = PaymentV1
  Profile1.conversionFactor = 100
  Profile2.conversionFactor = 100
  RootStep.renderedAmount = RootStep.effect.maxAmount * Profile1.conversionFactor
  DownstreamStep.renderedAmount = DownstreamStep.effect.maxAmount * Profile2.conversionFactor
  Profile1.contextDigest = CanonicalContextDigest
  Profile2.contextDigest = CanonicalContextDigest
  RootStep.executionContextDigest = CanonicalContextDigest
  DownstreamStep.executionContextDigest = CanonicalContextDigest
}

pred fieldSemanticsExact {
  RootStep.effectFields in {OperationField, TargetField, AmountField, CurrencyField, DestinationField, RiskScoreField}
  DownstreamStep.effectFields in {OperationField, TargetField, AmountField, CurrencyField, DestinationField, RiskScoreField}
  RootStep.effectFields in RootDecision.fields
  DownstreamStep.effectFields in RootDecision.fields
  RootStep.sourceOperation = OperationField
  RootStep.sourceTarget = TargetField
  RootStep.sourceAmount = AmountField
  RootStep.sourceCurrency = CurrencyField
  DownstreamStep.sourceOperation = OperationField
  DownstreamStep.sourceTarget = TargetField
  DownstreamStep.sourceAmount = AmountField
  DownstreamStep.sourceCurrency = CurrencyField
}

pred monotoneContractive {
  coreScopeLeq[MonotoneProfile.ceiling, MonotoneRelation.inputA]
  coreScopeLeq[MonotoneProfile.ceiling, MonotoneRelation.inputB]
  coreScopeLeq[MonotoneRelation.effectA, MonotoneRelation.inputA]
  coreScopeLeq[MonotoneRelation.effectB, MonotoneRelation.inputB]
}

pred monotoneRelationExact {
  MonotoneProfile.contextDigest = CanonicalContextDigest
  let m = MonotoneRelation |
    scopeLeq[m.inputA,m.inputB] implies scopeLeq[m.effectA,m.effectB]
}

fact CanonicalEnvironment {
  RootStep.committed = On
  DownstreamStep.committed = On
  RootStep.decision = RootDecision
  RootStep.profile = Profile1
  DownstreamStep.profile = Profile2
  no RootStep.upstream
  RootStep.upstream = none
  MonotoneProfile.contextDigest = CanonicalContextDigest
}

fact rootDecisionIdentityExact {
  RootStep.sourceDecisionId = CanonicalDecisionId
}

fact derivationRuleIdentityExact {
  Profile1.derivationRule = NoDerivation
  Profile2.derivationRule = NoDerivation
  RootStep.profile = Profile1
  DownstreamStep.profile = Profile2
}

fact sourceAmountFieldExact {
  RootStep.sourceAmount = AmountField
  DownstreamStep.sourceAmount = AmountField
}

fact audienceAttenuationExact {
  Profile1.ceiling.audiences in RootDecision.authority.audiences
  RootStep.effect.audiences in Profile1.ceiling.audiences
  Profile2.ceiling.audiences in RootStep.effect.audiences
  DownstreamStep.effect.audiences in Profile2.ceiling.audiences
}

fact step1CoreAttenuationExact {
  coreScopeLeq[Profile1.ceiling, RootDecision.authority]
  coreScopeLeq[RootStep.effect, Profile1.ceiling]
}

fact step2ProfileCoreNarrowed {
  coreScopeLeq[Profile2.ceiling, RootStep.effect]
}

fact step2EffectCoreNarrowed {
  coreScopeLeq[DownstreamStep.effect, Profile2.ceiling]
}

fact chainLinkExact {
  DownstreamStep.upstream = RootStep
  DownstreamStep.inputAuthority = RootStep.effect
  DownstreamStep.sourceEffectId = CanonicalEffectId
}

fact identityExact {
  Profile1.id = CanonicalProfile1Id
  Profile2.id = CanonicalProfile2Id
  Profile1.implementationHash = CanonicalImplementationHash1
  Profile2.implementationHash = CanonicalImplementationHash2
}

fact schemaConversionContextExact {
  Profile1.sourceSchema = PaymentV1
  Profile1.targetSchema = PaymentV1
  Profile2.sourceSchema = Profile1.targetSchema
  Profile2.targetSchema = PaymentV1
  Profile1.conversionFactor = 100
  Profile2.conversionFactor = 100
  RootStep.renderedAmount = RootStep.effect.maxAmount * Profile1.conversionFactor
  DownstreamStep.renderedAmount = DownstreamStep.effect.maxAmount * Profile2.conversionFactor
  Profile1.contextDigest = CanonicalContextDigest
  Profile2.contextDigest = CanonicalContextDigest
  RootStep.executionContextDigest = CanonicalContextDigest
  DownstreamStep.executionContextDigest = CanonicalContextDigest
}

fact fieldSemanticsExact {
  RootStep.effectFields in {OperationField, TargetField, AmountField, CurrencyField, DestinationField, RiskScoreField}
  DownstreamStep.effectFields in {OperationField, TargetField, AmountField, CurrencyField, DestinationField, RiskScoreField}
  RootStep.effectFields in RootDecision.fields
  DownstreamStep.effectFields in RootDecision.fields
  RootStep.sourceOperation = OperationField
  RootStep.sourceTarget = TargetField
  RootStep.sourceAmount = AmountField
  RootStep.sourceCurrency = CurrencyField
  DownstreamStep.sourceOperation = OperationField
  DownstreamStep.sourceTarget = TargetField
  DownstreamStep.sourceAmount = AmountField
  DownstreamStep.sourceCurrency = CurrencyField
}

fact monotonicityDeclared {
  MonotoneProfile.monotonicityDeclared = On
}

fact monotonicityContract {
  monotoneContractive
  monotoneRelationExact
}

fact AttenuationContracts {
  RootStep.committed = On
  DownstreamStep.committed = On
  DownstreamStep.upstream = RootStep
  RootStep.inputAuthority = RootDecision.authority
}

fact ScopeExtensionality {
  all a,b: Scope |
    (a.operations = b.operations
      and a.targets = b.targets
      and a.audiences = b.audiences
      and a.currencies = b.currencies
      and a.maxAmount = b.maxAmount) implies a = b
}

assert ScopeOrderReflexive {
  all s: Scope | scopeLeq[s,s]
}

assert ScopeOrderTransitive {
  all a,b,c: Scope |
    scopeLeq[a,b] and scopeLeq[b,c] implies scopeLeq[a,c]
}

assert ScopeOrderAntisymmetric {
  all a,b: Scope |
    scopeLeq[a,b] and scopeLeq[b,a] implies a = b
}

assert NonExpandingSemanticTransformCompositionExact {
  RootStep.committed = On
  DownstreamStep.committed = On
  RootStep.inputAuthority = RootDecision.authority
  DownstreamStep.upstream = RootStep
  DownstreamStep.inputAuthority = RootStep.effect
  DownstreamStep.decision = RootStep.decision
  coreScopeLeq[Profile1.ceiling, RootDecision.authority]
  coreScopeLeq[RootStep.effect, Profile1.ceiling]
  coreScopeLeq[Profile2.ceiling, RootStep.effect]
  coreScopeLeq[DownstreamStep.effect, Profile2.ceiling]
  audienceAttenuationExact
  coreScopeLeq[DownstreamStep.effect, RootDecision.authority]
  RootStep.sourceDecisionId = CanonicalDecisionId
  DownstreamStep.sourceDecisionId = CanonicalDecisionId
  RootStep.effectId = CanonicalEffectId
  DownstreamStep.effectId != RootStep.effectId
  DownstreamStep.sourceEffectId = CanonicalEffectId
  Profile1.id = CanonicalProfile1Id
  Profile2.id = CanonicalProfile2Id
  Profile1.implementationHash = CanonicalImplementationHash1
  Profile2.implementationHash = CanonicalImplementationHash2
  Profile2.derivationRule = NoDerivation
  DownstreamStep.sourceAmount = AmountField
  MonotoneProfile.monotonicityDeclared = On
  MonotoneProfile.contextDigest = CanonicalContextDigest
  monotoneContractive
  monotoneRelationExact
}

pred CanonicalTwoHopTransformWitness {
  RootStep.committed = On
  DownstreamStep.committed = On
  RootStep.decision = RootDecision
  RootStep.decision.authority.operations = TransferOp
  RootStep.decision.authority.targets = AliceTarget + BobTarget
  RootStep.decision.authority.audiences = PaymentsAudience
  RootStep.decision.authority.currencies = USD
  RootStep.decision.authority.maxAmount = 100
  RootDecision.id = CanonicalDecisionId
  RootDecision.fields = OperationField + TargetField + AmountField + CurrencyField
  RootDecision.sourceSchema = PaymentV1
  RootStep.effectId = CanonicalEffectId
  RootStep.inputAuthority = RootDecision.authority
  RootStep.effect.operations = TransferOp
  RootStep.effect.targets = AliceTarget
  RootStep.effect.audiences = PaymentsAudience
  RootStep.effect.currencies = USD
  RootStep.effect.maxAmount = 40
  RootStep.effectFields = OperationField + TargetField + AmountField + CurrencyField
  RootStep.sourceDecisionId = CanonicalDecisionId
  RootStep.sourceOperation = OperationField
  RootStep.sourceTarget = TargetField
  RootStep.sourceAmount = AmountField
  RootStep.sourceCurrency = CurrencyField
  Profile1.id = CanonicalProfile1Id
  Profile1.implementationHash = CanonicalImplementationHash1
  Profile1.derivationRule = NoDerivation
  Profile1.ceiling.operations = TransferOp
  Profile1.ceiling.targets = AliceTarget
  Profile1.ceiling.audiences = PaymentsAudience
  Profile1.ceiling.currencies = USD
  Profile1.ceiling.maxAmount = 60
  Profile1.sourceSchema = PaymentV1
  Profile1.targetSchema = PaymentV1
  Profile1.conversionFactor = 100
  Profile1.contextDigest = CanonicalContextDigest
  RootStep.renderedAmount = 4000
  RootStep.executionContextDigest = CanonicalContextDigest
  DownstreamStep.profile = Profile2
  DownstreamStep.decision = RootDecision
  DownstreamStep.effectId = AlternateEffectId
  DownstreamStep.effect.operations = TransferOp
  DownstreamStep.effect.targets = AliceTarget
  DownstreamStep.effect.audiences = PaymentsAudience
  DownstreamStep.effect.currencies = USD
  DownstreamStep.effect.maxAmount = 20
  DownstreamStep.effectFields = OperationField + TargetField + AmountField + CurrencyField
  DownstreamStep.sourceDecisionId = CanonicalDecisionId
  DownstreamStep.sourceEffectId = CanonicalEffectId
  DownstreamStep.inputAuthority = RootStep.effect
  DownstreamStep.sourceOperation = OperationField
  DownstreamStep.sourceTarget = TargetField
  DownstreamStep.sourceAmount = AmountField
  DownstreamStep.sourceCurrency = CurrencyField
  DownstreamStep.renderedAmount = 2000
  DownstreamStep.executionContextDigest = CanonicalContextDigest
  DownstreamStep.upstream = RootStep
  Profile2.id = CanonicalProfile2Id
  Profile2.implementationHash = CanonicalImplementationHash2
  Profile2.derivationRule = NoDerivation
  Profile2.ceiling.operations = TransferOp
  Profile2.ceiling.targets = AliceTarget
  Profile2.ceiling.audiences = PaymentsAudience
  Profile2.ceiling.currencies = USD
  Profile2.ceiling.maxAmount = 30
  Profile2.sourceSchema = PaymentV1
  Profile2.targetSchema = PaymentV1
  Profile2.conversionFactor = 100
  Profile2.contextDigest = CanonicalContextDigest
  MonotoneProfile.monotonicityDeclared = On
  MonotoneProfile.ceiling.operations = TransferOp
  MonotoneProfile.ceiling.targets = AliceTarget
  MonotoneProfile.ceiling.audiences = PaymentsAudience
  MonotoneProfile.ceiling.currencies = USD
  MonotoneProfile.ceiling.maxAmount = 5
  MonotoneProfile.contextDigest = CanonicalContextDigest
  MonotoneRelation.inputA.operations = TransferOp
  MonotoneRelation.inputA.targets = AliceTarget
  MonotoneRelation.inputA.audiences = PaymentsAudience
  MonotoneRelation.inputA.currencies = USD
  MonotoneRelation.inputA.maxAmount = 10
  MonotoneRelation.inputB.operations = TransferOp
  MonotoneRelation.inputB.targets = AliceTarget
  MonotoneRelation.inputB.audiences = PaymentsAudience
  MonotoneRelation.inputB.currencies = USD
  MonotoneRelation.inputB.maxAmount = 20
  MonotoneRelation.effectA.operations = TransferOp
  MonotoneRelation.effectA.targets = AliceTarget
  MonotoneRelation.effectA.audiences = PaymentsAudience
  MonotoneRelation.effectA.currencies = USD
  MonotoneRelation.effectA.maxAmount = 8
  MonotoneRelation.effectB.operations = TransferOp
  MonotoneRelation.effectB.targets = AliceTarget
  MonotoneRelation.effectB.audiences = PaymentsAudience
  MonotoneRelation.effectB.currencies = USD
  MonotoneRelation.effectB.maxAmount = 12
}

pred UpstreamDecisionIdentitySubstitutionWitness {
  RootStep.committed = On
  RootStep.sourceDecisionId = AlternateDecisionId
}

pred DerivationRuleSubstitutionWitness {
  DownstreamStep.committed = On
  Profile2.derivationRule = AlternateDerivation
}

pred SourceAmountSubstitutionWitness {
  DownstreamStep.committed = On
  DownstreamStep.sourceAmount = CurrencyField
}

pred AudienceWideningWitness {
  DownstreamStep.committed = On
  Profile2.ceiling.audiences = PaymentsAudience + AdminAudience
  DownstreamStep.effect.audiences = PaymentsAudience
}

pred SecondHopProfileCeilingWideningWitness {
  DownstreamStep.committed = On
  Profile2.ceiling.maxAmount = 70
  DownstreamStep.effect.maxAmount = 20
}

pred SecondHopEffectWideningWitness {
  DownstreamStep.committed = On
  DownstreamStep.effect.maxAmount = 50
  Profile2.ceiling.maxAmount = 30
}

pred ChainLinkSubstitutionWitness {
  DownstreamStep.committed = On
  DownstreamStep.upstream = none
  DownstreamStep.sourceEffectId = AlternateEffectId
  DownstreamStep.inputAuthority.maxAmount = 60
  DownstreamStep.effect.maxAmount = 20
}

pred MissingMonotonicityDeclarationWitness {
  MonotoneProfile.monotonicityDeclared = Off
}

pred MonotonicityViolationWitness {
  MonotoneRelation.effectA.maxAmount = 8
  MonotoneRelation.effectB.maxAmount = 3
  MonotoneRelation.inputA.maxAmount = 10
  MonotoneRelation.inputB.maxAmount = 20
}

pred DownstreamImplementationSubstitutionWitness {
  DownstreamStep.committed = On
  Profile2.implementationHash = AlternateImplementationHash
}

check ScopeOrderReflexive for 12 but 8 Scope, 6 TransformStep, 3 TransformProfile, 4 Decision
check ScopeOrderTransitive for 12 but 8 Scope, 6 TransformStep, 3 TransformProfile, 4 Decision
check ScopeOrderAntisymmetric for 12 but 8 Scope, 6 TransformStep, 3 TransformProfile, 4 Decision
check NonExpandingSemanticTransformCompositionExact for 12 but 8 Scope, 6 TransformStep, 3 TransformProfile, 4 Decision
run CanonicalTwoHopTransformWitness for 12 but 8 Scope, 6 TransformStep, 3 TransformProfile, 4 Decision
run UpstreamDecisionIdentitySubstitutionWitness for 12 but 8 Scope, 6 TransformStep, 3 TransformProfile, 4 Decision
run DerivationRuleSubstitutionWitness for 12 but 8 Scope, 6 TransformStep, 3 TransformProfile, 4 Decision
run SourceAmountSubstitutionWitness for 12 but 8 Scope, 6 TransformStep, 3 TransformProfile, 4 Decision
run AudienceWideningWitness for 12 but 8 Scope, 6 TransformStep, 3 TransformProfile, 4 Decision
run SecondHopProfileCeilingWideningWitness for 12 but 8 Scope, 6 TransformStep, 3 TransformProfile, 4 Decision
run SecondHopEffectWideningWitness for 12 but 8 Scope, 6 TransformStep, 3 TransformProfile, 4 Decision
run ChainLinkSubstitutionWitness for 12 but 8 Scope, 6 TransformStep, 3 TransformProfile, 4 Decision
run MissingMonotonicityDeclarationWitness for 12 but 8 Scope, 6 TransformStep, 3 TransformProfile, 4 Decision
run MonotonicityViolationWitness for 12 but 8 Scope, 6 TransformStep, 3 TransformProfile, 4 Decision
run DownstreamImplementationSubstitutionWitness for 12 but 8 Scope, 6 TransformStep, 3 TransformProfile, 4 Decision
