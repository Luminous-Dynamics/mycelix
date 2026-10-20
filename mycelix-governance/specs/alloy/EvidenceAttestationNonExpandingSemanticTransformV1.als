module EvidenceAttestationNonExpandingSemanticTransformV1

enum Bit { On, Off }

abstract sig Operation {}
one sig TransferOp, RefundOp extends Operation {}

abstract sig Target {}
one sig AliceTarget, BobTarget extends Target {}

abstract sig Currency {}
one sig USD, EUR extends Currency {}

abstract sig Field {}
one sig OperationField, TargetField, AmountField, CurrencyField, DestinationField, RiskScoreField extends Field {}

abstract sig Schema {}
one sig PaymentV1, PaymentV2 extends Schema {}

abstract sig ProfileId {}
one sig CanonicalProfileId, AlternateProfileId extends ProfileId {}

abstract sig ImplementationHash {}
one sig CanonicalImplementationHash, AlternateImplementationHash extends ImplementationHash {}

abstract sig ContextDigest {}
one sig CanonicalContextDigest, AlternateContextDigest extends ContextDigest {}

abstract sig DerivationRule {}
one sig NoDerivation, CanonicalDerivation, AlternateDerivation extends DerivationRule {}

sig Scope {
  operations: set Operation,
  targets: set Target,
  currencies: set Currency,
  maxAmount: one Int
}

sig Decision {
  id: one Int,
  authority: one Scope,
  fields: set Field,
  sourceSchema: one Schema
}

sig TransformProfile {
  id: one ProfileId,
  implementationHash: one ImplementationHash,
  ceiling: one Scope,
  derivedFields: set Field,
  declaredDerivationFields: set Field,
  sourceSchema: one Schema,
  targetSchema: one Schema,
  conversionFactor: one Int,
  contextDigest: one ContextDigest
}

sig EffectProjection {
  decision: one Decision,
  profile: one TransformProfile,
  effectScope: one Scope,
  effectFields: set Field,
  defaultedFields: set Field,
  externalFields: set Field,
  sourceOperation: one Field,
  sourceTarget: one Field,
  sourceAmount: one Field,
  sourceCurrency: one Field,
  renderedAmount: one Int,
  executionContextDigest: one ContextDigest,
  usedDerivationRule: one DerivationRule,
  committed: one Bit
}

pred scopeLeq[a,b: Scope] {
  a.operations in b.operations
  a.targets in b.targets
  a.currencies in b.currencies
  a.maxAmount <= b.maxAmount
}

fact ProfileCeilingNarrowed {
  all p: EffectProjection |
    p.committed = On implies
      scopeLeq[p.profile.ceiling, p.decision.authority]
}

fact EffectScopeNarrowed {
  all p: EffectProjection |
    p.committed = On implies
      scopeLeq[p.effectScope, p.profile.ceiling]
}

fact EffectScopeConsistent {
  all p: EffectProjection |
    p.committed = On implies
      p.effectScope.operations = {TransferOp}
      and p.effectScope.targets = {AliceTarget}
      and p.effectScope.currencies = {USD}
      and p.effectScope.maxAmount = p.renderedAmount / (p.profile.conversionFactor)
}

fact EffectFieldsKnown {
  all p: EffectProjection |
    p.committed = On implies
      p.effectFields in {OperationField, TargetField, AmountField, CurrencyField, DestinationField, RiskScoreField}
}

fact EffectFieldsBacked {
  all p: EffectProjection |
    p.committed = On implies
      (p.effectFields - p.profile.derivedFields - p.defaultedFields) in p.decision.fields
}

fact DeclaredDerivationExact {
  all p: EffectProjection |
    p.committed = On implies
      p.profile.derivedFields in p.profile.declaredDerivationFields
      and (some p.profile.derivedFields implies p.usedDerivationRule = CanonicalDerivation)
      and (no p.profile.derivedFields implies p.usedDerivationRule = NoDerivation)
}

fact FieldSourcesExact {
  all p: EffectProjection |
    p.committed = On implies
      p.sourceOperation = OperationField
      and p.sourceTarget = TargetField
      and p.sourceAmount = AmountField
      and p.sourceCurrency = CurrencyField
}

fact ProfileIdentityExact {
  all p: EffectProjection |
    p.committed = On implies
      p.profile.id = CanonicalProfileId
}

fact ImplementationIdentityExact {
  all p: EffectProjection |
    p.committed = On implies
      p.profile.implementationHash = CanonicalImplementationHash
}

fact SchemaIdentityExact {
  all p: EffectProjection |
    p.committed = On implies
      p.profile.sourceSchema = PaymentV1
      and p.profile.targetSchema = PaymentV1
      and p.decision.sourceSchema = PaymentV1
}

fact ConversionRuleExact {
  all p: EffectProjection |
    p.committed = On implies
      p.profile.conversionFactor = 100
      and p.renderedAmount = p.effectScope.maxAmount * p.profile.conversionFactor / 1
}

fact NoImplicitDefaults {
  all p: EffectProjection |
    p.committed = On implies
      no p.defaultedFields
}

fact NoExternalEnrichment {
  all p: EffectProjection |
    p.committed = On implies
      no p.externalFields
}

fact DeterministicContextExact {
  all p: EffectProjection |
    p.committed = On implies
      p.profile.contextDigest = CanonicalContextDigest
      and p.executionContextDigest = CanonicalContextDigest
}

assert NonExpandingSemanticTransformExact {
  all p: EffectProjection |
    p.committed = On implies
      scopeLeq[p.profile.ceiling, p.decision.authority]
      and scopeLeq[p.effectScope, p.profile.ceiling]
      and p.effectFields in {OperationField, TargetField, AmountField, CurrencyField, DestinationField, RiskScoreField}
      and (p.effectFields - p.profile.derivedFields - p.defaultedFields) in p.decision.fields
      and p.profile.derivedFields in p.profile.declaredDerivationFields
      and p.sourceOperation = OperationField
      and p.sourceTarget = TargetField
      and p.sourceAmount = AmountField
      and p.sourceCurrency = CurrencyField
      and p.profile.id = CanonicalProfileId
      and p.profile.implementationHash = CanonicalImplementationHash
      and p.profile.sourceSchema = PaymentV1
      and p.profile.targetSchema = PaymentV1
      and p.profile.conversionFactor = 100
      and p.renderedAmount = p.effectScope.maxAmount * p.profile.conversionFactor / 1
      and no p.defaultedFields
      and no p.externalFields
      and p.profile.contextDigest = CanonicalContextDigest
      and p.executionContextDigest = CanonicalContextDigest
      and (some p.profile.derivedFields implies p.usedDerivationRule = CanonicalDerivation)
      and (no p.profile.derivedFields implies p.usedDerivationRule = NoDerivation)
}

pred CanonicalTransformWitness {
  some d: Decision, p: TransformProfile, e: EffectProjection |
    e.committed = On
    and e.decision = d
    and e.profile = p
    and d.authority.operations = {TransferOp}
    and d.authority.targets = {AliceTarget, BobTarget}
    and d.authority.currencies = {USD}
    and d.authority.maxAmount = 100
    and d.fields = {OperationField, TargetField, AmountField, CurrencyField}
    and d.sourceSchema = PaymentV1
    and p.id = CanonicalProfileId
    and p.implementationHash = CanonicalImplementationHash
    and p.ceiling.operations = {TransferOp}
    and p.ceiling.targets = {AliceTarget}
    and p.ceiling.currencies = {USD}
    and p.ceiling.maxAmount = 50
    and p.derivedFields = none
    and p.declaredDerivationFields = none
    and p.sourceSchema = PaymentV1
    and p.targetSchema = PaymentV1
    and p.conversionFactor = 100
    and p.contextDigest = CanonicalContextDigest
    and e.effectScope.operations = {TransferOp}
    and e.effectScope.targets = {AliceTarget}
    and e.effectScope.currencies = {USD}
    and e.effectScope.maxAmount = 40
    and e.effectFields = {OperationField, TargetField, AmountField, CurrencyField}
    and no e.defaultedFields
    and no e.externalFields
    and e.sourceOperation = OperationField
    and e.sourceTarget = TargetField
    and e.sourceAmount = AmountField
    and e.sourceCurrency = CurrencyField
    and e.renderedAmount = 4000
    and e.executionContextDigest = CanonicalContextDigest
    and e.usedDerivationRule = NoDerivation
}

pred UndeclaredDerivationWitness {
  some d: Decision, p: TransformProfile, e: EffectProjection |
    e.committed = On
    and e.decision = d
    and e.profile = p
    and d.fields = {OperationField, TargetField, AmountField, CurrencyField}
    and p.derivedFields = {DestinationField}
    and p.declaredDerivationFields = none
    and e.effectFields = {OperationField, TargetField, AmountField, CurrencyField, DestinationField}
    and e.usedDerivationRule = NoDerivation
    and e.profile.id = CanonicalProfileId
    and e.profile.implementationHash = CanonicalImplementationHash
    and e.profile.conversionFactor = 100
    and e.renderedAmount = 4000
    and e.effectScope.maxAmount = 40
    and no e.defaultedFields
    and no e.externalFields
    and e.executionContextDigest = CanonicalContextDigest
}

pred ProfileCeilingWideningWitness {
  some e: EffectProjection |
    e.committed = On
    and e.profile.ceiling.maxAmount = 150
    and e.effectScope.maxAmount = 40
    and e.profile.ceiling.targets = {AliceTarget, BobTarget}
}

pred PrivilegeWideningWitness {
  some e: EffectProjection |
    e.committed = On
    and e.effectScope.maxAmount = 60
    and e.profile.ceiling.maxAmount = 50
    and e.renderedAmount = 6000
}

pred CrossFieldContaminationWitness {
  some e: EffectProjection |
    e.committed = On
    and e.sourceTarget = AmountField
    and e.sourceOperation = OperationField
    and e.sourceAmount = AmountField
    and e.sourceCurrency = CurrencyField
}

pred ProfileSubstitutionWitness {
  some e: EffectProjection |
    e.committed = On
    and e.profile.id = AlternateProfileId
    and e.profile.implementationHash = CanonicalImplementationHash
}

pred ImplementationHashSubstitutionWitness {
  some e: EffectProjection |
    e.committed = On
    and e.profile.id = CanonicalProfileId
    and e.profile.implementationHash = AlternateImplementationHash
}

pred ConversionRuleSubstitutionWitness {
  some e: EffectProjection |
    e.committed = On
    and e.profile.conversionFactor = 50
    and e.renderedAmount = 2000
    and e.effectScope.maxAmount = 40
}

pred DefaultReconstructionWitness {
  some e: EffectProjection |
    e.committed = On
    and e.defaultedFields = {CurrencyField}
}

pred ExternalEnrichmentWitness {
  some e: EffectProjection |
    e.committed = On
    and e.externalFields = {RiskScoreField}
}

pred UnboundContextWitness {
  some e: EffectProjection |
    e.committed = On
    and e.executionContextDigest = AlternateContextDigest
    and e.profile.contextDigest = CanonicalContextDigest
}

check NonExpandingSemanticTransformExact for 12 but 8 Scope, 6 EffectProjection, 3 Decision, 3 TransformProfile

run CanonicalTransformWitness for 12 but 8 Scope, 6 EffectProjection, 3 Decision, 3 TransformProfile
run ProfileCeilingWideningWitness for 12 but 8 Scope, 6 EffectProjection, 3 Decision, 3 TransformProfile
run UndeclaredDerivationWitness for 12 but 8 Scope, 6 EffectProjection, 3 Decision, 3 TransformProfile
run PrivilegeWideningWitness for 12 but 8 Scope, 6 EffectProjection, 3 Decision, 3 TransformProfile
run CrossFieldContaminationWitness for 12 but 8 Scope, 6 EffectProjection, 3 Decision, 3 TransformProfile
run ProfileSubstitutionWitness for 12 but 8 Scope, 6 EffectProjection, 3 Decision, 3 TransformProfile
run ImplementationHashSubstitutionWitness for 12 but 8 Scope, 6 EffectProjection, 3 Decision, 3 TransformProfile
run ConversionRuleSubstitutionWitness for 12 but 8 Scope, 6 EffectProjection, 3 Decision, 3 TransformProfile
run DefaultReconstructionWitness for 12 but 8 Scope, 6 EffectProjection, 3 Decision, 3 TransformProfile
run ExternalEnrichmentWitness for 12 but 8 Scope, 6 EffectProjection, 3 Decision, 3 TransformProfile
run UnboundContextWitness for 12 but 8 Scope, 6 EffectProjection, 3 Decision, 3 TransformProfile
