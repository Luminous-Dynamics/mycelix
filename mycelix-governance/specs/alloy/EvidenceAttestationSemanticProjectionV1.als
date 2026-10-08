module EvidenceAttestationSemanticProjectionV1

enum Bit { On, Off }

abstract sig Field {}
one sig Operation, Target, Amount, Currency, Destination, UnknownExtra extends Field {}

abstract sig Schema {}
one sig PaymentV1, PaymentV2 extends Schema {}

abstract sig AtomValue {}
one sig TransferValue, AliceTarget, Amount100, Amount200, UsdValue extends AtomValue {}

sig Decision {
  authorizedFields: set Field,
  decisionValueFields: set Field,
  operationValue: one AtomValue,
  targetValue: one AtomValue,
  amountValue: one AtomValue,
  currencyValue: one AtomValue,
  schema: one Schema
}

sig EffectProjection {
  decision: one Decision,
  effectFields: set Field,
  projectedOperation: one AtomValue,
  projectedTarget: one AtomValue,
  projectedAmount: one AtomValue,
  projectedCurrency: one AtomValue,
  sourceOperation: one Field,
  sourceTarget: one Field,
  sourceAmount: one Field,
  sourceCurrency: one Field,
  defaultedFields: set Field,
  schema: one Schema,
  committed: one Bit
}

fact EffectFieldsKnown {
  all p: EffectProjection |
    p.committed = On implies
      p.effectFields in {Operation, Target, Amount, Currency, Destination}
}

fact EffectFieldsAuthorized {
  all p: EffectProjection |
    p.committed = On implies
      p.effectFields in p.decision.authorizedFields
}

fact RequiredFieldsPresent {
  all p: EffectProjection |
    p.committed = On implies
      {Operation, Target, Amount, Currency} in p.effectFields
}

fact ProjectionSourcesExact {
  all p: EffectProjection |
    p.committed = On implies
      p.sourceOperation = Operation and
      p.sourceTarget = Target and
      p.sourceAmount = Amount and
      p.sourceCurrency = Currency
}

fact ProjectedValuesConserved {
  all p: EffectProjection |
    p.committed = On implies
      (Operation in p.effectFields implies p.projectedOperation = p.decision.operationValue) and
      (Target in p.effectFields implies p.projectedTarget = p.decision.targetValue) and
      (Amount in p.effectFields implies p.projectedAmount = p.decision.amountValue) and
      (Currency in p.effectFields implies p.projectedCurrency = p.decision.currencyValue)
}

fact NoImplicitDefaultAuthority {
  all p: EffectProjection |
    p.committed = On implies
      p.defaultedFields in p.decision.decisionValueFields
}

fact SchemaProfileExact {
  all p: EffectProjection |
    p.committed = On implies
      p.schema = p.decision.schema
}

assert SemanticProjectionExact {
  all p: EffectProjection |
    p.committed = On implies
      p.effectFields in {Operation, Target, Amount, Currency, Destination} and
      p.effectFields in p.decision.authorizedFields and
      {Operation, Target, Amount, Currency} in p.effectFields and
      p.sourceOperation = Operation and
      p.sourceTarget = Target and
      p.sourceAmount = Amount and
      p.sourceCurrency = Currency and
      (Operation in p.effectFields implies p.projectedOperation = p.decision.operationValue) and
      (Target in p.effectFields implies p.projectedTarget = p.decision.targetValue) and
      (Amount in p.effectFields implies p.projectedAmount = p.decision.amountValue) and
      (Currency in p.effectFields implies p.projectedCurrency = p.decision.currencyValue) and
      p.defaultedFields in p.decision.decisionValueFields and
      p.schema = p.decision.schema
}

pred CanonicalProjectionWitness {
  some d: Decision, p: EffectProjection |
    p.committed = On and p.decision = d and
    d.authorizedFields = {Operation, Target, Amount, Currency} and
    d.decisionValueFields = {Operation, Target, Amount, Currency} and
    p.effectFields = {Operation, Target, Amount, Currency} and
    p.defaultedFields = none and
    d.operationValue = TransferValue and d.targetValue = AliceTarget and
    d.amountValue = Amount100 and d.currencyValue = UsdValue and
    p.projectedOperation = TransferValue and p.projectedTarget = AliceTarget and
    p.projectedAmount = Amount100 and p.projectedCurrency = UsdValue and
    p.sourceOperation = Operation and p.sourceTarget = Target and
    p.sourceAmount = Amount and p.sourceCurrency = Currency and
    d.schema = PaymentV1 and p.schema = PaymentV1
}

pred UncoveredFieldWitness {
  some d: Decision, p: EffectProjection |
    p.committed = On and p.decision = d and
    d.authorizedFields = {Operation, Target, Amount, Currency} and
    d.decisionValueFields = {Operation, Target, Amount, Currency} and
    p.effectFields = {Operation, Target, Amount, Currency, Destination} and
    Destination not in d.authorizedFields and
    p.defaultedFields = none and
    d.operationValue = TransferValue and d.targetValue = AliceTarget and
    d.amountValue = Amount100 and d.currencyValue = UsdValue and
    p.projectedOperation = TransferValue and p.projectedTarget = AliceTarget and
    p.projectedAmount = Amount100 and p.projectedCurrency = UsdValue and
    p.sourceOperation = Operation and p.sourceTarget = Target and
    p.sourceAmount = Amount and p.sourceCurrency = Currency and
    d.schema = PaymentV1 and p.schema = PaymentV1
}

pred UnknownFieldWitness {
  some d: Decision, p: EffectProjection |
    p.committed = On and p.decision = d and
    d.authorizedFields = {Operation, Target, Amount, Currency, UnknownExtra} and
    d.decisionValueFields = {Operation, Target, Amount, Currency} and
    p.effectFields = {Operation, Target, Amount, Currency, UnknownExtra} and
    p.defaultedFields = none and
    d.operationValue = TransferValue and d.targetValue = AliceTarget and
    d.amountValue = Amount100 and d.currencyValue = UsdValue and
    p.projectedOperation = TransferValue and p.projectedTarget = AliceTarget and
    p.projectedAmount = Amount100 and p.projectedCurrency = UsdValue and
    p.sourceOperation = Operation and p.sourceTarget = Target and
    p.sourceAmount = Amount and p.sourceCurrency = Currency and
    d.schema = PaymentV1 and p.schema = PaymentV1
}

pred ValueSubstitutionWitness {
  some d: Decision, p: EffectProjection |
    p.committed = On and p.decision = d and
    d.authorizedFields = {Operation, Target, Amount, Currency} and
    d.decisionValueFields = {Operation, Target, Amount, Currency} and
    p.effectFields = {Operation, Target, Amount, Currency} and
    p.projectedAmount != d.amountValue and
    p.projectedOperation = d.operationValue and
    p.projectedTarget = d.targetValue and
    p.projectedCurrency = d.currencyValue and
    p.sourceOperation = Operation and p.sourceTarget = Target and
    p.sourceAmount = Amount and p.sourceCurrency = Currency and
    p.defaultedFields = none and
    d.schema = PaymentV1 and p.schema = PaymentV1
}

pred SourceSubstitutionWitness {
  some d: Decision, p: EffectProjection |
    p.committed = On and p.decision = d and
    d.authorizedFields = {Operation, Target, Amount, Currency} and
    d.decisionValueFields = {Operation, Target, Amount, Currency} and
    p.effectFields = {Operation, Target, Amount, Currency} and
    p.sourceOperation != Operation and
    p.sourceTarget = Target and p.sourceAmount = Amount and
    p.sourceCurrency = Currency and
    p.projectedOperation = d.operationValue and
    p.projectedTarget = d.targetValue and
    p.projectedAmount = d.amountValue and
    p.projectedCurrency = d.currencyValue and
    p.defaultedFields = none and
    d.schema = PaymentV1 and p.schema = PaymentV1
}

pred DefaultWitness {
  some d: Decision, p: EffectProjection |
    p.committed = On and p.decision = d and
    d.authorizedFields = {Operation, Target, Amount, Currency} and
    d.decisionValueFields = {Operation, Target, Amount} and
    p.effectFields = {Operation, Target, Amount, Currency} and
    p.defaultedFields = {Currency} and
    d.operationValue = TransferValue and d.targetValue = AliceTarget and
    d.amountValue = Amount100 and d.currencyValue = UsdValue and
    p.projectedOperation = TransferValue and p.projectedTarget = AliceTarget and
    p.projectedAmount = Amount100 and p.projectedCurrency = UsdValue and
    p.sourceOperation = Operation and p.sourceTarget = Target and
    p.sourceAmount = Amount and p.sourceCurrency = Currency and
    d.schema = PaymentV1 and p.schema = PaymentV1
}

pred SchemaSubstitutionWitness {
  some d: Decision, p: EffectProjection |
    p.committed = On and p.decision = d and
    d.authorizedFields = {Operation, Target, Amount, Currency} and
    d.decisionValueFields = {Operation, Target, Amount, Currency} and
    p.effectFields = {Operation, Target, Amount, Currency} and
    p.defaultedFields = none and
    d.operationValue = TransferValue and d.targetValue = AliceTarget and
    d.amountValue = Amount100 and d.currencyValue = UsdValue and
    p.projectedOperation = TransferValue and p.projectedTarget = AliceTarget and
    p.projectedAmount = Amount100 and p.projectedCurrency = UsdValue and
    p.sourceOperation = Operation and p.sourceTarget = Target and
    p.sourceAmount = Amount and p.sourceCurrency = Currency and
    d.schema = PaymentV1 and p.schema = PaymentV2
}

pred RequiredOmissionWitness {
  some d: Decision, p: EffectProjection |
    p.committed = On and p.decision = d and
    d.authorizedFields = {Operation, Target, Amount, Currency} and
    d.decisionValueFields = {Operation, Target, Amount, Currency} and
    p.effectFields = {Operation, Target, Amount} and
    p.defaultedFields = none and
    d.operationValue = TransferValue and d.targetValue = AliceTarget and
    d.amountValue = Amount100 and d.currencyValue = UsdValue and
    p.projectedOperation = TransferValue and p.projectedTarget = AliceTarget and
    p.projectedAmount = Amount100 and p.projectedCurrency = UsdValue and
    p.sourceOperation = Operation and p.sourceTarget = Target and
    p.sourceAmount = Amount and p.sourceCurrency = Currency and
    d.schema = PaymentV1 and p.schema = PaymentV1
}

check SemanticProjectionExact for 12 but 6 Decision, 6 EffectProjection, 6 AtomValue

run CanonicalProjectionWitness for 12 but 6 Decision, 6 EffectProjection, 6 AtomValue
run UncoveredFieldWitness for 12 but 6 Decision, 6 EffectProjection, 6 AtomValue
run UnknownFieldWitness for 12 but 6 Decision, 6 EffectProjection, 6 AtomValue
run ValueSubstitutionWitness for 12 but 6 Decision, 6 EffectProjection, 6 AtomValue
run SourceSubstitutionWitness for 12 but 6 Decision, 6 EffectProjection, 6 AtomValue
run DefaultWitness for 12 but 6 Decision, 6 EffectProjection, 6 AtomValue
run SchemaSubstitutionWitness for 12 but 6 Decision, 6 EffectProjection, 6 AtomValue
run RequiredOmissionWitness for 12 but 6 Decision, 6 EffectProjection, 6 AtomValue
