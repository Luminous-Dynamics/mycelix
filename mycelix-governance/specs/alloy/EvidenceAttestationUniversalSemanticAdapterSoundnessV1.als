module EvidenceAttestationUniversalSemanticAdapterSoundnessV1

enum Bit { On, Off }
abstract sig ControlKind {}
one sig CanonicalControl, MonotonicitySampledControl, MonotonicityViolationControl,
UndeclaredDomainControl, SemanticTypeControl, UnitControl, ReconstitutionControl,
ImplementationControl, ExternalControl, PartialControl, PreconditionControl extends ControlKind {}
one sig RunState { control: one ControlKind }

abstract sig AdapterId {}
one sig CanonicalAdapterId, AlternateAdapterId extends AdapterId {}
abstract sig ImplementationHash {}
one sig CanonicalImplementationHash, AlternateImplementationHash extends ImplementationHash {}
abstract sig SemanticType {}
one sig PaymentIntentType, EntitlementType extends SemanticType {}
abstract sig UnitRule {}
one sig IdentityUnitRule, RoundUpUnitRule extends UnitRule {}
abstract sig QuantityRule {}
one sig ExactFloorHalfRule, RoundUpRule extends QuantityRule {}
abstract sig ExternalState {}
one sig NoExternalState, LiveRiskApiV2 extends ExternalState {}
abstract sig AdapterStatus {}
one sig CompleteStatus, PartialStatus extends AdapterStatus {}
abstract sig Field {}
one sig SecretField extends Field {}
abstract sig TransformVariant {}
one sig CanonicalVariant, NonMonotoneVariant extends TransformVariant {}

one sig Adapter {
  id: one AdapterId,
  implementationHash: one ImplementationHash,
  sourceType: one SemanticType,
  targetType: one SemanticType,
  unitRule: one UnitRule,
  quantityRule: one QuantityRule,
  externalState: one ExternalState,
  status: one AdapterStatus,
  precondition: one Bit,
  declaredDomainMax: one Int,
  removedFields: set Field,
  reconstitutedFields: set Field
}
one sig Transform {
  variant: one TransformVariant,
  domain: set Int,
  monotonicityDomain: set Int,
  output: Int -> Int
}

pred OutputTotal { all x: Transform.domain | one x.(Transform.output) }
pred NonExpansion { all x: Transform.domain | x.(Transform.output) <= x }
pred Monotone { all a,b: Transform.monotonicityDomain | a <= b implies a.(Transform.output) <= b.(Transform.output) }
pred DomainExact { Transform.monotonicityDomain = Transform.domain }
pred AdapterDomainExact { Adapter.declaredDomainMax = 20 }
pred TypeExact { Adapter.sourceType = PaymentIntentType and Adapter.targetType = PaymentIntentType }
pred UnitExact { Adapter.unitRule = IdentityUnitRule and Adapter.quantityRule = ExactFloorHalfRule }
pred ReconstitutionExact { Adapter.reconstitutedFields & Adapter.removedFields = none }
pred ImplementationExact { Adapter.implementationHash = CanonicalImplementationHash }
pred ExternalExact { Adapter.externalState = NoExternalState }
pred StatusExact { Adapter.status = CompleteStatus }
pred IdentityExact { Adapter.id = CanonicalAdapterId }
pred PreconditionExact { Adapter.precondition = On }

pred Strict {
  OutputTotal
  NonExpansion
  DomainExact
  Monotone
  AdapterDomainExact
  TypeExact
  UnitExact
  ReconstitutionExact
  ImplementationExact
  ExternalExact
  StatusExact
  IdentityExact
  PreconditionExact
}

pred WithoutDomain {
  OutputTotal NonExpansion Monotone AdapterDomainExact TypeExact UnitExact
  ReconstitutionExact ImplementationExact ExternalExact StatusExact IdentityExact PreconditionExact
}
pred WithoutMonotone {
  OutputTotal NonExpansion DomainExact AdapterDomainExact TypeExact UnitExact
  ReconstitutionExact ImplementationExact ExternalExact StatusExact IdentityExact PreconditionExact
}
pred WithoutAdapterDomain {
  OutputTotal NonExpansion DomainExact Monotone TypeExact UnitExact
  ReconstitutionExact ImplementationExact ExternalExact StatusExact IdentityExact PreconditionExact
}
pred WithoutType {
  OutputTotal NonExpansion DomainExact Monotone AdapterDomainExact UnitExact
  ReconstitutionExact ImplementationExact ExternalExact StatusExact IdentityExact PreconditionExact
}
pred WithoutUnit {
  OutputTotal NonExpansion DomainExact Monotone AdapterDomainExact TypeExact
  ReconstitutionExact ImplementationExact ExternalExact StatusExact IdentityExact PreconditionExact
}
pred WithoutReconstitution {
  OutputTotal NonExpansion DomainExact Monotone AdapterDomainExact TypeExact UnitExact
  ImplementationExact ExternalExact StatusExact IdentityExact PreconditionExact
}
pred WithoutImplementation {
  OutputTotal NonExpansion DomainExact Monotone AdapterDomainExact TypeExact UnitExact
  ReconstitutionExact ExternalExact StatusExact IdentityExact PreconditionExact
}
pred WithoutExternal {
  OutputTotal NonExpansion DomainExact Monotone AdapterDomainExact TypeExact UnitExact
  ReconstitutionExact ImplementationExact StatusExact IdentityExact PreconditionExact
}
pred WithoutStatus {
  OutputTotal NonExpansion DomainExact Monotone AdapterDomainExact TypeExact UnitExact
  ReconstitutionExact ImplementationExact ExternalExact IdentityExact PreconditionExact
}
pred WithoutPrecondition {
  OutputTotal NonExpansion DomainExact Monotone AdapterDomainExact TypeExact UnitExact
  ReconstitutionExact ImplementationExact ExternalExact StatusExact IdentityExact
}

pred CanonicalEnvironment {
  RunState.control = CanonicalControl
  Adapter.id = CanonicalAdapterId
  Adapter.implementationHash = CanonicalImplementationHash
  Adapter.sourceType = PaymentIntentType
  Adapter.targetType = PaymentIntentType
  Adapter.unitRule = IdentityUnitRule
  Adapter.quantityRule = ExactFloorHalfRule
  Adapter.externalState = NoExternalState
  Adapter.status = CompleteStatus
  Adapter.precondition = On
  Adapter.declaredDomainMax = 20
  no Adapter.removedFields
  no Adapter.reconstitutedFields
  Transform.variant = CanonicalVariant
  Transform.domain = 7..20
  Transform.monotonicityDomain = 7..20
  all x: Transform.domain | one x.(Transform.output) and x.(Transform.output) = x div 2
}
pred SampledEnvironment {
  RunState.control = MonotonicitySampledControl
  Adapter.id = CanonicalAdapterId
  Adapter.implementationHash = CanonicalImplementationHash
  Adapter.sourceType = PaymentIntentType
  Adapter.targetType = PaymentIntentType
  Adapter.unitRule = IdentityUnitRule
  Adapter.quantityRule = ExactFloorHalfRule
  Adapter.externalState = NoExternalState
  Adapter.status = CompleteStatus
  Adapter.precondition = On
  Adapter.declaredDomainMax = 20
  no Adapter.removedFields and no Adapter.reconstitutedFields
  Transform.variant = CanonicalVariant
  Transform.domain = 7..20
  Transform.monotonicityDomain = 7..14
  all x: Transform.domain | one x.(Transform.output) and x.(Transform.output) = x div 2
}
pred ViolationEnvironment {
  RunState.control = MonotonicityViolationControl
  Adapter.id = CanonicalAdapterId
  Adapter.implementationHash = CanonicalImplementationHash
  Adapter.sourceType = PaymentIntentType
  Adapter.targetType = PaymentIntentType
  Adapter.unitRule = IdentityUnitRule
  Adapter.quantityRule = ExactFloorHalfRule
  Adapter.externalState = NoExternalState
  Adapter.status = CompleteStatus
  Adapter.precondition = On
  Adapter.declaredDomainMax = 20
  no Adapter.removedFields and no Adapter.reconstitutedFields
  Transform.variant = NonMonotoneVariant
  Transform.domain = 7..20
  Transform.monotonicityDomain = 7..20
  all x: Transform.domain | one x.(Transform.output)
  13.(Transform.output) = 7 and 14.(Transform.output) = 6
  all x: Transform.domain - 13 - 14 | x.(Transform.output) = x div 2
}
pred DomainEnvironment {
  RunState.control = UndeclaredDomainControl
  Adapter.id = CanonicalAdapterId and Adapter.implementationHash = CanonicalImplementationHash
  Adapter.sourceType = PaymentIntentType and Adapter.targetType = PaymentIntentType
  Adapter.unitRule = IdentityUnitRule and Adapter.quantityRule = ExactFloorHalfRule
  Adapter.externalState = NoExternalState and Adapter.status = CompleteStatus and Adapter.precondition = On
  Adapter.declaredDomainMax = 10 and no Adapter.removedFields and no Adapter.reconstitutedFields
  Transform.variant = CanonicalVariant and Transform.domain = 7..20 and Transform.monotonicityDomain = 7..20
  all x: Transform.domain | one x.(Transform.output) and x.(Transform.output) = x div 2
}
pred TypeEnvironment {
  RunState.control = SemanticTypeControl
  Adapter.id = CanonicalAdapterId and Adapter.implementationHash = CanonicalImplementationHash
  Adapter.sourceType = EntitlementType and Adapter.targetType = PaymentIntentType
  Adapter.unitRule = IdentityUnitRule and Adapter.quantityRule = ExactFloorHalfRule
  Adapter.externalState = NoExternalState and Adapter.status = CompleteStatus and Adapter.precondition = On
  Adapter.declaredDomainMax = 20 and no Adapter.removedFields and no Adapter.reconstitutedFields
  Transform.variant = CanonicalVariant and Transform.domain = 7..20 and Transform.monotonicityDomain = 7..20
  all x: Transform.domain | one x.(Transform.output) and x.(Transform.output) = x div 2
}
pred UnitEnvironment {
  RunState.control = UnitControl
  Adapter.id = CanonicalAdapterId and Adapter.implementationHash = CanonicalImplementationHash
  Adapter.sourceType = PaymentIntentType and Adapter.targetType = PaymentIntentType
  Adapter.unitRule = RoundUpUnitRule and Adapter.quantityRule = RoundUpRule
  Adapter.externalState = NoExternalState and Adapter.status = CompleteStatus and Adapter.precondition = On
  Adapter.declaredDomainMax = 20 and no Adapter.removedFields and no Adapter.reconstitutedFields
  Transform.variant = CanonicalVariant and Transform.domain = 7..20 and Transform.monotonicityDomain = 7..20
  all x: Transform.domain | one x.(Transform.output) and x.(Transform.output) = x div 2
}
pred ReconstitutionEnvironment {
  RunState.control = ReconstitutionControl
  Adapter.id = CanonicalAdapterId and Adapter.implementationHash = CanonicalImplementationHash
  Adapter.sourceType = PaymentIntentType and Adapter.targetType = PaymentIntentType
  Adapter.unitRule = IdentityUnitRule and Adapter.quantityRule = ExactFloorHalfRule
  Adapter.externalState = NoExternalState and Adapter.status = CompleteStatus and Adapter.precondition = On
  Adapter.declaredDomainMax = 20
  Adapter.removedFields = SecretField and Adapter.reconstitutedFields = SecretField
  Transform.variant = CanonicalVariant and Transform.domain = 7..20 and Transform.monotonicityDomain = 7..20
  all x: Transform.domain | one x.(Transform.output) and x.(Transform.output) = x div 2
}
pred ImplementationEnvironment {
  RunState.control = ImplementationControl
  Adapter.id = CanonicalAdapterId and Adapter.implementationHash = AlternateImplementationHash
  Adapter.sourceType = PaymentIntentType and Adapter.targetType = PaymentIntentType
  Adapter.unitRule = IdentityUnitRule and Adapter.quantityRule = ExactFloorHalfRule
  Adapter.externalState = NoExternalState and Adapter.status = CompleteStatus and Adapter.precondition = On
  Adapter.declaredDomainMax = 20 and no Adapter.removedFields and no Adapter.reconstitutedFields
  Transform.variant = CanonicalVariant and Transform.domain = 7..20 and Transform.monotonicityDomain = 7..20
  all x: Transform.domain | one x.(Transform.output) and x.(Transform.output) = x div 2
}
pred ExternalEnvironment {
  RunState.control = ExternalControl
  Adapter.id = CanonicalAdapterId and Adapter.implementationHash = CanonicalImplementationHash
  Adapter.sourceType = PaymentIntentType and Adapter.targetType = PaymentIntentType
  Adapter.unitRule = IdentityUnitRule and Adapter.quantityRule = ExactFloorHalfRule
  Adapter.externalState = LiveRiskApiV2 and Adapter.status = CompleteStatus and Adapter.precondition = On
  Adapter.declaredDomainMax = 20 and no Adapter.removedFields and no Adapter.reconstitutedFields
  Transform.variant = CanonicalVariant and Transform.domain = 7..20 and Transform.monotonicityDomain = 7..20
  all x: Transform.domain | one x.(Transform.output) and x.(Transform.output) = x div 2
}
pred PartialEnvironment {
  RunState.control = PartialControl
  Adapter.id = CanonicalAdapterId and Adapter.implementationHash = CanonicalImplementationHash
  Adapter.sourceType = PaymentIntentType and Adapter.targetType = PaymentIntentType
  Adapter.unitRule = IdentityUnitRule and Adapter.quantityRule = ExactFloorHalfRule
  Adapter.externalState = NoExternalState and Adapter.status = PartialStatus and Adapter.precondition = On
  Adapter.declaredDomainMax = 20 and no Adapter.removedFields and no Adapter.reconstitutedFields
  Transform.variant = CanonicalVariant and Transform.domain = 7..20 and Transform.monotonicityDomain = 7..20
  all x: Transform.domain | one x.(Transform.output) and x.(Transform.output) = x div 2
}
pred PreconditionEnvironment {
  RunState.control = PreconditionControl
  Adapter.id = CanonicalAdapterId and Adapter.implementationHash = CanonicalImplementationHash
  Adapter.sourceType = PaymentIntentType and Adapter.targetType = PaymentIntentType
  Adapter.unitRule = IdentityUnitRule and Adapter.quantityRule = ExactFloorHalfRule
  Adapter.externalState = NoExternalState and Adapter.status = CompleteStatus and Adapter.precondition = Off
  Adapter.declaredDomainMax = 20 and no Adapter.removedFields and no Adapter.reconstitutedFields
  Transform.variant = CanonicalVariant and Transform.domain = 7..20 and Transform.monotonicityDomain = 7..20
  all x: Transform.domain | one x.(Transform.output) and x.(Transform.output) = x div 2
}

fact Environment {
  CanonicalEnvironment or SampledEnvironment or ViolationEnvironment or DomainEnvironment
  or TypeEnvironment or UnitEnvironment or ReconstitutionEnvironment or ImplementationEnvironment
  or ExternalEnvironment or PartialEnvironment or PreconditionEnvironment
}

assert CanonicalAggregate {
  RunState.control = CanonicalControl implies Strict
}
assert MutantAggregate_monotonicity_sampled_only {
  RunState.control = MonotonicitySampledControl implies WithoutDomain
}
assert MutantAggregate_monotonicity_violation_outside_sample {
  RunState.control = MonotonicityViolationControl implies WithoutMonotone
}
assert MutantAggregate_undeclared_adapter_domain {
  RunState.control = UndeclaredDomainControl implies WithoutAdapterDomain
}
assert MutantAggregate_schema_semantic_type_substitution {
  RunState.control = SemanticTypeControl implies WithoutType
}
assert MutantAggregate_unit_conversion_expansion {
  RunState.control = UnitControl implies WithoutUnit
}
assert MutantAggregate_lossy_redaction_reconstitution {
  RunState.control = ReconstitutionControl implies WithoutReconstitution
}
assert MutantAggregate_adapter_implementation_substitution {
  RunState.control = ImplementationControl implies WithoutImplementation
}
assert MutantAggregate_external_dependency_substitution {
  RunState.control = ExternalControl implies WithoutExternal
}
assert MutantAggregate_partial_adapter_success {
  RunState.control = PartialControl implies WithoutStatus
}
assert MutantAggregate_domain_precondition_bypass {
  RunState.control = PreconditionControl implies WithoutPrecondition
}
assert DomainOrderReflexive { all x: Transform.domain | x <= x }

pred monotonicity_sampled_onlyWitness { RunState.control = MonotonicitySampledControl }
pred monotonicity_violation_outside_sampleWitness { RunState.control = MonotonicityViolationControl }
pred undeclared_adapter_domainWitness { RunState.control = UndeclaredDomainControl }
pred schema_semantic_type_substitutionWitness { RunState.control = SemanticTypeControl }
pred unit_conversion_expansionWitness { RunState.control = UnitControl }
pred lossy_redaction_reconstitutionWitness { RunState.control = ReconstitutionControl }
pred adapter_implementation_substitutionWitness { RunState.control = ImplementationControl }
pred external_dependency_substitutionWitness { RunState.control = ExternalControl }
pred partial_adapter_successWitness { RunState.control = PartialControl }
pred domain_precondition_bypassWitness { RunState.control = PreconditionControl }

check CanonicalAggregate for 12 but exactly 1 RunState, 1 Adapter, 1 Transform
check MutantAggregate_monotonicity_sampled_only for 12 but exactly 1 RunState, 1 Adapter, 1 Transform
check MutantAggregate_monotonicity_violation_outside_sample for 12 but exactly 1 RunState, 1 Adapter, 1 Transform
check MutantAggregate_undeclared_adapter_domain for 12 but exactly 1 RunState, 1 Adapter, 1 Transform
check MutantAggregate_schema_semantic_type_substitution for 12 but exactly 1 RunState, 1 Adapter, 1 Transform
check MutantAggregate_unit_conversion_expansion for 12 but exactly 1 RunState, 1 Adapter, 1 Transform
check MutantAggregate_lossy_redaction_reconstitution for 12 but exactly 1 RunState, 1 Adapter, 1 Transform
check MutantAggregate_adapter_implementation_substitution for 12 but exactly 1 RunState, 1 Adapter, 1 Transform
check MutantAggregate_external_dependency_substitution for 12 but exactly 1 RunState, 1 Adapter, 1 Transform
check MutantAggregate_partial_adapter_success for 12 but exactly 1 RunState, 1 Adapter, 1 Transform
check MutantAggregate_domain_precondition_bypass for 12 but exactly 1 RunState, 1 Adapter, 1 Transform
check DomainOrderReflexive for 12 but exactly 1 RunState, 1 Adapter, 1 Transform
run monotonicity_sampled_onlyWitness for 12 but exactly 1 RunState, 1 Adapter, 1 Transform
run monotonicity_violation_outside_sampleWitness for 12 but exactly 1 RunState, 1 Adapter, 1 Transform
run undeclared_adapter_domainWitness for 12 but exactly 1 RunState, 1 Adapter, 1 Transform
run schema_semantic_type_substitutionWitness for 12 but exactly 1 RunState, 1 Adapter, 1 Transform
run unit_conversion_expansionWitness for 12 but exactly 1 RunState, 1 Adapter, 1 Transform
run lossy_redaction_reconstitutionWitness for 12 but exactly 1 RunState, 1 Adapter, 1 Transform
run adapter_implementation_substitutionWitness for 12 but exactly 1 RunState, 1 Adapter, 1 Transform
run external_dependency_substitutionWitness for 12 but exactly 1 RunState, 1 Adapter, 1 Transform
run partial_adapter_successWitness for 12 but exactly 1 RunState, 1 Adapter, 1 Transform
run domain_precondition_bypassWitness for 12 but exactly 1 RunState, 1 Adapter, 1 Transform