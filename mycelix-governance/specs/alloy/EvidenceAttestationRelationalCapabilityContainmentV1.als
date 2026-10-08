module EvidenceAttestationRelationalCapabilityContainmentV1

abstract sig Operation {}
one sig TransferOp, RefundOp extends Operation {}
abstract sig Target {}
one sig AliceTarget, BobTarget, AcctAliceTarget, AcctBobTarget, AnyTarget extends Target {}
abstract sig Audience {}
one sig PaymentsAudience, AdminAudience, AnyAudience extends Audience {}
abstract sig Currency {}
one sig USD, EUR, AnyCurrency extends Currency {}
abstract sig ArgumentClass {}
one sig StandardArg, SensitiveArg, AnyArgument extends ArgumentClass {}
abstract sig SetId {}
one sig CanonicalId, AttackerId extends SetId {}
abstract sig Control {}
one sig CanonicalControl, CartesianControl, TargetCurrencyControl, OperationArgumentControl,
  AudienceTargetControl, WildcardControl, IdentityControl, NormalizationControl,
  ProfileWideningControl, EffectOutsideRelationControl, DownstreamBypassControl extends Control {}

one sig RunState {
  control: one Control
}

sig Capability {
  op: one Operation,
  target: one Target,
  audience: one Audience,
  currency: one Currency,
  argumentClass: one ArgumentClass,
  bound: one Int
}
one sig PTransfer, PRefund, CTransfer, CRefund, DTransfer, CX extends Capability {}
one sig ParentSet, ChildSet, DownstreamSet extends CapabilitySet {}
sig CapabilitySet {
  id: one SetId,
  members: set Capability
}

pred targetEquivalent[a,b: Target] {
  a = b
  or (a = AliceTarget and b = AcctAliceTarget)
  or (a = AcctAliceTarget and b = AliceTarget)
  or (a = BobTarget and b = AcctBobTarget)
  or (a = AcctBobTarget and b = BobTarget)
}

pred catLeq[c,p: Target] { p = AnyTarget or targetEquivalent[c,p] }
pred audLeq[c,p: Audience] { p = AnyAudience or c = p }
pred curLeq[c,p: Currency] { p = AnyCurrency or c = p }
pred argLeq[c,p: ArgumentClass] { p = AnyArgument or c = p }

pred capLeq[c,p: Capability] {
  c.op = p.op
  catLeq[c.target,p.target]
  audLeq[c.audience,p.audience]
  curLeq[c.currency,p.currency]
  argLeq[c.argumentClass,p.argumentClass]
  c.bound <= p.bound
}

pred setLeq[a,b: CapabilitySet] {
  all c: a.members | some p: b.members | capLeq[c,p]
}

pred admitted[s: CapabilitySet, x: Capability] {
  some c: s.members | capLeq[x,c]
}

pred semEquivalent[a,b: CapabilitySet] {
  all x: Capability | admitted[a,x] iff admitted[b,x]
}

pred canonicalEnvironment {
  RunState.control = CanonicalControl
  ParentSet.id = CanonicalId
  ChildSet.id = CanonicalId
  DownstreamSet.id = CanonicalId
  PTransfer.op = TransferOp and PTransfer.target = AliceTarget and PTransfer.audience = PaymentsAudience and PTransfer.currency = USD and PTransfer.argumentClass = StandardArg and PTransfer.bound = 100
  PRefund.op = RefundOp and PRefund.target = BobTarget and PRefund.audience = PaymentsAudience and PRefund.currency = EUR and PRefund.argumentClass = StandardArg and PRefund.bound = 50
  CTransfer.op = TransferOp and CTransfer.target = AliceTarget and CTransfer.audience = PaymentsAudience and CTransfer.currency = USD and CTransfer.argumentClass = StandardArg and CTransfer.bound = 60
  CRefund.op = RefundOp and CRefund.target = BobTarget and CRefund.audience = PaymentsAudience and CRefund.currency = EUR and CRefund.argumentClass = StandardArg and CRefund.bound = 30
  DTransfer.op = TransferOp and DTransfer.target = AliceTarget and DTransfer.audience = PaymentsAudience and DTransfer.currency = USD and DTransfer.argumentClass = StandardArg and DTransfer.bound = 20
  ParentSet.members = PTransfer + PRefund
  ChildSet.members = CTransfer + CRefund
  DownstreamSet.members = DTransfer
}

pred mutantEnvironment {
  (RunState.control = CartesianControl or RunState.control = TargetCurrencyControl or
   RunState.control = OperationArgumentControl or RunState.control = AudienceTargetControl or
   RunState.control = WildcardControl or RunState.control = NormalizationControl or
   RunState.control = ProfileWideningControl or RunState.control = EffectOutsideRelationControl or
   RunState.control = DownstreamBypassControl)
  ParentSet.id = CanonicalId
  PTransfer.op = TransferOp and PTransfer.target = AliceTarget and PTransfer.audience = PaymentsAudience and PTransfer.currency = USD and PTransfer.argumentClass = StandardArg and PTransfer.bound = 100
  PRefund.op = RefundOp and PRefund.target = BobTarget and PRefund.audience = PaymentsAudience and PRefund.currency = EUR and PRefund.argumentClass = StandardArg and PRefund.bound = 50
  CTransfer.op = TransferOp and CTransfer.target = AliceTarget and CTransfer.audience = PaymentsAudience and CTransfer.currency = USD and CTransfer.argumentClass = StandardArg and CTransfer.bound = 60
  CRefund.op = RefundOp and CRefund.target = BobTarget and CRefund.audience = PaymentsAudience and CRefund.currency = EUR and CRefund.argumentClass = StandardArg and CRefund.bound = 30
  DTransfer.op = TransferOp and DTransfer.target = AliceTarget and DTransfer.audience = PaymentsAudience and DTransfer.currency = USD and DTransfer.argumentClass = StandardArg and DTransfer.bound = 20
  (RunState.control = CartesianControl implies (CX.op = TransferOp and CX.target = BobTarget and CX.audience = PaymentsAudience and CX.currency = EUR and CX.argumentClass = StandardArg and CX.bound = 20))
  (RunState.control = TargetCurrencyControl implies (CX.op = TransferOp and CX.target = AliceTarget and CX.audience = PaymentsAudience and CX.currency = EUR and CX.argumentClass = StandardArg and CX.bound = 20))
  (RunState.control = OperationArgumentControl implies (CX.op = RefundOp and CX.target = AliceTarget and CX.audience = PaymentsAudience and CX.currency = USD and CX.argumentClass = SensitiveArg and CX.bound = 10))
  (RunState.control = AudienceTargetControl implies (CX.op = TransferOp and CX.target = BobTarget and CX.audience = AdminAudience and CX.currency = USD and CX.argumentClass = StandardArg and CX.bound = 10))
  (RunState.control = WildcardControl implies (CX.op = TransferOp and CX.target = AnyTarget and CX.audience = PaymentsAudience and CX.currency = USD and CX.argumentClass = StandardArg and CX.bound = 20))
  (RunState.control = NormalizationControl implies (CX.op = TransferOp and CX.target = AcctBobTarget and CX.audience = PaymentsAudience and CX.currency = USD and CX.argumentClass = StandardArg and CX.bound = 20))
  (RunState.control = ProfileWideningControl implies (CX.op = TransferOp and CX.target = BobTarget and CX.audience = PaymentsAudience and CX.currency = EUR and CX.argumentClass = StandardArg and CX.bound = 20))
  (RunState.control = EffectOutsideRelationControl implies (CX.op = RefundOp and CX.target = AliceTarget and CX.audience = PaymentsAudience and CX.currency = EUR and CX.argumentClass = StandardArg and CX.bound = 10))
  (RunState.control = DownstreamBypassControl implies (CX.op = TransferOp and CX.target = BobTarget and CX.audience = PaymentsAudience and CX.currency = USD and CX.argumentClass = StandardArg and CX.bound = 10))
  ParentSet.members = PTransfer + PRefund
  ChildSet.members = CTransfer + CRefund + CX
  DownstreamSet.members = DTransfer
}

fact Environment {
  canonicalEnvironment or mutantEnvironment or (RunState.control = IdentityControl and
    ParentSet.id = AttackerId and ChildSet.id = CanonicalId and DownstreamSet.id = CanonicalId and
    ParentSet.members = PTransfer + PRefund and ChildSet.members = CTransfer + CRefund and DownstreamSet.members = DTransfer)
}

assert CapabilitySetOrderReflexive { all s: CapabilitySet | setLeq[s,s] }
assert CapabilitySetOrderTransitive { all a,b,c: CapabilitySet | setLeq[a,b] and setLeq[b,c] implies setLeq[a,c] }
assert CapabilitySetOrderAntisymmetricModuloEquivalence {
  all a,b: CapabilitySet | setLeq[a,b] and setLeq[b,a] implies semEquivalent[a,b]
}
assert CanonicalAggregate {
  RunState.control = CanonicalControl implies (setLeq[ChildSet,ParentSet] and setLeq[DownstreamSet,ChildSet] and setLeq[DownstreamSet,ParentSet])
}
assert MutantAggregate_cartesian_recombination {
  RunState.control = CartesianControl implies not setLeq[ChildSet,ParentSet]
}
assert MutantAggregate_target_currency_correlation {
  RunState.control = TargetCurrencyControl implies not setLeq[ChildSet,ParentSet]
}
assert MutantAggregate_operation_argument_correlation {
  RunState.control = OperationArgumentControl implies not setLeq[ChildSet,ParentSet]
}
assert MutantAggregate_audience_target_correlation {
  RunState.control = AudienceTargetControl implies not setLeq[ChildSet,ParentSet]
}
assert MutantAggregate_wildcard_expansion {
  RunState.control = WildcardControl implies not setLeq[ChildSet,ParentSet]
}
assert MutantAggregate_capability_set_identity_substitution {
  RunState.control = IdentityControl implies ParentSet.id = CanonicalId
}
assert MutantAggregate_tuple_normalization_substitution {
  RunState.control = NormalizationControl implies not setLeq[ChildSet,ParentSet]
}
assert MutantAggregate_profile_widening_unchanged_marginals {
  RunState.control = ProfileWideningControl implies not setLeq[ChildSet,ParentSet]
}
assert MutantAggregate_effect_outside_relation_inside_marginals {
  RunState.control = EffectOutsideRelationControl implies not setLeq[ChildSet,ParentSet]
}
assert MutantAggregate_downstream_relational_subset_bypass {
  RunState.control = DownstreamBypassControl implies not setLeq[DownstreamSet,ChildSet]
}

check CapabilitySetOrderReflexive for 12 but exactly 3 CapabilitySet, exactly 7 Capability, exactly 1 RunState
check CapabilitySetOrderTransitive for 12 but exactly 3 CapabilitySet, exactly 7 Capability, exactly 1 RunState
check CapabilitySetOrderAntisymmetricModuloEquivalence for 12 but exactly 3 CapabilitySet, exactly 7 Capability, exactly 1 RunState
check CanonicalAggregate for 12 but exactly 3 CapabilitySet, exactly 7 Capability, exactly 1 RunState
check MutantAggregate_cartesian_recombination for 12 but exactly 3 CapabilitySet, exactly 7 Capability, exactly 1 RunState
check MutantAggregate_target_currency_correlation for 12 but exactly 3 CapabilitySet, exactly 7 Capability, exactly 1 RunState
check MutantAggregate_operation_argument_correlation for 12 but exactly 3 CapabilitySet, exactly 7 Capability, exactly 1 RunState
check MutantAggregate_audience_target_correlation for 12 but exactly 3 CapabilitySet, exactly 7 Capability, exactly 1 RunState
check MutantAggregate_wildcard_expansion for 12 but exactly 3 CapabilitySet, exactly 7 Capability, exactly 1 RunState
check MutantAggregate_capability_set_identity_substitution for 12 but exactly 3 CapabilitySet, exactly 7 Capability, exactly 1 RunState
check MutantAggregate_tuple_normalization_substitution for 12 but exactly 3 CapabilitySet, exactly 7 Capability, exactly 1 RunState
check MutantAggregate_profile_widening_unchanged_marginals for 12 but exactly 3 CapabilitySet, exactly 7 Capability, exactly 1 RunState
check MutantAggregate_effect_outside_relation_inside_marginals for 12 but exactly 3 CapabilitySet, exactly 7 Capability, exactly 1 RunState
check MutantAggregate_downstream_relational_subset_bypass for 12 but exactly 3 CapabilitySet, exactly 7 Capability, exactly 1 RunState
run CanonicalAggregate for 12 but exactly 3 CapabilitySet, exactly 7 Capability, exactly 1 RunState
