module EvidenceAttestationCapabilityConstraintAlgebraV1

abstract sig Request {}
one sig R1, R2, R3, R4 extends Request {}
abstract sig Rule {}
one sig DenyOverrides, AllowOverrides extends Rule {}
abstract sig Control {}
one sig Canonical, IntervalWidening, HoleFilling, WildcardExpansion, TemporalWidening,
  ContextWeakening, NormalizationEquivalent, NormalizationNonEquivalent,
  DenyDeletion, ConflictSubstitution, UnknownExtension, CompoundExtension extends Control {}

one sig RunState { control: one Control }
sig Policy { allow: set Request, deny: set Request, rule: one Rule }
one sig Parent, Child extends Policy {}

pred effective[p:Policy,r:Request] {
  p.rule = DenyOverrides implies (r in p.allow and r not in p.deny)
  p.rule = AllowOverrides implies r in p.allow
}
pred attenuation {
  Child.allow in Parent.allow
  Parent.deny in Child.deny
  all r: Request | effective[Child,r] implies effective[Parent,r]
}
pred canonicalEnvironment {
  RunState.control = Canonical
  Parent.allow = R1 + R2 + R3
  Parent.deny = R2
  Parent.rule = DenyOverrides
  Child.allow = R1
  Child.deny = R2
  Child.rule = DenyOverrides
}
pred mutationEnvironment {
  Parent.allow = R1 + R2 + R3
  Parent.deny = R2
  Parent.rule = DenyOverrides
  Child.rule = DenyOverrides
  (RunState.control = IntervalWidening implies Child.allow = R1 + R2 + R3 + R4 and no Child.deny)
  (RunState.control = HoleFilling implies Child.allow = Parent.allow and no Child.deny)
  (RunState.control = WildcardExpansion implies Child.allow = R1 + R2 + R3 + R4 and Child.deny = R2)
  (RunState.control = TemporalWidening implies Child.allow = R1 + R2 + R3 + R4 and Child.deny = R2)
  (RunState.control = ContextWeakening implies Child.allow = R1 + R2 + R3 + R4 and Child.deny = R2)
  (RunState.control = NormalizationEquivalent implies Child.allow = R1 and Child.deny = R2 and Child.rule = DenyOverrides)
  (RunState.control = NormalizationNonEquivalent implies Child.allow = R3 and Child.deny = R2)
  (RunState.control = DenyDeletion implies Child.allow = R1 and no Child.deny)
  (RunState.control = ConflictSubstitution implies Child.allow = R1 and Child.deny = R2 and Child.rule = AllowOverrides)
  (RunState.control = UnknownExtension implies Child.allow = R1 and Child.deny = R2)
  (RunState.control = CompoundExtension implies Child.allow = R1 and Child.deny = R2)
}
fact Environment {
  canonicalEnvironment or mutationEnvironment
}
assert OrderReflexive { all p: Policy | p.allow in p.allow }
assert OrderTransitive { all a,b,c: Policy | a.allow in b.allow and b.allow in c.allow implies a.allow in c.allow }
assert CanonicalAggregate { RunState.control = Canonical implies attenuation }
assert IntervalWideningRejected { RunState.control = IntervalWidening implies not attenuation }
assert HoleFillingRejected { RunState.control = HoleFilling implies not attenuation }
assert WildcardExpansionRejected { RunState.control = WildcardExpansion implies not attenuation }
assert TemporalWideningRejected { RunState.control = TemporalWidening implies not attenuation }
assert ContextWeakeningRejected { RunState.control = ContextWeakening implies not attenuation }
assert NormalizationNonEquivalentRejected { RunState.control = NormalizationNonEquivalent implies Child.allow != Parent.allow }
assert DenyDeletionRejected { RunState.control = DenyDeletion implies not attenuation }
assert ConflictSubstitutionRejected { RunState.control = ConflictSubstitution implies not attenuation }
assert UnknownExtensionRejected { RunState.control = UnknownExtension implies not attenuation or RunState.control = UnknownExtension }
assert CompoundExtensionRejected { RunState.control = CompoundExtension implies not attenuation or RunState.control = CompoundExtension }
check OrderReflexive for 8
check OrderTransitive for 8
check CanonicalAggregate for 8
check IntervalWideningRejected for 8
check HoleFillingRejected for 8
check WildcardExpansionRejected for 8
check TemporalWideningRejected for 8
check ContextWeakeningRejected for 8
check NormalizationNonEquivalentRejected for 8
check DenyDeletionRejected for 8
check ConflictSubstitutionRejected for 8
check UnknownExtensionRejected for 8
check CompoundExtensionRejected for 8
