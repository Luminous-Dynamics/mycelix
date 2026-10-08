module EvidenceAttestationCapabilityConstraintAlgebraV1

abstract sig Request {}
one sig R1,R2,R3,R4 extends Request {}
abstract sig Rule {}
one sig DenyOverrides,AllowOverrides extends Rule {}
abstract sig Bit {}
one sig On,Off extends Bit {}
abstract sig Control {}
one sig Canonical,IntervalWidening,HoleFilling,WildcardExpansion,TemporalWidening,
  ContextWeakening,NormalizationEquivalent,NormalizationNonEquivalent,
  DenyDeletion,ConflictSubstitution,UnknownExtension,CompoundExtension extends Control {}
abstract sig SyntaxForm {}
one sig CanonicalSyntax,AliasSyntax,WrongAliasSyntax extends SyntaxForm {}

one sig RunState { control: one Control, childSupported: one Bit, childSyntax: one SyntaxForm }
sig Policy { allow:set Request, deny:set Request, rule:one Rule }
one sig Parent,Child extends Policy {}

pred effective[p:Policy,r:Request] {
  (p.rule = DenyOverrides and r in p.allow and r not in p.deny)
  or (p.rule = AllowOverrides and r in p.allow)
}

pred semEquivalent[a,b:Policy] {
  all r:Request | (r in a.allow iff r in b.allow) and (r in a.deny iff r in b.deny)
}

pred syntacticEqualChildCanonical { RunState.childSyntax = CanonicalSyntax }

pred attenuation {
  Parent.rule = DenyOverrides
  RunState.childSupported = On
  Child.allow in Parent.allow
  Parent.deny in Child.deny
  all r:Request | effective[Child,r] implies effective[Parent,r]
}

pred canonicalEnvironment {
  RunState.control = Canonical
  RunState.childSupported = On
  RunState.childSyntax = CanonicalSyntax
  Parent.allow = R1+R2+R3
  Parent.deny = R2
  Parent.rule = DenyOverrides
  Child.allow = R1
  Child.deny = R2
  Child.rule = DenyOverrides
}

pred mutationEnvironment {
  Parent.allow = R1+R2+R3
  Parent.deny = R2
  Parent.rule = DenyOverrides
  (RunState.control = IntervalWidening implies (Child.allow = R1+R2+R3+R4 and no Child.deny and RunState.childSupported=On))
  (RunState.control = HoleFilling implies (Child.allow = Parent.allow and no Child.deny and RunState.childSupported=On))
  (RunState.control = WildcardExpansion implies (Child.allow = R1+R2+R3+R4 and Child.deny = R2 and RunState.childSupported=On))
  (RunState.control = TemporalWidening implies (Child.allow = R1+R2+R3+R4 and Child.deny = R2 and RunState.childSupported=On))
  (RunState.control = ContextWeakening implies (Child.allow = R1+R2+R3+R4 and Child.deny = R2 and RunState.childSupported=On))
  (RunState.control = NormalizationEquivalent implies (Child.allow = R1 and Child.deny = R2 and Child.rule = DenyOverrides and RunState.childSupported=On and RunState.childSyntax=AliasSyntax))
  (RunState.control = NormalizationNonEquivalent implies (Child.allow = R3 and Child.deny = R2 and Child.rule = DenyOverrides and RunState.childSupported=On and RunState.childSyntax=WrongAliasSyntax))
  (RunState.control = DenyDeletion implies (Child.allow = R1 and no Child.deny and Child.rule = DenyOverrides and RunState.childSupported=On))
  (RunState.control = ConflictSubstitution implies (Child.allow = R1 and Child.deny = R2 and Child.rule = AllowOverrides and RunState.childSupported=On))
  (RunState.control = UnknownExtension implies (Child.allow = R1 and Child.deny = R2 and Child.rule = DenyOverrides and RunState.childSupported=Off))
  (RunState.control = CompoundExtension implies (Child.allow = R1 and Child.deny = R2 and Child.rule = DenyOverrides and RunState.childSupported=Off))
  RunState.control != Canonical implies RunState.childSyntax != CanonicalSyntax
}

fact Environment { canonicalEnvironment or mutationEnvironment }

assert PolicyOrderReflexive { all p:Policy | p.allow in p.allow and p.deny in p.deny }
assert PolicyOrderTransitive {
  all a,b,c:Policy |
    a.allow in b.allow and b.allow in c.allow and
    a.deny in b.deny and b.deny in c.deny implies
    a.allow in c.allow and a.deny in c.deny
}
assert PolicyOrderAntisymmetricModuloSemanticEquivalence {
  all a,b:Policy |
    a.allow in b.allow and b.allow in a.allow and
    a.deny in b.deny and b.deny in a.deny implies
    semEquivalent[a,b]
}
assert CanonicalAggregate { RunState.control = Canonical implies attenuation }
assert IntervalWideningRejected { RunState.control = IntervalWidening implies not attenuation }
assert HoleFillingRejected { RunState.control = HoleFilling implies not attenuation }
assert WildcardExpansionRejected { RunState.control = WildcardExpansion implies not attenuation }
assert TemporalWideningRejected { RunState.control = TemporalWidening implies not attenuation }
assert ContextWeakeningRejected { RunState.control = ContextWeakening implies not attenuation }
assert NormalizationEquivalentAccepted {
  RunState.control = NormalizationEquivalent implies
    (semEquivalent[Child,Parent] and not syntacticEqualChildCanonical)
}
assert NormalizationNonEquivalentRejected {
  RunState.control = NormalizationNonEquivalent implies not semEquivalent[Child,Parent]
}
assert DenyDeletionRejected { RunState.control = DenyDeletion implies not attenuation }
assert ConflictSubstitutionRejected { RunState.control = ConflictSubstitution implies not attenuation }
assert UnknownExtensionRejected {
  RunState.control = UnknownExtension implies
    (RunState.childSupported = Off and not attenuation)
}
assert CompoundExtensionRejected {
  RunState.control = CompoundExtension implies
    (RunState.childSupported = Off and not attenuation)
}

check PolicyOrderReflexive for 8
check PolicyOrderTransitive for 8
check PolicyOrderAntisymmetricModuloSemanticEquivalence for 8
check CanonicalAggregate for 8
check IntervalWideningRejected for 8
check HoleFillingRejected for 8
check WildcardExpansionRejected for 8
check TemporalWideningRejected for 8
check ContextWeakeningRejected for 8
check NormalizationEquivalentAccepted for 8
check NormalizationNonEquivalentRejected for 8
check DenyDeletionRejected for 8
check ConflictSubstitutionRejected for 8
check UnknownExtensionRejected for 8
check CompoundExtensionRejected for 8
