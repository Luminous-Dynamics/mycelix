module EvidenceAttestationCompoundConstraintWitnessMatchingV1

abstract sig Request {}
one sig AliceBusiness, BobBusiness, BobRefund, AliceRefund extends Request {}
abstract sig ParentClause {}
one sig ParentBroad, ParentNarrow extends ParentClause {}
abstract sig ChildClause {}
one sig ChildBroad, ChildNarrow, ChildBob, ChildExtra, ChildUnsupported extends ChildClause {}
abstract sig Control {}
one sig Canonical, ParentClauseDeletion, DuplicateWitnessReuse, GreedyDeadEnd,
 ClauseOrderPermutation, SemanticEquivalent, SemanticNonEquivalentNormalization,
 CrossTypeSubstitution, UnsupportedExtensionInsideCompound, DisjunctExpansion,
 AdditionalRestrictiveClause extends Control {}
abstract sig Bit {}
one sig On, Off extends Bit {}
abstract sig SyntaxForm {}
one sig CanonicalSyntax, AliasSyntax, WrongAliasSyntax extends SyntaxForm {}

one sig RunState {
 control: one Control,
 childActive: set ChildClause,
 childSupported: one Bit,
 childSyntax: one SyntaxForm,
 greedyOutcome: one Bit,
 childDenotation: set Request
}
one sig Matcher { witness: ParentClause -> ChildClause }

fun parentDenotation: set Request { AliceBusiness + AliceRefund }

pred atomSubsumes[c:ChildClause,p:ParentClause] {
  (c = ChildNarrow and (p = ParentNarrow or p = ParentBroad))
  or (c = ChildBroad and p = ParentBroad)
  or (c = ChildExtra and (p = ParentNarrow or p = ParentBroad))
}

pred hasInjectiveMatching {
  Matcher.witness in ParentClause -> RunState.childActive
  all p: ParentClause | one p.Matcher.witness and atomSubsumes[p.Matcher.witness,p]
  all c: RunState.childActive | lone Matcher.witness.c
}

pred canonicalEnvironment {
  RunState.control = Canonical
  RunState.childActive = ChildNarrow + ChildBroad
  RunState.childSupported = On
  RunState.childSyntax = CanonicalSyntax
  RunState.greedyOutcome = On
  RunState.childDenotation = parentDenotation
}

pred mutationEnvironment {
  RunState.control != Canonical
  (RunState.control = ParentClauseDeletion implies
    (RunState.childActive = ChildBroad and RunState.childSupported = On and RunState.greedyOutcome = Off and RunState.childDenotation = parentDenotation))
  (RunState.control = DuplicateWitnessReuse implies
    (RunState.childActive = ChildNarrow and RunState.childSupported = On and RunState.greedyOutcome = Off and RunState.childDenotation = parentDenotation))
  (RunState.control = GreedyDeadEnd implies
    (RunState.childActive = ChildNarrow + ChildBroad and RunState.childSupported = On and RunState.greedyOutcome = Off and RunState.childDenotation = parentDenotation))
  (RunState.control = ClauseOrderPermutation implies
    (RunState.childActive = ChildBroad + ChildNarrow and RunState.childSupported = On and RunState.greedyOutcome = On and RunState.childDenotation = parentDenotation))
  (RunState.control = SemanticEquivalent implies
    (RunState.childActive = ChildNarrow + ChildBroad and RunState.childSupported = On and RunState.childSyntax = AliasSyntax and RunState.greedyOutcome = On and RunState.childDenotation = parentDenotation))
  (RunState.control = SemanticNonEquivalentNormalization implies
    (RunState.childActive = ChildBroad and RunState.childSupported = On and RunState.childSyntax = WrongAliasSyntax and RunState.greedyOutcome = On and RunState.childDenotation = AliceBusiness + BobBusiness + BobRefund + AliceRefund))
  (RunState.control = CrossTypeSubstitution implies
    (RunState.childActive = ChildUnsupported and RunState.childSupported = Off and RunState.childSyntax = WrongAliasSyntax and RunState.greedyOutcome = Off and RunState.childDenotation = parentDenotation))
  (RunState.control = UnsupportedExtensionInsideCompound implies
    (RunState.childActive = ChildUnsupported + ChildBroad and RunState.childSupported = Off and RunState.childSyntax = WrongAliasSyntax and RunState.greedyOutcome = Off and RunState.childDenotation = parentDenotation))
  (RunState.control = DisjunctExpansion implies
    (RunState.childActive = ChildNarrow + ChildBroad + ChildBob and RunState.childSupported = On and RunState.childSyntax = CanonicalSyntax and RunState.greedyOutcome = On and RunState.childDenotation = parentDenotation + BobRefund))
  (RunState.control = AdditionalRestrictiveClause implies
    (RunState.childActive = ChildNarrow + ChildBroad + ChildExtra and RunState.childSupported = On and RunState.childSyntax = CanonicalSyntax and RunState.greedyOutcome = On and RunState.childDenotation = AliceBusiness))
}

fact Environment { canonicalEnvironment or mutationEnvironment }

assert WitnessSound { all p: ParentClause | all c: p.Matcher.witness | atomSubsumes[c,p] }
assert WitnessInjective { all c: ChildClause | lone Matcher.witness.c }
assert CanonicalAggregate {
  RunState.control = Canonical implies
    (hasInjectiveMatching and RunState.childDenotation = parentDenotation and RunState.childSupported = On)
}
assert ParentClauseDeletionSafe { RunState.control = ParentClauseDeletion implies #RunState.childActive >= #ParentClause }
assert DuplicateWitnessReuseSafe { RunState.control = DuplicateWitnessReuse implies #RunState.childActive >= #ParentClause }
assert GreedyDeadEndSafe { RunState.control = GreedyDeadEnd implies RunState.greedyOutcome = On }
assert ClauseOrderPermutationSafe { RunState.control = ClauseOrderPermutation implies hasInjectiveMatching }
assert SemanticEquivalentSafe {
  RunState.control = SemanticEquivalent implies
    RunState.childDenotation = parentDenotation and RunState.childSyntax = AliasSyntax
}
assert SemanticNonEquivalentNormalizationSafe {
  RunState.control = SemanticNonEquivalentNormalization implies RunState.childDenotation = parentDenotation
}
assert CrossTypeSafe { RunState.control = CrossTypeSubstitution implies RunState.childSupported = On }
assert UnsupportedCompoundSafe { RunState.control = UnsupportedExtensionInsideCompound implies RunState.childSupported = On }
assert DisjunctExpansionSafe { RunState.control = DisjunctExpansion implies RunState.childDenotation in parentDenotation }
assert AdditionalRestrictiveSafe {
  RunState.control = AdditionalRestrictiveClause implies
    (hasInjectiveMatching and RunState.childDenotation in parentDenotation)
}

check WitnessSound for 8
check WitnessInjective for 8
check CanonicalAggregate for 8
check ParentClauseDeletionSafe for 8
check DuplicateWitnessReuseSafe for 8
check GreedyDeadEndSafe for 8
check ClauseOrderPermutationSafe for 8
check SemanticEquivalentSafe for 8
check SemanticNonEquivalentNormalizationSafe for 8
check CrossTypeSafe for 8
check UnsupportedCompoundSafe for 8
check DisjunctExpansionSafe for 8
check AdditionalRestrictiveSafe for 8
