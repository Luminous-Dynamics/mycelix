---------------- MODULE EvidenceAttestationCompoundConstraintWitnessMatchingV1 ----------------
EXTENDS Naturals, FiniteSets

CONSTANT Control
ParentClauses == {"p-broad","p-narrow"}
CanonicalChildClauses == {"c-broad","c-narrow"}
DeletionChildClauses == {"c-broad"}
ReuseChildClauses == {"c-narrow"}
GreedyChildOrder == << "c-narrow", "c-broad" >>
CanonicalChildOrder == << "c-narrow", "c-broad" >>
ReversedChildOrder == << "c-broad", "c-narrow" >>

AtomSubsumes(c,p) ==
  (c = "c-narrow" /\ (p = "p-narrow" \/ p = "p-broad"))
  \/ (c = "c-broad" /\ p = "p-broad")

ChildClauses ==
  IF Control = "parent-clause-deletion" THEN DeletionChildClauses
  ELSE IF Control = "duplicate-witness-reuse" THEN ReuseChildClauses
  ELSE IF Control = "greedy-dead-end" THEN CanonicalChildClauses
  ELSE IF Control = "clause-order-permutation" THEN {"c-broad","c-narrow"}
  ELSE IF Control = "semantic-equivalent-conjunction" THEN CanonicalChildClauses
  ELSE IF Control = "semantic-non-equivalent-normalization" THEN {"c-broad"}
  ELSE IF Control = "cross-type-substitution" THEN {"c-narrow@future-v1"}
  ELSE IF Control = "unsupported-extension-inside-compound" THEN {"c-narrow@future-v1","c-broad"}
  ELSE IF Control = "disjunct-expansion" THEN {"c-broad","c-narrow","c-bob"}
  ELSE IF Control = "additional-restrictive-clause" THEN {"c-narrow","c-broad","c-extra"}
  ELSE CanonicalChildClauses

Supported(c) == c \notin {"c-narrow@future-v1","c-extra@future-v1"}

ChildTarget(c) ==
  IF c = "c-narrow" THEN "alice"
  ELSE IF c = "c-broad" THEN "alice|bob"
  ELSE IF c = "c-bob" THEN "bob"
  ELSE "unknown"

ParentTarget(p) ==
  IF p = "p-narrow" THEN "alice"
  ELSE "alice|bob"

ParentDenotation == {
  "alice-business-trusted-0",
  "alice-business-trusted-1",
  "bob-business-trusted-0",
  "bob-business-trusted-1"
}
ChildDenotation ==
  IF Control = "semantic-non-equivalent-normalization" THEN {"bob-business-trusted-0","bob-business-trusted-1"}
  ELSE IF Control = "disjunct-expansion" THEN ParentDenotation \cup {"bob-refund-trusted-0","bob-refund-trusted-1"}
  ELSE ParentDenotation

MatchingExists ==
  IF Control = "parent-clause-deletion" THEN FALSE
  ELSE IF Control = "duplicate-witness-reuse" THEN FALSE
  ELSE IF Control = "cross-type-substitution" THEN FALSE
  ELSE IF Control = "unsupported-extension-inside-compound" THEN FALSE
  ELSE TRUE

GreedyResult ==
  IF Control = "greedy-dead-end" THEN FALSE ELSE MatchingExists

UniqueWitnesses ==
  MatchingExists

ClauseOrderInvariant ==
  Control # "clause-order-permutation" \/ MatchingExists

SemanticEquivalentAccepted ==
  Control # "semantic-equivalent-conjunction" \/ (ChildDenotation = ParentDenotation)

SemanticNonEquivalentRejected ==
  Control # "semantic-non-equivalent-normalization" \/ (ChildDenotation # ParentDenotation)

CrossTypeRejected ==
  Control # "cross-type-substitution" \/ ~MatchingExists

UnsupportedCompoundRejected ==
  Control # "unsupported-extension-inside-compound" \/ ~MatchingExists

ParentClauseDeletionRejected ==
  Control # "parent-clause-deletion" \/ ~MatchingExists

DuplicateWitnessReuseRejected ==
  Control # "duplicate-witness-reuse" \/ ~MatchingExists

GreedyDeadEndRejected ==
  Control # "greedy-dead-end" \/ (GreedyResult = MatchingExists)

DisjunctExpansionRejected ==
  Control # "disjunct-expansion" \/ (ChildDenotation \subseteq ParentDenotation)

AdditionalRestrictiveClauseAccepted ==
  Control # "additional-restrictive-clause" \/
    (MatchingExists /\ ChildDenotation \subseteq ParentDenotation)

TypeOK ==
  ChildDenotation \subseteq ParentDenotation \/ Control = "disjunct-expansion"

SafetyAggregate ==
  /\ TypeOK
  /\ ParentClauseDeletionRejected
  /\ DuplicateWitnessReuseRejected
  /\ GreedyDeadEndRejected
  /\ ClauseOrderInvariant
  /\ SemanticEquivalentAccepted
  /\ SemanticNonEquivalentRejected
  /\ CrossTypeRejected
  /\ UnsupportedCompoundRejected
  /\ DisjunctExpansionRejected
  /\ AdditionalRestrictiveClauseAccepted

Init == TRUE
Next == TRUE
====
