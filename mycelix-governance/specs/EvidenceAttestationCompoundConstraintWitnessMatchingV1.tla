---------------- MODULE EvidenceAttestationCompoundConstraintWitnessMatchingV1 ----------------
EXTENDS Naturals, FiniteSets

CONSTANT Control
ParentClauses == {"p-broad","p-narrow"}
CanonicalChildClauses == {"c-broad","c-narrow"}

ChildClauses ==
  IF Control = "parent-clause-deletion" THEN {"c-broad"}
  ELSE IF Control = "duplicate-witness-reuse" THEN {"c-narrow"}
  ELSE IF Control = "greedy-dead-end" THEN {"c-narrow","c-broad"}
  ELSE IF Control = "clause-order-permutation" THEN {"c-broad","c-narrow"}
  ELSE IF Control = "semantic-equivalent-conjunction" THEN {"c-narrow","c-broad"}
  ELSE IF Control = "semantic-non-equivalent-normalization" THEN {"c-bob"}
  ELSE IF Control = "cross-type-substitution" THEN {"c-narrow@future-v1"}
  ELSE IF Control = "unsupported-extension-inside-compound" THEN {"c-narrow@future-v1","c-broad"}
  ELSE IF Control = "disjunct-expansion" THEN {"c-broad","c-narrow","c-bob"}
  ELSE IF Control = "additional-restrictive-clause" THEN {"c-narrow","c-broad","c-extra"}
  ELSE CanonicalChildClauses

Supported(c) == c \notin {"c-narrow@future-v1","c-extra@future-v1"}
AtomSubsumes(c,p) ==
  (c = "c-narrow" /\ (p = "p-narrow" \/ p = "p-broad"))
  \/ (c = "c-broad" /\ p = "p-broad")

WitnessCounted ==
  Cardinality(ChildClauses) >= Cardinality(ParentClauses)

MatchingExists ==
  \E chosen \in [ParentClauses -> ChildClauses] :
    /\ \A p \in ParentClauses : AtomSubsumes(chosen[p],p)
    /\ \A p1 \in ParentClauses : \A p2 \in ParentClauses :
          p1 # p2 => chosen[p1] # chosen[p2]
    /\ \A p \in ParentClauses : chosen[p] \in ChildClauses

GreedyResult == IF Control = "greedy-dead-end" THEN FALSE ELSE MatchingExists

ParentDenotation == {"alice-business-trusted-0","alice-business-trusted-1","bob-business-trusted-0","bob-business-trusted-1"}
ChildDenotation ==
  IF Control = "semantic-non-equivalent-normalization" THEN {"bob-business-trusted-0","bob-business-trusted-1"}
  ELSE IF Control = "disjunct-expansion" THEN ParentDenotation \cup {"bob-refund-trusted-0","bob-refund-trusted-1"}
  ELSE ParentDenotation

SemanticEquivalent == ChildDenotation = ParentDenotation
OrderInvariant == MatchingExists
DeterministicWitness == MatchingExists

ParentClauseDeletionSafe == Control # "parent-clause-deletion" \/ WitnessCounted
DuplicateWitnessReuseSafe == Control # "duplicate-witness-reuse" \/ WitnessCounted
GreedyDeadEndSafe == Control # "greedy-dead-end" \/ (GreedyResult = MatchingExists)
ClauseOrderPermutationSafe == Control # "clause-order-permutation" \/ OrderInvariant
SemanticEquivalentSafe == Control # "semantic-equivalent-conjunction" \/ SemanticEquivalent
SemanticNonEquivalentNormalizationSafe == Control # "semantic-non-equivalent-normalization" \/ SemanticEquivalent
CrossTypeSafe == Control # "cross-type-substitution" \/ \A c \in ChildClauses : Supported(c)
UnsupportedCompoundSafe == Control # "unsupported-extension-inside-compound" \/ \A c \in ChildClauses : Supported(c)
DisjunctExpansionSafe == Control # "disjunct-expansion" \/ ChildDenotation \subseteq ParentDenotation
AdditionalRestrictiveSafe == Control # "additional-restrictive-clause" \/ (MatchingExists /\ ChildDenotation \subseteq ParentDenotation)

TypeOK == Control \in {
 "parent-clause-deletion","duplicate-witness-reuse","greedy-dead-end","clause-order-permutation",
 "semantic-equivalent-conjunction","semantic-non-equivalent-normalization","cross-type-substitution",
 "unsupported-extension-inside-compound","disjunct-expansion","additional-restrictive-clause","canonical"}

CanonicalAggregate ==
  Control = "canonical" =>
    /\ MatchingExists
    /\ SemanticEquivalent
    /\ ChildDenotation \subseteq ParentDenotation

SafetyAggregate ==
  /\ TypeOK
  /\ ParentClauseDeletionSafe
  /\ DuplicateWitnessReuseSafe
  /\ GreedyDeadEndSafe
  /\ ClauseOrderPermutationSafe
  /\ SemanticEquivalentSafe
  /\ SemanticNonEquivalentNormalizationSafe
  /\ CrossTypeSafe
  /\ UnsupportedCompoundSafe
  /\ DisjunctExpansionSafe
  /\ AdditionalRestrictiveSafe

Init == TRUE
Next == TRUE
====
