---------------- MODULE EvidenceAttestationCapabilityConstraintAlgebraV1 ----------------
EXTENDS Naturals, FiniteSets

CONSTANT Control

Requests == {"r1","r2","r3","r4"}
ParentAllow == {"r1","r2","r3"}
ParentDeny == {"r2"}
CanonicalChildAllow == {"r1"}
CanonicalChildDeny == {"r2"}
CanonicalConflictRule == "deny-overrides"

ChildAllow ==
  IF Control = "interval-widening" THEN {"r1","r2","r3","r4"}
  ELSE IF Control = "interval-hole-filling" THEN ParentAllow
  ELSE IF Control = "wildcard-expansion" THEN {"r1","r2","r3","r4"}
  ELSE IF Control = "temporal-window-widening" THEN {"r1","r2","r3","r4"}
  ELSE IF Control = "context-weakening" THEN {"r1","r2","r3","r4"}
  ELSE IF Control = "normalization-equivalence" THEN {"r1"}
  ELSE IF Control = "non-equivalent-normalization" THEN {"r3"}
  ELSE IF Control = "deny-set-deletion" THEN CanonicalChildAllow
  ELSE IF Control = "conflict-rule-substitution" THEN CanonicalChildAllow
  ELSE IF Control = "unknown-extension-treated-as-subsumed" THEN CanonicalChildAllow
  ELSE IF Control = "unsupported-compound-extension" THEN CanonicalChildAllow
  ELSE CanonicalChildAllow

ChildDeny ==
  IF Control = "interval-hole-filling" THEN {}
  ELSE IF Control = "deny-set-deletion" THEN {}
  ELSE CanonicalChildDeny

ChildConflictRule ==
  IF Control = "conflict-rule-substitution" THEN "allow-overrides"
  ELSE CanonicalConflictRule

ChildSupported ==
  Control # "unknown-extension-treated-as-subsumed"
    /\ Control # "unsupported-compound-extension"

ChildSyntaxTag ==
  IF Control = "normalization-equivalence" THEN "alias"
  ELSE IF Control = "non-equivalent-normalization" THEN "wrong-alias"
  ELSE "canonical"

SyntaxEqual == ChildSyntaxTag = "canonical"
SemanticEquivalent == ChildAllow = CanonicalChildAllow

Effective(allow,deny,rule) ==
  IF rule = "deny-overrides" THEN allow \ deny
  ELSE IF rule = "allow-overrides" THEN allow
  ELSE {}

AllowSubsumption == ChildAllow \subseteq ParentAllow
DenyPreservation == ParentDeny \subseteq ChildDeny
PolicyAttenuation == AllowSubsumption /\ DenyPreservation /\
  Effective(ChildAllow,ChildDeny,ChildConflictRule) \subseteq
  Effective(ParentAllow,ParentDeny,"deny-overrides")

SyntaxSemanticSeparation ==
  (Control = "normalization-equivalence") => (~SyntaxEqual /\ SemanticEquivalent)

NormalizationNonEquivalentRejected ==
  (Control = "non-equivalent-normalization") => ~SemanticEquivalent

UnsupportedExtensionFailClosed ==
  (Control = "unknown-extension-treated-as-subsumed" \/ Control = "unsupported-compound-extension")
    => ~ChildSupported

IntervalWideningRejected == Control # "interval-widening" \/ ~AllowSubsumption
IntervalHoleFillingRejected == Control # "interval-hole-filling" \/ ~PolicyAttenuation
WildcardExpansionRejected == Control # "wildcard-expansion" \/ ~AllowSubsumption
TemporalWideningRejected == Control # "temporal-window-widening" \/ ~AllowSubsumption
ContextWeakeningRejected == Control # "context-weakening" \/ ~AllowSubsumption
NormalizationEquivalenceAccepted ==
  Control # "normalization-equivalence" \/ (SemanticEquivalent /\ ~SyntaxEqual)
DenyDeletionRejected == Control # "deny-set-deletion" \/ ~DenyPreservation
ConflictRuleSubstitutionRejected == Control # "conflict-rule-substitution" \/ ~PolicyAttenuation
UnknownExtensionRejected == Control # "unknown-extension-treated-as-subsumed" \/ ~ChildSupported
CompoundExtensionRejected == Control # "unsupported-compound-extension" \/ ~ChildSupported

TypeOK ==
  ParentAllow \subseteq Requests
  /\ ParentDeny \subseteq Requests
  /\ ChildAllow \subseteq Requests
  /\ ChildDeny \subseteq Requests
  /\ ChildConflictRule \in {"deny-overrides","allow-overrides"}

CapabilitySetOrderReflexive == AllowSubsumption / Control # "canonical"
CapabilitySetOrderTransitive ==
  ({"r1"} \subseteq ParentAllow /\ ParentAllow \subseteq Requests) => {"r1"} \subseteq Requests

SafetyAggregate ==
  /\ TypeOK
  /\ SyntaxSemanticSeparation
  /\ NormalizationNonEquivalentRejected
  /\ UnsupportedExtensionFailClosed
  /\ IntervalWideningRejected
  /\ IntervalHoleFillingRejected
  /\ WildcardExpansionRejected
  /\ TemporalWideningRejected
  /\ ContextWeakeningRejected
  /\ NormalizationEquivalenceAccepted
  /\ DenyDeletionRejected
  /\ ConflictRuleSubstitutionRejected
  /\ UnknownExtensionRejected
  /\ CompoundExtensionRejected

Init == TRUE
Next == UNCHANGED << >>
====
