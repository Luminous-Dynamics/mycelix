---- MODULE DirectoryFabricCausalCutReceipts ----
EXTENDS Naturals, FiniteSets, TLC

(*
  Finite reference model for the Directory Fabric composition boundary.

  The safe model requires:
    1. a verified dependency closure over the exact dependency set;
    2. every required authority position covered componentwise;
    3. a permit policy result before an Allow decision;
    4. an Allow decision before an effect attempt;
    5. no retry while an effect outcome is Unknown;
    6. at most one committed effect per logical operation.

  Named mutants are enabled through constants:
    SkipClosureCompleteness -- verifier labels an omitted-dependency cut complete;
    ScalarFrontier          -- sums sequence-like progress instead of matching each dependency;
    AllowRetryUnknown       -- allows a new attempt while the prior effect is Unknown;
    AllowDuplicateCommit    -- permits another effect commit for one logical operation.

  This model is finite and abstract. It does not model cryptographic signatures,
  complete graph discovery, a real Holochain conductor, trusted time, or effect
  owner independence.
*)

CONSTANTS
    Membership,
    Issuer,
    Revocation,
    ExplicitDeny,
    SkipClosureCompleteness,
    ScalarFrontier,
    AllowRetryUnknown,
    AllowDuplicateCommit

Dependencies == {Membership, Issuer, Revocation, ExplicitDeny}

Weight(d) ==
    IF d = Issuer THEN 5 ELSE 1

RequiredScalarHeight == Cardinality(Dependencies)

VARIABLES
    cutDependencies,
    coveredDependencies,
    closureStatus,
    policyResult,
    decision,
    scalarHeight,
    effectStatus,
    attempts,
    retryWhileUnknown,
    commits

vars ==
    << cutDependencies,
       coveredDependencies,
       closureStatus,
       policyResult,
       decision,
       scalarHeight,
       effectStatus,
       attempts,
       retryWhileUnknown,
       commits >>

Init ==
    /\ cutDependencies = {}
    /\ coveredDependencies = {}
    /\ closureStatus = "UNVERIFIED"
    /\ policyResult = "NO_POLICY"
    /\ decision = "NONE"
    /\ scalarHeight = 0
    /\ effectStatus = "NONE"
    /\ attempts = 0
    /\ retryWhileUnknown = 0
    /\ commits = 0

TypeOK ==
    /\ cutDependencies \subseteq Dependencies
    /\ coveredDependencies \subseteq Dependencies
    /\ closureStatus \in {"UNVERIFIED", "VERIFIED_COMPLETE", "INCOMPLETE"}
    /\ policyResult \in {"NO_POLICY", "PERMIT", "FORBID", "DEFAULT_DENY"}
    /\ decision \in {"NONE", "ALLOW", "DENY", "INDETERMINATE"}
    /\ scalarHeight \in Nat
    /\ effectStatus \in {"NONE", "UNKNOWN", "NO_EFFECT", "COMMITTED"}
    /\ attempts \in 0..3
    /\ retryWhileUnknown \in Nat
    /\ commits \in Nat

BuildCompleteCut ==
    /\ closureStatus = "UNVERIFIED"
    /\ cutDependencies' = Dependencies
    /\ UNCHANGED <<coveredDependencies, closureStatus, policyResult, decision,
                    scalarHeight, effectStatus, attempts, retryWhileUnknown, commits>>

BuildCutOmittingRevocation ==
    /\ closureStatus = "UNVERIFIED"
    /\ cutDependencies' = Dependencies \ {Revocation}
    /\ UNCHANGED <<coveredDependencies, closureStatus, policyResult, decision,
                    scalarHeight, effectStatus, attempts, retryWhileUnknown, commits>>

VerifyClosure ==
    /\ closureStatus = "UNVERIFIED"
    /\ closureStatus' =
          IF SkipClosureCompleteness
          THEN "VERIFIED_COMPLETE"
          ELSE IF cutDependencies = Dependencies
               THEN "VERIFIED_COMPLETE"
               ELSE "INCOMPLETE"
    /\ UNCHANGED <<cutDependencies, coveredDependencies, policyResult, decision,
                    scalarHeight, effectStatus, attempts, retryWhileUnknown, commits>>

ObserveDependency ==
    \E d \in (cutDependencies \ coveredDependencies):
        /\ coveredDependencies' = coveredDependencies \cup {d}
        /\ scalarHeight' = scalarHeight + Weight(d)
        /\ UNCHANGED <<cutDependencies, closureStatus, policyResult, decision,
                        effectStatus, attempts, retryWhileUnknown, commits>>

SetPermit ==
    /\ policyResult = "NO_POLICY"
    /\ policyResult' = "PERMIT"
    /\ UNCHANGED <<cutDependencies, coveredDependencies, closureStatus, decision,
                    scalarHeight, effectStatus, attempts, retryWhileUnknown, commits>>

SetForbid ==
    /\ policyResult = "NO_POLICY"
    /\ policyResult' = "FORBID"
    /\ UNCHANGED <<cutDependencies, coveredDependencies, closureStatus, decision,
                    scalarHeight, effectStatus, attempts, retryWhileUnknown, commits>>

SetDefaultDeny ==
    /\ policyResult = "NO_POLICY"
    /\ policyResult' = "DEFAULT_DENY"
    /\ UNCHANGED <<cutDependencies, coveredDependencies, closureStatus, decision,
                    scalarHeight, effectStatus, attempts, retryWhileUnknown, commits>>

ClosureOK ==
    closureStatus = "VERIFIED_COMPLETE"

FrontierOK ==
    IF ScalarFrontier
    THEN scalarHeight >= RequiredScalarHeight
    ELSE cutDependencies \subseteq coveredDependencies

Decide ==
    /\ decision = "NONE"
    /\ policyResult # "NO_POLICY"
    /\ decision' =
          IF policyResult = "FORBID"
          THEN "DENY"
          ELSE IF policyResult = "PERMIT" /\ ClosureOK /\ FrontierOK
               THEN "ALLOW"
               ELSE IF policyResult = "DEFAULT_DENY" /\ ClosureOK /\ FrontierOK
                    THEN "DENY"
                    ELSE "INDETERMINATE"
    /\ UNCHANGED <<cutDependencies, coveredDependencies, closureStatus,
                    policyResult, scalarHeight, effectStatus, attempts,
                    retryWhileUnknown, commits>>

StartEffectAttempt ==
    /\ decision = "ALLOW"
    /\ attempts < 3
    /\ (attempts = 0
        \/ effectStatus = "NO_EFFECT"
        \/ (AllowRetryUnknown /\ effectStatus = "UNKNOWN"))
    /\ attempts' = attempts + 1
    /\ retryWhileUnknown' =
          IF effectStatus = "UNKNOWN"
          THEN retryWhileUnknown + 1
          ELSE retryWhileUnknown
    /\ effectStatus' = "UNKNOWN"
    /\ UNCHANGED <<cutDependencies, coveredDependencies, closureStatus,
                    policyResult, decision, scalarHeight, commits>>

ReconcileNoEffect ==
    /\ effectStatus = "UNKNOWN"
    /\ effectStatus' = "NO_EFFECT"
    /\ UNCHANGED <<cutDependencies, coveredDependencies, closureStatus,
                    policyResult, decision, scalarHeight, attempts,
                    retryWhileUnknown, commits>>

CommitEffect ==
    /\ decision = "ALLOW"
    /\ effectStatus = "UNKNOWN"
    /\ commits = 0
    /\ effectStatus' = "COMMITTED"
    /\ commits' = commits + 1
    /\ UNCHANGED <<cutDependencies, coveredDependencies, closureStatus,
                    policyResult, decision, scalarHeight, attempts,
                    retryWhileUnknown>>

DuplicateCommitMutant ==
    /\ AllowDuplicateCommit
    /\ decision = "ALLOW"
    /\ effectStatus = "COMMITTED"
    /\ commits = 1
    /\ commits' = commits + 1
    /\ UNCHANGED <<cutDependencies, coveredDependencies, closureStatus,
                    policyResult, decision, scalarHeight, effectStatus,
                    attempts, retryWhileUnknown>>

Next ==
    \/ BuildCompleteCut
    \/ BuildCutOmittingRevocation
    \/ VerifyClosure
    \/ ObserveDependency
    \/ SetPermit
    \/ SetForbid
    \/ SetDefaultDeny
    \/ Decide
    \/ StartEffectAttempt
    \/ ReconcileNoEffect
    \/ CommitEffect
    \/ DuplicateCommitMutant

Spec == Init /\ [][Next]_vars

AllowRequiresCompleteCut ==
    decision = "ALLOW"
    =>
    /\ policyResult = "PERMIT"
    /\ closureStatus = "VERIFIED_COMPLETE"
    /\ cutDependencies = Dependencies
    /\ Dependencies \subseteq coveredDependencies

EffectRequiresAllow ==
    commits > 0 => decision = "ALLOW"

AtMostOneCommit ==
    commits <= 1

NoRetryWhileUnknown ==
    retryWhileUnknown = 0

NoCommitWithoutEffectAttempt ==
    commits <= attempts

=============================================================================
