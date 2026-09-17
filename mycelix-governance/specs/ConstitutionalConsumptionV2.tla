-------------------- MODULE ConstitutionalConsumptionV2 --------------------
EXTENDS Naturals, FiniteSets

(***************************************************************************
Temporal safety and non-vacuity model for constitutional consumption/finality.

The model preserves two distinct revocation orders:
- effective order: when the revocation is constitutionally effective;
- observation order: when this state machine learned that evidence.

A later observation may therefore carry an earlier authenticated effective
sequence. Ordinary model events still advance one monotonic logical clock.

Cutoff is either "Finality" or "Effect":
- Finality: accepted finalization is the irrevocable constitutional commit point.
- Effect: authority must remain live until the side effect is applied.
***************************************************************************)

CONSTANTS C1, C2, C3, U1, U2, MaxSeq, MaxUses, Cutoff

Claims == {C1, C2, C3}
UseSlots == {U1, U2}
Seqs == 1..MaxSeq
ClockValues == 0..MaxSeq

ASSUME MaxSeq \in Nat \ {0}
ASSUME MaxUses \in Nat \ {0}
ASSUME MaxUses <= Cardinality(UseSlots)
ASSUME Cutoff \in {"Finality", "Effect"}

(* C1 and C2 intentionally compete for U1; C3 targets U2. *)
ClaimUse == [c \in Claims |-> IF c = C3 THEN U2 ELSE U1]

VARIABLES pending,
          finalAt,
          effectAt,
          revocations,
          revocationObservedAt,
          fault,
          depsKnown,
          frozenFinalAt,
          frozenEffectAt,
          clock

vars == <<pending, finalAt, effectAt, revocations, revocationObservedAt,
          fault, depsKnown, frozenFinalAt, frozenEffectAt, clock>>

ZeroMap == [c \in Claims |-> 0]
ZeroRevocationObservationMap == [r \in Seqs |-> 0]

Init ==
    /\ pending = {}
    /\ finalAt = ZeroMap
    /\ effectAt = ZeroMap
    /\ revocations = {}
    /\ revocationObservedAt = ZeroRevocationObservationMap
    /\ fault = FALSE
    /\ depsKnown = TRUE
    /\ frozenFinalAt = ZeroMap
    /\ frozenEffectAt = ZeroMap
    /\ clock = 0

Advanceable == clock < MaxSeq
NextSeq == clock + 1

FinalizedClaims == {c \in Claims : finalAt[c] # 0}
FinalizedUses == {u \in UseSlots : \E c \in Claims : finalAt[c] # 0 /\ ClaimUse[c] = u}
EffectClaims == {c \in Claims : effectAt[c] # 0}

NoRevocationAtOrBefore(s) == \A r \in revocations : r > s

CompetingFinalizationAbsent(c) ==
    \A other \in Claims :
        (ClaimUse[other] = ClaimUse[c] /\ finalAt[other] # 0) => other = c

WouldContradictAcceptedHistory(r) ==
    IF Cutoff = "Finality"
    THEN \E c \in Claims : finalAt[c] # 0 /\ r <= finalAt[c]
    ELSE \E c \in Claims : effectAt[c] # 0 /\ r <= effectAt[c]

Claim(c) ==
    /\ Advanceable
    /\ c \in Claims
    /\ c \notin pending
    /\ pending' = pending \cup {c}
    /\ clock' = NextSeq
    /\ UNCHANGED <<finalAt, effectAt, revocations, revocationObservedAt,
                   fault, depsKnown, frozenFinalAt, frozenEffectAt>>

LoseDependencies ==
    /\ Advanceable
    /\ depsKnown
    /\ depsKnown' = FALSE
    /\ clock' = NextSeq
    /\ UNCHANGED <<pending, finalAt, effectAt, revocations,
                   revocationObservedAt, fault, frozenFinalAt, frozenEffectAt>>

RestoreDependencies ==
    /\ Advanceable
    /\ ~depsKnown
    /\ depsKnown' = TRUE
    /\ clock' = NextSeq
    /\ UNCHANGED <<pending, finalAt, effectAt, revocations,
                   revocationObservedAt, fault, frozenFinalAt, frozenEffectAt>>

Finalize(c) ==
    /\ Advanceable
    /\ c \in pending
    /\ finalAt[c] = 0
    /\ ~fault
    /\ depsKnown
    /\ NoRevocationAtOrBefore(NextSeq)
    /\ CompetingFinalizationAbsent(c)
    /\ Cardinality(FinalizedUses) < MaxUses
    /\ finalAt' = [finalAt EXCEPT ![c] = NextSeq]
    /\ clock' = NextSeq
    /\ UNCHANGED <<pending, effectAt, revocations, revocationObservedAt,
                   fault, depsKnown, frozenFinalAt, frozenEffectAt>>

ObserveRevocation(r) ==
    /\ Advanceable
    (* r is the authenticated EFFECTIVE sequence; NextSeq is observation order. *)
    /\ r \in 1..NextSeq
    /\ r \notin revocations
    /\ revocations' = revocations \cup {r}
    /\ revocationObservedAt' = [revocationObservedAt EXCEPT ![r] = NextSeq]
    /\ IF ~fault /\ WouldContradictAcceptedHistory(r)
          THEN /\ fault' = TRUE
               /\ frozenFinalAt' = finalAt
               /\ frozenEffectAt' = effectAt
          ELSE /\ fault' = fault
               /\ frozenFinalAt' = frozenFinalAt
               /\ frozenEffectAt' = frozenEffectAt
    /\ clock' = NextSeq
    /\ UNCHANGED <<pending, finalAt, effectAt, depsKnown>>

ExecutionStillAuthorized(s) ==
    IF Cutoff = "Effect"
    THEN NoRevocationAtOrBefore(s)
    ELSE TRUE

Execute(c) ==
    /\ Advanceable
    /\ c \in Claims
    /\ finalAt[c] # 0
    /\ effectAt[c] = 0
    /\ ~fault
    /\ depsKnown
    /\ NextSeq > finalAt[c]
    /\ ExecutionStillAuthorized(NextSeq)
    /\ effectAt' = [effectAt EXCEPT ![c] = NextSeq]
    /\ clock' = NextSeq
    /\ UNCHANGED <<pending, finalAt, revocations, revocationObservedAt,
                   fault, depsKnown, frozenFinalAt, frozenEffectAt>>

(* Duplicate delivery is explicitly idempotent: it does not mutate model state. *)
Redeliver(c) ==
    /\ c \in Claims
    /\ effectAt[c] # 0
    /\ UNCHANGED vars

Next ==
       (\E c \in Claims : Claim(c))
    \/ LoseDependencies
    \/ RestoreDependencies
    \/ (\E c \in Claims : Finalize(c))
    \/ (\E r \in Seqs : ObserveRevocation(r))
    \/ (\E c \in Claims : Execute(c))
    \/ (\E c \in Claims : Redeliver(c))

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ pending \subseteq Claims
    /\ finalAt \in [Claims -> (Seqs \cup {0})]
    /\ effectAt \in [Claims -> (Seqs \cup {0})]
    /\ revocations \subseteq Seqs
    /\ revocationObservedAt \in [Seqs -> ClockValues]
    /\ fault \in BOOLEAN
    /\ depsKnown \in BOOLEAN
    /\ frozenFinalAt \in [Claims -> (Seqs \cup {0})]
    /\ frozenEffectAt \in [Claims -> (Seqs \cup {0})]
    /\ clock \in ClockValues

RecordedEventsDoNotExceedClock ==
    /\ \A c \in Claims : finalAt[c] <= clock
    /\ \A c \in Claims : effectAt[c] <= clock
    /\ \A r \in revocations : r <= clock
    /\ \A r \in revocations : revocationObservedAt[r] <= clock

RevocationObservationConsistent ==
    /\ \A r \in Seqs : (r \in revocations) <=> revocationObservedAt[r] # 0
    /\ \A r \in revocations : r <= revocationObservedAt[r]

AtMostOneFinalizedPerUse ==
    \A u \in UseSlots :
        Cardinality({c \in Claims : finalAt[c] # 0 /\ ClaimUse[c] = u}) <= 1

NoEffectWithoutFinality ==
    \A c \in Claims : effectAt[c] # 0 => finalAt[c] # 0

EffectAfterFinality ==
    \A c \in Claims : effectAt[c] # 0 => effectAt[c] > finalAt[c]

FinalizedWithinBudget == Cardinality(FinalizedUses) <= MaxUses

FaultFreezesFinalityAndEffects ==
    fault => (finalAt = frozenFinalAt /\ effectAt = frozenEffectAt)

NoEffectForUnfinalizedCompetitor ==
    \A c \in Claims : effectAt[c] # 0 =>
        \A other \in Claims :
            (ClaimUse[other] = ClaimUse[c] /\ other # c) => finalAt[other] = 0

RevocationCutoffConsistentWhenFaultFree ==
    ~fault =>
        IF Cutoff = "Finality"
        THEN \A c \in Claims :
                 finalAt[c] # 0 => NoRevocationAtOrBefore(finalAt[c])
        ELSE \A c \in Claims :
                 effectAt[c] # 0 => NoRevocationAtOrBefore(effectAt[c])

(***************************************************************************
Temporal non-vacuity witnesses. These are state predicates, not safety claims.
Dedicated qualification configs assert their negations and expect TLC to find
the named invariant violation, proving each constitutional history is reachable
inside the recorded finite bound.
***************************************************************************)

EffectLateEarlierRevocationCancellationReached ==
    /\ Cutoff = "Effect"
    /\ ~fault
    /\ \E c \in Claims :
         \E r \in revocations :
           /\ finalAt[c] # 0
           /\ effectAt[c] = 0
           /\ r <= finalAt[c]
           /\ revocationObservedAt[r] > finalAt[c]

FinalityLateEarlierRevocationFaultReached ==
    /\ Cutoff = "Finality"
    /\ fault
    /\ \E c \in Claims :
         \E r \in revocations :
           /\ finalAt[c] # 0
           /\ r <= finalAt[c]
           /\ revocationObservedAt[r] > finalAt[c]

FinalityPostCommitRevocationEffectReached ==
    /\ Cutoff = "Finality"
    /\ ~fault
    /\ \E c \in Claims :
         \E r \in revocations :
           /\ finalAt[c] # 0
           /\ effectAt[c] # 0
           /\ r > finalAt[c]
           /\ revocationObservedAt[r] > finalAt[c]
           /\ effectAt[c] > revocationObservedAt[r]

EffectPostEffectContradictionFaultReached ==
    /\ Cutoff = "Effect"
    /\ fault
    /\ \E c \in Claims :
         \E r \in revocations :
           /\ effectAt[c] # 0
           /\ r <= effectAt[c]
           /\ revocationObservedAt[r] > effectAt[c]

NeverEffectLateEarlierRevocationCancellation ==
    ~EffectLateEarlierRevocationCancellationReached

NeverFinalityLateEarlierRevocationFault ==
    ~FinalityLateEarlierRevocationFaultReached

NeverFinalityPostCommitRevocationEffect ==
    ~FinalityPostCommitRevocationEffectReached

NeverEffectPostEffectContradictionFault ==
    ~EffectPostEffectContradictionFaultReached

Safety ==
    /\ TypeOK
    /\ RecordedEventsDoNotExceedClock
    /\ RevocationObservationConsistent
    /\ AtMostOneFinalizedPerUse
    /\ NoEffectWithoutFinality
    /\ EffectAfterFinality
    /\ FinalizedWithinBudget
    /\ FaultFreezesFinalityAndEffects
    /\ NoEffectForUnfinalizedCompetitor
    /\ RevocationCutoffConsistentWhenFaultFree

(***************************************************************************
Liveness is intentionally separate from Safety. A production liveness claim
must state assumptions about dependency recovery, witness availability and the
selected finality service. We do not smuggle those assumptions into safety.
***************************************************************************)

=============================================================================
