-------------------- MODULE ConstitutionalConsumptionV2 --------------------
EXTENDS Naturals, FiniteSets

(***************************************************************************
Temporal safety model for constitutional consumption/finality.

This model deliberately allows competing claims for the same use slot and
late observation of authenticated revocation order. Safety is defined by what
may FINALIZE and what may reach a real/public EFFECT, not by pretending
conflicting attempts never occur.

Ordinary model events advance a single monotonic logical clock. A revocation
observation is itself a later event, but it may carry an authenticated earlier
effective sequence. This models late evidence without allowing normal events
to jump backward in logical time.

Cutoff is either "Finality" or "Effect":
- Finality: finalization is the irrevocable commitment point.
- Effect: authority must still be unrevoked at the effect sequence.

The model is bounded by MaxSeq and the finite claim/use constants supplied in
the selected configuration file.
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
          fault,
          depsKnown,
          frozenFinalAt,
          frozenEffectAt,
          clock

vars == <<pending, finalAt, effectAt, revocations, fault, depsKnown,
          frozenFinalAt, frozenEffectAt, clock>>

ZeroMap == [c \in Claims |-> 0]

Init ==
    /\ pending = {}
    /\ finalAt = ZeroMap
    /\ effectAt = ZeroMap
    /\ revocations = {}
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
    /\ UNCHANGED <<finalAt, effectAt, revocations, fault, depsKnown,
                   frozenFinalAt, frozenEffectAt>>

LoseDependencies ==
    /\ Advanceable
    /\ depsKnown
    /\ depsKnown' = FALSE
    /\ clock' = NextSeq
    /\ UNCHANGED <<pending, finalAt, effectAt, revocations, fault,
                   frozenFinalAt, frozenEffectAt>>

RestoreDependencies ==
    /\ Advanceable
    /\ ~depsKnown
    /\ depsKnown' = TRUE
    /\ clock' = NextSeq
    /\ UNCHANGED <<pending, finalAt, effectAt, revocations, fault,
                   frozenFinalAt, frozenEffectAt>>

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
    /\ UNCHANGED <<pending, effectAt, revocations, fault, depsKnown,
                   frozenFinalAt, frozenEffectAt>>

ObserveRevocation(r) ==
    /\ Advanceable
    (* r is the authenticated EFFECTIVE sequence, not arrival sequence. *)
    /\ r \in 1..NextSeq
    /\ r \notin revocations
    /\ revocations' = revocations \cup {r}
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
    /\ UNCHANGED <<pending, finalAt, revocations, fault, depsKnown,
                   frozenFinalAt, frozenEffectAt>>

(* Duplicate delivery is explicitly idempotent: it does not mutate model state. *)
Redeliver(c) ==
    /\ c \in Claims
    /\ effectAt[c] # 0
    /\ UNCHANGED vars

Next ==
       \E c \in Claims : Claim(c)
    \/ LoseDependencies
    \/ RestoreDependencies
    \/ \E c \in Claims : Finalize(c)
    \/ \E r \in Seqs : ObserveRevocation(r)
    \/ \E c \in Claims : Execute(c)
    \/ \E c \in Claims : Redeliver(c)

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ pending \subseteq Claims
    /\ finalAt \in [Claims -> (Seqs \cup {0})]
    /\ effectAt \in [Claims -> (Seqs \cup {0})]
    /\ revocations \subseteq Seqs
    /\ fault \in BOOLEAN
    /\ depsKnown \in BOOLEAN
    /\ frozenFinalAt \in [Claims -> (Seqs \cup {0})]
    /\ frozenEffectAt \in [Claims -> (Seqs \cup {0})]
    /\ clock \in ClockValues

RecordedEventsDoNotExceedClock ==
    /\ \A c \in Claims : finalAt[c] <= clock
    /\ \A c \in Claims : effectAt[c] <= clock
    /\ \A r \in revocations : r <= clock

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

(*
The transition guard is not itself the theorem.  When no integrity fault has
been raised, every accepted commit point must remain consistent with all
revocation evidence currently known.  Late contradictory evidence is allowed,
but it must move the model into fault rather than leave an accepted history
silently contradictory.
*)
RevocationCutoffConsistentWhenFaultFree ==
    ~fault =>
        IF Cutoff = "Finality"
        THEN \A c \in Claims :
                 finalAt[c] # 0 => NoRevocationAtOrBefore(finalAt[c])
        ELSE \A c \in Claims :
                 effectAt[c] # 0 => NoRevocationAtOrBefore(effectAt[c])

Safety ==
    /\ TypeOK
    /\ RecordedEventsDoNotExceedClock
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
