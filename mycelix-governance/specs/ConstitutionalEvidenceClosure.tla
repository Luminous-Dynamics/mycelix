---------------- MODULE ConstitutionalEvidenceClosure ----------------
EXTENDS Naturals, FiniteSets

(***************************************************************************
Dual-order finality/revocation provenance and evidence-closure abstraction.

Effective order and verifier observation order may either:
- share one normalized constitutional order (`SharedComparable`), or
- belong to independent domains (`Independent`).

Numeric effective<=observed comparison is performed ONLY in SharedComparable
mode. In Independent mode, observation monotonicity remains meaningful while the
numeric values of the two order domains are intentionally incomparable.

DetectionOnly never creates a closure watermark. Finality effective at/after an
already-known revocation is rejected before closure checks. New admissible
finality observed inside a closed interval is quarantined and faults. After a
fault, canonical finality/closure freezes while new evidence may still be kept.
***************************************************************************)

CONSTANTS MaxSeq, Profile, OrderMode

Seqs == 1..MaxSeq
ClockValues == 0..MaxSeq
Profiles == {"LocalIdempotent", "DetectionOnly", "WitnessedSingleSpend", "StrongConsensus"}
OrderModes == {"SharedComparable", "Independent"}

ASSUME MaxSeq \in Nat \ {0}
ASSUME Profile \in Profiles
ASSUME OrderMode \in OrderModes

VARIABLES clock,
          finalities,
          finalityObservedAt,
          rejectedFinalities,
          quarantinedFinalities,
          closureContradictions,
          revocations,
          revocationObservedAt,
          closureThrough,
          closureObservedAt,
          fault

vars == <<clock, finalities, finalityObservedAt, rejectedFinalities,
          quarantinedFinalities, closureContradictions, revocations,
          revocationObservedAt, closureThrough, closureObservedAt, fault>>

ZeroMap == [s \in Seqs |-> 0]

Init ==
    /\ clock = 0
    /\ finalities = {}
    /\ finalityObservedAt = ZeroMap
    /\ rejectedFinalities = {}
    /\ quarantinedFinalities = {}
    /\ closureContradictions = {}
    /\ revocations = {}
    /\ revocationObservedAt = ZeroMap
    /\ closureThrough = 0
    /\ closureObservedAt = 0
    /\ fault = FALSE

Advanceable == clock < MaxSeq
NextSeq == clock + 1
ClosureAllowed == Profile # "DetectionOnly"

OrderCompatible(effective, observed) ==
    IF OrderMode = "SharedComparable"
    THEN effective <= observed
    ELSE TRUE

KnownRevocationBlocks(f) == \E r \in revocations : r <= f

(***************************************************************************
AdvanceObservation represents another authenticated event in the observation
stream. It is model scaffolding for delayed evidence, not constitutional
progress or a liveness claim.
***************************************************************************)
AdvanceObservation ==
    /\ Advanceable
    /\ clock' = NextSeq
    /\ UNCHANGED <<finalities, finalityObservedAt, rejectedFinalities,
                   quarantinedFinalities, closureContradictions, revocations,
                   revocationObservedAt, closureThrough, closureObservedAt,
                   fault>>

ObserveRevocation(r) ==
    /\ Advanceable
    /\ r \in Seqs
    /\ OrderCompatible(r, NextSeq)
    /\ r \notin revocations
    /\ revocations' = revocations \cup {r}
    /\ revocationObservedAt' = [revocationObservedAt EXCEPT ![r] = NextSeq]
    /\ clock' = NextSeq
    /\ UNCHANGED <<finalities, finalityObservedAt, rejectedFinalities,
                   quarantinedFinalities, closureContradictions,
                   closureThrough, closureObservedAt, fault>>

ObserveFinality(f) ==
    /\ Advanceable
    /\ f \in Seqs
    /\ OrderCompatible(f, NextSeq)
    /\ f \notin (finalities \cup rejectedFinalities \cup quarantinedFinalities)
    /\ IF fault
          THEN /\ quarantinedFinalities' = quarantinedFinalities \cup {f}
               /\ finalityObservedAt' = [finalityObservedAt EXCEPT ![f] = NextSeq]
               /\ finalities' = finalities
               /\ rejectedFinalities' = rejectedFinalities
               /\ closureContradictions' = closureContradictions
               /\ fault' = fault
          ELSE IF KnownRevocationBlocks(f)
               THEN /\ rejectedFinalities' = rejectedFinalities \cup {f}
                    /\ finalityObservedAt' = [finalityObservedAt EXCEPT ![f] = NextSeq]
                    /\ finalities' = finalities
                    /\ quarantinedFinalities' = quarantinedFinalities
                    /\ closureContradictions' = closureContradictions
                    /\ fault' = fault
               ELSE IF closureThrough # 0 /\ f <= closureThrough
                    THEN /\ quarantinedFinalities' = quarantinedFinalities \cup {f}
                         /\ closureContradictions' = closureContradictions \cup {f}
                         /\ finalityObservedAt' = [finalityObservedAt EXCEPT ![f] = NextSeq]
                         /\ fault' = TRUE
                         /\ finalities' = finalities
                         /\ rejectedFinalities' = rejectedFinalities
                    ELSE /\ finalities' = finalities \cup {f}
                         /\ finalityObservedAt' = [finalityObservedAt EXCEPT ![f] = NextSeq]
                         /\ rejectedFinalities' = rejectedFinalities
                         /\ quarantinedFinalities' = quarantinedFinalities
                         /\ closureContradictions' = closureContradictions
                         /\ fault' = fault
    /\ clock' = NextSeq
    /\ UNCHANGED <<revocations, revocationObservedAt,
                   closureThrough, closureObservedAt>>

AcceptClosure(n) ==
    /\ Advanceable
    /\ ~fault
    /\ ClosureAllowed
    /\ n \in Seqs
    /\ OrderCompatible(n, NextSeq)
    /\ n >= closureThrough
    /\ closureThrough' = n
    /\ closureObservedAt' = NextSeq
    /\ clock' = NextSeq
    /\ UNCHANGED <<finalities, finalityObservedAt, rejectedFinalities,
                   quarantinedFinalities, closureContradictions, revocations,
                   revocationObservedAt, fault>>

Next ==
       AdvanceObservation
    \/ (\E r \in Seqs : ObserveRevocation(r))
    \/ (\E f \in Seqs : ObserveFinality(f))
    \/ (\E n \in Seqs : AcceptClosure(n))

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ clock \in ClockValues
    /\ finalities \subseteq Seqs
    /\ rejectedFinalities \subseteq Seqs
    /\ quarantinedFinalities \subseteq Seqs
    /\ closureContradictions \subseteq Seqs
    /\ finalityObservedAt \in [Seqs -> ClockValues]
    /\ revocations \subseteq Seqs
    /\ revocationObservedAt \in [Seqs -> ClockValues]
    /\ closureThrough \in ClockValues
    /\ closureObservedAt \in ClockValues
    /\ fault \in BOOLEAN

EvidenceSetsDisjoint ==
    /\ finalities \cap rejectedFinalities = {}
    /\ finalities \cap quarantinedFinalities = {}
    /\ rejectedFinalities \cap quarantinedFinalities = {}
    /\ closureContradictions \subseteq quarantinedFinalities

ObservationOrderConsistent ==
    /\ \A f \in (finalities \cup rejectedFinalities \cup quarantinedFinalities) :
           /\ finalityObservedAt[f] # 0
           /\ OrderCompatible(f, finalityObservedAt[f])
           /\ finalityObservedAt[f] <= clock
    /\ \A r \in revocations :
           /\ revocationObservedAt[r] # 0
           /\ OrderCompatible(r, revocationObservedAt[r])
           /\ revocationObservedAt[r] <= clock
    /\ closureThrough # 0 =>
           /\ closureObservedAt # 0
           /\ OrderCompatible(closureThrough, closureObservedAt)
           /\ closureObservedAt <= clock

DetectionOnlyHasNoClosure ==
    Profile = "DetectionOnly" => closureThrough = 0

ClosedEvidenceWasObservedBeforeClosure ==
    closureThrough # 0 =>
        \A f \in finalities :
            f <= closureThrough => finalityObservedAt[f] <= closureObservedAt

RejectedFinalityHasKnownRevocation ==
    \A f \in rejectedFinalities : \E r \in revocations : r <= f

ClosureContradictionsAreClosedEvidence ==
    \A f \in closureContradictions :
        /\ closureThrough # 0
        /\ f <= closureThrough

FaultHasClosureContradiction ==
    fault => closureContradictions # {}

(***************************************************************************
Reachability predicates used by independent non-vacuity qualification.
***************************************************************************)
LatePreRevocationFinalityAcceptedReached ==
    \E r \in revocations :
        \E f \in finalities :
            /\ f < r
            /\ finalityObservedAt[f] > revocationObservedAt[r]

PostClosureContradictionFaultReached ==
    /\ fault
    /\ closureContradictions # {}

ClosureReached == closureThrough # 0

IndependentIncomparableAcceptedReached ==
    /\ OrderMode = "Independent"
    /\ \E f \in finalities : f > finalityObservedAt[f]

NeverLatePreRevocationFinalityAccepted ==
    ~LatePreRevocationFinalityAcceptedReached

NeverPostClosureContradictionFault ==
    ~PostClosureContradictionFaultReached

NeverClosureReached ==
    ~ClosureReached

NeverIndependentIncomparableAccepted ==
    ~IndependentIncomparableAcceptedReached

Safety ==
    /\ TypeOK
    /\ EvidenceSetsDisjoint
    /\ ObservationOrderConsistent
    /\ DetectionOnlyHasNoClosure
    /\ ClosedEvidenceWasObservedBeforeClosure
    /\ RejectedFinalityHasKnownRevocation
    /\ ClosureContradictionsAreClosedEvidence
    /\ FaultHasClosureContradiction

=============================================================================
