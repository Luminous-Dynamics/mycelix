---------------- MODULE TemporalFrontierNoninterference ----------------
EXTENDS Naturals, FiniteSets, TLC

CONSTANTS OwnerA, OwnerB, BaseA, BaseB, CorrectionA, CorrectionB,
          PolicyEpoch, QueryInstant

Owners == {OwnerA, OwnerB}
BaseReceipts == {BaseA, BaseB}
Corrections == {CorrectionA, CorrectionB}
ReceiptUniverse == BaseReceipts \cup Corrections

ASSUME OwnerA # OwnerB
ASSUME BaseA # BaseB
ASSUME CorrectionA # CorrectionB
ASSUME BaseA # CorrectionA
ASSUME BaseA # CorrectionB
ASSUME BaseB # CorrectionA
ASSUME BaseB # CorrectionB

ReceiptOwner(receipt) ==
    IF receipt \in {BaseA, CorrectionA} THEN OwnerA ELSE OwnerB

ReceiptSequence(receipt) ==
    IF receipt \in BaseReceipts THEN 1 ELSE 2

CorrectionFor(owner) ==
    IF owner = OwnerA THEN CorrectionA ELSE CorrectionB

(* F0 is a vector of owner-local cuts, not a scalar/global sequence. *)
HistoricalCut == [owner \in Owners |-> 1]

Prefix(world, cut) ==
    {receipt \in world :
        ReceiptSequence(receipt) <= cut[ReceiptOwner(receipt)]}

CorrectionAccepted(owner, world, cut) ==
    CorrectionFor(owner) \in Prefix(world, cut)

(* Each correction represents a later accepted VALID_TO assertion whose
   business-time endpoint is at/before QueryInstant. It therefore changes
   the current point-query result, but only after entering that owner's cut. *)
QueryResult(world, cut) ==
    {owner \in Owners : ~CorrectionAccepted(owner, world, cut)}

ExpectedCertificate(world) ==
    [ frontier |-> HistoricalCut,
      accepted_prefix |-> Prefix(world, HistoricalCut),
      policy_epoch |-> PolicyEpoch,
      query_instant |-> QueryInstant,
      result |-> QueryResult(world, HistoricalCut) ]

VARIABLES visibleA, visibleB,
          currentCutA, currentCutB,
          replayA, replayB,
          certificateA, certificateB

vars == <<visibleA, visibleB,
          currentCutA, currentCutB,
          replayA, replayB,
          certificateA, certificateB>>

Init ==
    /\ visibleA = BaseReceipts
    /\ visibleB = BaseReceipts
    /\ currentCutA = HistoricalCut
    /\ currentCutB = HistoricalCut
    /\ replayA = Owners
    /\ replayB = Owners
    /\ certificateA = ExpectedCertificate(BaseReceipts)
    /\ certificateB = ExpectedCertificate(BaseReceipts)

ObserveCorrectionA(owner) ==
    /\ owner \in Owners
    /\ CorrectionFor(owner) \notin visibleA
    /\ visibleA' = visibleA \cup {CorrectionFor(owner)}
    /\ UNCHANGED <<visibleB, currentCutA, currentCutB,
                   replayA, replayB, certificateA, certificateB>>

ObserveCorrectionB(owner) ==
    /\ owner \in Owners
    /\ CorrectionFor(owner) \notin visibleB
    /\ visibleB' = visibleB \cup {CorrectionFor(owner)}
    /\ UNCHANGED <<visibleA, currentCutA, currentCutB,
                   replayA, replayB, certificateA, certificateB>>

AdvanceCutA(owner) ==
    /\ owner \in Owners
    /\ CorrectionFor(owner) \in visibleA
    /\ currentCutA[owner] = HistoricalCut[owner]
    /\ currentCutA' = [currentCutA EXCEPT ![owner] = 2]
    /\ UNCHANGED <<visibleA, visibleB, currentCutB,
                   replayA, replayB, certificateA, certificateB>>

AdvanceCutB(owner) ==
    /\ owner \in Owners
    /\ CorrectionFor(owner) \in visibleB
    /\ currentCutB[owner] = HistoricalCut[owner]
    /\ currentCutB' = [currentCutB EXCEPT ![owner] = 2]
    /\ UNCHANGED <<visibleA, visibleB, currentCutA,
                   replayA, replayB, certificateA, certificateB>>

ReplayAtHistoricalCutA ==
    /\ replayA' = QueryResult(visibleA, HistoricalCut)
    /\ UNCHANGED <<visibleA, visibleB, currentCutA, currentCutB,
                   replayB, certificateA, certificateB>>

ReplayAtHistoricalCutB ==
    /\ replayB' = QueryResult(visibleB, HistoricalCut)
    /\ UNCHANGED <<visibleA, visibleB, currentCutA, currentCutB,
                   replayA, certificateA, certificateB>>

Next ==
    \/ \E owner \in Owners : ObserveCorrectionA(owner)
    \/ \E owner \in Owners : ObserveCorrectionB(owner)
    \/ \E owner \in Owners : AdvanceCutA(owner)
    \/ \E owner \in Owners : AdvanceCutB(owner)
    \/ ReplayAtHistoricalCutA
    \/ ReplayAtHistoricalCutB

(* Negative-control actions model two independent implementation bugs. *)
ReplayUsingLatestCutA ==
    /\ replayA' = QueryResult(visibleA, currentCutA)
    /\ UNCHANGED <<visibleA, visibleB, currentCutA, currentCutB,
                   replayB, certificateA, certificateB>>

RelabelHistoricalCertificateA ==
    /\ certificateA' = [certificateA EXCEPT !.frontier = currentCutA]
    /\ UNCHANGED <<visibleA, visibleB, currentCutA, currentCutB,
                   replayA, replayB, certificateB>>

(* Test each mutation independently. A failure in one mutant must not mask
   a missing counterexample in the other. *)
BuggyReplayNext ==
    \/ Next
    \/ ReplayUsingLatestCutA

BuggyCertificateNext ==
    \/ Next
    \/ RelabelHistoricalCertificateA

BuggyReplaySpec == Init /\ [][BuggyReplayNext]_vars
BuggyCertificateSpec == Init /\ [][BuggyCertificateNext]_vars

(* Kept as a combined exploratory spec; qualification runs the two mutants
   above separately and checks the exact invariant each one violates. *)
BuggyNext ==
    \/ BuggyReplayNext
    \/ RelabelHistoricalCertificateA

Spec == Init /\ [][Next]_vars
BuggySpec == Init /\ [][BuggyNext]_vars

TypeOK ==
    /\ visibleA \subseteq ReceiptUniverse
    /\ visibleB \subseteq ReceiptUniverse
    /\ currentCutA \in [Owners -> {1, 2}]
    /\ currentCutB \in [Owners -> {1, 2}]
    /\ replayA \subseteq Owners
    /\ replayB \subseteq Owners

SameHistoricalPrefix ==
    Prefix(visibleA, HistoricalCut) = Prefix(visibleB, HistoricalCut)

HistoricalReplayBound ==
    /\ replayA = QueryResult(visibleA, HistoricalCut)
    /\ replayB = QueryResult(visibleB, HistoricalCut)

HistoricalReplayEquality ==
    replayA = replayB

NoHindsight ==
    SameHistoricalPrefix =>
        QueryResult(visibleA, HistoricalCut) =
        QueryResult(visibleB, HistoricalCut)

ExactHistoricalCertificateBinding ==
    /\ certificateA = ExpectedCertificate(visibleA)
    /\ certificateB = ExpectedCertificate(visibleB)

HistoricalCertificatesUseF0 ==
    /\ certificateA.frontier = HistoricalCut
    /\ certificateB.frontier = HistoricalCut

=============================================================================
