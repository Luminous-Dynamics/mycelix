---------------- MODULE OracleReceiptImmutability ----------------
EXTENDS Naturals, FiniteSets, TLC

ReceiptA == "receipt:A"
ReceiptB == "receipt:B"
Receipts == {ReceiptA, ReceiptB}

VARIABLES chain, nextSequence, answerA, answerB
vars == <<chain, nextSequence, answerA, answerB>>

Init ==
    /\ chain = {}
    /\ nextSequence = 1
    /\ answerA = "unset"
    /\ answerB = "unset"

AppendOriginal ==
    /\ chain = {}
    /\ nextSequence = 1
    /\ chain' = {ReceiptA}
    /\ nextSequence' = 2
    /\ answerA' = "affirm"
    /\ UNCHANGED answerB

AppendCorrection ==
    /\ chain = {ReceiptA}
    /\ nextSequence = 2
    /\ ReceiptB \notin chain
    /\ chain' = chain \cup {ReceiptB}
    /\ nextSequence' = 3
    /\ answerB' = "reject"
    /\ UNCHANGED answerA

KeepState ==
    /\ chain = Receipts
    /\ UNCHANGED vars

Next == AppendOriginal \/ AppendCorrection \/ KeepState
Spec == Init /\ [][Next]_vars

(* Faulty implementation rewrites the old receipt instead of appending. *)
MutantRewrite ==
    /\ chain = {ReceiptA}
    /\ answerA = "affirm"
    /\ answerA' = "reject"
    /\ UNCHANGED <<chain, nextSequence, answerB>>

MutantNext == Next \/ MutantRewrite
MutantSpec == Init /\ [][MutantNext]_vars

TypeOK ==
    /\ chain \subseteq Receipts
    /\ nextSequence \in {1, 2, 3}
    /\ answerA \in {"unset", "affirm", "reject"}
    /\ answerB \in {"unset", "affirm", "reject"}

AppendOnly == chain' = chain \/ chain \subseteq chain'
NoOverwrite == answerA = "affirm" => answerA' = "affirm"
SequenceMonotone == nextSequence' >= nextSequence

=============================================================================
