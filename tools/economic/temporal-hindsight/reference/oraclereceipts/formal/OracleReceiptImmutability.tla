---------------- MODULE OracleReceiptImmutability ----------------
EXTENDS Naturals, FiniteSets, TLC

ReceiptA == "receipt:A"
ReceiptB == "receipt:B"
Receipts == {ReceiptA, ReceiptB}
Answers == {"unset", "affirm", "reject"}

VARIABLES chain, nextSequence, answers
vars == <<chain, nextSequence, answers>>

Init ==
    /\ chain = {}
    /\ nextSequence = 1
    /\ answers = [r \in Receipts |-> "unset"]

AppendOriginal ==
    /\ chain = {}
    /\ nextSequence = 1
    /\ chain' = {ReceiptA}
    /\ nextSequence' = 2
    /\ answers' = [answers EXCEPT ![ReceiptA] = "affirm"]

AppendCorrection ==
    /\ chain = {ReceiptA}
    /\ nextSequence = 2
    /\ ReceiptB \notin chain
    /\ chain' = chain \cup {ReceiptB}
    /\ nextSequence' = 3
    /\ answers' = [answers EXCEPT ![ReceiptB] = "reject"]

KeepState ==
    /\ chain = Receipts
    /\ UNCHANGED vars

Next == AppendOriginal \/ AppendCorrection \/ KeepState
Spec == Init /\ [][Next]_vars

(* Faulty implementation rewrites the original receipt's answer. *)
MutantRewrite ==
    /\ chain = {ReceiptA}
    /\ answers[ReceiptA] = "affirm"
    /\ answers' = [answers EXCEPT ![ReceiptA] = "reject"]
    /\ UNCHANGED <<chain, nextSequence>>

MutantNext == Next \/ MutantRewrite
MutantSpec == Init /\ [][MutantNext]_vars

TypeOK ==
    /\ chain \subseteq Receipts
    /\ nextSequence \in {1, 2, 3}
    /\ answers \in [Receipts -> Answers]

SequenceConsistent == nextSequence = Cardinality(chain) + 1
CorrectionHasPredecessor == ReceiptB \in chain => ReceiptA \in chain
OriginalReceiptImmutable == ReceiptA \in chain => answers[ReceiptA] = "affirm"

=============================================================================
