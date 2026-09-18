------------------------- MODULE ConstitutionalEffectOutbox -------------------------
EXTENDS Naturals, FiniteSets

CONSTANT MaxStep

Ops == {"A", "B"}
NoOp == "None"
Phases == {
    "Absent",
    "Prepared",
    "EffectPending",
    "UnknownOutcome",
    "EffectObserved",
    "ReceiptCommitted",
    "Aborted",
    "IntegrityHalted"
}
StepValues == 0..MaxStep

VARIABLES step, phase, committedOp, outbox,
          externalEffect, effectCount, observedEffect, receipt,
          integrityHalt, worker, inFlight, callerAck,
          committedHistory, effectHistory, receiptHistory, callerAckHistory,
          deliveryAttempts, crashCount,
          reconciledSuccess, reconciledNoEffect, contradictionSeen

vars == <<step, phase, committedOp, outbox,
          externalEffect, effectCount, observedEffect, receipt,
          integrityHalt, worker, inFlight, callerAck,
          committedHistory, effectHistory, receiptHistory, callerAckHistory,
          deliveryAttempts, crashCount,
          reconciledSuccess, reconciledNoEffect, contradictionSeen>>

Init ==
    /\ step = 0
    /\ phase = [op \in Ops |-> "Absent"]
    /\ committedOp = NoOp
    /\ outbox = NoOp
    /\ externalEffect = NoOp
    /\ effectCount = 0
    /\ observedEffect = NoOp
    /\ receipt = NoOp
    /\ integrityHalt = FALSE
    /\ worker = NoOp
    /\ inFlight = NoOp
    /\ callerAck = NoOp
    /\ committedHistory = {}
    /\ effectHistory = {}
    /\ receiptHistory = {}
    /\ callerAckHistory = {}
    /\ deliveryAttempts = 0
    /\ crashCount = 0
    /\ reconciledSuccess = FALSE
    /\ reconciledNoEffect = FALSE
    /\ contradictionSeen = FALSE

Advanceable == step < MaxStep
NextStep == step + 1

Prepare(op) ==
    /\ Advanceable
    /\ ~integrityHalt
    /\ committedOp = NoOp
    /\ phase[op] = "Absent"
    /\ phase' = [phase EXCEPT ![op] = "Prepared"]
    /\ step' = NextStep
    /\ UNCHANGED <<committedOp, outbox, externalEffect, effectCount,
                   observedEffect, receipt, integrityHalt, worker, inFlight,
                   callerAck, committedHistory, effectHistory, receiptHistory,
                   callerAckHistory, deliveryAttempts, crashCount,
                   reconciledSuccess, reconciledNoEffect, contradictionSeen>>

CommitAndEnqueue(op) ==
    /\ Advanceable
    /\ ~integrityHalt
    /\ committedOp = NoOp
    /\ phase[op] = "Prepared"
    /\ committedOp' = op
    /\ outbox' = op
    /\ phase' = [x \in Ops |->
          IF x = op THEN "EffectPending"
          ELSE IF phase[x] = "Prepared" THEN "Aborted"
          ELSE phase[x]]
    /\ committedHistory' = committedHistory \cup {op}
    /\ step' = NextStep
    /\ UNCHANGED <<externalEffect, effectCount, observedEffect, receipt,
                   integrityHalt, worker, inFlight, callerAck,
                   effectHistory, receiptHistory, callerAckHistory,
                   deliveryAttempts, crashCount,
                   reconciledSuccess, reconciledNoEffect, contradictionSeen>>

ClaimWork(op) ==
    /\ Advanceable
    /\ ~integrityHalt
    /\ committedOp = op
    /\ phase[op] = "EffectPending"
    /\ worker = NoOp
    /\ worker' = op
    /\ step' = NextStep
    /\ UNCHANGED <<phase, committedOp, outbox, externalEffect, effectCount,
                   observedEffect, receipt, integrityHalt, inFlight, callerAck,
                   committedHistory, effectHistory, receiptHistory,
                   callerAckHistory, deliveryAttempts, crashCount,
                   reconciledSuccess, reconciledNoEffect, contradictionSeen>>

StartRequest(op) ==
    /\ Advanceable
    /\ ~integrityHalt
    /\ committedOp = op
    /\ outbox = op
    /\ phase[op] = "EffectPending"
    /\ worker = op
    /\ inFlight = NoOp
    /\ inFlight' = op
    /\ step' = NextStep
    /\ UNCHANGED <<phase, committedOp, outbox, externalEffect, effectCount,
                   observedEffect, receipt, integrityHalt, worker, callerAck,
                   committedHistory, effectHistory, receiptHistory,
                   callerAckHistory, deliveryAttempts, crashCount,
                   reconciledSuccess, reconciledNoEffect, contradictionSeen>>

ExternalDeliver(op) ==
    /\ Advanceable
    /\ ~integrityHalt
    /\ committedOp = op
    /\ outbox = op
    /\ inFlight = op
    /\ externalEffect \in {NoOp, op}
    /\ externalEffect' = op
    /\ effectCount' = 1
    /\ effectHistory' = effectHistory \cup {op}
    /\ deliveryAttempts' = deliveryAttempts + 1
    /\ step' = NextStep
    /\ UNCHANGED <<phase, committedOp, outbox, observedEffect, receipt,
                   integrityHalt, worker, inFlight, callerAck,
                   committedHistory, receiptHistory, callerAckHistory,
                   crashCount, reconciledSuccess, reconciledNoEffect,
                   contradictionSeen>>

ReceiveSuccessAck(op) ==
    /\ Advanceable
    /\ ~integrityHalt
    /\ committedOp = op
    /\ phase[op] = "EffectPending"
    /\ inFlight = op
    /\ externalEffect = op
    /\ phase' = [phase EXCEPT ![op] = "EffectObserved"]
    /\ observedEffect' = op
    /\ worker' = NoOp
    /\ inFlight' = NoOp
    /\ step' = NextStep
    /\ UNCHANGED <<committedOp, outbox, externalEffect, effectCount, receipt,
                   integrityHalt, callerAck, committedHistory, effectHistory,
                   receiptHistory, callerAckHistory, deliveryAttempts,
                   crashCount, reconciledSuccess, reconciledNoEffect,
                   contradictionSeen>>

LoseOrTimeoutAck(op) ==
    /\ Advanceable
    /\ ~integrityHalt
    /\ committedOp = op
    /\ phase[op] = "EffectPending"
    /\ inFlight = op
    /\ phase' = [phase EXCEPT ![op] = "UnknownOutcome"]
    /\ worker' = NoOp
    /\ inFlight' = NoOp
    /\ step' = NextStep
    /\ UNCHANGED <<committedOp, outbox, externalEffect, effectCount,
                   observedEffect, receipt, integrityHalt, callerAck,
                   committedHistory, effectHistory, receiptHistory,
                   callerAckHistory, deliveryAttempts, crashCount,
                   reconciledSuccess, reconciledNoEffect, contradictionSeen>>

ReconcileSuccess(op) ==
    /\ Advanceable
    /\ ~integrityHalt
    /\ committedOp = op
    /\ phase[op] = "UnknownOutcome"
    /\ externalEffect = op
    /\ phase' = [phase EXCEPT ![op] = "EffectObserved"]
    /\ observedEffect' = op
    /\ reconciledSuccess' = TRUE
    /\ step' = NextStep
    /\ UNCHANGED <<committedOp, outbox, externalEffect, effectCount, receipt,
                   integrityHalt, worker, inFlight, callerAck,
                   committedHistory, effectHistory, receiptHistory,
                   callerAckHistory, deliveryAttempts, crashCount,
                   reconciledNoEffect, contradictionSeen>>

ReconcileNoEffect(op) ==
    /\ Advanceable
    /\ ~integrityHalt
    /\ committedOp = op
    /\ phase[op] = "UnknownOutcome"
    /\ externalEffect = NoOp
    /\ phase' = [phase EXCEPT ![op] = "EffectPending"]
    /\ reconciledNoEffect' = TRUE
    /\ step' = NextStep
    /\ UNCHANGED <<committedOp, outbox, externalEffect, effectCount,
                   observedEffect, receipt, integrityHalt, worker, inFlight,
                   callerAck, committedHistory, effectHistory, receiptHistory,
                   callerAckHistory, deliveryAttempts, crashCount,
                   reconciledSuccess, contradictionSeen>>

CommitReceipt(op) ==
    /\ Advanceable
    /\ ~integrityHalt
    /\ committedOp = op
    /\ phase[op] = "EffectObserved"
    /\ observedEffect = op
    /\ receipt = NoOp
    /\ phase' = [phase EXCEPT ![op] = "ReceiptCommitted"]
    /\ receipt' = op
    /\ receiptHistory' = receiptHistory \cup {op}
    /\ step' = NextStep
    /\ UNCHANGED <<committedOp, outbox, externalEffect, effectCount,
                   observedEffect, integrityHalt, worker, inFlight, callerAck,
                   committedHistory, effectHistory, callerAckHistory,
                   deliveryAttempts, crashCount, reconciledSuccess,
                   reconciledNoEffect, contradictionSeen>>

AcknowledgeCaller(op) ==
    /\ Advanceable
    /\ receipt = op
    /\ callerAck = NoOp
    /\ callerAck' = op
    /\ callerAckHistory' = callerAckHistory \cup {op}
    /\ step' = NextStep
    /\ UNCHANGED <<phase, committedOp, outbox, externalEffect, effectCount,
                   observedEffect, receipt, integrityHalt, worker, inFlight,
                   committedHistory, effectHistory, receiptHistory,
                   deliveryAttempts, crashCount, reconciledSuccess,
                   reconciledNoEffect, contradictionSeen>>

Crash ==
    /\ Advanceable
    /\ worker' = NoOp
    /\ inFlight' = NoOp
    /\ callerAck' = NoOp
    /\ crashCount' = crashCount + 1
    /\ step' = NextStep
    /\ UNCHANGED <<phase, committedOp, outbox, externalEffect, effectCount,
                   observedEffect, receipt, integrityHalt, committedHistory,
                   effectHistory, receiptHistory, callerAckHistory,
                   deliveryAttempts, reconciledSuccess, reconciledNoEffect,
                   contradictionSeen>>

ObserveContradiction(op) ==
    /\ Advanceable
    /\ ~integrityHalt
    /\ committedOp = op
    /\ phase[op] \in {"UnknownOutcome", "EffectObserved", "ReceiptCommitted"}
    /\ integrityHalt' = TRUE
    /\ contradictionSeen' = TRUE
    /\ phase' = [phase EXCEPT ![op] = "IntegrityHalted"]
    /\ worker' = NoOp
    /\ inFlight' = NoOp
    /\ callerAck' = NoOp
    /\ step' = NextStep
    /\ UNCHANGED <<committedOp, outbox, externalEffect, effectCount,
                   observedEffect, receipt, committedHistory, effectHistory,
                   receiptHistory, callerAckHistory, deliveryAttempts,
                   crashCount, reconciledSuccess, reconciledNoEffect>>

Next ==
       (\E op \in Ops : Prepare(op))
    \/ (\E op \in Ops : CommitAndEnqueue(op))
    \/ (\E op \in Ops : ClaimWork(op))
    \/ (\E op \in Ops : StartRequest(op))
    \/ (\E op \in Ops : ExternalDeliver(op))
    \/ (\E op \in Ops : ReceiveSuccessAck(op))
    \/ (\E op \in Ops : LoseOrTimeoutAck(op))
    \/ (\E op \in Ops : ReconcileSuccess(op))
    \/ (\E op \in Ops : ReconcileNoEffect(op))
    \/ (\E op \in Ops : CommitReceipt(op))
    \/ (\E op \in Ops : AcknowledgeCaller(op))
    \/ Crash
    \/ (\E op \in Ops : ObserveContradiction(op))

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ step \in StepValues
    /\ phase \in [Ops -> Phases]
    /\ committedOp \in Ops \cup {NoOp}
    /\ outbox \in Ops \cup {NoOp}
    /\ externalEffect \in Ops \cup {NoOp}
    /\ effectCount \in 0..1
    /\ observedEffect \in Ops \cup {NoOp}
    /\ receipt \in Ops \cup {NoOp}
    /\ integrityHalt \in BOOLEAN
    /\ worker \in Ops \cup {NoOp}
    /\ inFlight \in Ops \cup {NoOp}
    /\ callerAck \in Ops \cup {NoOp}
    /\ committedHistory \subseteq Ops
    /\ effectHistory \subseteq Ops
    /\ receiptHistory \subseteq Ops
    /\ callerAckHistory \subseteq Ops
    /\ deliveryAttempts \in StepValues
    /\ crashCount \in StepValues
    /\ reconciledSuccess \in BOOLEAN
    /\ reconciledNoEffect \in BOOLEAN
    /\ contradictionSeen \in BOOLEAN

AtMostOneCommittedOpPerUse ==
    /\ Cardinality(committedHistory) <= 1
    /\ (committedOp = NoOp) = (committedHistory = {})
    /\ committedOp # NoOp => committedOp \in committedHistory

EffectRequiresConstitutionalCommit ==
    externalEffect # NoOp =>
        /\ committedOp = externalEffect
        /\ outbox = externalEffect
        /\ externalEffect \in committedHistory

ReceiptRequiresObservedEffect ==
    receipt # NoOp =>
        /\ observedEffect = receipt
        /\ receipt \in effectHistory
        /\ receipt \in receiptHistory

ExternalEffectIdentityMatchesCommittedOp ==
    externalEffect = NoOp \/ externalEffect = committedOp

RetryDoesNotChangeOperationIdentity ==
    /\ (outbox = NoOp \/ outbox = committedOp)
    /\ (worker = NoOp \/ worker = committedOp)
    /\ (inFlight = NoOp \/ inFlight = committedOp)

UnknownOutcomeBlocksConflictingOp ==
    \A op \in Ops : phase[op] = "UnknownOutcome" =>
        /\ committedOp = op
        /\ \A other \in Ops \ {op} : phase[other] \in {"Absent", "Aborted"}

CrashPreservesDurableHistory ==
    /\ (committedHistory # {} => committedOp # NoOp)
    /\ (effectHistory # {} => externalEffect # NoOp)
    /\ (receiptHistory # {} => receipt # NoOp)

ReceiptMonotonic ==
    /\ Cardinality(receiptHistory) <= 1
    /\ (receiptHistory = {} \/ receipt \in receiptHistory)

IntegrityHaltDoesNotEraseEffect ==
    integrityHalt =>
        /\ (effectHistory = {} \/ externalEffect # NoOp)
        /\ (receiptHistory = {} \/ receipt # NoOp)

CallerAckIsNotConstitutionalState ==
    /\ (callerAck = NoOp \/ callerAck = receipt)
    /\ callerAckHistory \subseteq receiptHistory

LogicalEffectAtMostOnce ==
    /\ effectCount \in 0..1
    /\ (effectCount = 0) = (externalEffect = NoOp)

OutboxBeforeEffect ==
    externalEffect # NoOp => outbox = externalEffect

(***************************************************************************
Named reachability predicates. These are non-vacuity evidence only.
***************************************************************************)
CrashAfterPrepareReached ==
    crashCount > 0 /\ committedOp = NoOp /\ \E op \in Ops : phase[op] = "Prepared"

CrashAfterCommitBeforeEffectReached ==
    crashCount > 0 /\ committedOp # NoOp /\ externalEffect = NoOp
    /\ phase[committedOp] = "EffectPending"

UnknownOutcomeWithEffectReached ==
    committedOp # NoOp /\ phase[committedOp] = "UnknownOutcome"
    /\ externalEffect = committedOp

UnknownOutcomeNoEffectReached ==
    committedOp # NoOp /\ phase[committedOp] = "UnknownOutcome"
    /\ externalEffect = NoOp

ReconcileSuccessReached == reconciledSuccess
ReconcileNoEffectReached == reconciledNoEffect

DuplicateDeliveryReached ==
    deliveryAttempts >= 2 /\ effectCount = 1 /\ externalEffect = committedOp

ReceiptAckLostReached ==
    receipt # NoOp /\ callerAck = NoOp /\ callerAckHistory = {receipt} /\ crashCount > 0

ContradictionHaltReached == contradictionSeen /\ integrityHalt

NeverCrashAfterPrepareReached == ~CrashAfterPrepareReached
NeverCrashAfterCommitBeforeEffectReached == ~CrashAfterCommitBeforeEffectReached
NeverUnknownOutcomeWithEffectReached == ~UnknownOutcomeWithEffectReached
NeverUnknownOutcomeNoEffectReached == ~UnknownOutcomeNoEffectReached
NeverReconcileSuccessReached == ~ReconcileSuccessReached
NeverReconcileNoEffectReached == ~ReconcileNoEffectReached
NeverDuplicateDeliveryReached == ~DuplicateDeliveryReached
NeverReceiptAckLostReached == ~ReceiptAckLostReached
NeverContradictionHaltReached == ~ContradictionHaltReached

=============================================================================
