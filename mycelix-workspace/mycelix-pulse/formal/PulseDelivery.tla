---- MODULE PulseDelivery ----
EXTENDS Naturals, FiniteSets

(*
Small bounded abstraction of one logical message and one recipient.
This is a design model, not a proof of the Rust implementation or a
production storage backend. Run with TLC using PulseDelivery.cfg.

Crash/restart is modeled explicitly. An attempt interrupted after send
authorization is recovered as "unknown", never inferred to have failed
or succeeded. A later verified receipt may resolve that unknown outcome.
*)
CONSTANT MaxEpoch

VARIABLES
  outboxState,
  inboxCommitted,
  inFlight,
  receiptEvidence,
  delivered,
  revoked,
  expired,
  workerEpoch,
  leaseEpoch,
  workerRunning,
  recoveryRequired

vars == <<outboxState, inboxCommitted, inFlight, receiptEvidence,
          delivered, revoked, expired, workerEpoch, leaseEpoch,
          workerRunning, recoveryRequired>>

Init ==
  /\ outboxState = "pending"
  /\ inboxCommitted = FALSE
  /\ inFlight = FALSE
  /\ receiptEvidence = FALSE
  /\ delivered = FALSE
  /\ revoked = FALSE
  /\ expired = FALSE
  /\ workerEpoch = 0
  /\ leaseEpoch = 0
  /\ workerRunning = TRUE
  /\ recoveryRequired = FALSE

AcquireLease ==
  /\ workerRunning
  /\ leaseEpoch # workerEpoch
  /\ leaseEpoch' = workerEpoch
  /\ UNCHANGED <<outboxState, inboxCommitted, inFlight, receiptEvidence,
                 delivered, revoked, expired, workerEpoch, workerRunning,
                 recoveryRequired>>

RestartWorker ==
  /\ ~workerRunning
  /\ workerEpoch < MaxEpoch
  /\ workerEpoch' = workerEpoch + 1
  /\ workerRunning' = TRUE
  /\ UNCHANGED <<outboxState, inboxCommitted, inFlight, receiptEvidence,
                 delivered, revoked, expired, leaseEpoch, recoveryRequired>>

AuthorizeSend ==
  /\ workerRunning
  /\ outboxState = "pending"
  /\ ~revoked
  /\ ~expired
  /\ leaseEpoch = workerEpoch
  /\ inFlight' = TRUE
  /\ outboxState' = "attempted"
  /\ UNCHANGED <<inboxCommitted, receiptEvidence, delivered, revoked,
                 expired, workerEpoch, leaseEpoch, workerRunning,
                 recoveryRequired>>

ReceiverCommit ==
  /\ inFlight
  /\ ~inboxCommitted
  /\ inboxCommitted' = TRUE
  /\ receiptEvidence' = TRUE
  /\ inFlight' = FALSE
  /\ UNCHANGED <<outboxState, delivered, revoked, expired,
                 workerEpoch, leaseEpoch, workerRunning, recoveryRequired>>

DuplicateReceive ==
  /\ inFlight
  /\ inboxCommitted
  /\ inFlight' = FALSE
  /\ UNCHANGED <<outboxState, inboxCommitted, receiptEvidence, delivered,
                 revoked, expired, workerEpoch, leaseEpoch, workerRunning,
                 recoveryRequired>>

VerifyReceipt ==
  /\ workerRunning
  /\ leaseEpoch = workerEpoch
  /\ outboxState \in {"attempted", "unknown"}
  /\ receiptEvidence
  /\ inboxCommitted
  /\ delivered' = TRUE
  /\ outboxState' = "delivered"
  /\ recoveryRequired' = FALSE
  /\ UNCHANGED <<inboxCommitted, inFlight, receiptEvidence, revoked,
                 expired, workerEpoch, leaseEpoch, workerRunning>>

RecoverInterruptedAttempt ==
  /\ workerRunning
  /\ leaseEpoch = workerEpoch
  /\ recoveryRequired
  /\ outboxState = "attempted"
  /\ outboxState' = "unknown"
  /\ recoveryRequired' = FALSE
  /\ UNCHANGED <<inboxCommitted, inFlight, receiptEvidence, delivered,
                 revoked, expired, workerEpoch, leaseEpoch, workerRunning>>

Revoke ==
  /\ ~revoked
  /\ revoked' = TRUE
  /\ UNCHANGED <<outboxState, inboxCommitted, inFlight, receiptEvidence,
                 delivered, expired, workerEpoch, leaseEpoch, workerRunning,
                 recoveryRequired>>

Expire ==
  /\ ~expired
  /\ expired' = TRUE
  /\ UNCHANGED <<outboxState, inboxCommitted, inFlight, receiptEvidence,
                 delivered, revoked, workerEpoch, leaseEpoch, workerRunning,
                 recoveryRequired>>

Crash ==
  /\ workerRunning
  /\ workerRunning' = FALSE
  /\ recoveryRequired' = (outboxState = "attempted")
  /\ UNCHANGED <<outboxState, inboxCommitted, inFlight, receiptEvidence,
                 delivered, revoked, expired, workerEpoch, leaseEpoch>>

Next ==
  \/ AcquireLease
  \/ RestartWorker
  \/ AuthorizeSend
  \/ ReceiverCommit
  \/ DuplicateReceive
  \/ VerifyReceipt
  \/ RecoverInterruptedAttempt
  \/ Revoke
  \/ Expire
  \/ Crash

TypeOK ==
  /\ outboxState \in {"pending", "attempted", "unknown", "delivered"}
  /\ inboxCommitted \in BOOLEAN
  /\ inFlight \in BOOLEAN
  /\ receiptEvidence \in BOOLEAN
  /\ delivered \in BOOLEAN
  /\ revoked \in BOOLEAN
  /\ expired \in BOOLEAN
  /\ workerEpoch \in 0..MaxEpoch
  /\ leaseEpoch \in 0..MaxEpoch
  /\ workerRunning \in BOOLEAN
  /\ recoveryRequired \in BOOLEAN

DeliveredRequiresReceipt ==
  delivered => (receiptEvidence /\ inboxCommitted /\ outboxState = "delivered")

ReceiptRequiresInbox ==
  receiptEvidence => inboxCommitted

UnknownIsNotDelivered ==
  outboxState = "unknown" => ~delivered

Spec == Init /\ [][Next]_vars

====
