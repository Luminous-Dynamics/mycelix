---- MODULE PulseDelivery ----
EXTENDS Naturals, FiniteSets

(*
Small bounded abstraction of one logical message and one recipient.
This is a design model, not a proof of the Rust implementation or a
production storage backend. Run with TLC using PulseDelivery.cfg.
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
  leaseEpoch

vars == <<outboxState, inboxCommitted, inFlight, receiptEvidence,
          delivered, revoked, expired, workerEpoch, leaseEpoch>>

Init ==
  / outboxState = "pending"
  / inboxCommitted = FALSE
  / inFlight = FALSE
  / receiptEvidence = FALSE
  / delivered = FALSE
  / revoked = FALSE
  / expired = FALSE
  / workerEpoch = 0
  / leaseEpoch = 0

AcquireLease ==
  / leaseEpoch # workerEpoch
  / leaseEpoch' = workerEpoch
  / UNCHANGED <<outboxState, inboxCommitted, inFlight, receiptEvidence,
                 delivered, revoked, expired, workerEpoch>>

RestartWorker ==
  / workerEpoch < MaxEpoch
  / workerEpoch' = workerEpoch + 1
  / UNCHANGED <<outboxState, inboxCommitted, inFlight, receiptEvidence,
                 delivered, revoked, expired, leaseEpoch>>

AuthorizeSend ==
  / outboxState = "pending"
  / ~revoked
  / ~expired
  / leaseEpoch = workerEpoch
  / inFlight' = TRUE
  / outboxState' = "attempted"
  / UNCHANGED <<inboxCommitted, receiptEvidence, delivered, revoked,
                 expired, workerEpoch, leaseEpoch>>

ReceiverCommit ==
  / inFlight
  / ~inboxCommitted
  / inboxCommitted' = TRUE
  / receiptEvidence' = TRUE
  / inFlight' = FALSE
  / UNCHANGED <<outboxState, delivered, revoked, expired,
                 workerEpoch, leaseEpoch>>

DuplicateReceive ==
  / inFlight
  / inboxCommitted
  / inFlight' = FALSE
  / UNCHANGED <<outboxState, inboxCommitted, receiptEvidence, delivered,
                 revoked, expired, workerEpoch, leaseEpoch>>

VerifyReceipt ==
  / receiptEvidence
  / inboxCommitted
  / delivered' = TRUE
  / outboxState' = "delivered"
  / UNCHANGED <<inboxCommitted, inFlight, receiptEvidence, revoked,
                 expired, workerEpoch, leaseEpoch>>

Revoke ==
  / ~revoked
  / revoked' = TRUE
  / UNCHANGED <<outboxState, inboxCommitted, inFlight, receiptEvidence,
                 delivered, expired, workerEpoch, leaseEpoch>>

Expire ==
  / ~expired
  / expired' = TRUE
  / UNCHANGED <<outboxState, inboxCommitted, inFlight, receiptEvidence,
                 delivered, revoked, workerEpoch, leaseEpoch>>

Crash ==
  UNCHANGED vars

Next ==
  / AcquireLease
  / RestartWorker
  / AuthorizeSend
  / ReceiverCommit
  / DuplicateReceive
  / VerifyReceipt
  / Revoke
  / Expire
  / Crash

TypeOK ==
  / outboxState \in {"pending", "attempted", "delivered"}
  / inboxCommitted \in BOOLEAN
  / inFlight \in BOOLEAN
  / receiptEvidence \in BOOLEAN
  / delivered \in BOOLEAN
  / revoked \in BOOLEAN
  / expired \in BOOLEAN
  / workerEpoch \in 0..MaxEpoch
  / leaseEpoch \in 0..MaxEpoch

DeliveredRequiresReceipt ==
  delivered => (receiptEvidence /\ inboxCommitted /\ outboxState = "delivered")

InboxAtMostOne ==
  inboxCommitted \in BOOLEAN

NoSendAfterPolicyStop ==
  (revoked \/ expired) => ~AuthorizeSend

FencingIsExplicit ==
  AuthorizeSend => (leaseEpoch = workerEpoch)

Spec == Init /\ [][Next]_vars

====
