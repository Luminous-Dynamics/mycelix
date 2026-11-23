---------------- MODULE PCOEEffectOwnerPayment ----------------
EXTENDS Naturals, FiniteSets, TLC

CONSTANTS NoDigest, GoodDigest, OtherDigest, AllowUnsafeMutations

ASSUME AllowUnsafeMutations \in BOOLEAN
ASSUME NoDigest # GoodDigest
ASSUME GoodDigest # OtherDigest
ASSUME NoDigest # OtherDigest

OperationStates == {"Absent", "Prepared", "Committed"}
FeeStates == {"NotRequired", "Pending", "Unknown", "Delivered", "Rejected"}

VARIABLES
  opState,
  requestDigest,
  claimCount,
  batchCount,
  debitCount,
  creditCount,
  feeLiabilityCount,
  feeOutboxCount,
  paymentProjectionCount,
  receiptProjectionCount,
  feeDeliveryCount,
  feeState,
  responseLost,
  responseReturned,
  conflictObserved,
  tombstonePresent,
  clientCachePresent,
  retryCount

vars ==
  <<opState, requestDigest, claimCount, batchCount, debitCount,
    creditCount, feeLiabilityCount, feeOutboxCount,
    paymentProjectionCount, receiptProjectionCount,
    feeDeliveryCount, feeState, responseLost, responseReturned,
    conflictObserved, tombstonePresent, clientCachePresent, retryCount>>

Init ==
  /\ opState = "Absent"
  /\ requestDigest = NoDigest
  /\ claimCount = 0
  /\ batchCount = 0
  /\ debitCount = 0
  /\ creditCount = 0
  /\ feeLiabilityCount = 0
  /\ feeOutboxCount = 0
  /\ paymentProjectionCount = 0
  /\ receiptProjectionCount = 0
  /\ feeDeliveryCount = 0
  /\ feeState = "NotRequired"
  /\ responseLost = FALSE
  /\ responseReturned = FALSE
  /\ conflictObserved = FALSE
  /\ tombstonePresent = FALSE
  /\ clientCachePresent = TRUE
  /\ retryCount = 0

Prepare ==
  /\ opState = "Absent"
  /\ opState' = "Prepared"
  /\ requestDigest' = GoodDigest
  /\ claimCount' = 1
  /\ tombstonePresent' = TRUE
  /\ UNCHANGED <<batchCount, debitCount, creditCount, feeLiabilityCount,
                 feeOutboxCount, paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, feeState, responseLost, responseReturned,
                 conflictObserved, clientCachePresent, retryCount>>

CommitAtomicJournalBatch ==
  /\ opState = "Prepared"
  /\ requestDigest = GoodDigest
  /\ opState' = "Committed"
  /\ batchCount' = 1
  /\ debitCount' = 1
  /\ creditCount' = 1
  /\ feeLiabilityCount' = 1
  /\ feeOutboxCount' = 1
  /\ paymentProjectionCount' = 1
  /\ receiptProjectionCount' = 1
  /\ feeState' = "Pending"
  /\ UNCHANGED <<requestDigest, claimCount, feeDeliveryCount,
                 responseLost, responseReturned, conflictObserved,
                 tombstonePresent, clientCachePresent, retryCount>>

ExactRetry ==
  /\ opState \in {"Prepared", "Committed"}
  /\ requestDigest = GoodDigest
  /\ retryCount < 2
  /\ retryCount' = retryCount + 1
  /\ UNCHANGED <<opState, requestDigest, claimCount, batchCount, debitCount,
                 creditCount, feeLiabilityCount, feeOutboxCount,
                 paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, feeState, responseLost, responseReturned,
                 conflictObserved, tombstonePresent, clientCachePresent>>

ChangedPayloadConflict ==
  /\ opState \in {"Prepared", "Committed"}
  /\ requestDigest = GoodDigest
  /\ OtherDigest # requestDigest
  /\ conflictObserved' = TRUE
  /\ UNCHANGED <<opState, requestDigest, claimCount, batchCount, debitCount,
                 creditCount, feeLiabilityCount, feeOutboxCount,
                 paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, feeState, responseLost, responseReturned,
                 tombstonePresent, clientCachePresent, retryCount>>

LoseResponse ==
  /\ opState = "Committed"
  /\ responseReturned = FALSE
  /\ responseLost' = TRUE
  /\ UNCHANGED <<opState, requestDigest, claimCount, batchCount, debitCount,
                 creditCount, feeLiabilityCount, feeOutboxCount,
                 paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, feeState, responseReturned,
                 conflictObserved, tombstonePresent, clientCachePresent, retryCount>>

ReturnOriginalResult ==
  /\ opState = "Committed"
  /\ responseLost = TRUE
  /\ responseReturned' = TRUE
  /\ UNCHANGED <<opState, requestDigest, claimCount, batchCount, debitCount,
                 creditCount, feeLiabilityCount, feeOutboxCount,
                 paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, feeState, responseLost,
                 conflictObserved, tombstonePresent, clientCachePresent, retryCount>>

MarkFeeOutcomeUnknown ==
  /\ feeState = "Pending"
  /\ feeState' = "Unknown"
  /\ UNCHANGED <<opState, requestDigest, claimCount, batchCount, debitCount,
                 creditCount, feeLiabilityCount, feeOutboxCount,
                 paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, responseLost, responseReturned,
                 conflictObserved, tombstonePresent, clientCachePresent, retryCount>>

ReconcileFeeAsDelivered ==
  /\ feeState = "Unknown"
  /\ feeState' = "Delivered"
  /\ feeDeliveryCount' = 1
  /\ UNCHANGED <<opState, requestDigest, claimCount, batchCount, debitCount,
                 creditCount, feeLiabilityCount, feeOutboxCount,
                 paymentProjectionCount, receiptProjectionCount,
                 responseLost, responseReturned, conflictObserved,
                 tombstonePresent, clientCachePresent, retryCount>>

DeliverFee ==
  /\ feeState = "Pending"
  /\ feeState' = "Delivered"
  /\ feeDeliveryCount' = 1
  /\ UNCHANGED <<opState, requestDigest, claimCount, batchCount, debitCount,
                 creditCount, feeLiabilityCount, feeOutboxCount,
                 paymentProjectionCount, receiptProjectionCount,
                 responseLost, responseReturned, conflictObserved,
                 tombstonePresent, clientCachePresent, retryCount>>

ExpireClientCache ==
  /\ clientCachePresent = TRUE
  /\ tombstonePresent = TRUE
  /\ clientCachePresent' = FALSE
  /\ UNCHANGED <<opState, requestDigest, claimCount, batchCount, debitCount,
                 creditCount, feeLiabilityCount, feeOutboxCount,
                 paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, feeState, responseLost, responseReturned,
                 conflictObserved, tombstonePresent, retryCount>>

\* Deliberate mutation only: split a value leg into a separate effect / commit
\* outside the single Finance source-chain invocation. This is not a claim that
\* the current send_payment same-call writes can commit independently.
UnsafeDebitOutsideBatch ==
  /\ AllowUnsafeMutations = TRUE
  /\ opState = "Prepared"
  /\ debitCount = 0
  /\ debitCount' = debitCount + 1
  /\ UNCHANGED <<opState, requestDigest, claimCount, batchCount, creditCount,
                 feeLiabilityCount, feeOutboxCount, paymentProjectionCount,
                 receiptProjectionCount, feeDeliveryCount, feeState,
                 responseLost, responseReturned, conflictObserved,
                 tombstonePresent, clientCachePresent, retryCount>>

UnsafeDuplicateDebit ==
  /\ AllowUnsafeMutations = TRUE
  /\ opState = "Committed"
  /\ debitCount = 1
  /\ debitCount' = debitCount + 1
  /\ UNCHANGED <<opState, requestDigest, claimCount, batchCount, creditCount,
                 feeLiabilityCount, feeOutboxCount, paymentProjectionCount,
                 receiptProjectionCount, feeDeliveryCount, feeState,
                 responseLost, responseReturned, conflictObserved,
                 tombstonePresent, clientCachePresent, retryCount>>

Next ==
  \/ Prepare
  \/ CommitAtomicJournalBatch
  \/ ExactRetry
  \/ ChangedPayloadConflict
  \/ LoseResponse
  \/ ReturnOriginalResult
  \/ MarkFeeOutcomeUnknown
  \/ ReconcileFeeAsDelivered
  \/ DeliverFee
  \/ ExpireClientCache
  \/ UnsafeDebitOutsideBatch
  \/ UnsafeDuplicateDebit

Spec == Init /\ [][Next]_vars

TypeOK ==
  /\ opState \in OperationStates
  /\ requestDigest \in {NoDigest, GoodDigest, OtherDigest}
  /\ claimCount \in Nat
  /\ batchCount \in Nat
  /\ debitCount \in Nat
  /\ creditCount \in Nat
  /\ feeLiabilityCount \in Nat
  /\ feeOutboxCount \in Nat
  /\ paymentProjectionCount \in Nat
  /\ receiptProjectionCount \in Nat
  /\ feeDeliveryCount \in Nat
  /\ feeState \in FeeStates
  /\ responseLost \in BOOLEAN
  /\ responseReturned \in BOOLEAN
  /\ conflictObserved \in BOOLEAN
  /\ tombstonePresent \in BOOLEAN
  /\ clientCachePresent \in BOOLEAN
  /\ retryCount \in Nat

AtMostOneClaimAndBatch ==
  /\ claimCount <= 1
  /\ batchCount <= 1

NoPartialLocalValueEffect ==
  /\ debitCount = batchCount
  /\ creditCount = batchCount
  /\ feeLiabilityCount = batchCount

ProjectionMatchesJournal ==
  /\ paymentProjectionCount = batchCount
  /\ receiptProjectionCount = batchCount
  /\ feeOutboxCount = batchCount

CommittedMeansCompleteBatch ==
  (opState = "Committed") <=> (batchCount = 1)

NoResponseBeforeCommit ==
  responseReturned => (opState = "Committed")

FeeDeliveryBounded ==
  /\ feeDeliveryCount <= feeOutboxCount
  /\ (feeDeliveryCount = 1) => (feeState = "Delivered")

DurableOperationIdentity ==
  (opState # "Absent") => tombstonePresent

ExpiredCacheDoesNotEraseOperation ==
  (clientCachePresent = FALSE) => tombstonePresent

FeeDeliveryStateMatchesCount ==
  (feeState = "Delivered") <=> (feeDeliveryCount = 1)

Safety ==
  /\ TypeOK
  /\ AtMostOneClaimAndBatch
  /\ NoPartialLocalValueEffect
  /\ ProjectionMatchesJournal
  /\ CommittedMeansCompleteBatch
  /\ NoResponseBeforeCommit
  /\ FeeDeliveryBounded
  /\ DurableOperationIdentity
  /\ ExpiredCacheDoesNotEraseOperation
  /\ FeeDeliveryStateMatchesCount

====
