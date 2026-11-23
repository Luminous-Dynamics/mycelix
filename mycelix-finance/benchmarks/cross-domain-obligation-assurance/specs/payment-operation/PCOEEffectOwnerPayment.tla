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
  outboxWakeupPresent,
  outboxScanCount,
  chainHeadMoveCount,
  retryCount

vars ==
  <<opState, requestDigest, claimCount, batchCount, debitCount,
    creditCount, feeLiabilityCount, feeOutboxCount,
    paymentProjectionCount, receiptProjectionCount,
    feeDeliveryCount, feeState, responseLost, responseReturned,
    conflictObserved, tombstonePresent, clientCachePresent, outboxWakeupPresent, outboxScanCount, chainHeadMoveCount, retryCount>>

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
  /\ outboxWakeupPresent = FALSE
  /\ outboxScanCount = 0
  /\ chainHeadMoveCount = 0
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
                 conflictObserved, clientCachePresent, outboxWakeupPresent, outboxScanCount, retryCount, chainHeadMoveCount>>

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
  /\ outboxWakeupPresent' = TRUE
  /\ UNCHANGED <<requestDigest, claimCount, feeDeliveryCount,
                 responseLost, responseReturned, conflictObserved,
                 tombstonePresent, clientCachePresent, outboxScanCount, retryCount, chainHeadMoveCount>>

ExactRetry ==
  /\ opState \in {"Prepared", "Committed"}
  /\ requestDigest = GoodDigest
  /\ retryCount < 2
  /\ retryCount' = retryCount + 1
  /\ UNCHANGED <<opState, requestDigest, claimCount, batchCount, debitCount,
                 creditCount, feeLiabilityCount, feeOutboxCount,
                 paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, feeState, responseLost, responseReturned,
                 conflictObserved, tombstonePresent, clientCachePresent, outboxWakeupPresent, outboxScanCount, chainHeadMoveCount>>

ObserveChainHeadMoved ==
  /\ opState = "Committed"
  /\ chainHeadMoveCount < 2
  /\ chainHeadMoveCount' = chainHeadMoveCount + 1
  /\ UNCHANGED <<opState, requestDigest, claimCount, batchCount, debitCount,
                 creditCount, feeLiabilityCount, feeOutboxCount,
                 paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, feeState, responseLost, responseReturned,
                 conflictObserved, tombstonePresent, clientCachePresent,
                 outboxWakeupPresent, outboxScanCount, retryCount>>

ChangedPayloadConflict ==
  /\ opState \in {"Prepared", "Committed"}
  /\ requestDigest = GoodDigest
  /\ OtherDigest # requestDigest
  /\ conflictObserved' = TRUE
  /\ UNCHANGED <<opState, requestDigest, claimCount, batchCount, debitCount,
                 creditCount, feeLiabilityCount, feeOutboxCount,
                 paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, feeState, responseLost, responseReturned,
                 tombstonePresent, clientCachePresent, outboxWakeupPresent, outboxScanCount, retryCount, chainHeadMoveCount>>

LoseResponse ==
  /\ opState = "Committed"
  /\ responseReturned = FALSE
  /\ responseLost' = TRUE
  /\ UNCHANGED <<opState, requestDigest, claimCount, batchCount, debitCount,
                 creditCount, feeLiabilityCount, feeOutboxCount,
                 paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, feeState, responseReturned,
                 conflictObserved, tombstonePresent, clientCachePresent, outboxWakeupPresent, outboxScanCount, retryCount, chainHeadMoveCount>>

ReturnOriginalResult ==
  /\ opState = "Committed"
  /\ responseLost = TRUE
  /\ responseReturned' = TRUE
  /\ UNCHANGED <<opState, requestDigest, claimCount, batchCount, debitCount,
                 creditCount, feeLiabilityCount, feeOutboxCount,
                 paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, feeState, responseLost,
                 conflictObserved, tombstonePresent, clientCachePresent, outboxWakeupPresent, outboxScanCount, retryCount, chainHeadMoveCount>>

MarkFeeOutcomeUnknown ==
  /\ feeState = "Pending"
  /\ outboxWakeupPresent = TRUE
  /\ feeState' = "Unknown"
  /\ outboxWakeupPresent' = FALSE
  /\ UNCHANGED <<opState, requestDigest, claimCount, batchCount, debitCount,
                 creditCount, feeLiabilityCount, feeOutboxCount,
                 paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, responseLost, responseReturned,
                 conflictObserved, tombstonePresent, clientCachePresent, outboxScanCount, retryCount, chainHeadMoveCount>>

ReconcileFeeAsDelivered ==
  /\ feeState = "Unknown"
  /\ feeState' = "Delivered"
  /\ feeDeliveryCount' = 1
  /\ outboxWakeupPresent' = FALSE
  /\ UNCHANGED <<opState, requestDigest, claimCount, batchCount, debitCount,
                 creditCount, feeLiabilityCount, feeOutboxCount,
                 paymentProjectionCount, receiptProjectionCount,
                 responseLost, responseReturned, conflictObserved,
                 tombstonePresent, clientCachePresent, outboxScanCount, retryCount, chainHeadMoveCount>>

DeliverFee ==
  /\ feeState = "Pending"
  /\ outboxWakeupPresent = TRUE
  /\ feeState' = "Delivered"
  /\ feeDeliveryCount' = 1
  /\ outboxWakeupPresent' = FALSE
  /\ UNCHANGED <<opState, requestDigest, claimCount, batchCount, debitCount,
                 creditCount, feeLiabilityCount, feeOutboxCount,
                 paymentProjectionCount, receiptProjectionCount,
                 responseLost, responseReturned, conflictObserved,
                 tombstonePresent, clientCachePresent, outboxScanCount, retryCount, chainHeadMoveCount>>

ExpireClientCache ==
  /\ clientCachePresent = TRUE
  /\ tombstonePresent = TRUE
  /\ clientCachePresent' = FALSE
  /\ UNCHANGED <<opState, requestDigest, claimCount, batchCount, debitCount,
                 creditCount, feeLiabilityCount, feeOutboxCount,
                 paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, feeState, responseLost, responseReturned,
                 conflictObserved, tombstonePresent, outboxWakeupPresent, outboxScanCount, retryCount, chainHeadMoveCount>>

LosePostCommitWakeup ==
  /\ opState = "Committed"
  /\ feeState = "Pending"
  /\ outboxWakeupPresent = TRUE
  /\ outboxWakeupPresent' = FALSE
  /\ UNCHANGED <<opState, requestDigest, claimCount, batchCount, debitCount,
                 creditCount, feeLiabilityCount, feeOutboxCount,
                 paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, feeState, responseLost, responseReturned,
                 conflictObserved, tombstonePresent, clientCachePresent,
                 outboxScanCount, retryCount, chainHeadMoveCount>>

ScanDurableOutbox ==
  /\ feeState \in {"Pending", "Unknown"}
  /\ feeOutboxCount = 1
  /\ outboxWakeupPresent = FALSE
  /\ outboxScanCount < 2
  /\ outboxWakeupPresent' = TRUE
  /\ outboxScanCount' = outboxScanCount + 1
  /\ UNCHANGED <<opState, requestDigest, claimCount, batchCount, debitCount,
                 creditCount, feeLiabilityCount, feeOutboxCount,
                 paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, feeState, responseLost, responseReturned,
                 conflictObserved, tombstonePresent, clientCachePresent, retryCount, chainHeadMoveCount>>

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
                 tombstonePresent, clientCachePresent, outboxWakeupPresent, outboxScanCount, retryCount, chainHeadMoveCount>>

UnsafeDuplicateDebit ==
  /\ AllowUnsafeMutations = TRUE
  /\ opState = "Committed"
  /\ debitCount = 1
  /\ debitCount' = debitCount + 1
  /\ UNCHANGED <<opState, requestDigest, claimCount, batchCount, creditCount,
                 feeLiabilityCount, feeOutboxCount, paymentProjectionCount,
                 receiptProjectionCount, feeDeliveryCount, feeState,
                 responseLost, responseReturned, conflictObserved,
                 tombstonePresent, clientCachePresent, outboxWakeupPresent, outboxScanCount, retryCount, chainHeadMoveCount>>

Next ==
  \/ Prepare
  \/ CommitAtomicJournalBatch
  \/ ExactRetry
  \/ ObserveChainHeadMoved
  \/ ChangedPayloadConflict
  \/ LoseResponse
  \/ ReturnOriginalResult
  \/ MarkFeeOutcomeUnknown
  \/ ReconcileFeeAsDelivered
  \/ DeliverFee
  \/ ExpireClientCache
  \/ LosePostCommitWakeup
  \/ ScanDurableOutbox
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
  /\ outboxWakeupPresent \in BOOLEAN
  /\ outboxScanCount \in 0..2
  /\ chainHeadMoveCount \in 0..2
  /\ retryCount \in 0..2

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

OutboxRemainsDurableAcrossLostWakeup ==
  (feeState \in {"Pending", "Unknown"}) =>
    (feeOutboxCount = 1 /\ batchCount = 1)

ChainHeadRaceDoesNotDuplicateEffect ==
  (chainHeadMoveCount > 0) =>
    (batchCount = 1 /\ debitCount = 1 /\ creditCount = 1)

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
  /\ OutboxRemainsDurableAcrossLostWakeup
  /\ ChainHeadRaceDoesNotDuplicateEffect

====
