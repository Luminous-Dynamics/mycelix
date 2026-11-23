---------------- MODULE PCOEEffectOwnerPayment ----------------
EXTENDS Naturals, FiniteSets, TLC

CONSTANTS NoDigest, GoodDigest, OtherDigest, AllowUnsafeMutations

ASSUME AllowUnsafeMutations \in BOOLEAN
ASSUME NoDigest # GoodDigest
ASSUME GoodDigest # OtherDigest
ASSUME NoDigest # OtherDigest

OperationStates == {"Absent", "Prepared", "Debited", "Settled", "Conflicted"}
FeeStates == {"NotRequired", "Pending", "Unknown", "Delivered", "Rejected"}

VARIABLES
  opState,
  requestDigest,
  senderIntentCount,
  senderDebitCount,
  recipientClaimCount,
  recipientCreditCount,
  feeLiabilityCount,
  feeOutboxCount,
  paymentProjectionCount,
  receiptProjectionCount,
  feeDeliveryCount,
  feeState,
  senderResponseLost,
  senderResponseReturned,
  recipientResponseLost,
  recipientClaimObserved,
  recipientForkSiblingCount,
  recipientCreditSpendable,
  conflictObserved,
  tombstonePresent,
  clientCachePresent,
  outboxWakeupPresent,
  outboxScanCount,
  chainHeadMoveCount,
  retryCount

vars ==
  <<opState, requestDigest, senderIntentCount, senderDebitCount,
    recipientClaimCount, recipientCreditCount, feeLiabilityCount,
    feeOutboxCount, paymentProjectionCount, receiptProjectionCount,
    feeDeliveryCount, feeState, senderResponseLost, senderResponseReturned,
    recipientResponseLost, recipientClaimObserved, recipientForkSiblingCount, recipientCreditSpendable, conflictObserved,
    tombstonePresent, clientCachePresent, outboxWakeupPresent,
    outboxScanCount, chainHeadMoveCount, retryCount>>

Init ==
  /\ opState = "Absent"
  /\ requestDigest = NoDigest
  /\ senderIntentCount = 0
  /\ senderDebitCount = 0
  /\ recipientClaimCount = 0
  /\ recipientCreditCount = 0
  /\ feeLiabilityCount = 0
  /\ feeOutboxCount = 0
  /\ paymentProjectionCount = 0
  /\ receiptProjectionCount = 0
  /\ feeDeliveryCount = 0
  /\ feeState = "NotRequired"
  /\ senderResponseLost = FALSE
  /\ senderResponseReturned = FALSE
  /\ recipientResponseLost = FALSE
  /\ recipientClaimObserved = FALSE
  /\ recipientForkSiblingCount = 0
  /\ recipientCreditSpendable = FALSE
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
  /\ UNCHANGED <<senderIntentCount, senderDebitCount, recipientClaimCount,
                 recipientCreditCount, feeLiabilityCount, feeOutboxCount,
                 paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, feeState, senderResponseLost,
                 senderResponseReturned, recipientResponseLost,
                 recipientClaimObserved, conflictObserved, tombstonePresent,
                 clientCachePresent, outboxWakeupPresent, outboxScanCount,
                 chainHeadMoveCount, retryCount, recipientForkSiblingCount, recipientCreditSpendable>>

CommitSenderIntentAndDebit ==
  /\ opState = "Prepared"
  /\ requestDigest = GoodDigest
  /\ opState' = "Debited"
  /\ senderIntentCount' = 1
  /\ senderDebitCount' = 1
  /\ feeLiabilityCount' = 1
  /\ feeOutboxCount' = 1
  /\ paymentProjectionCount' = 1
  /\ feeState' = "Pending"
  /\ tombstonePresent' = TRUE
  /\ outboxWakeupPresent' = TRUE
  /\ UNCHANGED <<requestDigest, recipientClaimCount, recipientCreditCount,
                 receiptProjectionCount, feeDeliveryCount,
                 senderResponseLost, senderResponseReturned,
                 recipientResponseLost, recipientClaimObserved,
                 conflictObserved, clientCachePresent, outboxScanCount,
                 chainHeadMoveCount, retryCount, recipientForkSiblingCount, recipientCreditSpendable>>

ExactRetry ==
  /\ opState \in {"Prepared", "Debited", "Settled", "Conflicted"}
  /\ requestDigest = GoodDigest
  /\ retryCount < 3
  /\ retryCount' = retryCount + 1
  /\ UNCHANGED <<opState, requestDigest, senderIntentCount, senderDebitCount,
                 recipientClaimCount, recipientCreditCount, feeLiabilityCount,
                 feeOutboxCount, paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, feeState, senderResponseLost,
                 senderResponseReturned, recipientResponseLost,
                 recipientClaimObserved, conflictObserved, tombstonePresent,
                 clientCachePresent, outboxWakeupPresent, outboxScanCount,
                 chainHeadMoveCount, recipientForkSiblingCount, recipientCreditSpendable>>

ObserveChainHeadMoved ==
  /\ opState \in {"Debited", "Settled"}
  /\ chainHeadMoveCount < 2
  /\ chainHeadMoveCount' = chainHeadMoveCount + 1
  /\ UNCHANGED <<opState, requestDigest, senderIntentCount, senderDebitCount,
                 recipientClaimCount, recipientCreditCount, feeLiabilityCount,
                 feeOutboxCount, paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, feeState, senderResponseLost,
                 senderResponseReturned, recipientResponseLost,
                 recipientClaimObserved, conflictObserved, tombstonePresent,
                 clientCachePresent, outboxWakeupPresent, outboxScanCount,
                 retryCount, recipientForkSiblingCount, recipientCreditSpendable>>

ChangedPayloadConflict ==
  /\ opState \in {"Prepared", "Debited", "Settled"}
  /\ requestDigest = GoodDigest
  /\ OtherDigest # requestDigest
  /\ conflictObserved' = TRUE
  /\ UNCHANGED <<opState, requestDigest, senderIntentCount, senderDebitCount,
                 recipientClaimCount, recipientCreditCount, feeLiabilityCount,
                 feeOutboxCount, paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, feeState, senderResponseLost,
                 senderResponseReturned, recipientResponseLost,
                 recipientClaimObserved, tombstonePresent, clientCachePresent,
                 outboxWakeupPresent, outboxScanCount, chainHeadMoveCount,
                 retryCount, recipientForkSiblingCount, recipientCreditSpendable>>

LoseSenderResponse ==
  /\ opState \in {"Debited", "Settled"}
  /\ senderResponseReturned = FALSE
  /\ senderResponseLost' = TRUE
  /\ UNCHANGED <<opState, requestDigest, senderIntentCount, senderDebitCount,
                 recipientClaimCount, recipientCreditCount, feeLiabilityCount,
                 feeOutboxCount, paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, feeState, senderResponseReturned,
                 recipientResponseLost, recipientClaimObserved,
                 conflictObserved, tombstonePresent, clientCachePresent,
                 outboxWakeupPresent, outboxScanCount, chainHeadMoveCount,
                 retryCount, recipientForkSiblingCount, recipientCreditSpendable>>

ReturnOriginalSenderResult ==
  /\ opState \in {"Debited", "Settled"}
  /\ senderResponseLost = TRUE
  /\ senderResponseReturned' = TRUE
  /\ UNCHANGED <<opState, requestDigest, senderIntentCount, senderDebitCount,
                 recipientClaimCount, recipientCreditCount, feeLiabilityCount,
                 feeOutboxCount, paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, feeState, senderResponseLost,
                 recipientResponseLost, recipientClaimObserved,
                 conflictObserved, tombstonePresent, clientCachePresent,
                 outboxWakeupPresent, outboxScanCount, chainHeadMoveCount,
                 retryCount, recipientForkSiblingCount, recipientCreditSpendable>>

CommitRecipientClaim ==
  /\ opState = "Debited"
  /\ recipientClaimCount = 0
  /\ recipientClaimCount' = 1
  /\ recipientCreditCount' = 1
  /\ receiptProjectionCount' = 1
  /\ recipientResponseLost' = FALSE
  /\ recipientClaimObserved' = TRUE
  /\ recipientCreditSpendable' = TRUE
  /\ opState' = "Settled"
  /\ UNCHANGED <<requestDigest, senderIntentCount, senderDebitCount,
                 feeLiabilityCount, feeOutboxCount, paymentProjectionCount,
                 feeDeliveryCount, feeState, senderResponseLost,
                 senderResponseReturned, conflictObserved, tombstonePresent,
                 clientCachePresent, outboxWakeupPresent, outboxScanCount,
                 chainHeadMoveCount, retryCount, recipientForkSiblingCount>>

CommitRecipientClaimLoseResponse ==
  /\ opState = "Debited"
  /\ recipientClaimCount = 0
  /\ recipientClaimCount' = 1
  /\ recipientCreditCount' = 1
  /\ receiptProjectionCount' = 1
  /\ recipientResponseLost' = TRUE
  /\ recipientClaimObserved' = FALSE
  /\ recipientCreditSpendable' = TRUE
  /\ UNCHANGED <<opState, requestDigest, senderIntentCount, senderDebitCount,
                 feeLiabilityCount, feeOutboxCount, paymentProjectionCount,
                 feeDeliveryCount, feeState, senderResponseLost,
                 senderResponseReturned, conflictObserved, tombstonePresent,
                 clientCachePresent, outboxWakeupPresent, outboxScanCount,
                 chainHeadMoveCount, retryCount, recipientForkSiblingCount>>

ObserveRecipientFork ==
  /\ opState \in {"Debited", "Settled"}
  /\ recipientClaimCount = 1
  /\ recipientCreditSpendable = TRUE
  /\ recipientForkSiblingCount = 0
  /\ recipientForkSiblingCount' = 1
  /\ recipientCreditSpendable' = FALSE
  /\ recipientClaimObserved' = TRUE
  /\ opState' = "Conflicted"
  /\ UNCHANGED <<requestDigest, senderIntentCount, senderDebitCount,
                 recipientClaimCount, recipientCreditCount, feeLiabilityCount,
                 feeOutboxCount, paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, feeState, senderResponseLost,
                 senderResponseReturned, recipientResponseLost, conflictObserved,
                 tombstonePresent, clientCachePresent, outboxWakeupPresent,
                 outboxScanCount, chainHeadMoveCount, retryCount>>

ReconcileRecipientClaim ==
  /\ opState = "Debited"
  /\ recipientClaimCount = 1
  /\ recipientCreditCount = 1
  /\ recipientClaimObserved = FALSE
  /\ recipientClaimObserved' = TRUE
  /\ opState' = "Settled"
  /\ UNCHANGED <<requestDigest, senderIntentCount, senderDebitCount,
                 recipientClaimCount, recipientCreditCount, feeLiabilityCount,
                 feeOutboxCount, paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, feeState, senderResponseLost,
                 senderResponseReturned, recipientResponseLost,
                 conflictObserved, tombstonePresent, clientCachePresent,
                 outboxWakeupPresent, outboxScanCount, chainHeadMoveCount,
                 retryCount, recipientForkSiblingCount, recipientCreditSpendable>>

MarkFeeOutcomeUnknown ==
  /\ feeState = "Pending"
  /\ outboxWakeupPresent = TRUE
  /\ feeState' = "Unknown"
  /\ outboxWakeupPresent' = FALSE
  /\ UNCHANGED <<opState, requestDigest, senderIntentCount, senderDebitCount,
                 recipientClaimCount, recipientCreditCount, feeLiabilityCount,
                 feeOutboxCount, paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, senderResponseLost, senderResponseReturned,
                 recipientResponseLost, recipientClaimObserved,
                 conflictObserved, tombstonePresent, clientCachePresent,
                 outboxScanCount, chainHeadMoveCount, retryCount, recipientForkSiblingCount, recipientCreditSpendable>>

ReconcileFeeAsDelivered ==
  /\ feeState = "Unknown"
  /\ feeState' = "Delivered"
  /\ feeDeliveryCount' = 1
  /\ outboxWakeupPresent' = FALSE
  /\ UNCHANGED <<opState, requestDigest, senderIntentCount, senderDebitCount,
                 recipientClaimCount, recipientCreditCount, feeLiabilityCount,
                 feeOutboxCount, paymentProjectionCount, receiptProjectionCount,
                 senderResponseLost, senderResponseReturned, conflictObserved,
                 tombstonePresent, clientCachePresent, outboxScanCount,
                 chainHeadMoveCount, recipientResponseLost,
                 recipientClaimObserved, retryCount, recipientForkSiblingCount, recipientCreditSpendable>>

DeliverFee ==
  /\ feeState = "Pending"
  /\ outboxWakeupPresent = TRUE
  /\ feeState' = "Delivered"
  /\ feeDeliveryCount' = 1
  /\ outboxWakeupPresent' = FALSE
  /\ UNCHANGED <<opState, requestDigest, senderIntentCount, senderDebitCount,
                 recipientClaimCount, recipientCreditCount, feeLiabilityCount,
                 feeOutboxCount, paymentProjectionCount, receiptProjectionCount,
                 senderResponseLost, senderResponseReturned, conflictObserved,
                 tombstonePresent, clientCachePresent, outboxScanCount,
                 chainHeadMoveCount, recipientResponseLost,
                 recipientClaimObserved, retryCount, recipientForkSiblingCount, recipientCreditSpendable>>

ExpireClientCache ==
  /\ clientCachePresent = TRUE
  /\ tombstonePresent = TRUE
  /\ clientCachePresent' = FALSE
  /\ UNCHANGED <<opState, requestDigest, senderIntentCount, senderDebitCount,
                 recipientClaimCount, recipientCreditCount, feeLiabilityCount,
                 feeOutboxCount, paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, feeState, senderResponseLost,
                 senderResponseReturned, recipientResponseLost,
                 recipientClaimObserved, conflictObserved, tombstonePresent,
                 outboxWakeupPresent, outboxScanCount, chainHeadMoveCount,
                 retryCount, recipientForkSiblingCount, recipientCreditSpendable>>

LosePostCommitWakeup ==
  /\ opState \in {"Debited", "Settled"}
  /\ feeState = "Pending"
  /\ outboxWakeupPresent = TRUE
  /\ outboxWakeupPresent' = FALSE
  /\ UNCHANGED <<opState, requestDigest, senderIntentCount, senderDebitCount,
                 recipientClaimCount, recipientCreditCount, feeLiabilityCount,
                 feeOutboxCount, paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, feeState, senderResponseLost,
                 senderResponseReturned, recipientResponseLost,
                 recipientClaimObserved, conflictObserved, tombstonePresent,
                 clientCachePresent, outboxScanCount, chainHeadMoveCount,
                 retryCount, recipientForkSiblingCount, recipientCreditSpendable>>

ScanDurableOutbox ==
  /\ feeState \in {"Pending", "Unknown"}
  /\ feeOutboxCount = 1
  /\ outboxWakeupPresent = FALSE
  /\ outboxScanCount < 2
  /\ outboxWakeupPresent' = TRUE
  /\ outboxScanCount' = outboxScanCount + 1
  /\ UNCHANGED <<opState, requestDigest, senderIntentCount, senderDebitCount,
                 recipientClaimCount, recipientCreditCount, feeLiabilityCount,
                 feeOutboxCount, paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, feeState, senderResponseLost,
                 senderResponseReturned, recipientResponseLost,
                 recipientClaimObserved, conflictObserved, tombstonePresent,
                 clientCachePresent, chainHeadMoveCount, retryCount, recipientForkSiblingCount, recipientCreditSpendable>>

\* Deliberate mutation: separate a sender debit from its immutable intent.
\* This is an unsafe split-write implementation, not a claim about the current
\* same-zome-call Holochain transaction, which is atomic.
UnsafeSenderDebitWithoutIntent ==
  /\ AllowUnsafeMutations = TRUE
  /\ opState = "Prepared"
  /\ senderDebitCount = 0
  /\ senderDebitCount' = senderDebitCount + 1
  /\ UNCHANGED <<opState, requestDigest, senderIntentCount,
                 recipientClaimCount, recipientCreditCount, feeLiabilityCount,
                 feeOutboxCount, paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, feeState, senderResponseLost,
                 senderResponseReturned, recipientResponseLost,
                 recipientClaimObserved, conflictObserved, tombstonePresent,
                 clientCachePresent, outboxWakeupPresent, outboxScanCount,
                 chainHeadMoveCount, retryCount, recipientForkSiblingCount, recipientCreditSpendable>>

UnsafeDuplicateSenderDebit ==
  /\ AllowUnsafeMutations = TRUE
  /\ opState = "Debited"
  /\ senderDebitCount = 1
  /\ senderDebitCount' = senderDebitCount + 1
  /\ UNCHANGED <<opState, requestDigest, senderIntentCount,
                 recipientClaimCount, recipientCreditCount, feeLiabilityCount,
                 feeOutboxCount, paymentProjectionCount, receiptProjectionCount,
                 feeDeliveryCount, feeState, senderResponseLost,
                 senderResponseReturned, recipientResponseLost,
                 recipientClaimObserved, conflictObserved, tombstonePresent,
                 clientCachePresent, outboxWakeupPresent, outboxScanCount,
                 chainHeadMoveCount, retryCount, recipientForkSiblingCount, recipientCreditSpendable>>

Next ==
  \/ Prepare
  \/ CommitSenderIntentAndDebit
  \/ ExactRetry
  \/ ObserveChainHeadMoved
  \/ ChangedPayloadConflict
  \/ LoseSenderResponse
  \/ ReturnOriginalSenderResult
  \/ CommitRecipientClaim
  \/ CommitRecipientClaimLoseResponse
  \/ ReconcileRecipientClaim
  \/ ObserveRecipientFork
  \/ MarkFeeOutcomeUnknown
  \/ ReconcileFeeAsDelivered
  \/ DeliverFee
  \/ ExpireClientCache
  \/ LosePostCommitWakeup
  \/ ScanDurableOutbox
  \/ UnsafeSenderDebitWithoutIntent
  \/ UnsafeDuplicateSenderDebit

Spec == Init /\ [][Next]_vars

TypeOK ==
  /\ opState \in OperationStates
  /\ requestDigest \in {NoDigest, GoodDigest, OtherDigest}
  /\ senderIntentCount \in 0..1
  /\ senderDebitCount \in 0..2
  /\ recipientClaimCount \in 0..1
  /\ recipientCreditCount \in 0..1
  /\ feeLiabilityCount \in 0..1
  /\ feeOutboxCount \in 0..1
  /\ paymentProjectionCount \in 0..1
  /\ receiptProjectionCount \in 0..1
  /\ feeDeliveryCount \in 0..1
  /\ feeState \in FeeStates
  /\ senderResponseLost \in BOOLEAN
  /\ senderResponseReturned \in BOOLEAN
  /\ recipientResponseLost \in BOOLEAN
  /\ recipientClaimObserved \in BOOLEAN
  /\ recipientForkSiblingCount \in 0..1
  /\ recipientCreditSpendable \in BOOLEAN
  /\ conflictObserved \in BOOLEAN
  /\ tombstonePresent \in BOOLEAN
  /\ clientCachePresent \in BOOLEAN
  /\ outboxWakeupPresent \in BOOLEAN
  /\ outboxScanCount \in 0..2
  /\ chainHeadMoveCount \in 0..2
  /\ retryCount \in 0..3

AtMostOneSenderEffect ==
  /\ senderIntentCount <= 1
  /\ senderDebitCount <= 1

AtMostOneRecipientEffect ==
  /\ recipientClaimCount <= 1
  /\ recipientCreditCount <= 1

SenderIntentDebitAtomicity ==
  /\ senderIntentCount = senderDebitCount
  /\ feeLiabilityCount = senderDebitCount
  /\ feeOutboxCount = senderDebitCount
  /\ paymentProjectionCount = senderDebitCount

NoRecipientCreditWithoutClaim ==
  recipientClaimCount = recipientCreditCount

RecipientClaimMatchesProjection ==
  receiptProjectionCount = recipientCreditCount

SettledRequiresBothOwnerLegs ==
  (opState = "Settled") =>
    (senderDebitCount = 1 /\ recipientCreditCount = 1 /\
     recipientClaimCount = 1 /\ recipientClaimObserved = TRUE /\ recipientCreditSpendable = TRUE /\ recipientForkSiblingCount = 0)

NoObservedClaimWithoutBothLegs ==
  recipientClaimObserved =>
    (senderIntentCount = 1 /\ senderDebitCount = 1 /\
     recipientClaimCount = 1 /\ recipientCreditCount = 1)

RecipientForkFreezesSpendability ==
  (recipientForkSiblingCount = 1) =>
    (opState = "Conflicted" /\ recipientCreditSpendable = FALSE)

NoSpendableCreditOnFork ==
  recipientCreditSpendable => recipientForkSiblingCount = 0

NoResponseBeforeSenderCommit ==
  senderResponseReturned => (senderIntentCount = 1 /\ senderDebitCount = 1)

FeeDeliveryBounded ==
  /\ feeDeliveryCount <= feeOutboxCount
  /\ (feeDeliveryCount = 1) => (feeState = "Delivered")

FeeDeliveryStateMatchesCount ==
  (feeState = "Delivered") <=> (feeDeliveryCount = 1)

DurableOperationIdentity ==
  (senderIntentCount = 1) => tombstonePresent

ExpiredCacheDoesNotEraseOperation ==
  (clientCachePresent = FALSE) => tombstonePresent

OutboxRemainsDurableAcrossLostWakeup ==
  (feeState \in {"Pending", "Unknown"}) =>
    (feeOutboxCount = 1 /\ senderDebitCount = 1)

ChainHeadRaceDoesNotDuplicateEffect ==
  (chainHeadMoveCount > 0) =>
    (senderDebitCount = 1 /\ senderIntentCount = 1)

NoPrematureSettlement ==
  (opState = "Settled") <=>
    (senderDebitCount = 1 /\ recipientCreditCount = 1 /\
     recipientClaimObserved = TRUE /\ recipientForkSiblingCount = 0 /\
     recipientCreditSpendable = TRUE)

Safety ==
  /\ TypeOK
  /\ AtMostOneSenderEffect
  /\ AtMostOneRecipientEffect
  /\ SenderIntentDebitAtomicity
  /\ NoRecipientCreditWithoutClaim
  /\ RecipientClaimMatchesProjection
  /\ SettledRequiresBothOwnerLegs
  /\ NoObservedClaimWithoutBothLegs
  /\ RecipientForkFreezesSpendability
  /\ NoSpendableCreditOnFork
  /\ NoResponseBeforeSenderCommit
  /\ FeeDeliveryBounded
  /\ FeeDeliveryStateMatchesCount
  /\ DurableOperationIdentity
  /\ ExpiredCacheDoesNotEraseOperation
  /\ OutboxRemainsDurableAcrossLostWakeup
  /\ ChainHeadRaceDoesNotDuplicateEffect
  /\ NoPrematureSettlement

====
