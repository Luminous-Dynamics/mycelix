---------------- MODULE SAPExecutionProfiles ----------------
EXTENDS Naturals, FiniteSets, TLC

CONSTANTS
  ProfileMode,
  FeeRequired,
  PayerOwner,
  BeneficiaryOwner,
  AllowUnsafeSplitLocal,
  AllowUnsafeEarlyComplete

ProfileModes == {"LocalAtomicPrincipal", "LocalAtomicTreasury", "OwnerBilateral"}
IsLocalProfile == ProfileMode \in {"LocalAtomicPrincipal", "LocalAtomicTreasury"}

ASSUME ProfileMode \in ProfileModes
ASSUME FeeRequired \in BOOLEAN
ASSUME PayerOwner \in {"alice", "same"}
ASSUME BeneficiaryOwner \in {"bob", "same"}
ASSUME AllowUnsafeSplitLocal \in BOOLEAN
ASSUME AllowUnsafeEarlyComplete \in BOOLEAN
ASSUME (ProfileMode = "LocalAtomicPrincipal" => FeeRequired = FALSE)
ASSUME (ProfileMode = "LocalAtomicTreasury" => FeeRequired = TRUE)
ASSUME (ProfileMode = "OwnerBilateral" => FeeRequired = FALSE)

VARIABLES
  localAtomicCommitted,
  localDebit,
  localCredit,
  paymentRecord,
  receiptRecord,
  treasuryOutboxPrepared,
  treasuryFeeCommitted,
  payerDebitCommitted,
  beneficiaryClaimCommitted,
  payerViewComplete,
  beneficiaryViewComplete,
  payerViewObserved,
  beneficiaryViewObserved,
  payerConflict,
  beneficiaryConflict,
  projection,
  operationComplete

vars ==
  <<localAtomicCommitted, localDebit, localCredit, paymentRecord,
    receiptRecord, treasuryOutboxPrepared, treasuryFeeCommitted,
    payerDebitCommitted, beneficiaryClaimCommitted, payerViewComplete,
    beneficiaryViewComplete, payerViewObserved, beneficiaryViewObserved,
    payerConflict, beneficiaryConflict, projection, operationComplete>>

Init ==
  /\ localAtomicCommitted = FALSE
  /\ localDebit = FALSE
  /\ localCredit = FALSE
  /\ paymentRecord = FALSE
  /\ receiptRecord = FALSE
  /\ treasuryOutboxPrepared = FALSE
  /\ treasuryFeeCommitted = FALSE
  /\ payerDebitCommitted = FALSE
  /\ beneficiaryClaimCommitted = FALSE
  /\ payerViewComplete = FALSE
  /\ beneficiaryViewComplete = FALSE
  /\ payerViewObserved = FALSE
  /\ beneficiaryViewObserved = FALSE
  /\ payerConflict = FALSE
  /\ beneficiaryConflict = FALSE
  /\ projection = "NotProjected"
  /\ operationComplete = FALSE

\* One source-chain transaction. These local facts become visible atomically:
\* payer debit, beneficiary credit, Payment, Receipt and (when required) outbox.
LocalAtomicCommit ==
  /\ IsLocalProfile
  /\ localAtomicCommitted = FALSE
  /\ localAtomicCommitted' = TRUE
  /\ localDebit' = TRUE
  /\ localCredit' = TRUE
  /\ paymentRecord' = TRUE
  /\ receiptRecord' = TRUE
  /\ treasuryOutboxPrepared' = FeeRequired
  /\ UNCHANGED <<treasuryFeeCommitted, payerDebitCommitted,
                 beneficiaryClaimCommitted, payerViewComplete,
                 beneficiaryViewComplete, payerViewObserved,
                 beneficiaryViewObserved, payerConflict,
                 beneficiaryConflict, projection, operationComplete>>

\* Deliberate unsafe mutant: publishes only the payer-side mutation and leaves
\* the rest of the local atomic write set absent.
UnsafeSplitLocalDebit ==
  /\ AllowUnsafeSplitLocal = TRUE
  /\ IsLocalProfile
  /\ localAtomicCommitted = FALSE
  /\ localDebit = FALSE
  /\ localDebit' = TRUE
  /\ UNCHANGED <<localAtomicCommitted, localCredit, paymentRecord,
                 receiptRecord, treasuryOutboxPrepared, treasuryFeeCommitted,
                 payerDebitCommitted, beneficiaryClaimCommitted,
                 payerViewComplete, beneficiaryViewComplete,
                 payerViewObserved, beneficiaryViewObserved,
                 payerConflict, beneficiaryConflict, projection,
                 operationComplete>>

TreasuryFeeCommit ==
  /\ ProfileMode = "LocalAtomicTreasury"
  /\ treasuryOutboxPrepared = TRUE
  /\ treasuryFeeCommitted = FALSE
  /\ treasuryFeeCommitted' = TRUE
  /\ UNCHANGED <<localAtomicCommitted, localDebit, localCredit,
                 paymentRecord, receiptRecord, treasuryOutboxPrepared,
                 payerDebitCommitted, beneficiaryClaimCommitted,
                 payerViewComplete, beneficiaryViewComplete,
                 payerViewObserved, beneficiaryViewObserved,
                 payerConflict, beneficiaryConflict, projection,
                 operationComplete>>

BilateralPayerCommit ==
  /\ ProfileMode = "OwnerBilateral"
  /\ payerDebitCommitted = FALSE
  /\ payerDebitCommitted' = TRUE
  /\ payerViewComplete' = FALSE
  /\ payerViewObserved' = FALSE
  /\ projection' = "NotProjected"
  /\ operationComplete' = FALSE
  /\ UNCHANGED <<localAtomicCommitted, localDebit, localCredit,
                 paymentRecord, receiptRecord, treasuryOutboxPrepared,
                 treasuryFeeCommitted, beneficiaryClaimCommitted,
                 beneficiaryViewComplete, beneficiaryViewObserved,
                 payerConflict, beneficiaryConflict>>

BilateralBeneficiaryCommit ==
  /\ ProfileMode = "OwnerBilateral"
  /\ beneficiaryClaimCommitted = FALSE
  /\ beneficiaryClaimCommitted' = TRUE
  /\ beneficiaryViewComplete' = FALSE
  /\ beneficiaryViewObserved' = FALSE
  /\ projection' = "NotProjected"
  /\ operationComplete' = FALSE
  /\ UNCHANGED <<localAtomicCommitted, localDebit, localCredit,
                 paymentRecord, receiptRecord, treasuryOutboxPrepared,
                 treasuryFeeCommitted, payerDebitCommitted,
                 payerViewComplete, payerViewObserved,
                 payerConflict, beneficiaryConflict>>

ObservePayerViewComplete ==
  /\ ProfileMode = "OwnerBilateral"
  /\ payerViewComplete = FALSE
  /\ payerViewComplete' = TRUE
  /\ payerViewObserved' = payerDebitCommitted
  /\ projection' = "NotProjected"
  /\ operationComplete' = FALSE
  /\ UNCHANGED <<localAtomicCommitted, localDebit, localCredit,
                 paymentRecord, receiptRecord, treasuryOutboxPrepared,
                 treasuryFeeCommitted, payerDebitCommitted,
                 beneficiaryClaimCommitted, beneficiaryViewComplete,
                 beneficiaryViewObserved, payerConflict,
                 beneficiaryConflict>>

ObservePayerViewIncomplete ==
  /\ ProfileMode = "OwnerBilateral"
  /\ payerViewComplete = FALSE
  /\ payerViewObserved' = payerDebitCommitted
  /\ projection' = "NotProjected"
  /\ operationComplete' = FALSE
  /\ UNCHANGED <<localAtomicCommitted, localDebit, localCredit,
                 paymentRecord, receiptRecord, treasuryOutboxPrepared,
                 treasuryFeeCommitted, payerDebitCommitted,
                 beneficiaryClaimCommitted, payerViewComplete,
                 beneficiaryViewComplete, beneficiaryViewObserved,
                 payerConflict, beneficiaryConflict>>

ObserveBeneficiaryViewComplete ==
  /\ ProfileMode = "OwnerBilateral"
  /\ beneficiaryViewComplete = FALSE
  /\ beneficiaryViewComplete' = TRUE
  /\ beneficiaryViewObserved' = beneficiaryClaimCommitted
  /\ projection' = "NotProjected"
  /\ operationComplete' = FALSE
  /\ UNCHANGED <<localAtomicCommitted, localDebit, localCredit,
                 paymentRecord, receiptRecord, treasuryOutboxPrepared,
                 treasuryFeeCommitted, payerDebitCommitted,
                 beneficiaryClaimCommitted, payerViewComplete,
                 payerViewObserved, payerConflict, beneficiaryConflict>>

ObserveBeneficiaryViewIncomplete ==
  /\ ProfileMode = "OwnerBilateral"
  /\ beneficiaryViewComplete = FALSE
  /\ beneficiaryViewObserved' = beneficiaryClaimCommitted
  /\ projection' = "NotProjected"
  /\ operationComplete' = FALSE
  /\ UNCHANGED <<localAtomicCommitted, localDebit, localCredit,
                 paymentRecord, receiptRecord, treasuryOutboxPrepared,
                 treasuryFeeCommitted, payerDebitCommitted,
                 beneficiaryClaimCommitted, payerViewComplete,
                 beneficiaryViewComplete, payerViewObserved,
                 payerConflict, beneficiaryConflict>>

ObservePayerConflict ==
  /\ ProfileMode = "OwnerBilateral"
  /\ payerConflict = FALSE
  /\ payerViewObserved' = TRUE
  /\ payerConflict' = TRUE
  /\ projection' = "NotProjected"
  /\ operationComplete' = FALSE
  /\ UNCHANGED <<localAtomicCommitted, localDebit, localCredit,
                 paymentRecord, receiptRecord, treasuryOutboxPrepared,
                 treasuryFeeCommitted, payerDebitCommitted,
                 beneficiaryClaimCommitted, payerViewComplete,
                 beneficiaryViewComplete, beneficiaryViewObserved,
                 beneficiaryConflict>>

ObserveBeneficiaryConflict ==
  /\ ProfileMode = "OwnerBilateral"
  /\ beneficiaryConflict = FALSE
  /\ beneficiaryViewObserved' = TRUE
  /\ beneficiaryConflict' = TRUE
  /\ projection' = "NotProjected"
  /\ operationComplete' = FALSE
  /\ UNCHANGED <<localAtomicCommitted, localDebit, localCredit,
                 paymentRecord, receiptRecord, treasuryOutboxPrepared,
                 treasuryFeeCommitted, payerDebitCommitted,
                 beneficiaryClaimCommitted, payerViewComplete,
                 beneficiaryViewComplete, payerViewObserved,
                 payerConflict>>

ProjectBilateral ==
  /\ ProfileMode = "OwnerBilateral"
  /\ projection' =
       IF payerConflict \/ beneficiaryConflict
       THEN "Conflict"
       ELSE IF ~payerViewComplete \/ ~beneficiaryViewComplete
       THEN "Indeterminate"
       ELSE IF payerViewObserved /\ beneficiaryViewObserved
       THEN "Supported"
       ELSE "Pending"
  /\ UNCHANGED <<localAtomicCommitted, localDebit, localCredit,
                 paymentRecord, receiptRecord, treasuryOutboxPrepared,
                 treasuryFeeCommitted, payerDebitCommitted,
                 beneficiaryClaimCommitted, payerViewComplete,
                 beneficiaryViewComplete, payerViewObserved,
                 beneficiaryViewObserved, payerConflict,
                 beneficiaryConflict, operationComplete>>

MarkOperationComplete ==
  /\ operationComplete = FALSE
  /\ ((IsLocalProfile
       /\ localAtomicCommitted
       /\ (~FeeRequired \/ treasuryFeeCommitted))
      \/ (ProfileMode = "OwnerBilateral" /\ projection = "Supported"))
  /\ operationComplete' = TRUE
  /\ UNCHANGED <<localAtomicCommitted, localDebit, localCredit,
                 paymentRecord, receiptRecord, treasuryOutboxPrepared,
                 treasuryFeeCommitted, payerDebitCommitted,
                 beneficiaryClaimCommitted, payerViewComplete,
                 beneficiaryViewComplete, payerViewObserved,
                 beneficiaryViewObserved, payerConflict,
                 beneficiaryConflict, projection>>

\* Deliberate unsafe mutant: interprets an outbox intent as Treasury delivery.
UnsafeCompleteAfterOutbox ==
  /\ AllowUnsafeEarlyComplete = TRUE
  /\ ProfileMode = "LocalAtomicTreasury"
  /\ treasuryOutboxPrepared = TRUE
  /\ treasuryFeeCommitted = FALSE
  /\ operationComplete = FALSE
  /\ operationComplete' = TRUE
  /\ UNCHANGED <<localAtomicCommitted, localDebit, localCredit,
                 paymentRecord, receiptRecord, treasuryOutboxPrepared,
                 treasuryFeeCommitted, payerDebitCommitted,
                 beneficiaryClaimCommitted, payerViewComplete,
                 beneficiaryViewComplete, payerViewObserved,
                 beneficiaryViewObserved, payerConflict,
                 beneficiaryConflict, projection>>

Next ==
  \/ LocalAtomicCommit
  \/ UnsafeSplitLocalDebit
  \/ TreasuryFeeCommit
  \/ BilateralPayerCommit
  \/ BilateralBeneficiaryCommit
  \/ ObservePayerViewComplete
  \/ ObservePayerViewIncomplete
  \/ ObserveBeneficiaryViewComplete
  \/ ObserveBeneficiaryViewIncomplete
  \/ ObservePayerConflict
  \/ ObserveBeneficiaryConflict
  \/ ProjectBilateral
  \/ MarkOperationComplete
  \/ UnsafeCompleteAfterOutbox

Spec == Init /\ [][Next]_vars

TypeOK ==
  /\ localAtomicCommitted \in BOOLEAN
  /\ localDebit \in BOOLEAN
  /\ localCredit \in BOOLEAN
  /\ paymentRecord \in BOOLEAN
  /\ receiptRecord \in BOOLEAN
  /\ treasuryOutboxPrepared \in BOOLEAN
  /\ treasuryFeeCommitted \in BOOLEAN
  /\ payerDebitCommitted \in BOOLEAN
  /\ beneficiaryClaimCommitted \in BOOLEAN
  /\ payerViewComplete \in BOOLEAN
  /\ beneficiaryViewComplete \in BOOLEAN
  /\ payerViewObserved \in BOOLEAN
  /\ beneficiaryViewObserved \in BOOLEAN
  /\ payerConflict \in BOOLEAN
  /\ beneficiaryConflict \in BOOLEAN
  /\ projection \in {"NotProjected", "Pending", "Indeterminate", "Supported", "Conflict"}
  /\ operationComplete \in BOOLEAN

LocalAtomicNoPartial ==
  IsLocalProfile =>
    /\ localDebit = localAtomicCommitted
    /\ localCredit = localAtomicCommitted
    /\ paymentRecord = localAtomicCommitted
    /\ receiptRecord = localAtomicCommitted
    /\ treasuryOutboxPrepared = (localAtomicCommitted /\ FeeRequired)

TreasuryRequiresOutbox ==
  treasuryFeeCommitted =>
    /\ ProfileMode = "LocalAtomicTreasury"
    /\ FeeRequired
    /\ localAtomicCommitted
    /\ treasuryOutboxPrepared

NoTreasuryEffectWhenFeeNotRequired ==
  ~FeeRequired => ~treasuryFeeCommitted

BilateralOwnerSeparation ==
  ProfileMode = "OwnerBilateral" => PayerOwner # BeneficiaryOwner

ExecutionModelIsolation ==
  /\ (IsLocalProfile =>
       /\ ~payerDebitCommitted
       /\ ~beneficiaryClaimCommitted
       /\ ~payerViewComplete
       /\ ~beneficiaryViewComplete)
  /\ (ProfileMode = "OwnerBilateral" =>
       /\ ~localAtomicCommitted
       /\ ~localDebit
       /\ ~localCredit
       /\ ~paymentRecord
       /\ ~receiptRecord
       /\ ~treasuryOutboxPrepared
       /\ ~treasuryFeeCommitted)

BilateralProjectionSound ==
  /\ (projection = "Supported" =>
       /\ ProfileMode = "OwnerBilateral"
       /\ payerViewComplete
       /\ beneficiaryViewComplete
       /\ payerViewObserved
       /\ beneficiaryViewObserved
       /\ ~payerConflict
       /\ ~beneficiaryConflict)
  /\ (projection = "Pending" =>
       /\ ProfileMode = "OwnerBilateral"
       /\ payerViewComplete
       /\ beneficiaryViewComplete
       /\ ~(payerViewObserved /\ beneficiaryViewObserved)
       /\ ~payerConflict
       /\ ~beneficiaryConflict)
  /\ (projection = "Indeterminate" =>
       /\ ProfileMode = "OwnerBilateral"
       /\ (~payerViewComplete \/ ~beneficiaryViewComplete)
       /\ ~payerConflict
       /\ ~beneficiaryConflict)
  /\ (projection = "Conflict" =>
       /\ ProfileMode = "OwnerBilateral"
       /\ (payerConflict \/ beneficiaryConflict))

CompletionRequiresEffects ==
  operationComplete =>
    IF IsLocalProfile
    THEN localAtomicCommitted /\ (~FeeRequired \/ treasuryFeeCommitted)
    ELSE projection = "Supported"

Safety ==
  /\ TypeOK
  /\ LocalAtomicNoPartial
  /\ TreasuryRequiresOutbox
  /\ NoTreasuryEffectWhenFeeNotRequired
  /\ BilateralOwnerSeparation
  /\ ExecutionModelIsolation
  /\ BilateralProjectionSound
  /\ CompletionRequiresEffects

====
