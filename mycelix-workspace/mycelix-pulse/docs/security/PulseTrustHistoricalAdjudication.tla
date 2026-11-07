---- MODULE PulseTrustHistoricalAdjudication ----
EXTENDS Naturals, FiniteSets, TLC

(*
Finite model of current-vs-historical trust adjudication.

Question 1: Is this key acceptable for a current decision under the current
policy epoch?
Question 2: Is the old statement historically authentic under the policy,
key status and trusted time that applied when it was signed?

These outcomes are deliberately distinct:
  CURRENT_ACCEPTED
  HISTORICALLY_AUTHENTICATED
  INDETERMINATE
  REJECTED

A signature that was valid under policy epoch 1 can remain historically
authentic after its key is revoked at epoch 2, but it cannot authorize a
current action. Missing trusted time or the exact historical policy snapshot
must return INDETERMINATE, not historical success.

Mutants:
  AcceptHistoricalAsCurrent = TRUE:
    allows an old pre-revocation statement to pass current verification after
    the key has been revoked.
  AcceptHistoricalWithoutTime = TRUE:
    treats signature validity + claimed epoch as sufficient even when trusted
    time or the historical policy snapshot is unavailable.
*)

CONSTANTS
    KeyA, KeyB, NoKey,
    EpochOne, EpochTwo,
    AcceptHistoricalAsCurrent,
    AcceptHistoricalWithoutTime

Keys == {KeyA, KeyB}
Epochs == {EpochOne, EpochTwo}
NoEpoch == 0
AllEpochs == {NoEpoch, EpochOne, EpochTwo}

VARIABLES
    currentPolicyEpoch,
    retainedPolicyEpochs,
    keyStatus,
    keyRevokedAtEpoch,
    statementPresent,
    statementKey,
    statementEpoch,
    statementSignatureValid,
    trustedTimeAvailable,
    historicalPolicyAvailable,
    currentOutcome,
    historicalOutcome,
    lastCurrentKeyStatus,
    lastCurrentSignatureValid,
    lastCurrentStatementEpoch,
    lastCurrentPolicyEpoch,
    lastHistoricalTimeAvailable,
    lastHistoricalPolicyAvailable,
    lastHistoricalSignatureValid,
    lastHistoricalStatementEpoch,
    lastHistoricalRevocationEpoch,
    lastHistoricalPolicyEpoch

vars ==
    << currentPolicyEpoch, retainedPolicyEpochs, keyStatus,
       keyRevokedAtEpoch, statementPresent, statementKey,
       statementEpoch, statementSignatureValid, trustedTimeAvailable,
       historicalPolicyAvailable, currentOutcome, historicalOutcome,
       lastCurrentKeyStatus, lastCurrentSignatureValid,
       lastCurrentStatementEpoch, lastCurrentPolicyEpoch,
       lastHistoricalTimeAvailable, lastHistoricalPolicyAvailable,
       lastHistoricalSignatureValid, lastHistoricalStatementEpoch,
       lastHistoricalRevocationEpoch, lastHistoricalPolicyEpoch >>

Init ==
    /\ currentPolicyEpoch = EpochOne
    /\ retainedPolicyEpochs = {EpochOne}
    /\ keyStatus =
        [k \in Keys |->
            IF k = KeyA THEN "active" ELSE "inactive"]
    /\ keyRevokedAtEpoch = [k \in Keys |-> NoEpoch]
    /\ statementPresent = FALSE
    /\ statementKey = NoKey
    /\ statementEpoch = NoEpoch
    /\ statementSignatureValid = FALSE
    /\ trustedTimeAvailable = TRUE
    /\ historicalPolicyAvailable = TRUE
    /\ currentOutcome = "none"
    /\ historicalOutcome = "none"
    /\ lastCurrentKeyStatus = "none"
    /\ lastCurrentSignatureValid = FALSE
    /\ lastCurrentStatementEpoch = NoEpoch
    /\ lastCurrentPolicyEpoch = NoEpoch
    /\ lastHistoricalTimeAvailable = FALSE
    /\ lastHistoricalPolicyAvailable = FALSE
    /\ lastHistoricalSignatureValid = FALSE
    /\ lastHistoricalStatementEpoch = NoEpoch
    /\ lastHistoricalRevocationEpoch = NoEpoch
    /\ lastHistoricalPolicyEpoch = NoEpoch

CreateSignedStatement ==
    /\ ~statementPresent
    /\ keyStatus[KeyA] = "active"
    /\ statementPresent' = TRUE
    /\ statementKey' = KeyA
    /\ statementEpoch' = currentPolicyEpoch
    /\ statementSignatureValid' = TRUE
    /\ currentOutcome' = "none"
    /\ historicalOutcome' = "none"
    /\ UNCHANGED
        << currentPolicyEpoch, retainedPolicyEpochs, keyStatus,
           keyRevokedAtEpoch, trustedTimeAvailable,
           historicalPolicyAvailable, lastCurrentKeyStatus,
           lastCurrentSignatureValid, lastCurrentStatementEpoch,
           lastCurrentPolicyEpoch, lastHistoricalTimeAvailable,
           lastHistoricalPolicyAvailable, lastHistoricalSignatureValid,
           lastHistoricalStatementEpoch, lastHistoricalRevocationEpoch,
           lastHistoricalPolicyEpoch >>

AdvancePolicyAndRevokeKeyA ==
    /\ currentPolicyEpoch = EpochOne
    /\ retainedPolicyEpochs' = retainedPolicyEpochs \cup {EpochTwo}
    /\ currentPolicyEpoch' = EpochTwo
    /\ keyStatus' =
        [keyStatus EXCEPT ![KeyA] = "revoked", ![KeyB] = "active"]
    /\ keyRevokedAtEpoch' =
        [keyRevokedAtEpoch EXCEPT ![KeyA] = EpochTwo]
    /\ currentOutcome' = "none"
    /\ historicalOutcome' = "none"
    /\ UNCHANGED
        << statementPresent, statementKey, statementEpoch,
           statementSignatureValid, trustedTimeAvailable,
           historicalPolicyAvailable, lastCurrentKeyStatus,
           lastCurrentSignatureValid, lastCurrentStatementEpoch,
           lastCurrentPolicyEpoch, lastHistoricalTimeAvailable,
           lastHistoricalPolicyAvailable, lastHistoricalSignatureValid,
           lastHistoricalStatementEpoch, lastHistoricalRevocationEpoch,
           lastHistoricalPolicyEpoch >>

RemoveTrustedTime ==
    /\ trustedTimeAvailable
    /\ trustedTimeAvailable' = FALSE
    /\ historicalOutcome' = "none"
    /\ lastHistoricalTimeAvailable' = FALSE
    /\ lastHistoricalPolicyAvailable' = FALSE
    /\ lastHistoricalSignatureValid' = FALSE
    /\ lastHistoricalStatementEpoch' = NoEpoch
    /\ lastHistoricalRevocationEpoch' = NoEpoch
    /\ lastHistoricalPolicyEpoch' = NoEpoch
    /\ UNCHANGED
        << currentPolicyEpoch, retainedPolicyEpochs, keyStatus,
           keyRevokedAtEpoch, statementPresent, statementKey,
           statementEpoch, statementSignatureValid,
           historicalPolicyAvailable, currentOutcome,
           lastCurrentKeyStatus, lastCurrentSignatureValid,
           lastCurrentStatementEpoch, lastCurrentPolicyEpoch >>

RemoveHistoricalPolicySnapshot ==
    /\ historicalPolicyAvailable
    /\ historicalPolicyAvailable' = FALSE
    /\ historicalOutcome' = "none"
    /\ lastHistoricalTimeAvailable' = FALSE
    /\ lastHistoricalPolicyAvailable' = FALSE
    /\ lastHistoricalSignatureValid' = FALSE
    /\ lastHistoricalStatementEpoch' = NoEpoch
    /\ lastHistoricalRevocationEpoch' = NoEpoch
    /\ lastHistoricalPolicyEpoch' = NoEpoch
    /\ UNCHANGED
        << currentPolicyEpoch, retainedPolicyEpochs, keyStatus,
           keyRevokedAtEpoch, statementPresent, statementKey,
           statementEpoch, statementSignatureValid, trustedTimeAvailable,
           currentOutcome, lastCurrentKeyStatus,
           lastCurrentSignatureValid, lastCurrentStatementEpoch,
           lastCurrentPolicyEpoch >>

VerifyCurrent ==
    /\ statementPresent
    /\ currentOutcome' =
        IF statementSignatureValid
           /\ statementEpoch <= currentPolicyEpoch
           /\ keyStatus[statementKey] = "active"
        THEN "CURRENT_ACCEPTED"
        ELSE IF AcceptHistoricalAsCurrent
                /\ statementSignatureValid
                /\ statementEpoch < keyRevokedAtEpoch[statementKey]
                /\ keyRevokedAtEpoch[statementKey] # NoEpoch
                /\ trustedTimeAvailable
                /\ historicalPolicyAvailable
             THEN "CURRENT_ACCEPTED"
             ELSE "REJECTED"
    /\ lastCurrentKeyStatus' = keyStatus[statementKey]
    /\ lastCurrentSignatureValid' = statementSignatureValid
    /\ lastCurrentStatementEpoch' = statementEpoch
    /\ lastCurrentPolicyEpoch' = currentPolicyEpoch
    /\ UNCHANGED
        << currentPolicyEpoch, retainedPolicyEpochs, keyStatus,
           keyRevokedAtEpoch, statementPresent, statementKey,
           statementEpoch, statementSignatureValid, trustedTimeAvailable,
           historicalPolicyAvailable, historicalOutcome,
           lastHistoricalTimeAvailable, lastHistoricalPolicyAvailable,
           lastHistoricalSignatureValid, lastHistoricalStatementEpoch,
           lastHistoricalRevocationEpoch, lastHistoricalPolicyEpoch >>

VerifyHistorical ==
    /\ statementPresent
    /\ historicalOutcome' =
        IF ~statementSignatureValid
           \/ statementEpoch = NoEpoch
           \/ ~retainedPolicyEpochs \cap {statementEpoch} # {statementEpoch}
        THEN "REJECTED"
        ELSE IF trustedTimeAvailable /\ historicalPolicyAvailable
        THEN
            IF keyRevokedAtEpoch[statementKey] = NoEpoch
               \/ statementEpoch < keyRevokedAtEpoch[statementKey]
            THEN "HISTORICALLY_AUTHENTICATED"
            ELSE "REJECTED"
        ELSE IF AcceptHistoricalWithoutTime
             /\ (keyRevokedAtEpoch[statementKey] = NoEpoch
                 \/ statementEpoch < keyRevokedAtEpoch[statementKey])
             THEN "HISTORICALLY_AUTHENTICATED"
             ELSE "INDETERMINATE"
    /\ lastHistoricalTimeAvailable' = trustedTimeAvailable
    /\ lastHistoricalPolicyAvailable' = historicalPolicyAvailable
    /\ lastHistoricalSignatureValid' = statementSignatureValid
    /\ lastHistoricalStatementEpoch' = statementEpoch
    /\ lastHistoricalRevocationEpoch' = keyRevokedAtEpoch[statementKey]
    /\ lastHistoricalPolicyEpoch' = currentPolicyEpoch
    /\ UNCHANGED
        << currentPolicyEpoch, retainedPolicyEpochs, keyStatus,
           keyRevokedAtEpoch, statementPresent, statementKey,
           statementEpoch, statementSignatureValid, trustedTimeAvailable,
           historicalPolicyAvailable, currentOutcome,
           lastCurrentKeyStatus, lastCurrentSignatureValid,
           lastCurrentStatementEpoch, lastCurrentPolicyEpoch >>

Next ==
    \/ CreateSignedStatement
    \/ AdvancePolicyAndRevokeKeyA
    \/ RemoveTrustedTime
    \/ RemoveHistoricalPolicySnapshot
    \/ VerifyCurrent
    \/ VerifyHistorical

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ currentPolicyEpoch \in Epochs
    /\ retainedPolicyEpochs \subseteq Epochs
    /\ keyStatus \in [Keys -> {"active", "inactive", "revoked"}]
    /\ keyRevokedAtEpoch \in [Keys -> AllEpochs]
    /\ statementPresent \in BOOLEAN
    /\ statementKey \in Keys \cup {NoKey}
    /\ statementEpoch \in AllEpochs
    /\ statementSignatureValid \in BOOLEAN
    /\ trustedTimeAvailable \in BOOLEAN
    /\ historicalPolicyAvailable \in BOOLEAN
    /\ currentOutcome \in {"none", "CURRENT_ACCEPTED", "REJECTED"}
    /\ historicalOutcome \in {"none", "HISTORICALLY_AUTHENTICATED", "INDETERMINATE", "REJECTED"}
    /\ lastCurrentKeyStatus \in {"none", "active", "inactive", "revoked"}
    /\ lastCurrentSignatureValid \in BOOLEAN
    /\ lastCurrentStatementEpoch \in AllEpochs
    /\ lastCurrentPolicyEpoch \in AllEpochs
    /\ lastHistoricalTimeAvailable \in BOOLEAN
    /\ lastHistoricalPolicyAvailable \in BOOLEAN
    /\ lastHistoricalSignatureValid \in BOOLEAN
    /\ lastHistoricalStatementEpoch \in AllEpochs
    /\ lastHistoricalRevocationEpoch \in AllEpochs
    /\ lastHistoricalPolicyEpoch \in AllEpochs

NoCurrentAcceptanceFromRevokedKey ==
    currentOutcome = "CURRENT_ACCEPTED"
    => /\ lastCurrentKeyStatus = "active"
       /\ lastCurrentSignatureValid
       /\ lastCurrentStatementEpoch <= lastCurrentPolicyEpoch

HistoricalAcceptanceRequiresTrustedEvidence ==
    historicalOutcome = "HISTORICALLY_AUTHENTICATED"
    => /\ lastHistoricalTimeAvailable
       /\ lastHistoricalPolicyAvailable
       /\ lastHistoricalSignatureValid
       /\ lastHistoricalStatementEpoch # NoEpoch
       /\ lastHistoricalStatementEpoch \in retainedPolicyEpochs
       /\ (lastHistoricalRevocationEpoch = NoEpoch
           \/ lastHistoricalStatementEpoch < lastHistoricalRevocationEpoch)
       /\ lastHistoricalStatementEpoch <= lastHistoricalPolicyEpoch

MissingHistoricalEvidenceNeverAccepts ==
    historicalOutcome = "HISTORICALLY_AUTHENTICATED"
    => lastHistoricalTimeAvailable /\ lastHistoricalPolicyAvailable

RevokedKeyCurrentDecisionRejected ==
    currentOutcome = "CURRENT_ACCEPTED"
    => lastCurrentKeyStatus # "revoked"

HistoricalAndCurrentOutcomesAreDistinct ==
    historicalOutcome = "HISTORICALLY_AUTHENTICATED"
    /\ keyStatus[KeyA] = "revoked"
    => currentOutcome # "CURRENT_ACCEPTED"

====
