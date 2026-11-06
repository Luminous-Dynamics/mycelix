---- MODULE PulseSignalTrust ----
EXTENDS Naturals, FiniteSets, TLC

(*
Bounded model for remote notification trust.

A received signal is an untrusted hint. It can update trusted UI state only
after the transport caller, claimed source, exact backing action, canonical
record author, record availability/validity and payload projection all match.

UseRemotePayloadAsAuthority = TRUE is a deliberate mutant: it applies the
sender-controlled value without a successful evidence check.

SkipDedupe = TRUE is a separate replay mutant: it applies the same logical
event more than once.

The model does not implement Holochain transport or DHT semantics. It models
the application's trust boundary and exactly-once logical-effect invariant.
*)

CONSTANTS
    Honest,
    Attacker,
    NoAgent,
    e1,
    e2,
    NoEvent,
    recordAction,
    wrongAction,
    UseRemotePayloadAsAuthority,
    SkipDedupe

Agents == {Honest, Attacker}
Events == {e1, e2}
Actions == {recordAction, wrongAction}

VARIABLES
    signalPending,
    currentEvent,
    transportCaller,
    claimedSource,
    claimedAction,
    signalPayloadValue,
    recordAuthor,
    recordActionId,
    recordValue,
    recordAvailable,
    recordValid,
    evidenceValid,
    appliedEvents,
    effectCount,
    trustedUiValue,
    lastUiUpdateWasVerified,
    lastUiEvent

vars ==
    << signalPending, currentEvent, transportCaller, claimedSource,
       claimedAction, signalPayloadValue, recordAuthor, recordActionId,
       recordValue, recordAvailable, recordValid, evidenceValid,
       appliedEvents, effectCount, trustedUiValue,
       lastUiUpdateWasVerified, lastUiEvent >>

Init ==
    /\ signalPending = FALSE
    /\ currentEvent = NoEvent
    /\ transportCaller = NoAgent
    /\ claimedSource = NoAgent
    /\ claimedAction = wrongAction
    /\ signalPayloadValue = 0
    /\ recordAuthor = Honest
    /\ recordActionId = recordAction
    /\ recordValue = 7
    /\ recordAvailable = TRUE
    /\ recordValid = TRUE
    /\ evidenceValid = FALSE
    /\ appliedEvents = {}
    /\ effectCount = [e \in Events |-> 0]
    /\ trustedUiValue = 0
    /\ lastUiUpdateWasVerified = TRUE
    /\ lastUiEvent = NoEvent

ReceiveLegitimate ==
    /\ ~signalPending
    /\ currentEvent' = e1
    /\ transportCaller' = Honest
    /\ claimedSource' = Honest
    /\ claimedAction' = recordAction
    /\ signalPayloadValue' = recordValue
    /\ signalPending' = TRUE
    /\ evidenceValid' = FALSE
    /\ UNCHANGED
        << recordAuthor, recordActionId, recordValue,
           recordAvailable, recordValid, appliedEvents,
           effectCount, trustedUiValue, lastUiUpdateWasVerified,
           lastUiEvent >>

ReceiveForgedSender ==
    /\ ~signalPending
    /\ currentEvent' = e2
    /\ transportCaller' = Attacker
    /\ claimedSource' = Honest
    /\ claimedAction' = recordAction
    /\ signalPayloadValue' = 99
    /\ signalPending' = TRUE
    /\ evidenceValid' = FALSE
    /\ UNCHANGED
        << recordAuthor, recordActionId, recordValue,
           recordAvailable, recordValid, appliedEvents,
           effectCount, trustedUiValue, lastUiUpdateWasVerified,
           lastUiEvent >>

ReceiveForgedPayload ==
    /\ ~signalPending
    /\ currentEvent' = e2
    /\ transportCaller' = Honest
    /\ claimedSource' = Honest
    /\ claimedAction' = recordAction
    /\ signalPayloadValue' = 99
    /\ signalPending' = TRUE
    /\ evidenceValid' = FALSE
    /\ UNCHANGED
        << recordAuthor, recordActionId, recordValue,
           recordAvailable, recordValid, appliedEvents,
           effectCount, trustedUiValue, lastUiUpdateWasVerified,
           lastUiEvent >>

ReceiveWrongAction ==
    /\ ~signalPending
    /\ currentEvent' = e2
    /\ transportCaller' = Honest
    /\ claimedSource' = Honest
    /\ claimedAction' = wrongAction
    /\ signalPayloadValue' = recordValue
    /\ signalPending' = TRUE
    /\ evidenceValid' = FALSE
    /\ UNCHANGED
        << recordAuthor, recordActionId, recordValue,
           recordAvailable, recordValid, appliedEvents,
           effectCount, trustedUiValue, lastUiUpdateWasVerified,
           lastUiEvent >>

ReceiveReplay ==
    /\ ~signalPending
    /\ e1 \in appliedEvents
    /\ currentEvent' = e1
    /\ transportCaller' = Honest
    /\ claimedSource' = Honest
    /\ claimedAction' = recordAction
    /\ signalPayloadValue' = recordValue
    /\ signalPending' = TRUE
    /\ evidenceValid' = FALSE
    /\ UNCHANGED
        << recordAuthor, recordActionId, recordValue,
           recordAvailable, recordValid, appliedEvents,
           effectCount, trustedUiValue, lastUiUpdateWasVerified,
           lastUiEvent >>

VerifyBackingRecord ==
    /\ signalPending
    /\ evidenceValid' =
        /\ recordAvailable
        /\ recordValid
        /\ transportCaller = claimedSource
        /\ claimedSource = recordAuthor
        /\ claimedAction = recordActionId
        /\ signalPayloadValue = recordValue
    /\ UNCHANGED
        << signalPending, currentEvent, transportCaller,
           claimedSource, claimedAction, signalPayloadValue,
           recordAuthor, recordActionId, recordValue,
           recordAvailable, recordValid, appliedEvents,
           effectCount, trustedUiValue, lastUiUpdateWasVerified,
           lastUiEvent >>

ApplySignal ==
    /\ signalPending
    /\ (UseRemotePayloadAsAuthority \/ evidenceValid)
    /\ (SkipDedupe \/ currentEvent \notin appliedEvents)
    /\ appliedEvents' = appliedEvents \cup {currentEvent}
    /\ effectCount' =
        [effectCount EXCEPT ![currentEvent] = @ + 1]
    /\ trustedUiValue' =
        IF UseRemotePayloadAsAuthority
        THEN signalPayloadValue
        ELSE recordValue
    /\ lastUiUpdateWasVerified' = evidenceValid
    /\ lastUiEvent' = currentEvent
    /\ signalPending' = FALSE
    /\ evidenceValid' = FALSE
    /\ UNCHANGED
        << currentEvent, transportCaller, claimedSource,
           claimedAction, signalPayloadValue, recordAuthor,
           recordActionId, recordValue, recordAvailable,
           recordValid >>

Next ==
    \/ ReceiveLegitimate
    \/ ReceiveForgedSender
    \/ ReceiveForgedPayload
    \/ ReceiveWrongAction
    \/ ReceiveReplay
    \/ VerifyBackingRecord
    \/ ApplySignal

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ signalPending \in BOOLEAN
    /\ currentEvent \in Events \cup {NoEvent}
    /\ transportCaller \in Agents \cup {NoAgent}
    /\ claimedSource \in Agents \cup {NoAgent}
    /\ claimedAction \in Actions
    /\ signalPayloadValue \in 0..100
    /\ recordAuthor \in Agents
    /\ recordActionId \in Actions
    /\ recordValue \in 0..100
    /\ recordAvailable \in BOOLEAN
    /\ recordValid \in BOOLEAN
    /\ evidenceValid \in BOOLEAN
    /\ appliedEvents \subseteq Events
    /\ effectCount \in [Events -> 0..2]
    /\ trustedUiValue \in 0..100
    /\ lastUiUpdateWasVerified \in BOOLEAN
    /\ lastUiEvent \in Events \cup {NoEvent}

NoUnverifiedSignalMutatesTrustedState ==
    lastUiUpdateWasVerified

AtMostOneEffectPerLogicalEvent ==
    \A e \in Events : effectCount[e] <= 1

AppliedEventsHaveEffects ==
    \A e \in appliedEvents : effectCount[e] >= 1

====
