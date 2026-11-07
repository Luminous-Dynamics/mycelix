---- MODULE PulseChainRestore ----
EXTENDS Naturals, FiniteSets, TLC

(*
Bounded model of Holochain 0.7.1 public source-chain restoration.

The model distinguishes:
- original AgentPubKey private-key availability;
- restore request and AwaitingRestore status;
- peer availability;
- fetched actions vs validator-accepted public actions;
- completion of every cell chain;
- explicit app enable;
- non-restoration of private entries and host capability grants;
- fresh host grant re-issuance after restore.

UseEarlyEnableMutant permits EnableApp before every required cell chain is
complete.
UseNoPeerEmptyMutant treats "no peers available yet" as an empty successful
restore. Both mutants must violate EnableRequiresAllCellsRestored.

This is an abstract contract model, not a model of Holochain's implementation.
*)

CONSTANTS
    HasOriginalAgentKey,
    UseEarlyEnableMutant,
    UseNoPeerEmptyMutant,
    UseMissingKeyMutant,
    UseLegacyGrantRestoreMutant

Cells == {"cell-1", "cell-2"}
Action1 == "cell-1-action-1"
Action2 == "cell-2-action-1"
InvalidAction == "forged-entry-action"
RequiredActions == {Action1, Action2}
AllActions == RequiredActions \cup {InvalidAction}
ValidActions == RequiredActions
ActionCell ==
    [action \in RequiredActions |->
        IF action = Action1 THEN "cell-1" ELSE "cell-2"]

VARIABLES
    processRunning,
    peerAvailable,
    restoreRequested,
    appStatus,
    fetchedActions,
    restoredActions,
    rejectedActions,
    privateRestored,
    freshGrantIssued,
    hostGrantActive,
    restoreGeneration

vars ==
    << processRunning, peerAvailable, restoreRequested, appStatus,
       fetchedActions, restoredActions, rejectedActions, privateRestored,
       freshGrantIssued, hostGrantActive, restoreGeneration >>

AllCellsRestored ==
    \A cell \in Cells :
        \A action \in RequiredActions :
            ActionCell[action] = cell => action \in restoredActions

Init ==
    /\ processRunning = TRUE
    /\ peerAvailable = FALSE
    /\ restoreRequested = FALSE
    /\ appStatus = "disabled"
    /\ fetchedActions = {}
    /\ restoredActions = {}
    /\ rejectedActions = {}
    /\ privateRestored = {}
    /\ freshGrantIssued = FALSE
    /\ hostGrantActive = FALSE
    /\ restoreGeneration = 0

(* Deliberately unsafe mutant: permit restore without the original agent key. *)
StartRestoreWithoutKey ==
    /\ UseMissingKeyMutant
    /\ ~HasOriginalAgentKey
    /\ processRunning
    /\ ~restoreRequested
    /\ appStatus = "disabled"
    /\ restoreRequested' = TRUE
    /\ appStatus' = "awaiting-restore"
    /\ restoreGeneration' = restoreGeneration + 1
    /\ UNCHANGED
        << processRunning, peerAvailable, fetchedActions,
           restoredActions, rejectedActions, privateRestored,
           freshGrantIssued, hostGrantActive >>

StartRestore ==
    /\ processRunning
    /\ HasOriginalAgentKey
    /\ ~restoreRequested
    /\ appStatus = "disabled"
    /\ restoreRequested' = TRUE
    /\ appStatus' = "awaiting-restore"
    /\ restoreGeneration' = restoreGeneration + 1
    /\ UNCHANGED
        << processRunning, peerAvailable, fetchedActions,
           restoredActions, rejectedActions, privateRestored,
           freshGrantIssued, hostGrantActive >>

PeerAppears ==
    /\ ~peerAvailable
    /\ peerAvailable' = TRUE
    /\ UNCHANGED
        << processRunning, restoreRequested, appStatus, fetchedActions,
           restoredActions, rejectedActions, privateRestored,
           freshGrantIssued, hostGrantActive, restoreGeneration >>

PeerDisappears ==
    /\ peerAvailable
    /\ peerAvailable' = FALSE
    /\ UNCHANGED
        << processRunning, restoreRequested, appStatus, fetchedActions,
           restoredActions, rejectedActions, privateRestored,
           freshGrantIssued, hostGrantActive, restoreGeneration >>

FetchFromPeer(action) ==
    /\ processRunning
    /\ restoreRequested
    /\ appStatus = "awaiting-restore"
    /\ peerAvailable
    /\ action \in AllActions
    /\ action \notin fetchedActions
    /\ fetchedActions' = fetchedActions \cup {action}
    /\ UNCHANGED
        << processRunning, peerAvailable, restoreRequested, appStatus,
           restoredActions, rejectedActions, privateRestored,
           freshGrantIssued, hostGrantActive, restoreGeneration >>

ValidateFetchedAction(action) ==
    /\ processRunning
    /\ appStatus = "awaiting-restore"
    /\ action \in fetchedActions
    /\ action \notin restoredActions
    /\ action \notin rejectedActions
    /\ IF action \in ValidActions
          THEN
              /\ restoredActions' = restoredActions \cup {action}
              /\ UNCHANGED rejectedActions
          ELSE
              /\ rejectedActions' = rejectedActions \cup {action}
              /\ UNCHANGED restoredActions
    /\ UNCHANGED
        << processRunning, peerAvailable, restoreRequested, appStatus,
           fetchedActions, privateRestored, freshGrantIssued,
           hostGrantActive, restoreGeneration >>

EnableApp ==
    /\ processRunning
    /\ appStatus = "awaiting-restore"
    /\ (UseEarlyEnableMutant \/ AllCellsRestored)
    /\ appStatus' = "enabled"
    /\ UNCHANGED
        << processRunning, peerAvailable, restoreRequested,
           fetchedActions, restoredActions, rejectedActions,
           privateRestored, freshGrantIssued, hostGrantActive,
           restoreGeneration >>

TreatNoPeersAsEmptySuccess ==
    /\ UseNoPeerEmptyMutant
    /\ processRunning
    /\ restoreRequested
    /\ appStatus = "awaiting-restore"
    /\ ~peerAvailable
    /\ ~AllCellsRestored
    /\ appStatus' = "enabled"
    /\ UNCHANGED
        << processRunning, peerAvailable, restoreRequested,
           fetchedActions, restoredActions, rejectedActions,
           privateRestored, freshGrantIssued, hostGrantActive,
           restoreGeneration >>

(* Deliberately unsafe mutant: import an old application claim as if it
   were a live conductor CapGrant. Restoration may recover public metadata,
   but host capability state must be re-issued through the target conductor. *)
RestoreLegacyHostGrant ==
    /\ UseLegacyGrantRestoreMutant
    /\ processRunning
    /\ restoreRequested
    /\ appStatus = "awaiting-restore"
    /\ restoredActions # {}
    /\ ~hostGrantActive
    /\ hostGrantActive' = TRUE
    /\ UNCHANGED
        << processRunning, peerAvailable, restoreRequested, appStatus,
           fetchedActions, restoredActions, rejectedActions,
           privateRestored, freshGrantIssued, restoreGeneration >>

Crash ==
    /\ processRunning
    /\ processRunning' = FALSE
    /\ UNCHANGED
        << peerAvailable, restoreRequested, appStatus, fetchedActions,
           restoredActions, rejectedActions, privateRestored,
           freshGrantIssued, hostGrantActive, restoreGeneration >>

Restart ==
    /\ ~processRunning
    /\ processRunning' = TRUE
    /\ UNCHANGED
        << peerAvailable, restoreRequested, appStatus, fetchedActions,
           restoredActions, rejectedActions, privateRestored,
           freshGrantIssued, hostGrantActive, restoreGeneration >>

ReissueFreshHostGrant ==
    /\ processRunning
    /\ appStatus = "enabled"
    /\ ~freshGrantIssued
    /\ freshGrantIssued' = TRUE
    /\ hostGrantActive' = TRUE
    /\ UNCHANGED
        << processRunning, peerAvailable, restoreRequested, appStatus,
           fetchedActions, restoredActions, rejectedActions,
           privateRestored, restoreGeneration >>

Next ==
    \/ StartRestore
    \/ StartRestoreWithoutKey
    \/ PeerAppears
    \/ PeerDisappears
    \/ \E action \in AllActions : FetchFromPeer(action)
    \/ \E action \in AllActions : ValidateFetchedAction(action)
    \/ EnableApp
    \/ TreatNoPeersAsEmptySuccess
    \/ RestoreLegacyHostGrant
    \/ Crash
    \/ Restart
    \/ ReissueFreshHostGrant

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ processRunning \in BOOLEAN
    /\ peerAvailable \in BOOLEAN
    /\ restoreRequested \in BOOLEAN
    /\ appStatus \in {"disabled", "awaiting-restore", "enabled"}
    /\ fetchedActions \subseteq AllActions
    /\ restoredActions \subseteq AllActions
    /\ rejectedActions \subseteq AllActions
    /\ privateRestored = {}
    /\ freshGrantIssued \in BOOLEAN
    /\ hostGrantActive \in BOOLEAN
    /\ restoreGeneration \in Nat

RestoreRequiresOriginalKey ==
    restoreRequested => HasOriginalAgentKey

EnableRequiresAllCellsRestored ==
    appStatus = "enabled" => AllCellsRestored

OnlyValidatedPublicActionsRestored ==
    /\ restoredActions \subseteq ValidActions
    /\ restoredActions \cap rejectedActions = {}

PrivateDataIsNotRestored ==
    privateRestored = {}

HostCapabilityMustBeFreshlyReissued ==
    hostGrantActive => freshGrantIssued

NoPrematureEnableWhilePeersMissing ==
    appStatus = "enabled" /\ ~peerAvailable
    => AllCellsRestored

====
