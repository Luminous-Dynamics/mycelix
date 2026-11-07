---- MODULE PulseCellScopeIsolation ----
EXTENDS Naturals, FiniteSets, TLC

(*
Finite contract model for Holochain's per-cell capability scope.

Two independent host grants share the same grantee and read function:
  GrantA -> CellA, secretA, read
  GrantB -> CellB, secretB, read

The grant identity is local to the target cell. A grant is not an hApp-wide
(principal, function) permission. A read grant does not authorize a write
function. Revoking GrantA must not delete GrantB merely because both grants
share the same grantee and function name.

Mutants:
  UseGlobalCellLookup = TRUE:
    forgets the target CellId while looking up grants.
  IgnoreFunctionScope = TRUE:
    treats any function grant as authority for every function in the cell.
  RevokeByPrincipalFunction = TRUE:
    deletes every grant with the same grantee/function tuple instead of the
    exact Grant ActionHash.
*)

CONSTANTS
    CellA, CellB, NoCell,
    Caller, OtherAgent, NoAgent,
    GrantA, GrantB, NoGrant,
    SecretA, SecretB, WrongSecret, NoSecret,
    ReadFn, WriteFn, NoFunction,
    OpARead, OpBReadUsingA, OpAWrite, OpBReadUsingB,
    OpAReadAfterRevoke, OpBReadAfterRevoke, OpWrongCaller, OpWrongSecret,
    UseGlobalCellLookup,
    IgnoreFunctionScope,
    RevokeByPrincipalFunction

Cells == {CellA, CellB}
Agents == {Caller, OtherAgent}
Grants == {GrantA, GrantB}
Functions == {ReadFn, WriteFn}
Ops == {
    OpARead,
    OpBReadUsingA,
    OpAWrite,
    OpBReadUsingB,
    OpAReadAfterRevoke,
    OpBReadAfterRevoke,
    OpWrongCaller,
    OpWrongSecret
}

GrantCell ==
    [g \in Grants |->
        IF g = GrantA THEN CellA ELSE CellB]

GrantGrantee ==
    [g \in Grants |-> Caller]

GrantSecret ==
    [g \in Grants |->
        IF g = GrantA THEN SecretA ELSE SecretB]

GrantFunctions ==
    [g \in Grants |-> {ReadFn}]

VARIABLES
    issued,
    hostActive,
    claims,
    revokedA,
    doneOps,
    lastDecision,
    lastGrant,
    lastCell,
    lastFunction,
    lastActor,
    lastCellMatches,
    lastFunctionMatches,
    lastClaimPresent,
    lastCallerMatches,
    lastSecretMatches,
    lastGrantActive

vars ==
    << issued, hostActive, claims, revokedA, doneOps,
       lastDecision, lastGrant, lastCell, lastFunction, lastActor,
       lastCellMatches, lastFunctionMatches, lastClaimPresent,
       lastCallerMatches, lastSecretMatches, lastGrantActive >>

Init ==
    /\ issued = {}
    /\ hostActive = {}
    /\ claims = {}
    /\ revokedA = FALSE
    /\ doneOps = {}
    /\ lastDecision = "none"
    /\ lastGrant = NoGrant
    /\ lastCell = NoCell
    /\ lastFunction = NoFunction
    /\ lastActor = NoAgent
    /\ lastCellMatches = FALSE
    /\ lastFunctionMatches = FALSE
    /\ lastClaimPresent = FALSE
    /\ lastCallerMatches = FALSE
    /\ lastSecretMatches = FALSE
    /\ lastGrantActive = FALSE

IssueGrant(g) ==
    /\ g \in Grants
    /\ g \notin issued
    /\ issued' = issued \cup {g}
    /\ hostActive' = hostActive \cup {g}
    /\ UNCHANGED
        << claims, revokedA, doneOps,
           lastDecision, lastGrant, lastCell, lastFunction, lastActor,
           lastCellMatches, lastFunctionMatches, lastClaimPresent,
           lastCallerMatches, lastSecretMatches, lastGrantActive >>

DeliverClaim(g) ==
    /\ g \in issued
    /\ <<g, Caller>> \notin claims
    /\ claims' = claims \cup {<<g, Caller>>}
    /\ UNCHANGED
        << issued, hostActive, revokedA, doneOps,
           lastDecision, lastGrant, lastCell, lastFunction, lastActor,
           lastCellMatches, lastFunctionMatches, lastClaimPresent,
           lastCallerMatches, lastSecretMatches, lastGrantActive >>

RevokeGrantA ==
    /\ GrantA \in hostActive
    /\ ~revokedA
    /\ hostActive' =
        IF RevokeByPrincipalFunction
        THEN
            {g \in hostActive :
                ~((GrantGrantee[g] = Caller) /\ (GrantFunctions[g] = {ReadFn}))}
        ELSE hostActive \ {GrantA}
    /\ revokedA' = TRUE
    /\ UNCHANGED
        << issued, claims, doneOps,
           lastDecision, lastGrant, lastCell, lastFunction, lastActor,
           lastCellMatches, lastFunctionMatches, lastClaimPresent,
           lastCallerMatches, lastSecretMatches, lastGrantActive >>

Call(op, cell, fn, actor, presentedSecret) ==
    /\ op \in Ops
    /\ op \notin doneOps
    /\ cell \in Cells
    /\ fn \in Functions
    /\ actor \in Agents
    /\ LET candidates ==
            {g \in Grants :
                /\ g \in hostActive
                /\ <<g, actor>> \in claims
                /\ (GrantGrantee[g] = actor)
                /\ (UseGlobalCellLookup \/ (GrantCell[g] = cell))
                /\ (IgnoreFunctionScope \/ (fn \in GrantFunctions[g]))
                /\ (GrantSecret[g] = presentedSecret)}
           hostAllows == candidates # {}
           selected ==
                IF hostAllows
                THEN CHOOSE g \in candidates : TRUE
                ELSE NoGrant
           cellScopeOK ==
                IF hostAllows THEN (GrantCell[selected] = cell) ELSE FALSE
           functionScopeOK ==
                IF hostAllows THEN (fn \in GrantFunctions[selected]) ELSE FALSE
           claimOK ==
                IF hostAllows THEN (<<selected, actor>> \in claims) ELSE FALSE
           callerOK ==
                IF hostAllows THEN (GrantGrantee[selected] = actor) ELSE FALSE
           secretOK ==
                IF hostAllows THEN (GrantSecret[selected] = presentedSecret) ELSE FALSE
           activeOK ==
                IF hostAllows THEN (selected \in hostActive) ELSE FALSE
       IN
           /\ doneOps' = doneOps \cup {op}
           /\ lastDecision' =
                IF hostAllows THEN "AUTHORIZED" ELSE "UNAUTHORIZED"
           /\ lastGrant' = selected
           /\ lastCell' = cell
           /\ lastFunction' = fn
           /\ lastActor' = actor
           /\ lastCellMatches' = cellScopeOK
           /\ lastFunctionMatches' = functionScopeOK
           /\ lastClaimPresent' = claimOK
           /\ lastCallerMatches' = callerOK
           /\ lastSecretMatches' = secretOK
           /\ lastGrantActive' = activeOK
           /\ UNCHANGED
                << issued, hostActive, claims, revokedA >>

TryCellARead ==
    Call(OpARead, CellA, ReadFn, Caller, SecretA)

TryCellBReadWithASecret ==
    Call(OpBReadUsingA, CellB, ReadFn, Caller, SecretA)

TryCellAWrite ==
    Call(OpAWrite, CellA, WriteFn, Caller, SecretA)

TryCellBReadWithBSecret ==
    Call(OpBReadUsingB, CellB, ReadFn, Caller, SecretB)

TryCellAReadAfterRevoke ==
    Call(OpAReadAfterRevoke, CellA, ReadFn, Caller, SecretA)

TryCellBReadAfterRevoke ==
    Call(OpBReadAfterRevoke, CellB, ReadFn, Caller, SecretB)

TryWrongCaller ==
    Call(OpWrongCaller, CellA, ReadFn, OtherAgent, SecretA)

TryWrongSecret ==
    Call(OpWrongSecret, CellA, ReadFn, Caller, WrongSecret)

Next ==
    \/ \E g \in Grants : IssueGrant(g)
    \/ \E g \in Grants : DeliverClaim(g)
    \/ RevokeGrantA
    \/ TryCellARead
    \/ TryCellBReadWithASecret
    \/ TryCellAWrite
    \/ TryCellBReadWithBSecret
    \/ TryCellAReadAfterRevoke
    \/ TryCellBReadAfterRevoke
    \/ TryWrongCaller
    \/ TryWrongSecret

Spec == Init /\ [][Next]_vars

SentinelsAreDistinct ==
    /\ NoCell \notin Cells
    /\ NoAgent \notin Agents
    /\ NoGrant \notin Grants
    /\ NoSecret \notin {SecretA, SecretB, WrongSecret}
    /\ NoFunction \notin Functions
    /\ GrantA # GrantB
    /\ CellA # CellB

TypeOK ==
    /\ issued \subseteq Grants
    /\ hostActive \subseteq Grants
    /\ hostActive \subseteq issued
    /\ claims \subseteq {<<g, a>> : g \in issued, a \in Agents}
    /\ revokedA \in BOOLEAN
    /\ doneOps \subseteq Ops
    /\ lastDecision \in {"none", "AUTHORIZED", "UNAUTHORIZED"}
    /\ lastGrant \in Grants \cup {NoGrant}
    /\ lastCell \in Cells \cup {NoCell}
    /\ lastFunction \in Functions \cup {NoFunction}
    /\ lastActor \in Agents \cup {NoAgent}
    /\ lastCellMatches \in BOOLEAN
    /\ lastFunctionMatches \in BOOLEAN
    /\ lastClaimPresent \in BOOLEAN
    /\ lastCallerMatches \in BOOLEAN
    /\ lastSecretMatches \in BOOLEAN
    /\ lastGrantActive \in BOOLEAN

NoCrossCellGrantBorrow ==
    lastDecision = "AUTHORIZED" => lastCellMatches

NoFunctionScopeWidening ==
    lastDecision = "AUTHORIZED" => lastFunctionMatches

AssignedClaimAndSecretAreExact ==
    lastDecision = "AUTHORIZED"
    =>
        /\ lastClaimPresent
        /\ lastCallerMatches
        /\ lastSecretMatches
        /\ lastGrantActive

NoCollateralRevocation ==
    /\ revokedA => GrantA \notin hostActive
    /\ (revokedA /\ GrantB \in issued) => GrantB \in hostActive

HostAuthorityIsPerCellGrant ==
    (lastDecision # "AUTHORIZED")
    \/ (lastGrantActive /\ (lastGrant \in Grants))

====
