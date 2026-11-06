---- MODULE PulseMailboxValidation ----
EXTENDS Naturals, FiniteSets, TLC

(*
Abstract contract model for SharedMailbox integrity validation.

The malicious coordinator is allowed to author any proposed transition. The
production integrity validator is modeled separately and checks the original
committed mailbox state, not the proposed member list.

UseProposedStateAuthority = TRUE is a deliberate mutant: it determines the
author's privilege from the proposed membership/role map. An attacker who adds
itself as Admin can therefore self-authorize.

IgnoreProtectedMutation = TRUE is a second mutant: it accepts an otherwise
valid member append even when unrelated protected mailbox state was changed.

This model does not simulate Holochain source-chain signatures, validation
receipts, DHT gossip, or conductor internals. It isolates the transition rule.
*)

CONSTANTS
    Owner, Admin, Attacker, NewMember, NoActor, NoRole,
    UseProposedStateAuthority, IgnoreProtectedMutation

Agents == {Owner, Admin, Attacker, NewMember}
OwnerRole == "Owner"
AdminRole == "Admin"
ViewerRole == "Viewer"
Roles == {OwnerRole, AdminRole, ViewerRole, NoRole}
InitialMembers == {Owner, Admin}

VARIABLES
    mailboxMembers,
    mailboxRoles,
    mailboxVersion,
    observerMembers,
    observerRoles,
    proposalPending,
    proposalAuthor,
    proposalAdded,
    proposalAddedBy,
    proposalRole,
    proposalProtectedMutation,
    proposalBaseVersion,
    proposalValidation,
    lastAuthored,
    lastAuthor,
    lastAdded,
    lastProtectedMutation,
    lastAccepted,
    lastRejected

vars ==
    << mailboxMembers, mailboxRoles, mailboxVersion,
       observerMembers, observerRoles,
       proposalPending, proposalAuthor, proposalAdded,
       proposalAddedBy, proposalRole, proposalProtectedMutation,
       proposalBaseVersion, proposalValidation, lastAuthored,
       lastAuthor, lastAdded, lastProtectedMutation,
       lastAccepted, lastRejected >>

Init ==
    /\ mailboxMembers = InitialMembers
    /\ mailboxRoles =
        [a \in Agents |->
            IF a = Owner THEN OwnerRole
            ELSE IF a = Admin THEN AdminRole
            ELSE NoRole]
    /\ mailboxVersion = 0
    /\ observerMembers = InitialMembers
    /\ observerRoles =
        [a \in Agents |->
            IF a = Owner THEN OwnerRole
            ELSE IF a = Admin THEN AdminRole
            ELSE NoRole]
    /\ proposalPending = FALSE
    /\ proposalAuthor = NoActor
    /\ proposalAdded = NoActor
    /\ proposalAddedBy = NoActor
    /\ proposalRole = NoRole
    /\ proposalProtectedMutation = FALSE
    /\ proposalBaseVersion = 0
    /\ proposalValidation = "none"
    /\ lastAuthored = FALSE
    /\ lastAuthor = NoActor
    /\ lastAdded = NoActor
    /\ lastProtectedMutation = FALSE
    /\ lastAccepted = FALSE
    /\ lastRejected = FALSE

SubmitForgedSelfAdmin ==
    /\ ~proposalPending
    /\ Attacker \notin mailboxMembers
    /\ proposalPending' = TRUE
    /\ proposalAuthor' = Attacker
    /\ proposalAdded' = Attacker
    /\ proposalAddedBy' = Attacker
    /\ proposalRole' = AdminRole
    /\ proposalProtectedMutation' = FALSE
    /\ proposalBaseVersion' = mailboxVersion
    /\ proposalValidation' = "pending"
    /\ lastAuthored' = TRUE
    /\ lastAuthor' = Attacker
    /\ lastAdded' = Attacker
    /\ lastProtectedMutation' = FALSE
    /\ lastAccepted' = FALSE
    /\ lastRejected' = FALSE
    /\ UNCHANGED
        << mailboxMembers, mailboxRoles, mailboxVersion,
           observerMembers, observerRoles >>

SubmitAdminDirectAppend ==
    /\ ~proposalPending
    /\ Admin \in mailboxMembers
    /\ NewMember \notin mailboxMembers
    /\ proposalPending' = TRUE
    /\ proposalAuthor' = Admin
    /\ proposalAdded' = NewMember
    /\ proposalAddedBy' = Admin
    /\ proposalRole' = ViewerRole
    /\ proposalProtectedMutation' = FALSE
    /\ proposalBaseVersion' = mailboxVersion
    /\ proposalValidation' = "pending"
    /\ lastAuthored' = TRUE
    /\ lastAuthor' = Admin
    /\ lastAdded' = NewMember
    /\ lastProtectedMutation' = FALSE
    /\ lastAccepted' = FALSE
    /\ lastRejected' = FALSE
    /\ UNCHANGED
        << mailboxMembers, mailboxRoles, mailboxVersion,
           observerMembers, observerRoles >>

SubmitOwnerAppend ==
    /\ ~proposalPending
    /\ NewMember \notin mailboxMembers
    /\ proposalPending' = TRUE
    /\ proposalAuthor' = Owner
    /\ proposalAdded' = NewMember
    /\ proposalAddedBy' = Owner
    /\ proposalRole' = ViewerRole
    /\ proposalProtectedMutation' = FALSE
    /\ proposalBaseVersion' = mailboxVersion
    /\ proposalValidation' = "pending"
    /\ lastAuthored' = TRUE
    /\ lastAuthor' = Owner
    /\ lastAdded' = NewMember
    /\ lastProtectedMutation' = FALSE
    /\ lastAccepted' = FALSE
    /\ lastRejected' = FALSE
    /\ UNCHANGED
        << mailboxMembers, mailboxRoles, mailboxVersion,
           observerMembers, observerRoles >>

SubmitOwnerAppendWithMutation ==
    /\ ~proposalPending
    /\ NewMember \notin mailboxMembers
    /\ proposalPending' = TRUE
    /\ proposalAuthor' = Owner
    /\ proposalAdded' = NewMember
    /\ proposalAddedBy' = Owner
    /\ proposalRole' = ViewerRole
    /\ proposalProtectedMutation' = TRUE
    /\ proposalBaseVersion' = mailboxVersion
    /\ proposalValidation' = "pending"
    /\ lastAuthored' = TRUE
    /\ lastAuthor' = Owner
    /\ lastAdded' = NewMember
    /\ lastProtectedMutation' = TRUE
    /\ lastAccepted' = FALSE
    /\ lastRejected' = FALSE
    /\ UNCHANGED
        << mailboxMembers, mailboxRoles, mailboxVersion,
           observerMembers, observerRoles >>

ValidateProposal ==
    /\ proposalPending
    /\ proposalValidation = "pending"
    /\ LET authorRole ==
               IF UseProposedStateAuthority
               THEN IF proposalAuthor = proposalAdded
                    THEN proposalRole
                    ELSE mailboxRoles[proposalAuthor]
               ELSE mailboxRoles[proposalAuthor]
           authorityOK ==
               IF UseProposedStateAuthority
               THEN authorRole \in {OwnerRole, AdminRole}
               ELSE proposalAuthor = Owner
           shapeOK ==
               /\ proposalBaseVersion = mailboxVersion
               /\ proposalAdded \notin mailboxMembers
               /\ proposalAddedBy = proposalAuthor
               /\ proposalRole # OwnerRole
           protectedOK ==
               IgnoreProtectedMutation \/ ~proposalProtectedMutation
           valid == authorityOK /\ shapeOK /\ protectedOK
       IN
           /\ proposalValidation' =
                IF valid THEN "accepted" ELSE "rejected"
           /\ lastAccepted' = FALSE
           /\ lastRejected' = ~valid
           /\ UNCHANGED
                << mailboxMembers, mailboxRoles, mailboxVersion,
                   observerMembers, observerRoles,
                   proposalPending, proposalAuthor, proposalAdded,
                   proposalAddedBy, proposalRole,
                   proposalProtectedMutation, proposalBaseVersion,
                   lastAuthored, lastAuthor, lastAdded,
                   lastProtectedMutation >>

IntegrateAcceptedProposal ==
    /\ proposalPending
    /\ proposalValidation = "accepted"
    /\ mailboxMembers' = mailboxMembers \cup {proposalAdded}
    /\ mailboxRoles' =
        [mailboxRoles EXCEPT ![proposalAdded] = proposalRole]
    /\ mailboxVersion' = mailboxVersion + 1
    /\ proposalPending' = FALSE
    /\ proposalValidation' = "integrated"
    /\ lastAccepted' = TRUE
    /\ lastRejected' = FALSE
    /\ UNCHANGED
        << observerMembers, observerRoles,
           proposalAuthor, proposalAdded, proposalAddedBy,
           proposalRole, proposalProtectedMutation,
           proposalBaseVersion, lastAuthored, lastAuthor,
           lastAdded, lastProtectedMutation >>

RejectProposal ==
    /\ proposalPending
    /\ proposalValidation = "rejected"
    /\ proposalPending' = FALSE
    /\ proposalValidation' = "rejected"
    /\ lastRejected' = TRUE
    /\ UNCHANGED
        << mailboxMembers, mailboxRoles, mailboxVersion,
           observerMembers, observerRoles,
           proposalAuthor, proposalAdded, proposalAddedBy,
           proposalRole, proposalProtectedMutation,
           proposalBaseVersion, lastAuthored, lastAuthor,
           lastAdded, lastProtectedMutation, lastAccepted >>

ConvergeObserver ==
    /\ observerMembers # mailboxMembers
    /\ observerMembers' = mailboxMembers
    /\ observerRoles' = mailboxRoles
    /\ UNCHANGED
        << mailboxMembers, mailboxRoles, mailboxVersion,
           proposalPending, proposalAuthor, proposalAdded,
           proposalAddedBy, proposalRole,
           proposalProtectedMutation, proposalBaseVersion,
           proposalValidation, lastAuthored, lastAuthor,
           lastAdded, lastProtectedMutation,
           lastAccepted, lastRejected >>

Next ==
    \/ SubmitForgedSelfAdmin
    \/ SubmitAdminDirectAppend
    \/ SubmitOwnerAppend
    \/ SubmitOwnerAppendWithMutation
    \/ ValidateProposal
    \/ IntegrateAcceptedProposal
    \/ RejectProposal
    \/ ConvergeObserver

Spec == Init /\ [][Next]_vars

NoSentinelCollision ==
    /\ NoActor \notin Agents
    /\ NoRole \notin {OwnerRole, AdminRole, ViewerRole}

TypeOK ==
    /\ mailboxMembers \subseteq Agents
    /\ mailboxRoles \in [Agents -> Roles]
    /\ mailboxVersion \in Nat
    /\ observerMembers \subseteq Agents
    /\ observerRoles \in [Agents -> Roles]
    /\ proposalPending \in BOOLEAN
    /\ proposalAuthor \in Agents \cup {NoActor}
    /\ proposalAdded \in Agents \cup {NoActor}
    /\ proposalAddedBy \in Agents \cup {NoActor}
    /\ proposalRole \in Roles
    /\ proposalProtectedMutation \in BOOLEAN
    /\ proposalBaseVersion \in Nat
    /\ proposalValidation \in
        {"none", "pending", "accepted", "rejected", "integrated"}
    /\ lastAuthored \in BOOLEAN
    /\ lastAuthor \in Agents \cup {NoActor}
    /\ lastAdded \in Agents \cup {NoActor}
    /\ lastProtectedMutation \in BOOLEAN
    /\ lastAccepted \in BOOLEAN
    /\ lastRejected \in BOOLEAN

OnlyOriginalOwnerMayUpdate ==
    lastAccepted => lastAuthor = Owner

ProtectedFieldsRemainImmutable ==
    lastAccepted => ~lastProtectedMutation

AttackerCannotSelfPromote ==
    Attacker \notin mailboxMembers

RejectedActionNeverIntegrated ==
    lastRejected => lastAdded \notin mailboxMembers

ObserverIsSubsetOfIntegratedState ==
    observerMembers \subseteq mailboxMembers

ObserverRoleMatchesIntegratedState ==
    \A a \in observerMembers : observerRoles[a] = mailboxRoles[a]

====
