---- MODULE PulseConcurrentGrantIdempotency ----
EXTENDS Naturals, FiniteSets, TLC

(*
Two-worker model of concurrent, retry-safe capability issuance.

A logical idempotency key is modeled as one shared key. DigestA/DigestB are
canonical request digests. In the safe configuration both workers submit the
same digest; the digest-conflict mutant gives WorkerB a different digest.

The safe implementation uses an atomic reservation before any host grant is
created. Both workers may observe absence, but only one can reserve the key.
The other waits and later reconciles to the same grant identity.

Mutants:
  UseAtomicReservation = FALSE:
    both workers can act on stale absence observations and create distinct
    host grants for the same logical key.

  EnforceDigestBinding = FALSE:
    a caller with a different canonical request digest can reuse the existing
    key and receive the existing grant rather than a conflict.

  EnforceRevocationMonotonic = FALSE:
    a retry after a committed revoke can mint a replacement grant under the
    same logical key, resurrecting authority.

This abstracts the reservation/store transaction. It does not assume that
Holochain source-chain actions and conductor admin grants are one atomic
transaction; that is a runtime question to test separately.
*)

CONSTANTS
    WorkerA, WorkerB, NoWorker,
    GrantA, GrantB, NoGrant,
    DigestA, DigestB, NoDigest,
    UseAtomicReservation,
    EnforceDigestBinding,
    EnforceRevocationMonotonic

Workers == {WorkerA, WorkerB}
CandidateGrants == {GrantA, GrantB}
Digests == {DigestA, DigestB}

RequestDigest(w) ==
    IF w = WorkerA THEN DigestA ELSE DigestB

VARIABLES
    reservationOwner,
    reservationDigest,
    logicalStatus,
    grants,
    grantForKey,
    grantDigests,
    grantStates,
    revokedGrants,
    workerPhase,
    workerGrant

vars ==
    << reservationOwner, reservationDigest, logicalStatus, grants,
       grantForKey, grantDigests, grantStates, revokedGrants,
       workerPhase, workerGrant >>

Init ==
    /\ reservationOwner = NoWorker
    /\ reservationDigest = NoDigest
    /\ logicalStatus = "absent"
    /\ grants = {}
    /\ grantForKey = NoGrant
    /\ grantDigests = [g \in CandidateGrants |-> NoDigest]
    /\ grantStates = [g \in CandidateGrants |-> "absent"]
    /\ revokedGrants = {}
    /\ workerPhase = [w \in Workers |-> "idle"]
    /\ workerGrant = [w \in Workers |-> NoGrant]

ObserveAbsence(w) ==
    /\ workerPhase[w] = "idle"
    /\ logicalStatus = "absent"
    /\ workerPhase' = [workerPhase EXCEPT ![w] = "observed_absent"]
    /\ UNCHANGED
        << reservationOwner, reservationDigest, logicalStatus, grants,
           grantForKey, grantDigests, grantStates, revokedGrants,
           workerGrant >>

ReserveKeyAtomically(w) ==
    /\ UseAtomicReservation
    /\ logicalStatus = "absent"
    /\ reservationOwner = NoWorker
    /\ workerPhase[w] \in {"idle", "observed_absent"}
    /\ reservationOwner' = w
    /\ reservationDigest' = RequestDigest(w)
    /\ logicalStatus' = "reserved"
    /\ workerPhase' = [workerPhase EXCEPT ![w] = "reserved"]
    /\ UNCHANGED
        << grants, grantForKey, grantDigests, grantStates, revokedGrants,
           workerGrant >>

WaitForReservation(w) ==
    /\ logicalStatus = "reserved"
    /\ reservationOwner # w
    /\ workerPhase[w] = "observed_absent"
    /\ workerPhase' = [workerPhase EXCEPT ![w] = "waiting"]
    /\ UNCHANGED
        << reservationOwner, reservationDigest, logicalStatus, grants,
           grantForKey, grantDigests, grantStates, revokedGrants,
           workerGrant >>

CreateGrantUnderReservation(w, g) ==
    /\ UseAtomicReservation
    /\ reservationOwner = w
    /\ logicalStatus = "reserved"
    /\ g \in CandidateGrants
    /\ g \notin grants
    /\ grants' = grants \cup {g}
    /\ grantForKey' = g
    /\ grantDigests' = [grantDigests EXCEPT ![g] = reservationDigest]
    /\ grantStates' = [grantStates EXCEPT ![g] = "active"]
    /\ logicalStatus' = "created"
    /\ workerPhase' = [workerPhase EXCEPT ![w] = "ack_lost"]
    /\ workerGrant' = [workerGrant EXCEPT ![w] = g]
    /\ UNCHANGED
        << reservationOwner, reservationDigest, revokedGrants >>

UnsafeCreateAfterStaleAbsence(w, g) ==
    /\ ~UseAtomicReservation
    /\ workerPhase[w] = "observed_absent"
    /\ g \in CandidateGrants
    /\ g \notin grants
    /\ grants' = grants \cup {g}
    /\ grantForKey' = g
    /\ grantDigests' = [grantDigests EXCEPT ![g] =
          IF reservationDigest = NoDigest
          THEN RequestDigest(w)
          ELSE reservationDigest]
    /\ grantStates' = [grantStates EXCEPT ![g] = "active"]
    /\ reservationDigest' =
          IF reservationDigest = NoDigest
          THEN RequestDigest(w)
          ELSE reservationDigest
    /\ logicalStatus' = "created"
    /\ workerPhase' = [workerPhase EXCEPT ![w] = "ack_lost"]
    /\ workerGrant' = [workerGrant EXCEPT ![w] = g]
    /\ UNCHANGED
        << reservationOwner, revokedGrants >>

RejectDigestConflict(w) ==
    /\ EnforceDigestBinding
    /\ logicalStatus \in {"reserved", "created", "revoked"}
    /\ RequestDigest(w) # reservationDigest
    /\ workerPhase[w] \in {"idle", "observed_absent", "waiting", "ack_lost"}
    /\ workerPhase' = [workerPhase EXCEPT ![w] = "conflict"]
    /\ UNCHANGED
        << reservationOwner, reservationDigest, logicalStatus, grants,
           grantForKey, grantDigests, grantStates, revokedGrants,
           workerGrant >>

RetryExistingGrant(w) ==
    /\ logicalStatus = "created"
    /\ grantForKey \in grants
    /\ workerPhase[w] \in {"idle", "observed_absent", "waiting", "ack_lost"}
    /\ (~EnforceDigestBinding \/ RequestDigest(w) = reservationDigest)
    /\ workerPhase' = [workerPhase EXCEPT ![w] = "acknowledged"]
    /\ workerGrant' = [workerGrant EXCEPT ![w] = grantForKey]
    /\ UNCHANGED
        << reservationOwner, reservationDigest, logicalStatus, grants,
           grantForKey, grantDigests, grantStates, revokedGrants >>

RevokeCanonicalGrant ==
    /\ logicalStatus = "created"
    /\ grantForKey \in grants
    /\ grantStates[grantForKey] = "active"
    /\ logicalStatus' = "revoked"
    /\ grantStates' = [grantStates EXCEPT ![grantForKey] = "revoked"]
    /\ revokedGrants' = revokedGrants \cup {grantForKey}
    /\ UNCHANGED
        << reservationOwner, reservationDigest, grants, grantForKey,
           grantDigests, workerPhase, workerGrant >>

RetryAfterRevocationSafely(w) ==
    /\ EnforceRevocationMonotonic
    /\ logicalStatus = "revoked"
    /\ workerPhase[w] \in {"idle", "observed_absent", "waiting", "ack_lost"}
    /\ workerPhase' = [workerPhase EXCEPT ![w] = "revoked_result"]
    /\ workerGrant' = [workerGrant EXCEPT ![w] = grantForKey]
    /\ UNCHANGED
        << reservationOwner, reservationDigest, logicalStatus, grants,
           grantForKey, grantDigests, grantStates, revokedGrants >>

UnsafeResurrectionAfterRevocation(w, g) ==
    /\ ~EnforceRevocationMonotonic
    /\ logicalStatus = "revoked"
    /\ workerPhase[w] \in {"idle", "observed_absent", "waiting", "ack_lost"}
    /\ g \in CandidateGrants
    /\ g \notin grants
    /\ grants' = grants \cup {g}
    /\ grantForKey' = g
    /\ grantDigests' = [grantDigests EXCEPT ![g] = reservationDigest]
    /\ grantStates' = [grantStates EXCEPT ![g] = "active"]
    /\ logicalStatus' = "created"
    /\ workerPhase' = [workerPhase EXCEPT ![w] = "ack_lost"]
    /\ workerGrant' = [workerGrant EXCEPT ![w] = g]
    /\ UNCHANGED
        << reservationOwner, reservationDigest, revokedGrants >>

Next ==
    \/ \E w \in Workers : ObserveAbsence(w)
    \/ \E w \in Workers : ReserveKeyAtomically(w)
    \/ \E w \in Workers : WaitForReservation(w)
    \/ \E w \in Workers, g \in CandidateGrants : CreateGrantUnderReservation(w, g)
    \/ \E w \in Workers, g \in CandidateGrants : UnsafeCreateAfterStaleAbsence(w, g)
    \/ \E w \in Workers : RejectDigestConflict(w)
    \/ \E w \in Workers : RetryExistingGrant(w)
    \/ RevokeCanonicalGrant
    \/ \E w \in Workers : RetryAfterRevocationSafely(w)
    \/ \E w \in Workers, g \in CandidateGrants : UnsafeResurrectionAfterRevocation(w, g)

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ reservationOwner \in Workers \cup {NoWorker}
    /\ reservationDigest \in Digests \cup {NoDigest}
    /\ logicalStatus \in {"absent", "reserved", "created", "revoked"}
    /\ grants \subseteq CandidateGrants
    /\ grantForKey \in CandidateGrants \cup {NoGrant}
    /\ grantDigests \in [CandidateGrants -> (Digests \cup {NoDigest})]
    /\ grantStates \in [CandidateGrants -> {"absent", "active", "revoked"}]
    /\ revokedGrants \subseteq CandidateGrants
    /\ workerPhase \in [Workers ->
        {"idle", "observed_absent", "reserved", "waiting", "ack_lost",
         "acknowledged", "conflict", "revoked_result"}]
    /\ workerGrant \in [Workers -> (CandidateGrants \cup {NoGrant})]

AtMostOneGrantPerKey ==
    Cardinality(grants) <= 1

ReservationBeforeCreation ==
    grants # {} => reservationOwner \in Workers

GrantDigestMatchesReservation ==
    \A g \in grants : grantDigests[g] = reservationDigest

CanonicalGrantExists ==
    logicalStatus \in {"created", "revoked"} => grantForKey \in grants

ConflictingRequestNeverSucceeds ==
    \A w \in Workers :
        (RequestDigest(w) # reservationDigest
         /\ workerPhase[w] = "acknowledged") => FALSE

NoResurrectionAfterRevocation ==
    revokedGrants # {} =>
        \A g \in grants : grantStates[g] = "revoked"

AcknowledgedWorkerGetsCanonicalGrant ==
    \A w \in Workers :
        workerPhase[w] \in {"acknowledged", "revoked_result"}
        => workerGrant[w] = grantForKey

NoGrantBeforeReservation ==
    logicalStatus = "absent" => grants = {}

====
