---------------- MODULE PCOEAuthorityEpochTransition ----------------
EXTENDS Naturals, FiniteSets, TLC

CONSTANTS
  Witnesses, OldWitnesses, NewWitnesses, Faulty,
  OldQuorum, NewQuorum, DigestA, DigestB,
  AllowUnsafeOldOnlyCertificate,
  AllowUnsafeNewOnlyCertificate,
  AllowUnsafeDirectSwitch,
  AllowUnsafeJournalLoss,
  AllowUnsafeDigestRewrite,
  AllowUnsafeOldPostBarrier,
  AllowUnsafeNewPreActivation

NoDigest == "NoTransitionDigest"
Phases == {"StableOld", "JointPrepared", "JointCertified", "StableNew"}

ASSUME OldWitnesses \subseteq Witnesses
ASSUME NewWitnesses \subseteq Witnesses
ASSUME Cardinality(OldWitnesses) = 4
ASSUME Cardinality(NewWitnesses) = 4
ASSUME Cardinality(Faulty) <= 1
ASSUME Faulty \subseteq Witnesses
ASSUME OldQuorum = 3
ASSUME NewQuorum = 3
ASSUME DigestA # DigestB
ASSUME DigestA # NoDigest
ASSUME DigestB # NoDigest
ASSUME AllowUnsafeOldOnlyCertificate \in BOOLEAN
ASSUME AllowUnsafeNewOnlyCertificate \in BOOLEAN
ASSUME AllowUnsafeDirectSwitch \in BOOLEAN
ASSUME AllowUnsafeJournalLoss \in BOOLEAN
ASSUME AllowUnsafeDigestRewrite \in BOOLEAN
ASSUME AllowUnsafeOldPostBarrier \in BOOLEAN
ASSUME AllowUnsafeNewPreActivation \in BOOLEAN

HonestWitnesses == Witnesses \ Faulty
VARIABLES
  phase,
  proposalDigest,
  lockedDigest,
  votes,
  oldSignersA,
  oldSignersB,
  newSignersA,
  newSignersB,
  certificateA,
  certificateB,
  oldEpochClosed,
  newEpochActive,
  oldEffectA,
  oldEffectB,
  newEffectA,
  newEffectB,
  spendableA,
  spendableB,
  oldPostBarrierEffect,
  newPreActivationEffect

AllSignersA == oldSignersA \\cup newSignersA
AllSignersB == oldSignersB \\cup newSignersB

vars ==
  <<phase, proposalDigest, lockedDigest, votes,
    oldSignersA, oldSignersB, newSignersA, newSignersB,
    certificateA, certificateB, oldEpochClosed, newEpochActive,
    oldEffectA, oldEffectB, newEffectA, newEffectB,
    spendableA, spendableB, oldPostBarrierEffect, newPreActivationEffect>>

Init ==
  /\ phase = "StableOld"
  /\ proposalDigest = NoDigest
  /\ lockedDigest = NoDigest
  /\ votes = [w \in Witnesses |-> {}]
  /\ oldSignersA = {}
  /\ oldSignersB = {}
  /\ newSignersA = {}
  /\ newSignersB = {}
  /\ certificateA = FALSE
  /\ certificateB = FALSE
  /\ oldEpochClosed = FALSE
  /\ newEpochActive = FALSE
  /\ oldEffectA = FALSE
  /\ oldEffectB = FALSE
  /\ newEffectA = FALSE
  /\ newEffectB = FALSE
  /\ spendableA = FALSE
  /\ spendableB = FALSE
  /\ oldPostBarrierEffect = FALSE
  /\ newPreActivationEffect = FALSE

PrepareTransitionA ==
  /\ phase = "StableOld"
  /\ proposalDigest = NoDigest
  /\ lockedDigest = NoDigest
  /\ phase' = "JointPrepared"
  /\ proposalDigest' = DigestA
  /\ lockedDigest' = DigestA
  /\ UNCHANGED <<votes, oldSignersA, oldSignersB, newSignersA, newSignersB,
                 certificateA, certificateB, oldEpochClosed, newEpochActive,
                 oldEffectA, oldEffectB, newEffectA, newEffectB,
                 spendableA, spendableB, oldPostBarrierEffect, newPreActivationEffect>>

PrepareTransitionB ==
  /\ phase = "StableOld"
  /\ proposalDigest = NoDigest
  /\ lockedDigest = NoDigest
  /\ phase' = "JointPrepared"
  /\ proposalDigest' = DigestB
  /\ lockedDigest' = DigestB
  /\ UNCHANGED <<votes, oldSignersA, oldSignersB, newSignersA, newSignersB,
                 certificateA, certificateB, oldEpochClosed, newEpochActive,
                 oldEffectA, oldEffectB, newEffectA, newEffectB,
                 spendableA, spendableB, oldPostBarrierEffect, newPreActivationEffect>>

\* A logical witness has one vote journal regardless of membership in both sets.
\* A single authenticated signature counts once in each set in which that
\* logical identity is a member; it is never counted twice within one set.
SignTransitionA(w) ==
  /\ phase = "JointPrepared"
  /\ proposalDigest = DigestA
  /\ w \in (OldWitnesses \cup NewWitnesses)
  /\ DigestA \notin votes[w]
  /\ (w \in Faulty \/ votes[w] = {})
  /\ votes' = [votes EXCEPT ![w] = @ \cup {DigestA}]
  /\ oldSignersA' = oldSignersA \cup (OldWitnesses \cap {w})
  /\ newSignersA' = newSignersA \cup (NewWitnesses \cap {w})
  /\ UNCHANGED <<phase, proposalDigest, lockedDigest,
                 oldSignersB, newSignersB, certificateA, certificateB,
                 oldEpochClosed, newEpochActive, oldEffectA, oldEffectB,
                 newEffectA, newEffectB, spendableA, spendableB,
                 oldPostBarrierEffect, newPreActivationEffect>>

SignTransitionB(w) ==
  /\ phase = "JointPrepared"
  /\ proposalDigest = DigestB
  /\ w \in (OldWitnesses \cup NewWitnesses)
  /\ DigestB \notin votes[w]
  /\ (w \in Faulty \/ votes[w] = {})
  /\ votes' = [votes EXCEPT ![w] = @ \cup {DigestB}]
  /\ oldSignersB' = oldSignersB \cup (OldWitnesses \cap {w})
  /\ newSignersB' = newSignersB \cup (NewWitnesses \cap {w})
  /\ UNCHANGED <<phase, proposalDigest, lockedDigest,
                 oldSignersA, newSignersA, certificateA, certificateB,
                 oldEpochClosed, newEpochActive, oldEffectA, oldEffectB,
                 newEffectA, newEffectB, spendableA, spendableB,
                 oldPostBarrierEffect, newPreActivationEffect>>

CertifyJointA ==
  /\ phase = "JointPrepared"
  /\ proposalDigest = DigestA
  /\ Cardinality(oldSignersA) >= OldQuorum
  /\ Cardinality(newSignersA) >= NewQuorum
  /\ certificateA' = TRUE
  /\ phase' = "JointCertified"
  /\ UNCHANGED <<proposalDigest, lockedDigest, votes,
                 oldSignersA, oldSignersB, newSignersA, newSignersB,
                 certificateB, oldEpochClosed, newEpochActive,
                 oldEffectA, oldEffectB, newEffectA, newEffectB,
                 spendableA, spendableB, oldPostBarrierEffect, newPreActivationEffect>>

CertifyJointB ==
  /\ phase = "JointPrepared"
  /\ proposalDigest = DigestB
  /\ Cardinality(oldSignersB) >= OldQuorum
  /\ Cardinality(newSignersB) >= NewQuorum
  /\ certificateB' = TRUE
  /\ phase' = "JointCertified"
  /\ UNCHANGED <<proposalDigest, lockedDigest, votes,
                 oldSignersA, oldSignersB, newSignersA, newSignersB,
                 certificateA, oldEpochClosed, newEpochActive,
                 oldEffectA, oldEffectB, newEffectA, newEffectB,
                 spendableA, spendableB, oldPostBarrierEffect, newPreActivationEffect>>

ActivateNewEpoch ==
  /\ phase = "JointCertified"
  /\ (certificateA \/ certificateB)
  /\ oldEpochClosed = FALSE
  /\ newEpochActive = FALSE
  /\ phase' = "StableNew"
  /\ oldEpochClosed' = TRUE
  /\ newEpochActive' = TRUE
  /\ UNCHANGED <<proposalDigest, lockedDigest, votes,
                 oldSignersA, oldSignersB, newSignersA, newSignersB,
                 certificateA, certificateB, oldEffectA, oldEffectB,
                 newEffectA, newEffectB, spendableA, spendableB,
                 oldPostBarrierEffect, newPreActivationEffect>>

\* The previous effect record is preserved through activation. A new epoch
\* cannot create a second, conflicting owner effect for the same singleton key.
AdmitOldEffectA ==
  /\ phase = "StableOld"
  /\ ~oldEffectA /\ ~oldEffectB /\ ~newEffectA /\ ~newEffectB
  /\ oldEffectA' = TRUE
  /\ spendableA' = TRUE
  /\ UNCHANGED <<phase, proposalDigest, lockedDigest, votes,
                 oldSignersA, oldSignersB, newSignersA, newSignersB,
                 certificateA, certificateB, oldEpochClosed, newEpochActive,
                 oldEffectB, newEffectA, newEffectB, spendableB,
                 oldPostBarrierEffect, newPreActivationEffect>>

AdmitOldEffectB ==
  /\ phase = "StableOld"
  /\ ~oldEffectA /\ ~oldEffectB /\ ~newEffectA /\ ~newEffectB
  /\ oldEffectB' = TRUE
  /\ spendableB' = TRUE
  /\ UNCHANGED <<phase, proposalDigest, lockedDigest, votes,
                 oldSignersA, oldSignersB, newSignersA, newSignersB,
                 certificateA, certificateB, oldEpochClosed, newEpochActive,
                 oldEffectA, newEffectA, newEffectB, spendableA,
                 oldPostBarrierEffect, newPreActivationEffect>>

AdmitNewEffectA ==
  /\ phase = "StableNew"
  /\ newEpochActive = TRUE
  /\ ~oldEffectA /\ ~oldEffectB /\ ~newEffectA /\ ~newEffectB
  /\ newEffectA' = TRUE
  /\ spendableA' = TRUE
  /\ UNCHANGED <<phase, proposalDigest, lockedDigest, votes,
                 oldSignersA, oldSignersB, newSignersA, newSignersB,
                 certificateA, certificateB, oldEpochClosed, newEpochActive,
                 oldEffectA, oldEffectB, newEffectB, spendableB,
                 oldPostBarrierEffect, newPreActivationEffect>>

AdmitNewEffectB ==
  /\ phase = "StableNew"
  /\ newEpochActive = TRUE
  /\ ~oldEffectA /\ ~oldEffectB /\ ~newEffectA /\ ~newEffectB
  /\ newEffectB' = TRUE
  /\ spendableB' = TRUE
  /\ UNCHANGED <<phase, proposalDigest, lockedDigest, votes,
                 oldSignersA, oldSignersB, newSignersA, newSignersB,
                 certificateA, certificateB, oldEpochClosed, newEpochActive,
                 oldEffectA, oldEffectB, newEffectA, spendableA,
                 oldPostBarrierEffect, newPreActivationEffect>>

UnsafeOldOnlyCertificateA ==
  /\ AllowUnsafeOldOnlyCertificate = TRUE
  /\ phase = "JointPrepared"
  /\ proposalDigest = DigestA
  /\ Cardinality(oldSignersA) >= OldQuorum
  /\ Cardinality(newSignersA) < NewQuorum
  /\ certificateA' = TRUE
  /\ phase' = "JointCertified"
  /\ UNCHANGED <<proposalDigest, lockedDigest, votes,
                 oldSignersA, oldSignersB, newSignersA, newSignersB,
                 certificateB, oldEpochClosed, newEpochActive,
                 oldEffectA, oldEffectB, newEffectA, newEffectB,
                 spendableA, spendableB, oldPostBarrierEffect, newPreActivationEffect>>

UnsafeNewOnlyCertificateA ==
  /\ AllowUnsafeNewOnlyCertificate = TRUE
  /\ phase = "JointPrepared"
  /\ proposalDigest = DigestA
  /\ Cardinality(newSignersA) >= NewQuorum
  /\ Cardinality(oldSignersA) < OldQuorum
  /\ certificateA' = TRUE
  /\ phase' = "JointCertified"
  /\ UNCHANGED <<proposalDigest, lockedDigest, votes,
                 oldSignersA, oldSignersB, newSignersA, newSignersB,
                 certificateB, oldEpochClosed, newEpochActive,
                 oldEffectA, oldEffectB, newEffectA, newEffectB,
                 spendableA, spendableB, oldPostBarrierEffect, newPreActivationEffect>>

UnsafeActivateWithoutJointCertificate ==
  /\ AllowUnsafeDirectSwitch = TRUE
  /\ phase = "StableOld"
  /\ phase' = "StableNew"
  /\ oldEpochClosed' = TRUE
  /\ newEpochActive' = TRUE
  /\ UNCHANGED <<proposalDigest, lockedDigest, votes,
                 oldSignersA, oldSignersB, newSignersA, newSignersB,
                 certificateA, certificateB, oldEffectA, oldEffectB,
                 newEffectA, newEffectB, spendableA, spendableB,
                 oldPostBarrierEffect, newPreActivationEffect>>

UnsafeRewriteProposalDigest ==
  /\ AllowUnsafeDigestRewrite = TRUE
  /\ phase = "JointPrepared"
  /\ proposalDigest = DigestA
  /\ lockedDigest = DigestA
  /\ proposalDigest' = DigestB
  /\ UNCHANGED <<phase, lockedDigest, votes,
                 oldSignersA, oldSignersB, newSignersA, newSignersB,
                 certificateA, certificateB, oldEpochClosed, newEpochActive,
                 oldEffectA, oldEffectB, newEffectA, newEffectB,
                 spendableA, spendableB, oldPostBarrierEffect, newPreActivationEffect>>

UnsafeForgetHonestJournal(w) ==
  /\ AllowUnsafeJournalLoss = TRUE
  /\ w \in HonestWitnesses
  /\ votes[w] # {}
  /\ votes' = [votes EXCEPT ![w] = {}]
  /\ UNCHANGED <<phase, proposalDigest, lockedDigest,
                 oldSignersA, oldSignersB, newSignersA, newSignersB,
                 certificateA, certificateB, oldEpochClosed, newEpochActive,
                 oldEffectA, oldEffectB, newEffectA, newEffectB,
                 spendableA, spendableB, oldPostBarrierEffect, newPreActivationEffect>>

UnsafeOldEffectAfterBarrier ==
  /\ AllowUnsafeOldPostBarrier = TRUE
  /\ phase = "StableNew"
  /\ oldEpochClosed = TRUE
  /\ oldPostBarrierEffect = FALSE
  /\ oldPostBarrierEffect' = TRUE
  /\ UNCHANGED <<phase, proposalDigest, lockedDigest, votes,
                 oldSignersA, oldSignersB, newSignersA, newSignersB,
                 certificateA, certificateB, oldEpochClosed, newEpochActive,
                 oldEffectA, oldEffectB, newEffectA, newEffectB,
                 spendableA, spendableB, newPreActivationEffect>>

UnsafeNewEffectBeforeActivation ==
  /\ AllowUnsafeNewPreActivation = TRUE
  /\ phase # "StableNew"
  /\ newEpochActive = FALSE
  /\ newPreActivationEffect = FALSE
  /\ newPreActivationEffect' = TRUE
  /\ UNCHANGED <<phase, proposalDigest, lockedDigest, votes,
                 oldSignersA, oldSignersB, newSignersA, newSignersB,
                 certificateA, certificateB, oldEpochClosed, newEpochActive,
                 oldEffectA, oldEffectB, newEffectA, newEffectB,
                 spendableA, spendableB, oldPostBarrierEffect>>

Next ==
  \/ PrepareTransitionA
  \/ PrepareTransitionB
  \/ \E w \in (OldWitnesses \cup NewWitnesses) : SignTransitionA(w)
  \/ \E w \in (OldWitnesses \cup NewWitnesses) : SignTransitionB(w)
  \/ CertifyJointA
  \/ CertifyJointB
  \/ ActivateNewEpoch
  \/ AdmitOldEffectA
  \/ AdmitOldEffectB
  \/ AdmitNewEffectA
  \/ AdmitNewEffectB
  \/ UnsafeOldOnlyCertificateA
  \/ UnsafeNewOnlyCertificateA
  \/ UnsafeActivateWithoutJointCertificate
  \/ UnsafeRewriteProposalDigest
  \/ \E w \in HonestWitnesses : UnsafeForgetHonestJournal(w)
  \/ UnsafeOldEffectAfterBarrier
  \/ UnsafeNewEffectBeforeActivation

Spec == Init /\ [][Next]_vars

TypeOK ==
  /\ phase \in Phases
  /\ proposalDigest \in {NoDigest, DigestA, DigestB}
  /\ lockedDigest \in {NoDigest, DigestA, DigestB}
  /\ votes \in [Witnesses -> SUBSET {DigestA, DigestB}]
  /\ oldSignersA \subseteq OldWitnesses
  /\ oldSignersB \subseteq OldWitnesses
  /\ newSignersA \subseteq NewWitnesses
  /\ newSignersB \subseteq NewWitnesses
  /\ certificateA \in BOOLEAN
  /\ certificateB \in BOOLEAN
  /\ oldEpochClosed \in BOOLEAN
  /\ newEpochActive \in BOOLEAN
  /\ oldEffectA \in BOOLEAN
  /\ oldEffectB \in BOOLEAN
  /\ newEffectA \in BOOLEAN
  /\ newEffectB \in BOOLEAN
  /\ spendableA \in BOOLEAN
  /\ spendableB \in BOOLEAN
  /\ oldPostBarrierEffect \in BOOLEAN
  /\ newPreActivationEffect \in BOOLEAN

QuorumIntersectionBound ==
  /\ 2 * OldQuorum - Cardinality(OldWitnesses) >= Cardinality(Faulty \cap OldWitnesses) + 1
  /\ 2 * NewQuorum - Cardinality(NewWitnesses) >= Cardinality(Faulty \cap NewWitnesses) + 1

TransitionDigestImmutable ==
  (lockedDigest = NoDigest) \/ (proposalDigest = lockedDigest)

HonestNonEquivocation ==
  \A w \in HonestWitnesses : ~(w \in AllSignersA /\ w \in AllSignersB)

JointCertificateRequiresBothQuorums ==
  /\ (certificateA =>
        /\ proposalDigest = DigestA
        /\ Cardinality(oldSignersA) >= OldQuorum
        /\ Cardinality(newSignersA) >= NewQuorum)
  /\ (certificateB =>
        /\ proposalDigest = DigestB
        /\ Cardinality(oldSignersB) >= OldQuorum
        /\ Cardinality(newSignersB) >= NewQuorum)

NoConflictingTransitionCertificates ==
  ~(certificateA /\ certificateB)

NewEpochActivationRequiresJointCertificate ==
  /\ (newEpochActive => (phase = "StableNew" /\ (certificateA \/ certificateB)))
  /\ (oldEpochClosed => newEpochActive)

OldEpochClosedAfterActivation ==
  /\ (newEpochActive => oldEpochClosed)
  /\ ~oldPostBarrierEffect

NewAdmissionRequiresActivation ==
  /\ (newEffectA \/ newEffectB \/ newPreActivationEffect) => newEpochActive

NoConflictingOwnerEffects ==
  ~((oldEffectA \/ newEffectA) /\ (oldEffectB \/ newEffectB))

NoDuplicateOwnerEffects ==
  /\ ~(oldEffectA /\ newEffectA)
  /\ ~(oldEffectB /\ newEffectB)

SpendabilityRequiresOwnerEffect ==
  /\ (spendableA => (oldEffectA \/ newEffectA))
  /\ (spendableB => (oldEffectB \/ newEffectB))

NoConflictingSpendableEffects ==
  ~(spendableA /\ spendableB)

Safety ==
  /\ TypeOK
  /\ QuorumIntersectionBound
  /\ TransitionDigestImmutable
  /\ HonestNonEquivocation
  /\ JointCertificateRequiresBothQuorums
  /\ NoConflictingTransitionCertificates
  /\ NewEpochActivationRequiresJointCertificate
  /\ OldEpochClosedAfterActivation
  /\ NewAdmissionRequiresActivation
  /\ NoConflictingOwnerEffects
  /\ NoDuplicateOwnerEffects
  /\ SpendabilityRequiresOwnerEffect
  /\ NoConflictingSpendableEffects

====
