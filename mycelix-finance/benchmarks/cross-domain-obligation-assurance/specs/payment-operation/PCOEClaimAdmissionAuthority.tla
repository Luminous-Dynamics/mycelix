---------------- MODULE PCOEClaimAdmissionAuthority ----------------
EXTENDS Naturals, FiniteSets, TLC

CONSTANTS Witnesses, Faulty, Quorum, FingerprintA, FingerprintB,
          AllowUnsafeMinorityAcceptance, AllowUnsafeJournalLoss

ASSUME Cardinality(Witnesses) = 4
ASSUME Faulty \subseteq Witnesses
ASSUME Cardinality(Faulty) <= 1
ASSUME Quorum = 3
ASSUME FingerprintA # FingerprintB
ASSUME AllowUnsafeMinorityAcceptance \in BOOLEAN
ASSUME AllowUnsafeJournalLoss \in BOOLEAN

HonestWitnesses == Witnesses \ Faulty

VARIABLES
  votes,
  signersA,
  signersB,
  certificateA,
  certificateB,
  ownerEffectA,
  ownerEffectB,
  spendableA,
  spendableB

vars ==
  <<votes, signersA, signersB, certificateA, certificateB,
    ownerEffectA, ownerEffectB, spendableA, spendableB>>

Init ==
  /\ votes = [w \in Witnesses |-> {}]
  /\ signersA = {}
  /\ signersB = {}
  /\ certificateA = FALSE
  /\ certificateB = FALSE
  /\ ownerEffectA = FALSE
  /\ ownerEffectB = FALSE
  /\ spendableA = FALSE
  /\ spendableB = FALSE

SignA(w) ==
  /\ w \in Witnesses
  /\ FingerprintA \notin votes[w]
  /\ (w \in Faulty \/ votes[w] = {})
  /\ votes' = [votes EXCEPT ![w] = @ \cup {FingerprintA}]
  /\ signersA' = signersA \cup {w}
  /\ UNCHANGED <<signersB, certificateA, certificateB,
                 ownerEffectA, ownerEffectB, spendableA, spendableB>>

SignB(w) ==
  /\ w \in Witnesses
  /\ FingerprintB \notin votes[w]
  /\ (w \in Faulty \/ votes[w] = {})
  /\ votes' = [votes EXCEPT ![w] = @ \cup {FingerprintB}]
  /\ signersB' = signersB \cup {w}
  /\ UNCHANGED <<signersA, certificateA, certificateB,
                 ownerEffectA, ownerEffectB, spendableA, spendableB>>

CertifyA ==
  /\ certificateA = FALSE
  /\ Cardinality(signersA) >= Quorum
  /\ certificateA' = TRUE
  /\ UNCHANGED <<votes, signersA, signersB, certificateB,
                 ownerEffectA, ownerEffectB, spendableA, spendableB>>

CertifyB ==
  /\ certificateB = FALSE
  /\ Cardinality(signersB) >= Quorum
  /\ certificateB' = TRUE
  /\ UNCHANGED <<votes, signersA, signersB, certificateA,
                 ownerEffectA, ownerEffectB, spendableA, spendableB>>

CommitOwnerEffectA ==
  /\ certificateA = TRUE
  /\ certificateB = FALSE
  /\ ownerEffectA = FALSE
  /\ ownerEffectA' = TRUE
  /\ spendableA' = TRUE
  /\ UNCHANGED <<votes, signersA, signersB, certificateA, certificateB,
                 ownerEffectB, spendableB>>

CommitOwnerEffectB ==
  /\ certificateB = TRUE
  /\ certificateA = FALSE
  /\ ownerEffectB = FALSE
  /\ ownerEffectB' = TRUE
  /\ spendableB' = TRUE
  /\ UNCHANGED <<votes, signersA, signersB, certificateA, certificateB,
                 ownerEffectA, spendableA>>

\* Deliberate unsafe mutant: accept a certificate with one fewer than the
\* declared quorum. The intended counterexample is CertificateQuorum.
UnsafeCertifyAWithMinority ==
  /\ AllowUnsafeMinorityAcceptance = TRUE
  /\ certificateA = FALSE
  /\ Cardinality(signersA) = Quorum - 1
  /\ certificateA' = TRUE
  /\ UNCHANGED <<votes, signersA, signersB, certificateB,
                 ownerEffectA, ownerEffectB, spendableA, spendableB>>

UnsafeCertifyBWithMinority ==
  /\ AllowUnsafeMinorityAcceptance = TRUE
  /\ certificateB = FALSE
  /\ Cardinality(signersB) = Quorum - 1
  /\ certificateB' = TRUE
  /\ UNCHANGED <<votes, signersA, signersB, certificateA,
                 ownerEffectA, ownerEffectB, spendableA, spendableB>>

\* Deliberate durability mutant: a witness loses its non-equivocation journal
\* while its previously issued signature remains part of the public certificate
\* evidence. The witness can then sign the other fingerprint.
UnsafeForgetHonestJournal(w) ==
  /\ AllowUnsafeJournalLoss = TRUE
  /\ w \in HonestWitnesses
  /\ votes[w] # {}
  /\ votes' = [votes EXCEPT ![w] = {}]
  /\ UNCHANGED <<signersA, signersB, certificateA, certificateB,
                 ownerEffectA, ownerEffectB, spendableA, spendableB>>

Next ==
  \/ \E w \in Witnesses : SignA(w)
  \/ \E w \in Witnesses : SignB(w)
  \/ CertifyA
  \/ CertifyB
  \/ CommitOwnerEffectA
  \/ CommitOwnerEffectB
  \/ UnsafeCertifyAWithMinority
  \/ UnsafeCertifyBWithMinority
  \/ \E w \in HonestWitnesses : UnsafeForgetHonestJournal(w)

Spec == Init /\ [][Next]_vars

TypeOK ==
  /\ votes \in [Witnesses -> SUBSET {FingerprintA, FingerprintB}]
  /\ signersA \subseteq Witnesses
  /\ signersB \subseteq Witnesses
  /\ certificateA \in BOOLEAN
  /\ certificateB \in BOOLEAN
  /\ ownerEffectA \in BOOLEAN
  /\ ownerEffectB \in BOOLEAN
  /\ spendableA \in BOOLEAN
  /\ spendableB \in BOOLEAN

\* For n=3f+1 and q=2f+1, any two q-sized quorums intersect in at least
\* f+1 witnesses. With at most f Byzantine witnesses, at least one member of
\* that intersection is honest and must durably refuse a conflicting vote.
QuorumIntersectionBound ==
  2 * Quorum - Cardinality(Witnesses) >= Cardinality(Faulty) + 1

CertificateQuorum ==
  /\ (certificateA => Cardinality(signersA) >= Quorum)
  /\ (certificateB => Cardinality(signersB) >= Quorum)

HonestNonEquivocation ==
  \A w \in HonestWitnesses :
    ~(w \in signersA /\ w \in signersB)

NoConflictingCertificates ==
  ~(certificateA /\ certificateB)

EffectRequiresCertificate ==
  /\ (ownerEffectA => certificateA)
  /\ (ownerEffectB => certificateB)

SpendabilityRequiresOwnerEffect ==
  /\ (spendableA => ownerEffectA)
  /\ (spendableB => ownerEffectB)

NoConflictingSpendableEffects ==
  ~(spendableA /\ spendableB)

SpendabilityRequiresExclusiveCertificate ==
  /\ (spendableA => ~certificateB)
  /\ (spendableB => ~certificateA)

Safety ==
  /\ TypeOK
  /\ QuorumIntersectionBound
  /\ CertificateQuorum
  /\ HonestNonEquivocation
  /\ NoConflictingCertificates
  /\ EffectRequiresCertificate
  /\ SpendabilityRequiresOwnerEffect
  /\ NoConflictingSpendableEffects
  /\ SpendabilityRequiresExclusiveCertificate

====
