---------------- MODULE OwnerEpochExecutionAttestation ----------------
EXTENDS Naturals, FiniteSets, TLC

CONSTANTS
    Workers,
    Operations,
    MaxEpoch,
    MaxGeneration,
    MaxClaimEpoch,
    MaxLeaseGeneration,
    TrustedEpochProofs,
    TrustedFenceProofs

ASSUME /\ Workers # {}
       /\ Operations # {}
       /\ MaxEpoch \in Nat \ {0}
       /\ MaxGeneration \in Nat \ {0}
       /\ MaxClaimEpoch \in Nat \ {0}
       /\ MaxLeaseGeneration \in Nat \ {0}
       /\ TrustedEpochProofs # {}
       /\ TrustedFenceProofs # {}

NoClaim == "no-claim"
NoEffect == "no-effect"
NoAttestation == "no-attestation"
NoProof == "no-proof"

EffectRecordSet ==
    [operation : Operations,
     ownerEpoch : 1..MaxEpoch,
     generation : 1..MaxGeneration]

AttestationRecordSet ==
    [operation : Operations,
     ownerEpoch : 1..MaxEpoch,
     generation : 1..MaxGeneration,
     worker : Workers,
     claimEpoch : 1..MaxClaimEpoch]

TransitionRecordSet ==
    [previousEpoch : 1..MaxEpoch,
     newEpoch : 1..MaxEpoch,
     previousGeneration : 1..MaxGeneration,
     newGeneration : 1..MaxGeneration,
     authorityProof : TrustedEpochProofs,
     physicalFenceProof : TrustedFenceProofs]

VARIABLES
    ownerEpoch,
    generation,
    externalHighWater,
    leaseGeneration,
    externalLeaseHighWater,
    quarantined,
    currentPrimaryCanWrite,
    formerPrimaryCanWrite,
    oldPrimaryFenced,
    lastFenceProof,
    workerClaims,
    effects,
    attestations,
    replayResults,
    epochTransitions

vars ==
    << ownerEpoch,
       generation,
       externalHighWater,
       leaseGeneration,
       externalLeaseHighWater,
       quarantined,
       currentPrimaryCanWrite,
       formerPrimaryCanWrite,
       oldPrimaryFenced,
       lastFenceProof,
       workerClaims,
       effects,
       attestations,
       replayResults,
       epochTransitions >>

Init ==
    /\ ownerEpoch = 1
    /\ generation = 1
    /\ externalHighWater = 1
    /\ leaseGeneration = 1
    /\ externalLeaseHighWater = 1
    /\ quarantined = FALSE
    /\ currentPrimaryCanWrite = TRUE
    /\ formerPrimaryCanWrite = FALSE
    /\ oldPrimaryFenced = TRUE
    /\ lastFenceProof = NoProof
    /\ workerClaims = [op \in Operations |-> NoClaim]
    /\ effects = [op \in Operations |-> NoEffect]
    /\ attestations = [op \in Operations |-> NoAttestation]
    /\ replayResults = [op \in Operations |-> NoAttestation]
    /\ epochTransitions = {}

InstallWorkerClaim(op, worker, nextClaimEpoch) ==
    /\ op \in Operations
    /\ worker \in Workers
    /\ currentPrimaryCanWrite
    /\ ~quarantined
    /\ ownerEpoch = externalHighWater
    /\ leaseGeneration = externalLeaseHighWater
    /\ nextClaimEpoch \in 1..MaxClaimEpoch
    /\ IF workerClaims[op] = NoClaim
          THEN nextClaimEpoch > 0
          ELSE nextClaimEpoch > workerClaims[op].claimEpoch
    /\ workerClaims' =
          [workerClaims EXCEPT
              ![op] = [ownerEpoch |-> ownerEpoch,
                       claimEpoch |-> nextClaimEpoch,
                       worker |-> worker]]
    /\ UNCHANGED
          << ownerEpoch, generation, externalHighWater,
             leaseGeneration, externalLeaseHighWater,
             quarantined, currentPrimaryCanWrite,
             formerPrimaryCanWrite, oldPrimaryFenced,
             lastFenceProof, effects, attestations,
             replayResults, epochTransitions >>

FenceCurrentPrimary(proof) ==
    /\ currentPrimaryCanWrite
    /\ proof \in TrustedFenceProofs
    /\ currentPrimaryCanWrite' = FALSE
    /\ oldPrimaryFenced' = TRUE
    /\ lastFenceProof' = proof
    /\ UNCHANGED
          << ownerEpoch, generation, externalHighWater,
             leaseGeneration, externalLeaseHighWater,
             quarantined, formerPrimaryCanWrite,
             workerClaims, effects, attestations,
             replayResults, epochTransitions >>

FenceFormerPrimary(proof) ==
    /\ formerPrimaryCanWrite
    /\ proof \in TrustedFenceProofs
    /\ formerPrimaryCanWrite' = FALSE
    /\ oldPrimaryFenced' = TRUE
    /\ lastFenceProof' = proof
    /\ UNCHANGED
          << ownerEpoch, generation, externalHighWater,
             leaseGeneration, externalLeaseHighWater,
             quarantined, currentPrimaryCanWrite,
             workerClaims, effects, attestations,
             replayResults, epochTransitions >>

AdvanceOwnerEpoch(nextEpoch, nextGeneration, nextLeaseGeneration, authorityProof) ==
    /\ ~currentPrimaryCanWrite
    /\ ~formerPrimaryCanWrite
    /\ oldPrimaryFenced
    /\ lastFenceProof \in TrustedFenceProofs
    /\ authorityProof \in TrustedEpochProofs
    /\ nextEpoch \in 1..MaxEpoch
    /\ nextGeneration \in 1..MaxGeneration
    /\ nextLeaseGeneration \in 1..MaxLeaseGeneration
    /\ nextEpoch > externalHighWater
    /\ nextGeneration > generation
    /\ nextLeaseGeneration > externalLeaseHighWater
    /\ ownerEpoch' = nextEpoch
    /\ generation' = nextGeneration
    /\ externalHighWater' = nextEpoch
    /\ leaseGeneration' = nextLeaseGeneration
    /\ externalLeaseHighWater' = nextLeaseGeneration
    /\ quarantined' = FALSE
    /\ currentPrimaryCanWrite' = TRUE
    /\ epochTransitions' =
          epochTransitions \cup
          { [previousEpoch |-> ownerEpoch,
             newEpoch |-> nextEpoch,
             previousGeneration |-> generation,
             newGeneration |-> nextGeneration,
             authorityProof |-> authorityProof,
             physicalFenceProof |-> lastFenceProof] }
    /\ UNCHANGED
          << formerPrimaryCanWrite, oldPrimaryFenced,
             lastFenceProof, workerClaims, effects,
             attestations, replayResults >>

RestoreStaleBackup(restoredEpoch, restoredGeneration, restoredLeaseGeneration) ==
    /\ currentPrimaryCanWrite
    /\ restoredEpoch \in 1..ownerEpoch
    /\ restoredGeneration \in 1..generation
    /\ restoredLeaseGeneration \in 1..leaseGeneration
    /\ ownerEpoch' = restoredEpoch
    /\ generation' = restoredGeneration
    /\ leaseGeneration' = restoredLeaseGeneration
    /\ quarantined' = TRUE
    /\ currentPrimaryCanWrite' = FALSE
    /\ formerPrimaryCanWrite' = TRUE
    /\ oldPrimaryFenced' = FALSE
    /\ lastFenceProof' = NoProof
    /\ UNCHANGED
          << externalHighWater, externalLeaseHighWater,
             workerClaims, effects, attestations,
             replayResults, epochTransitions >>

CommitEffect(op, worker) ==
    /\ op \in Operations
    /\ worker \in Workers
    /\ currentPrimaryCanWrite
    /\ ~formerPrimaryCanWrite
    /\ ~quarantined
    /\ ownerEpoch = externalHighWater
    /\ leaseGeneration = externalLeaseHighWater
    /\ workerClaims[op] # NoClaim
    /\ workerClaims[op].ownerEpoch = ownerEpoch
    /\ workerClaims[op].worker = worker
    /\ effects[op] = NoEffect
    /\ effects' =
          [effects EXCEPT
              ![op] = [operation |-> op,
                       ownerEpoch |-> ownerEpoch,
                       generation |-> generation]]
    /\ attestations' =
          [attestations EXCEPT
              ![op] = [operation |-> op,
                       ownerEpoch |-> ownerEpoch,
                       generation |-> generation,
                       worker |-> worker,
                       claimEpoch |-> workerClaims[op].claimEpoch]]
    /\ UNCHANGED
          << ownerEpoch, generation, externalHighWater,
             leaseGeneration, externalLeaseHighWater,
             quarantined, currentPrimaryCanWrite,
             formerPrimaryCanWrite, oldPrimaryFenced,
             lastFenceProof, workerClaims,
             replayResults, epochTransitions >>

ReplayEffect(op, worker) ==
    /\ op \in Operations
    /\ worker \in Workers
    /\ currentPrimaryCanWrite
    /\ ~formerPrimaryCanWrite
    /\ ~quarantined
    /\ ownerEpoch = externalHighWater
    /\ leaseGeneration = externalLeaseHighWater
    /\ workerClaims[op] # NoClaim
    /\ workerClaims[op].ownerEpoch = ownerEpoch
    /\ workerClaims[op].worker = worker
    /\ effects[op] # NoEffect
    /\ attestations[op] # NoAttestation
    /\ replayResults' =
          [replayResults EXCEPT ![op] = attestations[op]]
    /\ UNCHANGED
          << ownerEpoch, generation, externalHighWater,
             leaseGeneration, externalLeaseHighWater,
             quarantined, currentPrimaryCanWrite,
             formerPrimaryCanWrite, oldPrimaryFenced,
             lastFenceProof, workerClaims, effects,
             attestations, epochTransitions >>

Next ==
    \/ \E op \in Operations, worker \in Workers,
          claimEpoch \in 1..MaxClaimEpoch :
          InstallWorkerClaim(op, worker, claimEpoch)
    \/ \E proof \in TrustedFenceProofs : FenceCurrentPrimary(proof)
    \/ \E proof \in TrustedFenceProofs : FenceFormerPrimary(proof)
    \/ \E nextEpoch \in 1..MaxEpoch,
          nextGeneration \in 1..MaxGeneration,
          nextLeaseGeneration \in 1..MaxLeaseGeneration,
          authorityProof \in TrustedEpochProofs :
          AdvanceOwnerEpoch(
              nextEpoch, nextGeneration, nextLeaseGeneration, authorityProof)
    \/ \E restoredEpoch \in 1..MaxEpoch,
          restoredGeneration \in 1..MaxGeneration,
          restoredLeaseGeneration \in 1..MaxLeaseGeneration :
          RestoreStaleBackup(
              restoredEpoch, restoredGeneration, restoredLeaseGeneration)
    \/ \E op \in Operations, worker \in Workers :
          CommitEffect(op, worker)
    \/ \E op \in Operations, worker \in Workers :
          ReplayEffect(op, worker)

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ ownerEpoch \in 1..MaxEpoch
    /\ generation \in 1..MaxGeneration
    /\ externalHighWater \in 1..MaxEpoch
    /\ leaseGeneration \in 1..MaxLeaseGeneration
    /\ externalLeaseHighWater \in 1..MaxLeaseGeneration
    /\ quarantined \in BOOLEAN
    /\ currentPrimaryCanWrite \in BOOLEAN
    /\ formerPrimaryCanWrite \in BOOLEAN
    /\ oldPrimaryFenced \in BOOLEAN
    /\ lastFenceProof \in ({NoProof} \cup TrustedFenceProofs)
    /\ workerClaims \in
          [Operations ->
              ({NoClaim} \cup
               [ownerEpoch : 1..MaxEpoch,
                claimEpoch : 1..MaxClaimEpoch,
                worker : Workers])]
    /\ effects \in [Operations -> ({NoEffect} \cup EffectRecordSet)]
    /\ attestations \in
          [Operations -> ({NoAttestation} \cup AttestationRecordSet)]
    /\ replayResults \in
          [Operations -> ({NoAttestation} \cup AttestationRecordSet)]
    /\ epochTransitions \subseteq TransitionRecordSet

NoSplitBrain ==
    ~(currentPrimaryCanWrite /\ formerPrimaryCanWrite)

OwnerAndLeaseHighWater ==
    /\ ownerEpoch <= externalHighWater
    /\ leaseGeneration <= externalLeaseHighWater
    /\ (quarantined
        \/ /\ ownerEpoch = externalHighWater
           /\ leaseGeneration = externalLeaseHighWater)

WritablePrimaryIsAdmitted ==
    currentPrimaryCanWrite =>
      /\ ~quarantined
      /\ ~formerPrimaryCanWrite
      /\ oldPrimaryFenced
      /\ ownerEpoch = externalHighWater
      /\ leaseGeneration = externalLeaseHighWater

EffectsHaveAtomicExecutionAttestations ==
    \A op \in Operations :
      IF effects[op] = NoEffect
      THEN attestations[op] = NoAttestation
      ELSE
        /\ attestations[op] # NoAttestation
        /\ effects[op].operation = attestations[op].operation
        /\ effects[op].ownerEpoch = attestations[op].ownerEpoch
        /\ effects[op].generation = attestations[op].generation
        /\ attestations[op].claimEpoch \in 1..MaxClaimEpoch

ReplayReturnsOriginalAttestation ==
    \A op \in Operations :
      replayResults[op] = NoAttestation
      \/ replayResults[op] = attestations[op]

TransitionEvidenceIsMonotonic ==
    \A transition \in epochTransitions :
      /\ transition.newEpoch > transition.previousEpoch
      /\ transition.newGeneration > transition.previousGeneration

EffectsHaveChronologicalTransitionEvidence ==
    \A op \in Operations :
      IF effects[op] = NoEffect
      THEN TRUE
      ELSE IF effects[op].ownerEpoch = 1
           THEN TRUE
           ELSE
             \E transition \in epochTransitions :
               /\ transition.newEpoch = effects[op].ownerEpoch
               /\ transition.previousEpoch < transition.newEpoch
               /\ transition.previousGeneration < transition.newGeneration
               /\ transition.newGeneration <= effects[op].generation
               /\ transition.authorityProof \in TrustedEpochProofs
               /\ transition.physicalFenceProof \in TrustedFenceProofs

OwnerEpochAdvancesOnlyAfterFencing ==
    []((ownerEpoch' > ownerEpoch) =>
        /\ oldPrimaryFenced
        /\ ~currentPrimaryCanWrite
        /\ ~formerPrimaryCanWrite)

ExternalHighWaterNeverRollsBack ==
    [](externalHighWater' >= externalHighWater
      /\ externalLeaseHighWater' >= externalLeaseHighWater)

NoEffectsDuringQuarantine ==
    [](effects' # effects => ~quarantined /\ currentPrimaryCanWrite)

=============================================================================
