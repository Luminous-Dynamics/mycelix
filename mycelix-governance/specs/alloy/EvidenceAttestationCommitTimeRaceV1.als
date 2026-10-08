module EvidenceAttestationCommitTimeRaceV1

open util/ordering[Step] as StepOrder
open util/ordering[Time] as TimeOrder

enum Bit { On, Off }
enum MutationKind { AuthorityEpochMutation, RequestMutation, PolicyEpochMutation, CapabilityExpiryMutation, NoMutation }

sig Step {}
sig Time {}
sig Epoch {}
sig RequestCommitment {}
sig PolicyEpoch {}

sig Decision {
  authorityEpoch: one Epoch,
  requestCommitment: one RequestCommitment,
  policyEpoch: one PolicyEpoch,
  capabilityExpiry: one Time,
  issuedAt: one Time
}

sig RaceTrace {
  decision: one Decision,
  preflightStep: one Step,
  mutationStep: one Step,
  commitStep: one Step,
  observedAuthorityEpoch: one Epoch,
  observedRequestCommitment: one RequestCommitment,
  observedPolicyEpoch: one PolicyEpoch,
  observedCapabilityExpiry: one Time,
  observedTime: one Time,
  currentAuthorityEpoch: one Epoch,
  currentRequestCommitment: one RequestCommitment,
  currentPolicyEpoch: one PolicyEpoch,
  currentCapabilityExpiry: one Time,
  currentTime: one Time,
  authorized: one Bit,
  committed: one Bit,
  mutation: one MutationKind
}

fact TraceOrdering {
  all r: RaceTrace |
    StepOrder/lt[r.preflightStep, r.mutationStep] and
    StepOrder/lt[r.mutationStep, r.commitStep]
}

fact PreflightSnapshotExact {
  all r: RaceTrace |
    r.observedAuthorityEpoch = r.decision.authorityEpoch and
    r.observedRequestCommitment = r.decision.requestCommitment and
    r.observedPolicyEpoch = r.decision.policyEpoch and
    r.observedCapabilityExpiry = r.decision.capabilityExpiry and
    r.observedTime = r.decision.issuedAt
}

fact CommitOnlyAfterCheck {
  all r: RaceTrace |
    r.committed = On implies r.authorized = On
}

fact CommitRequiresCurrentRevalidation {
  all r: RaceTrace |
    r.committed = On implies
      r.currentAuthorityEpoch = r.observedAuthorityEpoch and
      r.currentRequestCommitment = r.observedRequestCommitment and
      r.currentPolicyEpoch = r.observedPolicyEpoch and
      TimeOrder/lt[r.currentTime, r.currentCapabilityExpiry] and
      TimeOrder/lte[r.observedTime, r.currentTime]
}

pred ValidAtomicCommitWitness {
  some r: RaceTrace |
    r.authorized = On and
    r.committed = On and
    r.mutation = NoMutation and
    r.currentAuthorityEpoch = r.observedAuthorityEpoch and
    r.currentRequestCommitment = r.observedRequestCommitment and
    r.currentPolicyEpoch = r.observedPolicyEpoch and
    TimeOrder/lt[r.currentTime, r.currentCapabilityExpiry]
}

pred AuthorityEpochRaceWitness {
  some r: RaceTrace |
    r.authorized = On and
    r.committed = On and
    r.mutation = AuthorityEpochMutation and
    r.currentAuthorityEpoch != r.observedAuthorityEpoch and
    r.currentRequestCommitment = r.observedRequestCommitment and
    r.currentPolicyEpoch = r.observedPolicyEpoch and
    TimeOrder/lt[r.currentTime, r.currentCapabilityExpiry] and
    TimeOrder/lte[r.observedTime, r.currentTime]
}

pred RequestRaceWitness {
  some r: RaceTrace |
    r.authorized = On and
    r.committed = On and
    r.mutation = RequestMutation and
    r.currentAuthorityEpoch = r.observedAuthorityEpoch and
    r.currentRequestCommitment != r.observedRequestCommitment and
    r.currentPolicyEpoch = r.observedPolicyEpoch and
    TimeOrder/lt[r.currentTime, r.currentCapabilityExpiry] and
    TimeOrder/lte[r.observedTime, r.currentTime]
}

pred PolicyEpochRaceWitness {
  some r: RaceTrace |
    r.authorized = On and
    r.committed = On and
    r.mutation = PolicyEpochMutation and
    r.currentAuthorityEpoch = r.observedAuthorityEpoch and
    r.currentRequestCommitment = r.observedRequestCommitment and
    r.currentPolicyEpoch != r.observedPolicyEpoch and
    TimeOrder/lt[r.currentTime, r.currentCapabilityExpiry] and
    TimeOrder/lte[r.observedTime, r.currentTime]
}

pred CapabilityExpiryRaceWitness {
  some r: RaceTrace |
    r.authorized = On and
    r.committed = On and
    r.mutation = CapabilityExpiryMutation and
    r.currentAuthorityEpoch = r.observedAuthorityEpoch and
    r.currentRequestCommitment = r.observedRequestCommitment and
    r.currentPolicyEpoch = r.observedPolicyEpoch and
    not TimeOrder/lt[r.currentTime, r.currentCapabilityExpiry] and
    TimeOrder/lte[r.observedTime, r.currentTime]
}

assert CommitRequiresCurrentRevalidation {
  all r: RaceTrace |
    r.committed = On implies
      r.currentAuthorityEpoch = r.observedAuthorityEpoch and
      r.currentRequestCommitment = r.observedRequestCommitment and
      r.currentPolicyEpoch = r.observedPolicyEpoch and
      TimeOrder/lt[r.currentTime, r.currentCapabilityExpiry] and
      TimeOrder/lte[r.observedTime, r.currentTime]
}

assert CommitOnlyAfterCheck {
  all r: RaceTrace |
    r.committed = On implies r.authorized = On
}

assert PreflightSnapshotExact {
  all r: RaceTrace |
    r.observedAuthorityEpoch = r.decision.authorityEpoch and
    r.observedRequestCommitment = r.decision.requestCommitment and
    r.observedPolicyEpoch = r.decision.policyEpoch and
    r.observedCapabilityExpiry = r.decision.capabilityExpiry and
    r.observedTime = r.decision.issuedAt
}

run ValidAtomicCommitWitness
  for 12 but 3 Step, 3 Time, 3 Epoch, 3 RequestCommitment, 3 PolicyEpoch,
  3 Decision, 4 RaceTrace

run AuthorityEpochRaceWitness
  for 12 but 3 Step, 3 Time, 3 Epoch, 3 RequestCommitment, 3 PolicyEpoch,
  3 Decision, 4 RaceTrace

run RequestRaceWitness
  for 12 but 3 Step, 3 Time, 3 Epoch, 3 RequestCommitment, 3 PolicyEpoch,
  3 Decision, 4 RaceTrace

run PolicyEpochRaceWitness
  for 12 but 3 Step, 3 Time, 3 Epoch, 3 RequestCommitment, 3 PolicyEpoch,
  3 Decision, 4 RaceTrace

run CapabilityExpiryRaceWitness
  for 12 but 3 Step, 3 Time, 3 Epoch, 3 RequestCommitment, 3 PolicyEpoch,
  3 Decision, 4 RaceTrace

check CommitRequiresCurrentRevalidation
  for 12 but 3 Step, 3 Time, 3 Epoch, 3 RequestCommitment, 3 PolicyEpoch,
  3 Decision, 4 RaceTrace

check CommitOnlyAfterCheck
  for 12 but 3 Step, 3 Time, 3 Epoch, 3 RequestCommitment, 3 PolicyEpoch,
  3 Decision, 4 RaceTrace

check PreflightSnapshotExact
  for 12 but 3 Step, 3 Time, 3 Epoch, 3 RequestCommitment, 3 PolicyEpoch,
  3 Decision, 4 RaceTrace
