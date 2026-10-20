module EvidenceAttestationCommitTimeRaceV1

open util/ordering[Step] as StepOrder
open util/ordering[Time] as TimeOrder

enum Bit { On, Off }
enum MutationKind { AuthorityEpochMutation, RequestMutation, TargetMutation, PolicyEpochMutation, AdapterMutation, InvocationMutation, CapabilityExpiryMutation, TimeMutation, NoMutation }

sig Step {}
sig Time {}
sig Epoch {}
sig RequestCommitment {}
sig Target {}
sig PolicyEpoch {}
sig Adapter {}
sig Invocation {}

sig Decision {
  authorityEpoch: one Epoch,
  requestCommitment: one RequestCommitment,
  target: one Target,
  policyEpoch: one PolicyEpoch,
  adapter: one Adapter,
  invocation: one Invocation,
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
  observedTarget: one Target,
  observedPolicyEpoch: one PolicyEpoch,
  observedAdapter: one Adapter,
  observedInvocation: one Invocation,
  observedCapabilityExpiry: one Time,
  observedTime: one Time,
  currentAuthorityEpoch: one Epoch,
  currentRequestCommitment: one RequestCommitment,
  currentTarget: one Target,
  currentPolicyEpoch: one PolicyEpoch,
  currentAdapter: one Adapter,
  currentInvocation: one Invocation,
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
    r.observedTarget = r.decision.target and
    r.observedPolicyEpoch = r.decision.policyEpoch and
    r.observedAdapter = r.decision.adapter and
    r.observedInvocation = r.decision.invocation and
    r.observedCapabilityExpiry = r.decision.capabilityExpiry and
    r.observedTime = r.decision.issuedAt
}

fact CommitOnlyAfterCheck {
  all r: RaceTrace |
    r.committed = On implies r.authorized = On
}

fact AuthorityEpochRevalidated {
  all r: RaceTrace |
    r.committed = On implies r.currentAuthorityEpoch = r.observedAuthorityEpoch
}

fact RequestCommitmentRevalidated {
  all r: RaceTrace |
    r.committed = On implies r.currentRequestCommitment = r.observedRequestCommitment
}

fact TargetRevalidated {
  all r: RaceTrace |
    r.committed = On implies r.currentTarget = r.observedTarget
}

fact PolicyEpochRevalidated {
  all r: RaceTrace |
    r.committed = On implies r.currentPolicyEpoch = r.observedPolicyEpoch
}

fact AdapterIdentityRevalidated {
  all r: RaceTrace |
    r.committed = On implies r.currentAdapter = r.observedAdapter
}

fact InvocationIdentityRevalidated {
  all r: RaceTrace |
    r.committed = On implies r.currentInvocation = r.observedInvocation
}

fact CapabilityExpiryUnchangedAtCommit {
  all r: RaceTrace |
    r.committed = On implies r.currentCapabilityExpiry = r.observedCapabilityExpiry
}

fact CapabilityCurrentAtCommit {
  all r: RaceTrace |
    r.committed = On implies TimeOrder/lt[r.currentTime, r.currentCapabilityExpiry]
}

fact CommitTimeMonotone {
  all r: RaceTrace |
    r.committed = On implies TimeOrder/lte[r.observedTime, r.currentTime]
}

pred ValidAtomicCommitWitness {
  some r: RaceTrace |
    r.authorized = On and
    r.committed = On and
    r.mutation = NoMutation and
    r.currentAuthorityEpoch = r.observedAuthorityEpoch and
    r.currentRequestCommitment = r.observedRequestCommitment and
    r.currentTarget = r.observedTarget and
    r.currentPolicyEpoch = r.observedPolicyEpoch and
    r.currentAdapter = r.observedAdapter and
    r.currentInvocation = r.observedInvocation and
    r.currentCapabilityExpiry = r.observedCapabilityExpiry and
    TimeOrder/lt[r.currentTime, r.currentCapabilityExpiry] and
    TimeOrder/lte[r.observedTime, r.currentTime]
}

pred AuthorityEpochRaceWitness {
  some r: RaceTrace |
    r.authorized = On and r.committed = On and r.mutation = AuthorityEpochMutation and
    r.currentAuthorityEpoch != r.observedAuthorityEpoch and
    r.currentRequestCommitment = r.observedRequestCommitment and
    r.currentTarget = r.observedTarget and
    r.currentPolicyEpoch = r.observedPolicyEpoch and
    r.currentAdapter = r.observedAdapter and
    r.currentInvocation = r.observedInvocation and
    r.currentCapabilityExpiry = r.observedCapabilityExpiry and
    TimeOrder/lt[r.currentTime, r.currentCapabilityExpiry] and
    TimeOrder/lte[r.observedTime, r.currentTime]
}

pred RequestRaceWitness {
  some r: RaceTrace |
    r.authorized = On and r.committed = On and r.mutation = RequestMutation and
    r.currentAuthorityEpoch = r.observedAuthorityEpoch and
    r.currentRequestCommitment != r.observedRequestCommitment and
    r.currentTarget = r.observedTarget and
    r.currentPolicyEpoch = r.observedPolicyEpoch and
    r.currentAdapter = r.observedAdapter and
    r.currentInvocation = r.observedInvocation and
    r.currentCapabilityExpiry = r.observedCapabilityExpiry and
    TimeOrder/lt[r.currentTime, r.currentCapabilityExpiry] and
    TimeOrder/lte[r.observedTime, r.currentTime]
}

pred TargetRaceWitness {
  some r: RaceTrace |
    r.authorized = On and r.committed = On and r.mutation = TargetMutation and
    r.currentAuthorityEpoch = r.observedAuthorityEpoch and
    r.currentRequestCommitment = r.observedRequestCommitment and
    r.currentTarget != r.observedTarget and
    r.currentPolicyEpoch = r.observedPolicyEpoch and
    r.currentAdapter = r.observedAdapter and
    r.currentInvocation = r.observedInvocation and
    r.currentCapabilityExpiry = r.observedCapabilityExpiry and
    TimeOrder/lt[r.currentTime, r.currentCapabilityExpiry] and
    TimeOrder/lte[r.observedTime, r.currentTime]
}

pred PolicyEpochRaceWitness {
  some r: RaceTrace |
    r.authorized = On and r.committed = On and r.mutation = PolicyEpochMutation and
    r.currentAuthorityEpoch = r.observedAuthorityEpoch and
    r.currentRequestCommitment = r.observedRequestCommitment and
    r.currentTarget = r.observedTarget and
    r.currentPolicyEpoch != r.observedPolicyEpoch and
    r.currentAdapter = r.observedAdapter and
    r.currentInvocation = r.observedInvocation and
    r.currentCapabilityExpiry = r.observedCapabilityExpiry and
    TimeOrder/lt[r.currentTime, r.currentCapabilityExpiry] and
    TimeOrder/lte[r.observedTime, r.currentTime]
}

pred AdapterRaceWitness {
  some r: RaceTrace |
    r.authorized = On and r.committed = On and r.mutation = AdapterMutation and
    r.currentAuthorityEpoch = r.observedAuthorityEpoch and
    r.currentRequestCommitment = r.observedRequestCommitment and
    r.currentTarget = r.observedTarget and
    r.currentPolicyEpoch = r.observedPolicyEpoch and
    r.currentAdapter != r.observedAdapter and
    r.currentInvocation = r.observedInvocation and
    r.currentCapabilityExpiry = r.observedCapabilityExpiry and
    TimeOrder/lt[r.currentTime, r.currentCapabilityExpiry] and
    TimeOrder/lte[r.observedTime, r.currentTime]
}

pred InvocationRaceWitness {
  some r: RaceTrace |
    r.authorized = On and r.committed = On and r.mutation = InvocationMutation and
    r.currentAuthorityEpoch = r.observedAuthorityEpoch and
    r.currentRequestCommitment = r.observedRequestCommitment and
    r.currentTarget = r.observedTarget and
    r.currentPolicyEpoch = r.observedPolicyEpoch and
    r.currentAdapter = r.observedAdapter and
    r.currentInvocation != r.observedInvocation and
    r.currentCapabilityExpiry = r.observedCapabilityExpiry and
    TimeOrder/lt[r.currentTime, r.currentCapabilityExpiry] and
    TimeOrder/lte[r.observedTime, r.currentTime]
}

pred CapabilityExpiryRaceWitness {
  some r: RaceTrace |
    r.authorized = On and r.committed = On and r.mutation = CapabilityExpiryMutation and
    r.currentAuthorityEpoch = r.observedAuthorityEpoch and
    r.currentRequestCommitment = r.observedRequestCommitment and
    r.currentTarget = r.observedTarget and
    r.currentPolicyEpoch = r.observedPolicyEpoch and
    r.currentAdapter = r.observedAdapter and
    r.currentInvocation = r.observedInvocation and
    r.currentCapabilityExpiry != r.observedCapabilityExpiry and
    TimeOrder/lt[r.currentTime, r.currentCapabilityExpiry] and
    TimeOrder/lte[r.observedTime, r.currentTime]
}

pred TimeExpiryRaceWitness {
  some r: RaceTrace |
    r.authorized = On and r.committed = On and r.mutation = TimeMutation and
    r.currentAuthorityEpoch = r.observedAuthorityEpoch and
    r.currentRequestCommitment = r.observedRequestCommitment and
    r.currentTarget = r.observedTarget and
    r.currentPolicyEpoch = r.observedPolicyEpoch and
    r.currentAdapter = r.observedAdapter and
    r.currentInvocation = r.observedInvocation and
    r.currentCapabilityExpiry = r.observedCapabilityExpiry and
    not TimeOrder/lt[r.currentTime, r.currentCapabilityExpiry] and
    TimeOrder/lte[r.observedTime, r.currentTime]
}

assert CommitRequiresCurrentRevalidation {
  all r: RaceTrace |
    r.committed = On implies
      r.currentAuthorityEpoch = r.observedAuthorityEpoch and
      r.currentRequestCommitment = r.observedRequestCommitment and
      r.currentTarget = r.observedTarget and
      r.currentPolicyEpoch = r.observedPolicyEpoch and
      r.currentAdapter = r.observedAdapter and
      r.currentInvocation = r.observedInvocation and
      r.currentCapabilityExpiry = r.observedCapabilityExpiry and
      TimeOrder/lt[r.currentTime, r.currentCapabilityExpiry] and
      TimeOrder/lte[r.observedTime, r.currentTime]
}

assert AuthorityEpochRevalidated {
  all r: RaceTrace |
    r.committed = On implies r.currentAuthorityEpoch = r.observedAuthorityEpoch
}
assert RequestCommitmentRevalidated {
  all r: RaceTrace |
    r.committed = On implies r.currentRequestCommitment = r.observedRequestCommitment
}
assert TargetRevalidated {
  all r: RaceTrace |
    r.committed = On implies r.currentTarget = r.observedTarget
}
assert PolicyEpochRevalidated {
  all r: RaceTrace |
    r.committed = On implies r.currentPolicyEpoch = r.observedPolicyEpoch
}
assert AdapterIdentityRevalidated {
  all r: RaceTrace |
    r.committed = On implies r.currentAdapter = r.observedAdapter
}
assert InvocationIdentityRevalidated {
  all r: RaceTrace |
    r.committed = On implies r.currentInvocation = r.observedInvocation
}
assert CapabilityExpiryUnchangedAtCommit {
  all r: RaceTrace |
    r.committed = On implies r.currentCapabilityExpiry = r.observedCapabilityExpiry
}
assert CapabilityCurrentAtCommit {
  all r: RaceTrace |
    r.committed = On implies TimeOrder/lt[r.currentTime, r.currentCapabilityExpiry]
}
assert CommitTimeMonotone {
  all r: RaceTrace |
    r.committed = On implies TimeOrder/lte[r.observedTime, r.currentTime]
}

run ValidAtomicCommitWitness
  for 12 but 3 Step, 5 Time, 3 Epoch, 3 RequestCommitment, 3 Target, 3 PolicyEpoch,
  3 Adapter, 3 Invocation, 4 Decision, 6 RaceTrace

run AuthorityEpochRaceWitness
  for 12 but 3 Step, 5 Time, 3 Epoch, 3 RequestCommitment, 3 Target, 3 PolicyEpoch,
  3 Adapter, 3 Invocation, 4 Decision, 6 RaceTrace

run RequestRaceWitness
  for 12 but 3 Step, 5 Time, 3 Epoch, 3 RequestCommitment, 3 Target, 3 PolicyEpoch,
  3 Adapter, 3 Invocation, 4 Decision, 6 RaceTrace

run TargetRaceWitness
  for 12 but 3 Step, 5 Time, 3 Epoch, 3 RequestCommitment, 3 Target, 3 PolicyEpoch,
  3 Adapter, 3 Invocation, 4 Decision, 6 RaceTrace

run PolicyEpochRaceWitness
  for 12 but 3 Step, 5 Time, 3 Epoch, 3 RequestCommitment, 3 Target, 3 PolicyEpoch,
  3 Adapter, 3 Invocation, 4 Decision, 6 RaceTrace

run AdapterRaceWitness
  for 12 but 3 Step, 5 Time, 3 Epoch, 3 RequestCommitment, 3 Target, 3 PolicyEpoch,
  3 Adapter, 3 Invocation, 4 Decision, 6 RaceTrace

run InvocationRaceWitness
  for 12 but 3 Step, 5 Time, 3 Epoch, 3 RequestCommitment, 3 Target, 3 PolicyEpoch,
  3 Adapter, 3 Invocation, 4 Decision, 6 RaceTrace

run CapabilityExpiryRaceWitness
  for 12 but 3 Step, 5 Time, 3 Epoch, 3 RequestCommitment, 3 Target, 3 PolicyEpoch,
  3 Adapter, 3 Invocation, 4 Decision, 6 RaceTrace

run TimeExpiryRaceWitness
  for 12 but 3 Step, 5 Time, 3 Epoch, 3 RequestCommitment, 3 Target, 3 PolicyEpoch,
  3 Adapter, 3 Invocation, 4 Decision, 6 RaceTrace

check CommitRequiresCurrentRevalidation
  for 12 but 3 Step, 5 Time, 3 Epoch, 3 RequestCommitment, 3 Target, 3 PolicyEpoch,
  3 Adapter, 3 Invocation, 4 Decision, 6 RaceTrace
check AuthorityEpochRevalidated
  for 12 but 3 Step, 5 Time, 3 Epoch, 3 RequestCommitment, 3 Target, 3 PolicyEpoch,
  3 Adapter, 3 Invocation, 4 Decision, 6 RaceTrace
check RequestCommitmentRevalidated
  for 12 but 3 Step, 5 Time, 3 Epoch, 3 RequestCommitment, 3 Target, 3 PolicyEpoch,
  3 Adapter, 3 Invocation, 4 Decision, 6 RaceTrace
check TargetRevalidated
  for 12 but 3 Step, 5 Time, 3 Epoch, 3 RequestCommitment, 3 Target, 3 PolicyEpoch,
  3 Adapter, 3 Invocation, 4 Decision, 6 RaceTrace
check PolicyEpochRevalidated
  for 12 but 3 Step, 5 Time, 3 Epoch, 3 RequestCommitment, 3 Target, 3 PolicyEpoch,
  3 Adapter, 3 Invocation, 4 Decision, 6 RaceTrace
check AdapterIdentityRevalidated
  for 12 but 3 Step, 5 Time, 3 Epoch, 3 RequestCommitment, 3 Target, 3 PolicyEpoch,
  3 Adapter, 3 Invocation, 4 Decision, 6 RaceTrace
check InvocationIdentityRevalidated
  for 12 but 3 Step, 5 Time, 3 Epoch, 3 RequestCommitment, 3 Target, 3 PolicyEpoch,
  3 Adapter, 3 Invocation, 4 Decision, 6 RaceTrace
check CapabilityExpiryUnchangedAtCommit
  for 12 but 3 Step, 5 Time, 3 Epoch, 3 RequestCommitment, 3 Target, 3 PolicyEpoch,
  3 Adapter, 3 Invocation, 4 Decision, 6 RaceTrace
check CapabilityCurrentAtCommit
  for 12 but 3 Step, 5 Time, 3 Epoch, 3 RequestCommitment, 3 Target, 3 PolicyEpoch,
  3 Adapter, 3 Invocation, 4 Decision, 6 RaceTrace
check CommitTimeMonotone
  for 12 but 3 Step, 5 Time, 3 Epoch, 3 RequestCommitment, 3 Target, 3 PolicyEpoch,
  3 Adapter, 3 Invocation, 4 Decision, 6 RaceTrace
