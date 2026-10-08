module EvidenceAttestationDecisionEffectBindingV1

open util/ordering[Time] as TimeOrder
enum Bit { On, Off }

sig DecisionId {}
sig Epoch {}
sig RequestCommitment {}
sig Target {}
sig PolicyEpoch {}
sig Adapter {}
sig Invocation {}
sig Time {}
sig Agent {}

sig Capability {
  expiry: one Time
}

sig Decision {
  id: one DecisionId,
  authorityEpoch: one Epoch,
  requestCommitment: one RequestCommitment,
  target: one Target,
  policyEpoch: one PolicyEpoch,
  adapter: one Adapter,
  invocation: one Invocation,
  issuedAt: one Time,
  validityUntil: one Time,
  capability: one Capability,
  subject: one Agent
}

sig EffectContext {
  authorityEpoch: one Epoch,
  requestCommitment: one RequestCommitment,
  target: one Target,
  policyEpoch: one PolicyEpoch,
  adapter: one Adapter,
  invocation: one Invocation,
  now: one Time
}

sig EffectAdmission {
  decision: one Decision,
  context: one EffectContext,
  authorized: one Bit,
  recordedDecisionId: one DecisionId
}

pred basicDecision[dec: Decision, ctx: EffectContext] {
  dec.subject = dec.subject and
  TimeOrder/lte[dec.issuedAt, ctx.now]
}

fun matchingDecision[c: EffectContext, d: Decision]: one Decision { d }

fact DecisionIdentityBound {
  all e: EffectAdmission |
    e.authorized = On implies
      e.recordedDecisionId = e.decision.id
}

fact AuthorityEpochBound {
  all e: EffectAdmission |
    e.authorized = On implies
      e.context.authorityEpoch = e.decision.authorityEpoch
}

fact RequestCommitmentBound {
  all e: EffectAdmission |
    e.authorized = On implies
      e.context.requestCommitment = e.decision.requestCommitment
}

fact TargetBound {
  all e: EffectAdmission |
    e.authorized = On implies
      e.context.target = e.decision.target
}

fact PolicyEpochBound {
  all e: EffectAdmission |
    e.authorized = On implies
      e.context.policyEpoch = e.decision.policyEpoch
}

fact AdapterIdentityBound {
  all e: EffectAdmission |
    e.authorized = On implies
      e.context.adapter = e.decision.adapter
}

fact InvocationIdentityBound {
  all e: EffectAdmission |
    e.authorized = On implies
      e.context.invocation = e.decision.invocation
}

fact CapabilityCurrentAtEffect {
  all e: EffectAdmission |
    e.authorized = On implies
      TimeOrder/lt[e.context.now, e.decision.capability.expiry]
}

fact DecisionHorizonCurrentAtEffect {
  all e: EffectAdmission |
    e.authorized = On implies
      TimeOrder/lt[e.context.now, e.decision.validityUntil]
}

fact DecisionIssuedBeforeEffect {
  all e: EffectAdmission |
    TimeOrder/lte[e.decision.issuedAt, e.context.now]
}

pred ValidEffectAdmissionWitness {
  some e: EffectAdmission, d: Decision, c: EffectContext |
    e.decision = d and
    e.context = c and
    e.recordedDecisionId = d.id and
    e.authorized = On and
    d.authorityEpoch = c.authorityEpoch and
    d.requestCommitment = c.requestCommitment and
    d.target = c.target and
    d.policyEpoch = c.policyEpoch and
    d.adapter = c.adapter and
    d.invocation = c.invocation and
    TimeOrder/lte[d.issuedAt, c.now] and
    TimeOrder/lt[c.now, d.capability.expiry] and
    TimeOrder/lt[c.now, d.validityUntil]
}

pred DecisionIdentitySubstitutionWitness {
  some e: EffectAdmission |
    e.authorized = On and
    e.recordedDecisionId != e.decision.id and
    e.context.authorityEpoch = e.decision.authorityEpoch and
    e.context.requestCommitment = e.decision.requestCommitment and
    e.context.target = e.decision.target and
    e.context.policyEpoch = e.decision.policyEpoch and
    e.context.adapter = e.decision.adapter and
    e.context.invocation = e.decision.invocation and
    TimeOrder/lte[e.decision.issuedAt, e.context.now] and
    TimeOrder/lt[e.context.now, e.decision.capability.expiry] and
    TimeOrder/lt[e.context.now, e.decision.validityUntil]
}

pred AuthorityEpochSubstitutionWitness {
  some e: EffectAdmission |
    e.authorized = On and
    e.context.authorityEpoch != e.decision.authorityEpoch and
    e.context.requestCommitment = e.decision.requestCommitment and
    e.context.target = e.decision.target and
    e.context.policyEpoch = e.decision.policyEpoch and
    e.context.adapter = e.decision.adapter and
    e.context.invocation = e.decision.invocation and
    TimeOrder/lte[e.decision.issuedAt, e.context.now] and
    TimeOrder/lt[e.context.now, e.decision.capability.expiry] and
    TimeOrder/lt[e.context.now, e.decision.validityUntil]
}

pred RequestCommitmentSubstitutionWitness {
  some e: EffectAdmission |
    e.authorized = On and
    e.context.authorityEpoch = e.decision.authorityEpoch and
    e.context.requestCommitment != e.decision.requestCommitment and
    e.context.target = e.decision.target and
    e.context.policyEpoch = e.decision.policyEpoch and
    e.context.adapter = e.decision.adapter and
    e.context.invocation = e.decision.invocation and
    TimeOrder/lte[e.decision.issuedAt, e.context.now] and
    TimeOrder/lt[e.context.now, e.decision.capability.expiry] and
    TimeOrder/lt[e.context.now, e.decision.validityUntil]
}

pred TargetSubstitutionWitness {
  some e: EffectAdmission |
    e.authorized = On and
    e.context.authorityEpoch = e.decision.authorityEpoch and
    e.context.requestCommitment = e.decision.requestCommitment and
    e.context.target != e.decision.target and
    e.context.policyEpoch = e.decision.policyEpoch and
    e.context.adapter = e.decision.adapter and
    e.context.invocation = e.decision.invocation and
    TimeOrder/lte[e.decision.issuedAt, e.context.now] and
    TimeOrder/lt[e.context.now, e.decision.capability.expiry] and
    TimeOrder/lt[e.context.now, e.decision.validityUntil]
}

pred PolicyEpochSubstitutionWitness {
  some e: EffectAdmission |
    e.authorized = On and
    e.context.authorityEpoch = e.decision.authorityEpoch and
    e.context.requestCommitment = e.decision.requestCommitment and
    e.context.target = e.decision.target and
    e.context.policyEpoch != e.decision.policyEpoch and
    e.context.adapter = e.decision.adapter and
    e.context.invocation = e.decision.invocation and
    TimeOrder/lte[e.decision.issuedAt, e.context.now] and
    TimeOrder/lt[e.context.now, e.decision.capability.expiry] and
    TimeOrder/lt[e.context.now, e.decision.validityUntil]
}

pred AdapterSubstitutionWitness {
  some e: EffectAdmission |
    e.authorized = On and
    e.context.authorityEpoch = e.decision.authorityEpoch and
    e.context.requestCommitment = e.decision.requestCommitment and
    e.context.target = e.decision.target and
    e.context.policyEpoch = e.decision.policyEpoch and
    e.context.adapter != e.decision.adapter and
    e.context.invocation = e.decision.invocation and
    TimeOrder/lte[e.decision.issuedAt, e.context.now] and
    TimeOrder/lt[e.context.now, e.decision.capability.expiry] and
    TimeOrder/lt[e.context.now, e.decision.validityUntil]
}

pred InvocationSubstitutionWitness {
  some e: EffectAdmission |
    e.authorized = On and
    e.context.authorityEpoch = e.decision.authorityEpoch and
    e.context.requestCommitment = e.decision.requestCommitment and
    e.context.target = e.decision.target and
    e.context.policyEpoch = e.decision.policyEpoch and
    e.context.adapter = e.decision.adapter and
    e.context.invocation != e.decision.invocation and
    TimeOrder/lte[e.decision.issuedAt, e.context.now] and
    TimeOrder/lt[e.context.now, e.decision.capability.expiry] and
    TimeOrder/lt[e.context.now, e.decision.validityUntil]
}

pred CapabilityExpiryWitness {
  some e: EffectAdmission |
    e.authorized = On and
    e.context.authorityEpoch = e.decision.authorityEpoch and
    e.context.requestCommitment = e.decision.requestCommitment and
    e.context.target = e.decision.target and
    e.context.policyEpoch = e.decision.policyEpoch and
    e.context.adapter = e.decision.adapter and
    e.context.invocation = e.decision.invocation and
    TimeOrder/lte[e.decision.issuedAt, e.context.now] and
    not TimeOrder/lt[e.context.now, e.decision.capability.expiry] and
    TimeOrder/lt[e.context.now, e.decision.validityUntil]
}

pred DecisionHorizonWitness {
  some e: EffectAdmission |
    e.authorized = On and
    e.context.authorityEpoch = e.decision.authorityEpoch and
    e.context.requestCommitment = e.decision.requestCommitment and
    e.context.target = e.decision.target and
    e.context.policyEpoch = e.decision.policyEpoch and
    e.context.adapter = e.decision.adapter and
    e.context.invocation = e.decision.invocation and
    TimeOrder/lte[e.decision.issuedAt, e.context.now] and
    TimeOrder/lt[e.context.now, e.decision.capability.expiry] and
    not TimeOrder/lt[e.context.now, e.decision.validityUntil]
}

assert DecisionToEffectAuthorityStillBound {
  all e: EffectAdmission |
    e.authorized = On implies
      e.recordedDecisionId = e.decision.id and
      e.context.authorityEpoch = e.decision.authorityEpoch and
      e.context.requestCommitment = e.decision.requestCommitment and
      e.context.target = e.decision.target and
      e.context.policyEpoch = e.decision.policyEpoch and
      e.context.adapter = e.decision.adapter and
      e.context.invocation = e.decision.invocation and
      TimeOrder/lte[e.decision.issuedAt, e.context.now] and
      TimeOrder/lt[e.context.now, e.decision.capability.expiry] and
      TimeOrder/lt[e.context.now, e.decision.validityUntil]
}

assert DecisionIdentityBound {
  all e: EffectAdmission | e.authorized = On implies e.recordedDecisionId = e.decision.id
}
assert AuthorityEpochBound {
  all e: EffectAdmission | e.authorized = On implies e.context.authorityEpoch = e.decision.authorityEpoch
}
assert RequestCommitmentBound {
  all e: EffectAdmission | e.authorized = On implies e.context.requestCommitment = e.decision.requestCommitment
}
assert TargetBound {
  all e: EffectAdmission | e.authorized = On implies e.context.target = e.decision.target
}
assert PolicyEpochBound {
  all e: EffectAdmission | e.authorized = On implies e.context.policyEpoch = e.decision.policyEpoch
}
assert AdapterIdentityBound {
  all e: EffectAdmission | e.authorized = On implies e.context.adapter = e.decision.adapter
}
assert InvocationIdentityBound {
  all e: EffectAdmission | e.authorized = On implies e.context.invocation = e.decision.invocation
}
assert CapabilityCurrentAtEffect {
  all e: EffectAdmission | e.authorized = On implies TimeOrder/lt[e.context.now, e.decision.capability.expiry]
}
assert DecisionHorizonCurrentAtEffect {
  all e: EffectAdmission | e.authorized = On implies TimeOrder/lt[e.context.now, e.decision.validityUntil]
}

run ValidEffectAdmissionWitness
  for 12 but 2 Resource, 2 Action, 2 Audience, 6 Time,
  2 Capability, 4 Decision, 4 EffectContext, 4 EffectAdmission,
  4 DecisionId, 4 Epoch, 4 RequestCommitment, 4 Target, 4 PolicyEpoch, 4 Adapter, 4 Invocation, 4 Agent

run DecisionIdentitySubstitutionWitness
  for 12 but 2 Resource, 2 Action, 2 Audience, 6 Time,
  2 Capability, 4 Decision, 4 EffectContext, 4 EffectAdmission,
  4 DecisionId, 4 Epoch, 4 RequestCommitment, 4 Target, 4 PolicyEpoch, 4 Adapter, 4 Invocation, 4 Agent

run AuthorityEpochSubstitutionWitness
  for 12 but 2 Resource, 2 Action, 2 Audience, 6 Time,
  2 Capability, 4 Decision, 4 EffectContext, 4 EffectAdmission,
  4 DecisionId, 4 Epoch, 4 RequestCommitment, 4 Target, 4 PolicyEpoch, 4 Adapter, 4 Invocation, 4 Agent

run RequestCommitmentSubstitutionWitness
  for 12 but 2 Resource, 2 Action, 2 Audience, 6 Time,
  2 Capability, 4 Decision, 4 EffectContext, 4 EffectAdmission,
  4 DecisionId, 4 Epoch, 4 RequestCommitment, 4 Target, 4 PolicyEpoch, 4 Adapter, 4 Invocation, 4 Agent

run TargetSubstitutionWitness
  for 12 but 2 Resource, 2 Action, 2 Audience, 6 Time,
  2 Capability, 4 Decision, 4 EffectContext, 4 EffectAdmission,
  4 DecisionId, 4 Epoch, 4 RequestCommitment, 4 Target, 4 PolicyEpoch, 4 Adapter, 4 Invocation, 4 Agent

run PolicyEpochSubstitutionWitness
  for 12 but 2 Resource, 2 Action, 2 Audience, 6 Time,
  2 Capability, 4 Decision, 4 EffectContext, 4 EffectAdmission,
  4 DecisionId, 4 Epoch, 4 RequestCommitment, 4 Target, 4 PolicyEpoch, 4 Adapter, 4 Invocation, 4 Agent

run AdapterSubstitutionWitness
  for 12 but 2 Resource, 2 Action, 2 Audience, 6 Time,
  2 Capability, 4 Decision, 4 EffectContext, 4 EffectAdmission,
  4 DecisionId, 4 Epoch, 4 RequestCommitment, 4 Target, 4 PolicyEpoch, 4 Adapter, 4 Invocation, 4 Agent

run InvocationSubstitutionWitness
  for 12 but 2 Resource, 2 Action, 2 Audience, 6 Time,
  2 Capability, 4 Decision, 4 EffectContext, 4 EffectAdmission,
  4 DecisionId, 4 Epoch, 4 RequestCommitment, 4 Target, 4 PolicyEpoch, 4 Adapter, 4 Invocation, 4 Agent

run CapabilityExpiryWitness
  for 12 but 2 Resource, 2 Action, 2 Audience, 6 Time,
  2 Capability, 4 Decision, 4 EffectContext, 4 EffectAdmission,
  4 DecisionId, 4 Epoch, 4 RequestCommitment, 4 Target, 4 PolicyEpoch, 4 Adapter, 4 Invocation, 4 Agent

run DecisionHorizonWitness
  for 12 but 2 Resource, 2 Action, 2 Audience, 6 Time,
  2 Capability, 4 Decision, 4 EffectContext, 4 EffectAdmission,
  4 DecisionId, 4 Epoch, 4 RequestCommitment, 4 Target, 4 PolicyEpoch, 4 Adapter, 4 Invocation, 4 Agent

check DecisionToEffectAuthorityStillBound
  for 12 but 2 Resource, 2 Action, 2 Audience, 6 Time,
  2 Capability, 4 Decision, 4 EffectContext, 4 EffectAdmission,
  4 DecisionId, 4 Epoch, 4 RequestCommitment, 4 Target, 4 PolicyEpoch, 4 Adapter, 4 Invocation, 4 Agent

check DecisionIdentityBound
  for 12 but 2 Resource, 2 Action, 2 Audience, 6 Time,
  2 Capability, 4 Decision, 4 EffectContext, 4 EffectAdmission,
  4 DecisionId, 4 Epoch, 4 RequestCommitment, 4 Target, 4 PolicyEpoch, 4 Adapter, 4 Invocation, 4 Agent

check AuthorityEpochBound
  for 12 but 2 Resource, 2 Action, 2 Audience, 6 Time,
  2 Capability, 4 Decision, 4 EffectContext, 4 EffectAdmission,
  4 DecisionId, 4 Epoch, 4 RequestCommitment, 4 Target, 4 PolicyEpoch, 4 Adapter, 4 Invocation, 4 Agent

check RequestCommitmentBound
  for 12 but 2 Resource, 2 Action, 2 Audience, 6 Time,
  2 Capability, 4 Decision, 4 EffectContext, 4 EffectAdmission,
  4 DecisionId, 4 Epoch, 4 RequestCommitment, 4 Target, 4 PolicyEpoch, 4 Adapter, 4 Invocation, 4 Agent

check TargetBound
  for 12 but 2 Resource, 2 Action, 2 Audience, 6 Time,
  2 Capability, 4 Decision, 4 EffectContext, 4 EffectAdmission,
  4 DecisionId, 4 Epoch, 4 RequestCommitment, 4 Target, 4 PolicyEpoch, 4 Adapter, 4 Invocation, 4 Agent

check PolicyEpochBound
  for 12 but 2 Resource, 2 Action, 2 Audience, 6 Time,
  2 Capability, 4 Decision, 4 EffectContext, 4 EffectAdmission,
  4 DecisionId, 4 Epoch, 4 RequestCommitment, 4 Target, 4 PolicyEpoch, 4 Adapter, 4 Invocation, 4 Agent

check AdapterIdentityBound
  for 12 but 2 Resource, 2 Action, 2 Audience, 6 Time,
  2 Capability, 4 Decision, 4 EffectContext, 4 EffectAdmission,
  4 DecisionId, 4 Epoch, 4 RequestCommitment, 4 Target, 4 PolicyEpoch, 4 Adapter, 4 Invocation, 4 Agent

check InvocationIdentityBound
  for 12 but 2 Resource, 2 Action, 2 Audience, 6 Time,
  2 Capability, 4 Decision, 4 EffectContext, 4 EffectAdmission,
  4 DecisionId, 4 Epoch, 4 RequestCommitment, 4 Target, 4 PolicyEpoch, 4 Adapter, 4 Invocation, 4 Agent

check CapabilityCurrentAtEffect
  for 12 but 2 Resource, 2 Action, 2 Audience, 6 Time,
  2 Capability, 4 Decision, 4 EffectContext, 4 EffectAdmission,
  4 DecisionId, 4 Epoch, 4 RequestCommitment, 4 Target, 4 PolicyEpoch, 4 Adapter, 4 Invocation, 4 Agent

check DecisionHorizonCurrentAtEffect
  for 12 but 2 Resource, 2 Action, 2 Audience, 6 Time,
  2 Capability, 4 Decision, 4 EffectContext, 4 EffectAdmission,
  4 DecisionId, 4 Epoch, 4 RequestCommitment, 4 Target, 4 PolicyEpoch, 4 Adapter, 4 Invocation, 4 Agent
