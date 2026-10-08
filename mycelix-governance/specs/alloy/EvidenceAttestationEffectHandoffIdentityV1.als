module EvidenceAttestationEffectHandoffIdentityV1

enum Bit { On, Off }

sig DecisionId {}
sig IntentId {}
sig OperationCommitment {}
sig UseCommitment {}
sig Target {}
sig Adapter {}
sig Invocation {}

sig Decision { id: one DecisionId }

sig EffectIntent {
  id: one IntentId,
  decision: one Decision,
  operation: one OperationCommitment,
  target: one Target,
  adapter: one Adapter,
  invocation: one Invocation
}

sig CapabilityUseCommit {
  id: one UseCommitment,
  decision: one Decision,
  intent: one EffectIntent,
  operation: one OperationCommitment,
  target: one Target,
  adapter: one Adapter,
  invocation: one Invocation
}

sig EffectAttempt {
  decision: one Decision,
  intent: one EffectIntent,
  use: one CapabilityUseCommit,
  recordedDecision: one DecisionId,
  recordedIntent: one IntentId,
  recordedOperation: one OperationCommitment,
  recordedUse: one UseCommitment,
  recordedTarget: one Target,
  recordedAdapter: one Adapter,
  recordedInvocation: one Invocation,
  committed: one Bit
}

fact IdentityConservation {
  all a: EffectAttempt |
    a.committed = On implies
      a.recordedDecision = a.decision.id and
      a.recordedIntent = a.intent.id and
      a.recordedOperation = a.intent.operation and
      a.recordedUse = a.use.id and
      a.recordedTarget = a.intent.target and
      a.recordedAdapter = a.intent.adapter and
      a.recordedInvocation = a.intent.invocation and
      a.use.decision = a.decision and
      a.use.intent = a.intent and
      a.use.operation = a.intent.operation and
      a.use.target = a.intent.target and
      a.use.adapter = a.intent.adapter and
      a.use.invocation = a.intent.invocation
}

pred ValidHandoffWitness {
  some a: EffectAttempt |
    a.committed = On and
    a.recordedDecision = a.decision.id and
    a.recordedIntent = a.intent.id and
    a.recordedOperation = a.intent.operation and
    a.recordedUse = a.use.id and
    a.recordedTarget = a.intent.target and
    a.recordedAdapter = a.intent.adapter and
    a.recordedInvocation = a.intent.invocation and
    a.use.decision = a.decision and
    a.use.intent = a.intent
}

pred DecisionSubstitutionWitness {
  some a: EffectAttempt, d2: Decision |
    a.committed = On and
    d2.id != a.decision.id and
    a.recordedDecision = d2.id
}

pred IntentSubstitutionWitness {
  some a: EffectAttempt, i2: EffectIntent |
    a.committed = On and
    i2.id != a.intent.id and
    a.recordedIntent = i2.id
}

pred OperationSubstitutionWitness {
  some a: EffectAttempt, op2: OperationCommitment |
    a.committed = On and
    op2 != a.intent.operation and
    a.recordedOperation = op2
}

pred UseSubstitutionWitness {
  some a: EffectAttempt, u2: UseCommitment |
    a.committed = On and
    u2 != a.use.id and
    a.recordedUse = u2
}

pred TargetSubstitutionWitness {
  some a: EffectAttempt, t2: Target |
    a.committed = On and
    t2 != a.intent.target and
    a.recordedTarget = t2
}

pred AdapterSubstitutionWitness {
  some a: EffectAttempt, ad2: Adapter |
    a.committed = On and
    ad2 != a.intent.adapter and
    a.recordedAdapter = ad2
}

pred InvocationSubstitutionWitness {
  some a: EffectAttempt, v2: Invocation |
    a.committed = On and
    v2 != a.intent.invocation and
    a.recordedInvocation = v2
}

assert IdentityConservation {
  all a: EffectAttempt |
    a.committed = On implies
      a.recordedDecision = a.decision.id and
      a.recordedIntent = a.intent.id and
      a.recordedOperation = a.intent.operation and
      a.recordedUse = a.use.id and
      a.recordedTarget = a.intent.target and
      a.recordedAdapter = a.intent.adapter and
      a.recordedInvocation = a.intent.invocation and
      a.use.decision = a.decision and
      a.use.intent = a.intent and
      a.use.operation = a.intent.operation and
      a.use.target = a.intent.target and
      a.use.adapter = a.intent.adapter and
      a.use.invocation = a.intent.invocation
}

check IdentityConservation for 12 but 4 Decision, 4 EffectIntent, 4 CapabilityUseCommit, 4 EffectAttempt,
  4 DecisionId, 4 IntentId, 4 OperationCommitment, 4 UseCommitment, 4 Target, 4 Adapter, 4 Invocation

run ValidHandoffWitness for 12 but 4 Decision, 4 EffectIntent, 4 CapabilityUseCommit, 4 EffectAttempt,
  4 DecisionId, 4 IntentId, 4 OperationCommitment, 4 UseCommitment, 4 Target, 4 Adapter, 4 Invocation

run DecisionSubstitutionWitness for 12 but 4 Decision, 4 EffectIntent, 4 CapabilityUseCommit, 4 EffectAttempt,
  4 DecisionId, 4 IntentId, 4 OperationCommitment, 4 UseCommitment, 4 Target, 4 Adapter, 4 Invocation

run IntentSubstitutionWitness for 12 but 4 Decision, 4 EffectIntent, 4 CapabilityUseCommit, 4 EffectAttempt,
  4 DecisionId, 4 IntentId, 4 OperationCommitment, 4 UseCommitment, 4 Target, 4 Adapter, 4 Invocation

run OperationSubstitutionWitness for 12 but 4 Decision, 4 EffectIntent, 4 CapabilityUseCommit, 4 EffectAttempt,
  4 DecisionId, 4 IntentId, 4 OperationCommitment, 4 UseCommitment, 4 Target, 4 Adapter, 4 Invocation

run UseSubstitutionWitness for 12 but 4 Decision, 4 EffectIntent, 4 CapabilityUseCommit, 4 EffectAttempt,
  4 DecisionId, 4 IntentId, 4 OperationCommitment, 4 UseCommitment, 4 Target, 4 Adapter, 4 Invocation

run TargetSubstitutionWitness for 12 but 4 Decision, 4 EffectIntent, 4 CapabilityUseCommit, 4 EffectAttempt,
  4 DecisionId, 4 IntentId, 4 OperationCommitment, 4 UseCommitment, 4 Target, 4 Adapter, 4 Invocation

run AdapterSubstitutionWitness for 12 but 4 Decision, 4 EffectIntent, 4 CapabilityUseCommit, 4 EffectAttempt,
  4 DecisionId, 4 IntentId, 4 OperationCommitment, 4 UseCommitment, 4 Target, 4 Adapter, 4 Invocation

run InvocationSubstitutionWitness for 12 but 4 Decision, 4 EffectIntent, 4 CapabilityUseCommit, 4 EffectAttempt,
  4 DecisionId, 4 IntentId, 4 OperationCommitment, 4 UseCommitment, 4 Target, 4 Adapter, 4 Invocation
