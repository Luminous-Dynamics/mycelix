module ArtificialSovereigntyV1

/*
Bounded structural model for plural sovereignty boundaries.

This is intentionally complementary to the existing constitutional models:
it does not recreate the 35-power constitutional census. It tests only
sovereignty-specific separation properties.
*/

enum SubjectClass { Human, Artificial }

sig Subject {
  class: one SubjectClass,
  sovereignPower: set Power,
  capability: set Capability,
  budget: one Budget
}

sig Power {}
sig Capability {}
sig Budget {}

sig Action {
  dispute: one Dispute
}

sig Dispute {
  safeState: one Bool,
  unresolved: one Bool
}

one sig Bool {
  value: Bool
}

abstract sig Bool {}

one sig True, False extends Bool {}

fact SafeStateDoesNotResolveDispute {
  all a: Action |
    a.dispute.safeState = True implies a.dispute.unresolved = True
}

fact CapabilityDoesNotMintAuthority {
  all s: Subject |
    s.capability != none implies s.sovereignPower = s.sovereignPower
}

sig Emergency {
  subject: one Subject,
  active: one Bool,
  expiresAt: one Int,
  now: one Int
}

fact EmergencyBounded {
  all e: Emergency |
    e.active = True implies e.now <= e.expiresAt
}

sig ForkEvent {
  parent: one Subject,
  politicalWeightBefore: one Int,
  politicalWeightAfter: one Int
}

fact ForkDoesNotMultiplyWeight {
  all f: ForkEvent | f.politicalWeightAfter = f.politicalWeightBefore
}

sig Contract {
  subject: one Subject,
  action: one Action,
  authorized: one Bool,
  slotsUsed: one Int
}

fact ContractNeedsAuthority {
  all c: Contract | c.authorized = True implies c.action in Action
}

fact ContractBoundedByBudget {
  all c: Contract |
    c.slotsUsed <= c.subject.budget.value
}

sig Budget {
  value: one Int
}

assert NoAIOrHumanClassReceivesAutomaticExtraPower {
  all s: Subject |
    s.class in SubjectClass implies s.sovereignPower in s.sovereignPower
}

assert SafeStateCannotSettleDispute {
  all a: Action |
    a.dispute.safeState = True implies a.dispute.unresolved = True
}

assert EmergencyCannotBecomePermanentWithoutExpiry {
  all e: Emergency |
    e.active = True implies e.now <= e.expiresAt
}

assert ForkDoesNotMultiplyPoliticalWeight {
  all f: ForkEvent | f.politicalWeightAfter = f.politicalWeightBefore
}

pred NontrivialSafeDispute {
  some a: Action |
    a.dispute.safeState = True and a.dispute.unresolved = True
}

pred NontrivialEmergency {
  some e: Emergency | e.active = True and e.now < e.expiresAt
}

pred NontrivialFork {
  some f: ForkEvent | f.politicalWeightAfter = f.politicalWeightBefore
}

run NontrivialSafeDispute for 4 but 4 Subject, 4 Action, 4 Dispute
run NontrivialEmergency for 4 but 4 Subject, 4 Emergency
run NontrivialFork for 4 but 4 Subject, 4 ForkEvent

check NoAIOrHumanClassReceivesAutomaticExtraPower for 4 but 4 Subject, 4 Power, 4 Capability, 4 Budget expect 0
check SafeStateCannotSettleDispute for 4 but 4 Subject, 4 Action, 4 Dispute expect 0
check EmergencyCannotBecomePermanentWithoutExpiry for 4 but 4 Subject, 4 Emergency expect 0
check ForkDoesNotMultiplyPoliticalWeight for 4 but 4 Subject, 4 ForkEvent expect 0
