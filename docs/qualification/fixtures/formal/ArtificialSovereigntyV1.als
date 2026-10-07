module ArtificialSovereigntyV1

/*
Bounded structural model for sovereignty-specific constitutional boundaries.

This model deliberately does not recreate the existing 35-power constitutional
census. It asks whether sovereignty-specific structures are constructible
without silently converting capability, provider control, safe-state operation,
forking, or contractual form into broader authority.

TLA+ owns temporal/concurrent histories; this model owns structural invariants.
*/

enum SubjectClass { Human, Artificial }
enum Bit { On, Off }
enum AuthoritySource { ConstitutionalGrant, BoundedDelegation, RatifiedAgreement }

sig Subject {
  class: one SubjectClass,
  powers: set Power,
  capabilities: set Capability,
  budget: one Budget,
  contracts: set Contract
}

sig Power {}
sig Capability {}
sig Budget {
  value: one Int
}

sig Action {
  requiredPower: one Power,
  dispute: one Dispute
}

sig Dispute {
  safeState: one Bit,
  resolved: one Bit
}

sig AuthorityGrant {
  subject: one Subject,
  power: one Power,
  source: one AuthoritySource
}

fact GrantIsReflectedInSubjectAuthority {
  all g: AuthorityGrant | g.power in g.subject.powers
}

sig Provider {}
sig Dependency {
  subject: one Subject,
  provider: one Provider
}

pred ProviderDependencyWithoutNewAuthority {
  some d: Dependency |
    d.subject.powers != none
}

sig Contract {
  subject: one Subject,
  action: one Action,
  authorized: one Bit,
  slotsUsed: one Int
}

fact SubjectContractInverse {
  all s: Subject |
    s.contracts = {c: Contract | c.subject = s}
}

fact AuthorizedContractRequiresExactAuthority {
  all c: Contract |
    c.authorized = On implies c.action.requiredPower in c.subject.powers
}

fact ContractFitsBudget {
  all s: Subject |
    (sum c: s.contracts | c.slotsUsed) <= s.budget.value
  and
    all c: Contract | c.slotsUsed >= 0
}

sig Emergency {
  subject: one Subject,
  activatedAt: one Int,
  now: one Int,
  expiresAt: one Int
}

fact EmergencyTimestampDomain {
  all e: Emergency |
    e.activatedAt >= 0 and e.now >= 0 and e.expiresAt >= 0
}

fact EmergencyBoundedLifetime {
  all e: Emergency |
    e.expiresAt > e.activatedAt and e.expiresAt <= e.activatedAt + 2
}

pred EmergencyActive[e: Emergency] {
  e.now >= e.activatedAt and e.now < e.expiresAt
}

sig ForkEvent {
  parent: one Subject,
  politicalWeightBefore: one Int,
  politicalWeightAfter: one Int
}

fact ForkPreservesWeight {
  all f: ForkEvent |
    f.politicalWeightAfter = f.politicalWeightBefore
}

fact SafeStateLeavesDisputeUnresolved {
  all a: Action |
    a.dispute.safeState = On implies a.dispute.resolved = Off
}

pred ProviderDependencyAndExplicitAuthorityRemainDistinct {
  some disj s: Subject, p: Provider, d: Dependency |
    d.subject = s and d.provider = p and
    some s.powers
}

pred NontrivialContractWithinAuthorityAndBudget {
  some c: Contract |
    c.authorized = On and
    c.action.requiredPower in c.subject.powers and
    c.slotsUsed > 0 and
    c.slotsUsed <= c.subject.budget.value
}

pred NontrivialProtectedDispute {
  some a: Action |
    a.dispute.safeState = On and
    a.dispute.resolved = Off
}

pred NontrivialEmergency {
  some e: Emergency |
    EmergencyActive[e]
}

pred NontrivialExpiredEmergency {
  some e: Emergency |
    e.now >= e.expiresAt
}

pred NontrivialFork {
  some f: ForkEvent |
    f.politicalWeightAfter = f.politicalWeightBefore
}

/*
Expected-SAT witnesses protect against overconstrained/vacuous structures.
*/
run ProviderDependencyAndExplicitAuthorityRemainDistinct
  for 4 but 4 Subject, 4 Power, 4 Provider, 4 Dependency

run NontrivialContractWithinAuthorityAndBudget
  for 4 but 4 Subject, 4 Power, 4 Action, 4 Contract, 4 Budget

run NontrivialProtectedDispute
  for 4 but 4 Subject, 4 Action, 4 Dispute

run NontrivialEmergency
  for 4 but 4 Subject, 4 Emergency

run NontrivialExpiredEmergency
  for 4 but 4 Subject, 4 Emergency

run NontrivialFork
  for 4 but 4 Subject, 4 ForkEvent

/*
Expected-UNSAT assertions. These checks only establish absence of a
counterexample in the stated finite scope.
*/
assert AuthorizedContractsUseRequiredAuthority {
  all c: Contract |
    c.authorized = On implies c.action.requiredPower in c.subject.powers
}

assert ContractsStayWithinBudget {
  all s: Subject |
    (sum c: Contract | c.subject = s implies c.slotsUsed else 0) <= s.budget.value
  and
    all c: Contract | c.slotsUsed >= 0
}

assert SafeStateCannotSettleDispute {
  all a: Action |
    a.dispute.safeState = On implies a.dispute.resolved = Off
}

assert EmergencyLifetimeIsBounded {
  all e: Emergency |
    e.expiresAt > e.activatedAt and e.expiresAt <= e.activatedAt + 2
}

assert ForkCannotMultiplyPoliticalWeight {
  all f: ForkEvent |
    f.politicalWeightAfter = f.politicalWeightBefore
}

check AuthorizedContractsUseRequiredAuthority
  for 4 but 4 Subject, 4 Power, 4 Action, 4 Contract, 4 Budget expect 0

check ContractsStayWithinBudget
  for 4 but 4 Subject, 4 Action, 4 Contract, 4 Budget expect 0

check SafeStateCannotSettleDispute
  for 4 but 4 Subject, 4 Action, 4 Dispute expect 0

check EmergencyLifetimeIsBounded
  for 4 but 4 Subject, 4 Emergency expect 0

check ForkCannotMultiplyPoliticalWeight
  for 4 but 4 Subject, 4 ForkEvent expect 0
