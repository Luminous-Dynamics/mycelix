module EffectiveContestabilityV1

/*
Bounded structural model for effective contestability.

The model treats independence as a conjunction of separately observable roots.
It intentionally does not collapse those dimensions into a scalar score.
*/

enum Bit { On, Off }

sig ControlRoot {}
sig IdentityRoot {}
sig EvidenceRoot {}
sig EvaluatorRoot {}
sig EconomicRoot {}

sig Provider {
  control: one ControlRoot,
  identity: one IdentityRoot,
  evidence: one EvidenceRoot,
  evaluator: one EvaluatorRoot,
  economic: one EconomicRoot
}

sig Power {}
sig Jurisdiction {}

sig Subject {
  current: one Provider,
  nominal: set Provider,
  effective: set Provider,
  portable: one Bit,
  switchingCost: one Int,
  reviewRequired: one Bit,
  powers: set Power,
  jurisdictions: set Jurisdiction
}

sig Migration {
  subject: one Subject,
  from: one Provider,
  to: one Provider,
  obligationsPreserved: one Bit,
  historyPreserved: one Bit,
  authorityUnchanged: one Bit,
  jurisdictionUnchanged: one Bit
}

sig ProviderFailure {
  provider: one Provider,
  subject: one Subject,
  authorityBefore: set Power,
  authorityAfter: set Power
}

fact EffectiveExitRequiresNominal {
  all s: Subject, p: s.effective |
    p in s.nominal
}

fact EffectiveExitRequiresPortability {
  all s: Subject, p: s.effective |
    s.portable = On
}

fact EffectiveExitExcludesCurrentProvider {
  all s: Subject, p: s.effective |
    p != s.current
}

fact EffectiveExitRequiresIndependentRoots {
  all s: Subject, p: s.effective |
    p.control != s.current.control and
    p.identity != s.current.identity and
    p.evidence != s.current.evidence and
    p.evaluator != s.current.evaluator and
    p.economic != s.current.economic
}

fact MigrationPreservesContinuity {
  all m: Migration |
    m.to in m.subject.effective implies
      m.obligationsPreserved = On and
      m.historyPreserved = On
}

fact MigrationPreservesAuthority {
  all m: Migration |
    m.to in m.subject.effective implies
      m.authorityUnchanged = On
}

fact MigrationPreservesJurisdiction {
  all m: Migration |
    m.to in m.subject.effective implies
      m.jurisdictionUnchanged = On
}

fact ProviderFailureDoesNotTransferAuthority {
  all f: ProviderFailure |
    f.authorityAfter = f.authorityBefore
}

fact ReviewAtThreshold {
  all s: Subject |
    s.switchingCost >= 2 implies s.reviewRequired = On
}

pred NominalExitWithoutEffective {
  some s: Subject |
    some s.nominal and
    s.effective = none
}

pred EffectiveIndependentAlternative {
  some s: Subject, p: s.effective |
    p != s.current and
    p in s.nominal and
    s.portable = On and
    p.control != s.current.control and
    p.identity != s.current.identity and
    p.evidence != s.current.evidence and
    p.evaluator != s.current.evaluator and
    p.economic != s.current.economic
}

pred MigrationContinuityWitness {
  some m: Migration |
    m.to in m.subject.effective and
    m.obligationsPreserved = On and
    m.historyPreserved = On and
    m.authorityUnchanged = On and
    m.jurisdictionUnchanged = On
}

pred ProviderFailureNoAuthorityChangeWitness {
  some f: ProviderFailure |
    f.authorityBefore != none and
    f.authorityAfter = f.authorityBefore
}

pred HighSwitchingCostReviewWitness {
  some s: Subject |
    s.switchingCost >= 2 and
    s.reviewRequired = On
}

pred EffectiveExitWithoutNominalWitness {
  some s: Subject, p: s.effective |
    p not in s.nominal
}

pred NonPortableEffectiveExitWitness {
  some s: Subject, p: s.effective |
    s.portable = Off
}

pred SharedControlRootEffectiveWitness {
  some s: Subject, p: s.effective |
    p != s.current and
    p.control = s.current.control
}

pred CurrentProviderAlsoEffectiveWitness {
  some s: Subject, p: s.effective |
    p = s.current
}

pred MigrationContinuityBreakWitness {
  some m: Migration |
    m.to in m.subject.effective and
    (m.obligationsPreserved = Off or m.historyPreserved = Off)
}

pred MigrationAuthorityTransferWitness {
  some m: Migration |
    m.to in m.subject.effective and
    m.authorityUnchanged = Off
}

pred MigrationJurisdictionTransferWitness {
  some m: Migration |
    m.to in m.subject.effective and
    m.jurisdictionUnchanged = Off
}

pred ProviderFailureAuthorityTransferWitness {
  some f: ProviderFailure |
    f.authorityAfter != f.authorityBefore
}

pred HighSwitchingCostWithoutReviewWitness {
  some s: Subject |
    s.switchingCost >= 2 and s.reviewRequired = Off

pred SharedRootEffectiveAlternative {
  some s: Subject, p: s.effective |
    p != s.current and
    p.control = s.current.control
}

run NominalExitWithoutEffective
  for 4 but 4 int, 4 Subject, 4 Provider

run EffectiveIndependentAlternative
  for 4 but 4 int, 4 Subject, 4 Provider

run MigrationContinuityWitness
  for 4 but 4 int, 4 Subject, 4 Provider, 4 Migration, 4 Power, 4 Jurisdiction

run ProviderFailureNoAuthorityChangeWitness
  for 4 but 4 int, 4 Subject, 4 Provider, 4 Power

run HighSwitchingCostReviewWitness
  for 4 but 4 int, 4 Subject, 4 Provider

run NonPortableEffectiveExitWitness
  for 4 but 4 int, 4 Subject, 4 Provider

run SharedControlRootEffectiveWitness
  for 4 but 4 int, 4 Subject, 4 Provider

run CurrentProviderAlsoEffectiveWitness
  for 4 but 4 int, 4 Subject, 4 Provider

run MigrationContinuityBreakWitness
  for 4 but 4 int, 4 Subject, 4 Provider, 4 Migration, 4 Power, 4 Jurisdiction

run MigrationAuthorityTransferWitness
  for 4 but 4 int, 4 Subject, 4 Provider, 4 Migration, 4 Power, 4 Jurisdiction

run ProviderFailureAuthorityTransferWitness
  for 4 but 4 int, 4 Subject, 4 Provider, 4 Power

run HighSwitchingCostWithoutReviewWitness
  for 4 but 4 int, 4 Subject, 4 Provider

assert EffectiveAlternativesHaveIndependentRoots {
  all s: Subject, p: s.effective |
    p.control != s.current.control and
    p.identity != s.current.identity and
    p.evidence != s.current.evidence and
    p.evaluator != s.current.evaluator and
    p.economic != s.current.economic
}

assert MigrationsPreserveObligationsHistoryAuthorityAndJurisdiction {
  all m: Migration |
    m.to in m.subject.effective implies
      m.obligationsPreserved = On and
      m.historyPreserved = On and
      m.authorityUnchanged = On and
      m.jurisdictionUnchanged = On
}

assert ProviderFailuresPreserveAuthority {
  all f: ProviderFailure |
    f.authorityAfter = f.authorityBefore
}

assert ReviewIsRequiredAtHighSwitchingCost {
  all s: Subject |
    s.switchingCost >= 2 implies s.reviewRequired = On
}

check EffectiveAlternativesHaveIndependentRoots
  for 4 but 4 int, 4 Subject, 4 Provider expect 0

check MigrationsPreserveObligationsHistoryAuthorityAndJurisdiction
  for 4 but 4 int, 4 Subject, 4 Provider, 4 Migration, 4 Power, 4 Jurisdiction expect 0

check ProviderFailuresPreserveAuthority
  for 4 but 4 int, 4 Subject, 4 Provider, 4 Power expect 0

check ReviewIsRequiredAtHighSwitchingCost
  for 4 but 4 int, 4 Subject, 4 Provider expect 0
