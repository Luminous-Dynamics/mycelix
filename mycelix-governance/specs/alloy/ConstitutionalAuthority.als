module ConstitutionalAuthority

/*
Bounded structural model for the Mycelix constitutional authority algebra.

This model is intentionally static. It asks whether unsafe constitutional
structures are constructible inside a finite scope. Temporal races belong in
TLA+.

The SovereignPower census is intended to correspond exactly to the Rust
`ConstitutionalPower` enum. MYC-CONST-003CR adds an independent drift gate so
that this correspondence cannot silently decay over time.
*/

enum PrincipalClass {
  Constituent,
  Deliberative,
  Stewardship,
  Justice,
  Integrity,
  CivicMandate,
  RightsDefender,
  PublicEvidence,
  FutureGenerations,
  FiscalObservatory,
  PublicService,
  ProsecutionService,
  AutomatedAgent
}

enum SovereignPower {
  ProposeOrdinaryLaw,
  EnactOrdinaryLaw,
  AppropriatePublicFunds,
  RatifyTreaty,
  AuthorizeEmergency,
  ConductLegislativeOversight,
  ExecuteLaw,
  ExecuteAppropriation,
  AdministerPublicService,
  DirectPublicAdministration,
  DeclareProvisionalEmergency,
  AdjudicateDispute,
  ConductConstitutionalReview,
  IssueJudicialRemedy,
  AuditPublicExpenditure,
  AuditAuthorityUse,
  InvestigatePublicIntegrity,
  PublishIntegrityFinding,
  ReferForProsecution,
  AdministerElection,
  CertifyMandate,
  AdministerRecall,
  AdministerInitiative,
  AdministerSortition,
  VerifyCivicEligibility,
  CallConstitutionalConvention,
  RatifyStructuralConstitution,
  RatifyFoundationalCovenant,
  WithdrawConstituentDelegation,
  InitiateRightsChallenge,
  PublishEvidenceAssessment,
  InitiateFutureGenerationsReview,
  PublishFiscalAssessment,
  CertifyPublicServiceQualification,
  InitiatePublicProsecution
}

enum Entitlement {
  RequestLawfulRecord,
  AccessSubmittedEvidence,
  ReceiveDecisionNotice,
  ObtainDecisionReasons,
  SubmitEvidence,
  ChallengePublicAction,
  SeekJudicialReview,
  PublishProtectedOversightReport,
  ReceiveProtectedDisclosure
}

fun Branches : set PrincipalClass {
  Deliberative + Stewardship + Justice + Integrity + CivicMandate
}

fun Guardians : set PrincipalClass {
  RightsDefender + PublicEvidence + FutureGenerations + FiscalObservatory +
  PublicService + ProsecutionService
}

sig Allocation {
  power: one SovereignPower,
  owner: one PrincipalClass
}

fact ExactlyOneAllocationPerPower {
  all p: SovereignPower | one a: Allocation | a.power = p
}

fact ConstitutionalOwnershipMap {
  all a: Allocation |
    (a.power in ProposeOrdinaryLaw + EnactOrdinaryLaw + AppropriatePublicFunds +
                RatifyTreaty + AuthorizeEmergency + ConductLegislativeOversight)
      iff a.owner = Deliberative
  all a: Allocation |
    (a.power in ExecuteLaw + ExecuteAppropriation + AdministerPublicService +
                DirectPublicAdministration + DeclareProvisionalEmergency)
      iff a.owner = Stewardship
  all a: Allocation |
    (a.power in AdjudicateDispute + ConductConstitutionalReview + IssueJudicialRemedy)
      iff a.owner = Justice
  all a: Allocation |
    (a.power in AuditPublicExpenditure + AuditAuthorityUse + InvestigatePublicIntegrity +
                PublishIntegrityFinding + ReferForProsecution)
      iff a.owner = Integrity
  all a: Allocation |
    (a.power in AdministerElection + CertifyMandate + AdministerRecall +
                AdministerInitiative + AdministerSortition + VerifyCivicEligibility)
      iff a.owner = CivicMandate
  all a: Allocation |
    (a.power in CallConstitutionalConvention + RatifyStructuralConstitution +
                RatifyFoundationalCovenant + WithdrawConstituentDelegation)
      iff a.owner = Constituent
  all a: Allocation |
    (a.power = InitiateRightsChallenge) iff a.owner = RightsDefender
  all a: Allocation |
    (a.power = PublishEvidenceAssessment) iff a.owner = PublicEvidence
  all a: Allocation |
    (a.power = InitiateFutureGenerationsReview) iff a.owner = FutureGenerations
  all a: Allocation |
    (a.power = PublishFiscalAssessment) iff a.owner = FiscalObservatory
  all a: Allocation |
    (a.power = CertifyPublicServiceQualification) iff a.owner = PublicService
  all a: Allocation |
    (a.power = InitiatePublicProsecution) iff a.owner = ProsecutionService
}

sig Holder {
  class: one PrincipalClass
}

sig PowerGrant {
  holder: one Holder,
  allocation: one Allocation
}

fact PowerGrantMatchesOwnerClass {
  all g: PowerGrant | g.holder.class = g.allocation.owner
}

sig EntitlementGrant {
  holder: one Holder,
  entitlement: one Entitlement
}

/* Entitlements deliberately do not imply a PowerGrant. */
pred EntitlementWithoutSovereignPower {
  some e: EntitlementGrant |
    no p: PowerGrant | p.holder = e.holder
}

/* Configurable separation-of-duty mechanism. */
enum Role {
  Investigator,
  Prosecutor,
  Judge,
  Executor,
  Auditor,
  ElectionAdministrator,
  Candidate,
  Certifier
}

sig Matter {}

sig RoleAssignment {
  holder: one Holder,
  role: one Role,
  matter: lone Matter
}

one sig SeparationProfile {
  staticBan: Role -> Role,
  dynamicBan: Role -> Role
}

fact SeparationRelationsAreSymmetricAndIrreflexive {
  SeparationProfile.staticBan = ~SeparationProfile.staticBan
  SeparationProfile.dynamicBan = ~SeparationProfile.dynamicBan
  no iden & SeparationProfile.staticBan
  no iden & SeparationProfile.dynamicBan
}

fact EnforceConfiguredSeparation {
  all disj a, b: RoleAssignment |
    a.holder = b.holder implies {
      not (a.role -> b.role in SeparationProfile.staticBan)
      (some a.matter and a.matter = b.matter) implies
        not (a.role -> b.role in SeparationProfile.dynamicBan)
    }
}

pred NontrivialSeparationConfiguration {
  some SeparationProfile.staticBan
  some SeparationProfile.dynamicBan
  some RoleAssignment
}

/* Independent concurrence is domain/holder-aware, not just key-count-aware. */
sig Envelope {}
sig Key {}

sig Approval {
  holder: one Holder,
  key: one Key,
  envelope: one Envelope
}

sig Concurrence {
  envelope: one Envelope,
  approvals: some Approval,
  requiredDomains: set PrincipalClass
}

fact ConcurrenceBindsOneEnvelopeAndUniqueHolders {
  all c: Concurrence | {
    c.approvals.envelope = c.envelope
    all disj a, b: c.approvals | a.holder != b.holder
    c.requiredDomains in c.approvals.holder.class
  }
}

assert EveryPowerHasExactlyOneOwner {
  all p: SovereignPower | one a: Allocation | a.power = p
}

assert AutomatedAgentOwnsNoSovereignPower {
  no a: Allocation | a.owner = AutomatedAgent
}

assert ConstituentSovereigntyIsNotABranch {
  Constituent not in Branches
}

assert BranchCannotOwnFoundationalRatification {
  no a: Allocation |
    a.owner in Branches and
    a.power in RatifyStructuralConstitution + RatifyFoundationalCovenant + WithdrawConstituentDelegation
}

assert StewardshipCannotCertifyMandate {
  no a: Allocation | a.owner = Stewardship and a.power = CertifyMandate
}

assert IntegrityCannotAdjudicate {
  no a: Allocation | a.owner = Integrity and a.power = AdjudicateDispute
}

assert JusticeCannotExecuteAppropriation {
  no a: Allocation | a.owner = Justice and a.power = ExecuteAppropriation
}

assert ConfiguredStaticSeparationCannotBeViolated {
  no disj a, b: RoleAssignment |
    a.holder = b.holder and a.role -> b.role in SeparationProfile.staticBan
}

assert ConfiguredDynamicSeparationCannotBeViolatedOnSameMatter {
  no disj a, b: RoleAssignment |
    a.holder = b.holder and some a.matter and a.matter = b.matter and
    a.role -> b.role in SeparationProfile.dynamicBan
}

assert ConcurrenceCannotCountOneHolderTwice {
  all c: Concurrence |
    all disj a, b: c.approvals | a.holder != b.holder
}

/* Negative control: raw key count alone can manufacture fake independence. */
pred WeakKeyCountCanFakeIndependence {
  some h: Holder, disj k1, k2: Key, e: Envelope |
    some disj a1, a2: Approval |
      a1.holder = h and a2.holder = h and
      a1.key = k1 and a2.key = k2 and
      a1.envelope = e and a2.envelope = e
}

/*
There are exactly 35 SovereignPower atoms and the ownership fact requires one
Allocation per power. Every command therefore scopes Allocation explicitly to
35. Without this, a small default scope can make the whole model UNSAT and turn
otherwise useful checks into vacuous success.
*/

/* Expected-SAT witness commands guard against vacuous model configurations. */
run EntitlementWithoutSovereignPower for 3 but exactly 35 Allocation, 6 Holder, 6 PowerGrant, 6 EntitlementGrant expect 1
run NontrivialSeparationConfiguration for 3 but exactly 35 Allocation, 8 Holder, 12 RoleAssignment, 5 Matter expect 1
run WeakKeyCountCanFakeIndependence for 3 but exactly 35 Allocation, 6 Holder, 6 Key, 8 Approval, 3 Envelope expect 1

/* Expected-UNSAT checks: no counterexample should exist within the stated scope. */
check EveryPowerHasExactlyOneOwner for 3 but exactly 35 Allocation expect 0
check AutomatedAgentOwnsNoSovereignPower for 3 but exactly 35 Allocation expect 0
check ConstituentSovereigntyIsNotABranch for 3 but exactly 35 Allocation expect 0
check BranchCannotOwnFoundationalRatification for 3 but exactly 35 Allocation expect 0
check StewardshipCannotCertifyMandate for 3 but exactly 35 Allocation expect 0
check IntegrityCannotAdjudicate for 3 but exactly 35 Allocation expect 0
check JusticeCannotExecuteAppropriation for 3 but exactly 35 Allocation expect 0
check ConfiguredStaticSeparationCannotBeViolated for 3 but exactly 35 Allocation, 8 Holder, 12 RoleAssignment, 5 Matter expect 0
check ConfiguredDynamicSeparationCannotBeViolatedOnSameMatter for 3 but exactly 35 Allocation, 8 Holder, 12 RoleAssignment, 5 Matter expect 0
check ConcurrenceCannotCountOneHolderTwice for 3 but exactly 35 Allocation, 8 Holder, 8 Key, 12 Approval, 4 Concurrence, 4 Envelope expect 0
