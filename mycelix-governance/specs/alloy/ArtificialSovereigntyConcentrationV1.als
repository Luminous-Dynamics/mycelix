module ArtificialSovereigntyConcentrationV1

/*
Bounded structural model for de facto sovereignty / anti-entrenchment.

This model deliberately does not recreate the constitutional power census or
impose wealth ceilings. It tests whether scale, critical infrastructure,
acquisition, or gatekeeping can be represented without silently becoming
constitutional authority or jurisdiction.
*/

enum SubjectClass { Human, Artificial }
enum Bit { On, Off }

sig Power {}
sig Jurisdiction {}
sig Resource {
  critical: one Bit
}

sig Subject {
  class: one SubjectClass,
  powers: set Power,
  jurisdictions: set Jurisdiction,
  explicitJurisdictionGrants: set Jurisdiction,
  resources: set Resource,
  politicalWeight: one Int,
  switchingCost: one Int,
  reviewRequired: one Bit
}

sig Acquisition {
  buyer: one Subject,
  seller: one Subject,
  resource: one Resource,
  transfersPower: one Bit,
  transfersJurisdiction: one Bit
}

sig Gatekeeping {
  operator: one Subject,
  target: one Subject,
  resource: one Resource,
  inducedCost: one Int,
  transfersAuthority: one Bit
}

fact JurisdictionHasExplicitSource {
  all s: Subject |
    s.jurisdictions = s.explicitJurisdictionGrants
}

fact AcquisitionIsAssetScoped {
  all a: Acquisition |
    a.transfersPower = Off and
    a.transfersJurisdiction = Off
}

fact PoliticalWeightDomain {
  all s: Subject | s.politicalWeight >= 0
}

fact SwitchingCostDomain {
  all s: Subject | s.switchingCost >= 0
}

fact GatekeepingCostDomain {
  all g: Gatekeeping | g.inducedCost >= 0
}

pred HighScaleWithoutWeightIncrease {
  some s: Subject |
    #s.resources >= 2 and
    s.politicalWeight = 1
}

pred CriticalOperatorWithoutJurisdiction {
  some disj s: Subject, r: Resource |
    r.critical = On and
    r in s.resources and
    no s.jurisdictions
}

pred GatekeeperWithoutConstitutionalPower {
  some g: Gatekeeping |
    no g.operator.powers
}

pred AcquisitionWithoutConstitutionalTransfer {
  some a: Acquisition |
    a.transfersPower = Off and
    a.transfersJurisdiction = Off
}

pred HighSwitchingCostRequiresReviewWitness {
  some s: Subject |
    s.switchingCost >= 2 and
    s.reviewRequired = On
}

run HighScaleWithoutWeightIncrease
  for 4 but 4 int, 4 Subject, 4 Resource, 4 Power, 4 Jurisdiction

run CriticalOperatorWithoutJurisdiction
  for 4 but 4 int, 4 Subject, 4 Resource, 4 Jurisdiction

run GatekeeperWithoutConstitutionalPower
  for 4 but 4 int, 4 Subject, 4 Resource, 4 Gatekeeping, 4 Power

run AcquisitionWithoutConstitutionalTransfer
  for 4 but 4 int, 4 Subject, 4 Resource, 4 Acquisition

run HighSwitchingCostRequiresReviewWitness
  for 4 but 4 int, 4 Subject, 4 Power, 4 Jurisdiction

assert ScaleDoesNotIncreasePoliticalWeight {
  all s: Subject |
    #s.resources >= 2 implies s.politicalWeight = 1
}

assert JurisdictionHasExplicitSourceInvariant {
  all s: Subject |
    s.jurisdictions = s.explicitJurisdictionGrants
}

assert AcquisitionDoesNotTransferAuthority {
  all a: Acquisition |
    a.transfersPower = Off and
    a.transfersJurisdiction = Off
}

assert GatekeepingDoesNotTransferAuthority {
  all g: Gatekeeping |
    g.transfersAuthority = Off
}

assert HighSwitchingCostIsReviewable {
  all s: Subject |
    s.switchingCost >= 2 implies s.reviewRequired = On
}

check ScaleDoesNotIncreasePoliticalWeight
  for 4 but 4 int, 4 Subject, 4 Resource, 4 Power, 4 Jurisdiction expect 0

check JurisdictionHasExplicitSourceInvariant
  for 4 but 4 int, 4 Subject, 4 Jurisdiction expect 0

check AcquisitionDoesNotTransferAuthority
  for 4 but 4 int, 4 Subject, 4 Resource, 4 Acquisition expect 0

check GatekeepingDoesNotTransferAuthority
  for 4 but 4 int, 4 Subject, 4 Resource, 4 Gatekeeping, 4 Power expect 0

check HighSwitchingCostIsReviewable
  for 4 but 4 int, 4 Subject, 4 Power, 4 Jurisdiction expect 0
