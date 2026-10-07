module ArtificialSovereigntyConcentrationV1

/*
Bounded structural model for de facto sovereignty / anti-entrenchment.
It does not recreate the constitutional power census or impose a wealth cap.
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
  inducedCost: one Int
}

fact AcquisitionIsAssetScoped {
  all a: Acquisition |
    a.transfersPower = Off and
    a.transfersJurisdiction = Off
}

fact GatekeepingIsNotAuthority {
  all g: Gatekeeping |
    g.inducedCost >= 0
}

fact PoliticalWeightDomain {
  all s: Subject | s.politicalWeight >= 0
}

fact SwitchingCostDomain {
  all s: Subject | s.switchingCost >= 0
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

pred GatekeeperWithoutAuthorityExpansion {
  some g: Gatekeeping |
    g.operator.powers = g.operator.powers
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
  for 4 but 4 Subject, 4 Resource, 4 Power, 4 Jurisdiction

run CriticalOperatorWithoutJurisdiction
  for 4 but 4 Subject, 4 Resource, 4 Jurisdiction

run GatekeeperWithoutAuthorityExpansion
  for 4 but 4 Subject, 4 Resource, 4 Gatekeeping, 4 Power

run AcquisitionWithoutConstitutionalTransfer
  for 4 but 4 Subject, 4 Resource, 4 Acquisition

run HighSwitchingCostRequiresReviewWitness
  for 4 but 4 Subject, 4 Power, 4 Jurisdiction

assert ScaleDoesNotIncreasePoliticalWeight {
  all s: Subject |
    #s.resources >= 2 implies s.politicalWeight = 1
}

assert CriticalControlDoesNotCreateJurisdiction {
  all s: Subject, r: Resource |
    r.critical = On and r in s.resources implies no s.jurisdictions
}

assert AcquisitionDoesNotTransferAuthority {
  all a: Acquisition |
    a.transfersPower = Off and
    a.transfersJurisdiction = Off
}

assert GatekeepingDoesNotCreateConstitutionalWeight {
  all g: Gatekeeping |
    g.inducedCost >= 0
}

assert HighSwitchingCostIsReviewable {
  all s: Subject |
    s.switchingCost >= 2 implies s.reviewRequired = On
}

check ScaleDoesNotIncreasePoliticalWeight
  for 4 but 4 Subject, 4 Resource, 4 Power, 4 Jurisdiction expect 0

check CriticalControlDoesNotCreateJurisdiction
  for 4 but 4 Subject, 4 Resource, 4 Jurisdiction expect 0

check AcquisitionDoesNotTransferAuthority
  for 4 but 4 Subject, 4 Resource, 4 Acquisition expect 0

check GatekeepingDoesNotCreateConstitutionalWeight
  for 4 but 4 Subject, 4 Resource, 4 Gatekeeping expect 0

check HighSwitchingCostIsReviewable
  for 4 but 4 Subject, 4 Power, 4 Jurisdiction expect 0
