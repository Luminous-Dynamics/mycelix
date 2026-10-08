module EffectiveContestabilityClosureV1

enum Bit { On, Off }
enum Domain { Control, Identity, Evidence, Evaluator, Economic, CriticalInfrastructure }

abstract sig Node {
  domain: one Domain,
  critical: one Bit,
  dependsOn: set Node
}

sig DependencyNode extends Node {}

sig Provider extends Node {
  controlRoot: one DependencyNode,
  identityRoot: one DependencyNode,
  evidenceRoot: one DependencyNode,
  evaluatorRoot: one DependencyNode,
  economicRoot: one DependencyNode
}

sig Subject {
  current: one Provider,
  nominal: set Provider,
  effective: set Provider,
  portable: one Bit
}

fact ProviderRootsAreDependencies {
  all p: Provider |
    p.controlRoot + p.identityRoot + p.evidenceRoot + p.evaluatorRoot + p.economicRoot in p.dependsOn
}

fact EffectiveAlternativeHasDisjointCriticalClosure {
  all s: Subject, p: s.effective |
    p != s.current and
    p in s.nominal and
    s.portable = On and
    p.controlRoot != s.current.controlRoot and
    p.identityRoot != s.current.identityRoot and
    p.evidenceRoot != s.current.evidenceRoot and
    p.evaluatorRoot != s.current.evaluatorRoot and
    p.economicRoot != s.current.economicRoot and
    no (p.*dependsOn & s.current.*dependsOn & {n: Node | n.critical = On})
}

pred NominalExitWithoutEffective {
  some s: Subject |
    some s.nominal and s.effective = none
}

pred EffectiveIndependentWitness {
  some s: Subject, p: s.effective |
    p != s.current and
    p in s.nominal and
    s.portable = On and
    p.controlRoot != s.current.controlRoot and
    p.identityRoot != s.current.identityRoot and
    p.evidenceRoot != s.current.evidenceRoot and
    p.evaluatorRoot != s.current.evaluatorRoot and
    p.economicRoot != s.current.economicRoot and
    no (p.*dependsOn & s.current.*dependsOn & {n: Node | n.critical = On})
}

pred SharedCriticalAncestorNominalWitness {
  some s: Subject, p: Provider, n: DependencyNode |
    p != s.current and
    p in s.nominal and
    p.controlRoot != s.current.controlRoot and
    p.identityRoot != s.current.identityRoot and
    p.evidenceRoot != s.current.evidenceRoot and
    p.evaluatorRoot != s.current.evaluatorRoot and
    p.economicRoot != s.current.economicRoot and
    n in p.*dependsOn and
    n in s.current.*dependsOn and
    n not in p.dependsOn and
    n not in s.current.dependsOn and
    n.critical = On
}

run NominalExitWithoutEffective
  for 6 but 6 Subject, 6 Provider, 12 Node

run EffectiveIndependentWitness
  for 6 but 6 Subject, 6 Provider, 12 Node

run SharedCriticalAncestorNominalWitness
  for 6 but 6 Subject, 6 Provider, 12 Node

assert EffectiveAlternativesHaveNoSharedCriticalDependency {
  all s: Subject, p: s.effective |
    no (p.*dependsOn & s.current.*dependsOn & {n: Node | n.critical = On})
}

assert DirectRootSeparationRemainsNecessary {
  all s: Subject, p: s.effective |
    p.controlRoot != s.current.controlRoot and
    p.identityRoot != s.current.identityRoot and
    p.evidenceRoot != s.current.evidenceRoot and
    p.evaluatorRoot != s.current.evaluatorRoot and
    p.economicRoot != s.current.economicRoot
}

check EffectiveAlternativesHaveNoSharedCriticalDependency
  for 6 but 6 Subject, 6 Provider, 12 Node expect 0

check DirectRootSeparationRemainsNecessary
  for 6 but 6 Subject, 6 Provider, 12 Node expect 0
