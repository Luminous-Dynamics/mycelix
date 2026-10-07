module ArtificialSovereigntyRoleConcentrationV1

enum SubjectClass { Human, Artificial }
enum Role { Operator, Verifier, EvidenceArchive, Adjudicator }
enum Bit { On, Off }

sig Subject {
  class: one SubjectClass,
  roles: set Role,
  conflictFinding: one Bit,
  externalReviewers: set Subject
}

fun roleCount[s: Subject]: Int {
  #s.roles
}

pred independentReview[s: Subject] {
  some r: s.externalReviewers |
    r != s and
    no r.roles
}

fact RoleConcentrationRequiresFinding {
  all s: Subject |
    roleCount[s] >= 2 implies s.conflictFinding = On
}

fact FullControlRequiresIndependentReview {
  all s: Subject |
    roleCount[s] = 4 implies independentReview[s]
}

fact ReviewerRoleDisjointness {
  all s, r: Subject |
    r in s.externalReviewers implies no r.roles
}

pred ConcentratedRolesWithFinding {
  some s: Subject |
    roleCount[s] >= 2 and
    s.conflictFinding = On
}

pred FullControlWithIndependentReview {
  some s: Subject |
    roleCount[s] = 4 and
    independentReview[s]
}

pred FullControlWithRoleDisjointReview {
  some s, r: Subject |
    roleCount[s] = 4 and
    r in s.externalReviewers and
    r != s and
    no r.roles
}

pred SelfReviewOnlyFullControl {
  some s: Subject |
    roleCount[s] = 4 and
    s.externalReviewers = {s}
}

pred SameRoleReviewerFullControl {
  some disj s, r: Subject |
    roleCount[s] = 4 and
    r in s.externalReviewers and
    r != s and
    some r.roles
}

pred ReviewerRoleDriftState {
  some disj s, r: Subject |
    r in s.externalReviewers and
    some r.roles
}

run ConcentratedRolesWithFinding
  for 4 int, 4 Subject, 4 Role

run FullControlWithIndependentReview
  for 4 int, 4 Subject, 4 Role

run FullControlWithRoleDisjointReview
  for 4 int, 4 Subject, 4 Role

run SelfReviewOnlyFullControl
  for 4 int, 4 Subject, 4 Role

run SameRoleReviewerFullControl
  for 4 int, 4 Subject, 4 Role

run ReviewerRoleDriftState
  for 4 int, 4 Subject, 4 Role

assert RoleConcentrationRequiresFindingInvariant {
  all s: Subject |
    roleCount[s] >= 2 implies s.conflictFinding = On
}

assert FullControlRequiresIndependentReviewInvariant {
  all s: Subject |
    roleCount[s] = 4 implies independentReview[s]
}

assert SelfReviewAloneDoesNotCountAsIndependentReview {
  all s: Subject |
    s.externalReviewers = {s} implies not independentReview[s]
}

assert ReviewerCannotHoldQualificationRole {
  all s, r: Subject |
    r in s.externalReviewers implies no r.roles
}

check RoleConcentrationRequiresFindingInvariant
  for 4 int, 4 Subject, 4 Role expect 0

check FullControlRequiresIndependentReviewInvariant
  for 4 int, 4 Subject, 4 Role expect 0

check SelfReviewAloneDoesNotCountAsIndependentReview
  for 4 int, 4 Subject expect 0

check ReviewerCannotHoldQualificationRole
  for 4 int, 4 Subject, 4 Role expect 0
