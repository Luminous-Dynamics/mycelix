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

fun criticalCount[s: Subject]: Int {
  #(s.roles & (Operator + Verifier + EvidenceArchive + Adjudicator))
}

pred independentReview[s: Subject] {
  some r: s.externalReviewers | r != s
}

fact RoleConcentrationRequiresFinding {
  all s: Subject |
    criticalCount[s] >= 2 implies s.conflictFinding = On
}

fact FullControlRequiresIndependentReview {
  all s: Subject |
    criticalCount[s] = 4 implies independentReview[s]
}

pred ConcentratedRolesWithFinding {
  some s: Subject |
    criticalCount[s] >= 2 and
    s.conflictFinding = On
}

pred FullControlWithIndependentReview {
  some s: Subject |
    criticalCount[s] = 4 and
    independentReview[s]
}

pred SelfReviewOnlyFullControl {
  some s: Subject |
    criticalCount[s] = 4 and
    s.externalReviewers = {s}
}

run ConcentratedRolesWithFinding
  for 4 int, 4 Subject, 4 Role

run FullControlWithIndependentReview
  for 4 int, 4 Subject, 4 Role

run SelfReviewOnlyFullControl
  for 4 int, 4 Subject, 4 Role

assert RoleConcentrationRequiresFindingInvariant {
  all s: Subject |
    criticalCount[s] >= 2 implies s.conflictFinding = On
}

assert FullControlRequiresIndependentReviewInvariant {
  all s: Subject |
    criticalCount[s] = 4 implies independentReview[s]
}

assert SelfReviewAloneDoesNotCountAsIndependentReview {
  all s: Subject |
    s.externalReviewers = s implies not independentReview[s]
}

check RoleConcentrationRequiresFindingInvariant
  for 4 int, 4 Subject, 4 Role expect 0

check FullControlRequiresIndependentReviewInvariant
  for 4 int, 4 Subject, 4 Role expect 0

check SelfReviewAloneDoesNotCountAsIndependentReview
  for 4 int, 4 Subject expect 0
