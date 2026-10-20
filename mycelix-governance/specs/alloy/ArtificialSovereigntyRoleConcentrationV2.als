module ArtificialSovereigntyRoleConcentrationV2

enum SubjectClass { Human, Artificial }
enum Role { Operator, Verifier, EvidenceArchive, Adjudicator }
enum Bit { On, Off }

sig Subject {
  class: one SubjectClass,
  roles: set Role,
  conflictFinding: one Bit
}

sig Review {
  subject: one Subject,
  reviewer: one Subject,
  active: one Bit
}

pred independentReview[s: Subject] {
  some v: Review |
    v.subject = s and
    v.active = On and
    v.reviewer != s and
    no v.reviewer.roles
}

fact RoleConcentrationRequiresFinding {
  all s: Subject |
    #s.roles >= 2 implies s.conflictFinding = On
}

fact FullControlRequiresIndependentReview {
  all s: Subject |
    #s.roles = 4 implies independentReview[s]
}

fact ActiveReviewRoleDisjointness {
  all v: Review |
    v.active = On implies
      v.reviewer != v.subject and
      no v.reviewer.roles
}

pred FullControlWithIndependentReview {
  some s: Subject |
    #s.roles = 4 and
    independentReview[s]
}

pred ConcentratedRolesWithFinding {
  some s: Subject |
    #s.roles >= 2 and
    s.conflictFinding = On
}

pred ClosedReviewReleasesRoleLock {
  some v: Review |
    v.active = Off and
    some v.reviewer.roles
}

pred SelfReviewOnlyFullControl {
  some s: Subject |
    #s.roles = 4 and
    some v: Review |
      v.subject = s and v.reviewer = s and v.active = On
}

pred SameRoleReviewerFullControl {
  some disj s, r: Subject |
    #s.roles = 4 and
    some v: Review |
      v.subject = s and v.reviewer = r and v.active = On and
      some r.roles
}

pred ReviewerRoleDriftState {
  some disj s, r: Subject |
    some v: Review |
      v.subject = s and v.reviewer = r and v.active = On and
      some r.roles
}

run ConcentratedRolesWithFinding
  for 4 int, 4 Subject, 4 Role, 4 Review

run FullControlWithIndependentReview
  for 4 int, 4 Subject, 4 Role, 4 Review

run ClosedReviewReleasesRoleLock
  for 4 int, 4 Subject, 4 Role, 4 Review

run SelfReviewOnlyFullControl
  for 4 int, 4 Subject, 4 Role, 4 Review

run SameRoleReviewerFullControl
  for 4 int, 4 Subject, 4 Role, 4 Review

run ReviewerRoleDriftState
  for 4 int, 4 Subject, 4 Role, 4 Review

assert RoleConcentrationRequiresFindingInvariant {
  all s: Subject |
    #s.roles >= 2 implies s.conflictFinding = On
}

assert FullControlRequiresIndependentReviewInvariant {
  all s: Subject |
    #s.roles = 4 implies independentReview[s]
}

assert ActiveReviewRoleDisjointnessInvariant {
  all v: Review |
    v.active = On implies
      v.reviewer != v.subject and
      no v.reviewer.roles
}

check RoleConcentrationRequiresFindingInvariant
  for 4 int, 4 Subject, 4 Role, 4 Review expect 0

check FullControlRequiresIndependentReviewInvariant
  for 4 int, 4 Subject, 4 Role, 4 Review expect 0

check ActiveReviewRoleDisjointnessInvariant
  for 4 int, 4 Subject, 4 Role, 4 Review expect 0
