module ArtificialSovereigntyRoleConcentrationV1

enum SubjectClass { Human, Artificial }
enum Bit { On, Off }
enum Role { Operator, Verifier, EvidenceArchive, Adjudicator }

sig Subject {
  class: one SubjectClass,
  roles: set Role,
  conflictFinding: one Bit,
  externalReview: one Bit
}

fact CriticalRoleConcentrationRequiresFinding {
  all s: Subject |
    #(s.roles & (Operator + Verifier + EvidenceArchive)) >= 2
      implies s.conflictFinding = On
}

fact FullControlRequiresExternalReview {
  all s: Subject |
    #(s.roles & (Operator + Verifier + EvidenceArchive)) = 3
      implies s.externalReview = On
}

pred ConcentratedRolesWithFinding {
  some s: Subject |
    #(s.roles & (Operator + Verifier + EvidenceArchive)) >= 2 and
    s.conflictFinding = On
}

pred FullControlWithExternalReview {
  some s: Subject |
    #(s.roles & (Operator + Verifier + EvidenceArchive)) = 3 and
    s.externalReview = On
}

pred AdjudicatorDistinctOrReviewed {
  some s: Subject |
    Adjudicator in s.roles and
    (s.conflictFinding = On or s.externalReview = On)
}

run ConcentratedRolesWithFinding
  for 4 int, 4 Subject, 4 Role

run FullControlWithExternalReview
  for 4 int, 4 Subject, 4 Role

run AdjudicatorDistinctOrReviewed
  for 4 int, 4 Subject, 4 Role

assert RoleConcentrationRequiresFinding {
  all s: Subject |
    #(s.roles & (Operator + Verifier + EvidenceArchive)) >= 2
      implies s.conflictFinding = On
}

assert FullControlRequiresExternalReview {
  all s: Subject |
    #(s.roles & (Operator + Verifier + EvidenceArchive)) = 3
      implies s.externalReview = On
}

check RoleConcentrationRequiresFinding
  for 4 int, 4 Subject, 4 Role expect 0

check FullControlRequiresExternalReview
  for 4 int, 4 Subject, 4 Role expect 0
