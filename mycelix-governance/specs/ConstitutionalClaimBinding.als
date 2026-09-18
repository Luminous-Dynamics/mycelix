module ConstitutionalClaimBinding

abstract sig ClaimId {}
abstract sig EnvelopeDigest {}
abstract sig Nonce {}
abstract sig UseIndex {}
abstract sig Jurisdiction {}
abstract sig Matter {}
abstract sig TargetDigest {}
abstract sig PayloadDigest {}
abstract sig BudgetId {}

sig Claim {
  claimId: one ClaimId,
  envelope: one EnvelopeDigest,
  nonce: one Nonce,
  useIndex: one UseIndex,
  jurisdiction: one Jurisdiction,
  matter: one Matter,
  target: one TargetDigest,
  payload: one PayloadDigest,
  budget: one BudgetId
}

sig Binding {
  claimId: one ClaimId,
  envelope: one EnvelopeDigest,
  nonce: one Nonce,
  useIndex: one UseIndex,
  jurisdiction: one Jurisdiction,
  matter: one Matter,
  target: one TargetDigest,
  payload: one PayloadDigest,
  budget: one BudgetId
}

pred SameClaimBody[c1, c2: Claim] {
  c1.claimId = c2.claimId
  c1.envelope = c2.envelope
  c1.nonce = c2.nonce
  c1.useIndex = c2.useIndex
  c1.jurisdiction = c2.jurisdiction
  c1.matter = c2.matter
  c1.target = c2.target
  c1.payload = c2.payload
  c1.budget = c2.budget
}

pred Matches[b: Binding, c: Claim] {
  b.claimId = c.claimId
  b.envelope = c.envelope
  b.nonce = c.nonce
  b.useIndex = c.useIndex
  b.jurisdiction = c.jurisdiction
  b.matter = c.matter
  b.target = c.target
  b.payload = c.payload
  b.budget = c.budget
}

/* Formal Claim atoms represent distinct semantic claim bodies. */
fact CanonicalClaimAtoms {
  all disj c1, c2: Claim | not SameClaimBody[c1, c2]
}

/* Every concrete claim has exactly one exact semantic binding. */
fact ExactBindingForEveryClaim {
  all c: Claim | one b: Binding | Matches[b, c]
}

/* Bindings in the modeled universe are backed by at least one claim. */
fact NoOrphanBindings {
  all b: Binding | some c: Claim | Matches[b, c]
}

assert BindingAuthenticatesAtMostOneClaim {
  all b: Binding, c1, c2: Claim |
    Matches[b, c1] and Matches[b, c2] implies c1 = c2
}

/*
 * Non-vacuity: the scope can contain two claims with the same legacy claimId
 * and all fields equal except payload. Exact binding must keep them distinct.
 */
pred LegacyIdCollisionCandidate {
  some disj c1, c2: Claim |
    c1.claimId = c2.claimId
    and c1.envelope = c2.envelope
    and c1.nonce = c2.nonce
    and c1.useIndex = c2.useIndex
    and c1.jurisdiction = c2.jurisdiction
    and c1.matter = c2.matter
    and c1.target = c2.target
    and c1.budget = c2.budget
    and c1.payload != c2.payload
}

check BindingAuthenticatesAtMostOneClaim for 4
run LegacyIdCollisionCandidate for 4
