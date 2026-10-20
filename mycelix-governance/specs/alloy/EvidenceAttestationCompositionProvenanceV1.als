module EvidenceAttestationCompositionProvenanceV1

sig Resource {}
sig Action {}

enum Bit { On, Off }

sig Capability {
  resource: one Resource,
  action: one Action
}

sig Claim {
  authorized: one Bit,
  grantBacked: one Bit,
  capability: one Capability
}

sig Composition {
  inputClaims: set Claim,
  contributorClaims: set Claim,
  authority: set Capability
}

fun validClaims: set Claim {
  { c: Claim |
    c.authorized = On and c.grantBacked = On
  }
}

fun expectedAuthority[comp: Composition]: set Capability {
  { cap: Capability |
    some c: comp.inputClaims |
      c.capability = cap
  }
}

fact AuthorizedClaimsGrantBacked {
  all c: Claim |
    c.authorized = On implies c.grantBacked = On
}

fact AuthorityOnlyFromInputs {
  all comp: Composition |
    comp.authority in expectedAuthority[comp]
}

fact ContributorsAuthorizeTheirCapabilities {
  all comp: Composition, c: comp.contributorClaims |
    c.authorized = On and
    c.grantBacked = On and
    c.capability in comp.authority
}

fact CompositionProvenanceMatchesInputs {
  all comp: Composition |
    comp.contributorClaims = comp.inputClaims
}

pred ValidCompositionWitness {
  some disj c1, c2: Claim, comp: Composition |
    c1.authorized = On and
    c1.grantBacked = On and
    c2.authorized = On and
    c2.grantBacked = On and
    c1.capability != c2.capability and
    comp.inputClaims = c1 + c2 and
    comp.contributorClaims = c1 + c2 and
    comp.authority = c1.capability + c2.capability
}

pred ContributorSubstitutionWitness {
  some disj c1, c2, substitute: Claim, comp: Composition |
    c1.authorized = On and
    c1.grantBacked = On and
    c2.authorized = On and
    c2.grantBacked = On and
    substitute.authorized = On and
    substitute.grantBacked = On and
    c2.capability = substitute.capability and
    c1.capability != c2.capability and
    comp.inputClaims = c1 + c2 and
    comp.contributorClaims = c1 + substitute and
    comp.authority = c1.capability + c2.capability
}

run ValidCompositionWitness
  for 6 but 6 Resource, 6 Action, 8 Capability, 6 Claim, 4 Composition

run ContributorSubstitutionWitness
  for 6 but 6 Resource, 6 Action, 8 Capability, 6 Claim, 4 Composition

assert CompositionProvenanceIsExact {
  all comp: Composition |
    comp.contributorClaims = comp.inputClaims
}

check CompositionProvenanceIsExact
  for 6 but 6 Resource, 6 Action, 8 Capability, 6 Claim, 4 Composition
