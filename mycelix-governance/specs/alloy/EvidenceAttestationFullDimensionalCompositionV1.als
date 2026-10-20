module EvidenceAttestationFullDimensionalCompositionV1

enum Bit { On, Off }

sig Resource {}
sig Action {}
sig Audience {}
sig Time {}
sig Agent {}

sig Capability {
  resource: one Resource,
  action: one Action,
  audience: one Audience,
  expiry: one Time
}

sig Claim {
  authorized: one Bit,
  grantBacked: one Bit,
  signerTrusted: one Bit,
  subject: one Agent,
  target: one Agent,
  capability: one Capability
}

sig Composition {
  inputClaims: set Claim,
  authority: set Capability,
  contributorClaims: Capability -> Claim,
  resourceContributors: Capability -> Claim,
  actionContributors: Capability -> Claim,
  audienceContributors: Capability -> Claim,
  expiryContributors: Capability -> Claim
}

fun expectedAuthority[comp: Composition]: set Capability {
  { cap: Capability |
    some c: comp.inputClaims | c.capability = cap
  }
}

fun expectedContributors[comp: Composition]: Capability -> Claim {
  { cap: Capability, c: comp.inputClaims |
    c.capability = cap
  }
}

fun expectedResourceContributors[comp: Composition]: Capability -> Claim {
  { cap: Capability, c: comp.inputClaims |
    c.capability.resource = cap.resource
  }
}

fun expectedActionContributors[comp: Composition]: Capability -> Claim {
  { cap: Capability, c: comp.inputClaims |
    c.capability.action = cap.action
  }
}

fun expectedAudienceContributors[comp: Composition]: Capability -> Claim {
  { cap: Capability, c: comp.inputClaims |
    c.capability.audience = cap.audience
  }
}

fun expectedExpiryContributors[comp: Composition]: Capability -> Claim {
  { cap: Capability, c: comp.inputClaims |
    c.capability.expiry = cap.expiry
  }
}

fact InputClaimsValid {
  all comp: Composition, c: comp.inputClaims |
    c.authorized = On and
    c.grantBacked = On and
    c.signerTrusted = On and
    c.subject = c.target
}

fact DimensionSourcesWithinInputs {
  all comp: Composition |
    comp.resourceContributors in (Capability -> comp.inputClaims) and
    comp.actionContributors in (Capability -> comp.inputClaims) and
    comp.audienceContributors in (Capability -> comp.inputClaims) and
    comp.expiryContributors in (Capability -> comp.inputClaims)
}

fact DimensionSourcesMatchCapabilities {
  all comp: Composition, cap: Capability |
    all c: cap.(comp.resourceContributors) |
      c.capability.resource = cap.resource
  all comp: Composition, cap: Capability |
    all c: cap.(comp.actionContributors) |
      c.capability.action = cap.action
  all comp: Composition, cap: Capability |
    all c: cap.(comp.audienceContributors) |
      c.capability.audience = cap.audience
  all comp: Composition, cap: Capability |
    all c: cap.(comp.expiryContributors) |
      c.capability.expiry = cap.expiry
}

fact DimensionSourcesAreExact {
  all comp: Composition |
    comp.resourceContributors = expectedResourceContributors[comp] and
    comp.actionContributors = expectedActionContributors[comp] and
    comp.audienceContributors = expectedAudienceContributors[comp] and
    comp.expiryContributors = expectedExpiryContributors[comp]
}

fact CompositionAtomicAndProvenance {
  all comp: Composition |
    comp.authority in expectedAuthority[comp] and
    comp.contributorClaims = expectedContributors[comp] and
    all cap: comp.authority | some cap.(comp.contributorClaims)
}

pred ValidFourDimensionalCompositionWitness {
  some disj c1, c2, c3, c4: Claim, comp: Composition |
    c1.authorized = On and c1.grantBacked = On and c1.signerTrusted = On and c1.subject = c1.target and
    c2.authorized = On and c2.grantBacked = On and c2.signerTrusted = On and c2.subject = c2.target and
    c3.authorized = On and c3.grantBacked = On and c3.signerTrusted = On and c3.subject = c3.target and
    c4.authorized = On and c4.grantBacked = On and c4.signerTrusted = On and c4.subject = c4.target and
    c1.capability != c2.capability and
    c1.capability != c3.capability and
    c1.capability != c4.capability and
    c2.capability != c3.capability and
    c2.capability != c4.capability and
    c3.capability != c4.capability and
    comp.inputClaims = c1 + c2 + c3 + c4 and
    comp.authority = c1.capability + c2.capability + c3.capability + c4.capability and
    comp.contributorClaims = expectedContributors[comp]
}

pred HybridSynthesisWitness {
  some disj c1, c2, c3, c4: Claim, hybrid: Capability, comp: Composition |
    c1.authorized = On and c1.grantBacked = On and c1.signerTrusted = On and c1.subject = c1.target and
    c2.authorized = On and c2.grantBacked = On and c2.signerTrusted = On and c2.subject = c2.target and
    c3.authorized = On and c3.grantBacked = On and c3.signerTrusted = On and c3.subject = c3.target and
    c4.authorized = On and c4.grantBacked = On and c4.signerTrusted = On and c4.subject = c4.target and

    hybrid.resource = c1.capability.resource and
    hybrid.action = c2.capability.action and
    hybrid.audience = c4.capability.audience and
    hybrid.expiry = c4.capability.expiry and

    hybrid not in (c1 + c2 + c3 + c4).capability and

    comp.inputClaims = c1 + c2 + c3 + c4 and
    comp.authority = c1.capability + c2.capability + c3.capability + c4.capability + hybrid and

    hybrid -> (c1 + c2 + c3 + c4) in comp.contributorClaims and
    hybrid -> (c1 + c3) in comp.resourceContributors and
    hybrid -> (c2 + c3) in comp.actionContributors and
    hybrid -> (c3 + c4) in comp.audienceContributors and
    hybrid -> (c1 + c4) in comp.expiryContributors
}

assert CompositionAtomAndProvenanceExact {
  all comp: Composition |
    comp.authority in expectedAuthority[comp] and
    comp.contributorClaims = expectedContributors[comp] and
    all cap: comp.authority | some cap.(comp.contributorClaims)
}

assert DimensionSourcesRemainValid {
  all comp: Composition |
    comp.resourceContributors = expectedResourceContributors[comp] and
    comp.actionContributors = expectedActionContributors[comp] and
    comp.audienceContributors = expectedAudienceContributors[comp] and
    comp.expiryContributors = expectedExpiryContributors[comp]
}

run ValidFourDimensionalCompositionWitness
  for 8 but 8 Resource, 8 Action, 8 Audience, 6 Time,
  12 Capability, 8 Claim, 4 Composition, 6 Agent

run HybridSynthesisWitness
  for 8 but 8 Resource, 8 Action, 8 Audience, 6 Time,
  12 Capability, 8 Claim, 4 Composition, 6 Agent

check CompositionAtomAndProvenanceExact
  for 8 but 8 Resource, 8 Action, 8 Audience, 6 Time,
  12 Capability, 8 Claim, 4 Composition, 6 Agent

check DimensionSourcesRemainValid
  for 8 but 8 Resource, 8 Action, 8 Audience, 6 Time,
  12 Capability, 8 Claim, 4 Composition, 6 Agent
