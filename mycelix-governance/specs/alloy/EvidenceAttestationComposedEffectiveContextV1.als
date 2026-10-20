module EvidenceAttestationComposedEffectiveContextV1

open util/ordering[Time] as TimeOrder
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
  contributorClaims: Capability -> Claim
}

sig Request {
  resource: one Resource,
  action: one Action,
  audience: one Audience,
  now: one Time
}

sig Decision {
  composition: one Composition,
  request: one Request,
  authorized: one Bit,
  contributors: set Claim,
  resourceContributors: set Claim,
  actionContributors: set Claim,
  audienceContributors: set Claim,
  temporalContributors: set Claim
}

fact CapabilityTupleUniqueness {
  all disj c1, c2: Capability |
    c1.resource != c2.resource or
    c1.action != c2.action or
    c1.audience != c2.audience or
    c1.expiry != c2.expiry
}

fun expectedAuthority[comp: Composition]: set Capability {
  { cap: Capability |
    some c: comp.inputClaims | c.capability = cap
  }
}

fun expectedContributors[comp: Composition, cap: Capability]: set Claim {
  { c: comp.inputClaims | c.capability = cap }
}

fun matchingCapabilities[comp: Composition, req: Request]: set Capability {
  { cap: comp.authority |
    cap.resource = req.resource and
    cap.action = req.action and
    cap.audience = req.audience and
    TimeOrder/lt[req.now, cap.expiry]
  }
}

fun resourceSources[comp: Composition, req: Request]: set Claim {
  { c: comp.inputClaims | c.capability.resource = req.resource }
}

fun actionSources[comp: Composition, req: Request]: set Claim {
  { c: comp.inputClaims | c.capability.action = req.action }
}

fun audienceSources[comp: Composition, req: Request]: set Claim {
  { c: comp.inputClaims | c.capability.audience = req.audience }
}

fun temporalSources[comp: Composition, req: Request]: set Claim {
  { c: comp.inputClaims | TimeOrder/lt[req.now, c.capability.expiry] }
}

fun expectedDecisionContributors[comp: Composition, req: Request]: set Claim {
  { c: comp.inputClaims |
    some cap: matchingCapabilities[comp, req] | c.capability = cap
  }
}

fact InputClaimsValid {
  all comp: Composition, c: comp.inputClaims |
    c.authorized = On and
    c.grantBacked = On and
    c.signerTrusted = On and
    c.subject = c.target
}

fact CompositionAtomicAndProvenance {
  all comp: Composition |
    comp.authority = expectedAuthority[comp] and
    all cap: comp.authority |
      cap.(comp.contributorClaims) =
        expectedContributors[comp, cap] and
      some cap.(comp.contributorClaims)
}

fact DecisionDimensionSourcesExact {
  all d: Decision |
    d.resourceContributors =
      resourceSources[d.composition, d.request] and
    d.actionContributors =
      actionSources[d.composition, d.request] and
    d.audienceContributors =
      audienceSources[d.composition, d.request] and
    d.temporalContributors =
      temporalSources[d.composition, d.request] and
    some d.resourceContributors and
    some d.actionContributors and
    some d.audienceContributors and
    some d.temporalContributors
}

fact DecisionRequiresSingleEffectiveAtom {
  all d: Decision |
    d.authorized = On implies
      some matchingCapabilities[d.composition, d.request] and
      d.contributors =
        expectedDecisionContributors[d.composition, d.request] and
      some d.contributors
}

pred ValidEffectiveContextWitness {
  some d: Decision, cap: Capability |
    d.authorized = On and
    cap in matchingCapabilities[d.composition, d.request] and
    d.contributors =
      expectedDecisionContributors[d.composition, d.request] and
    some d.contributors
}

pred ContextualLaunderingWitness {
  some d: Decision, cr, ca, cu, ct: Claim |
    d.authorized = On and
    no matchingCapabilities[d.composition, d.request] and
    cr in d.resourceContributors and
    ca in d.actionContributors and
    cu in d.audienceContributors and
    ct in d.temporalContributors and
    cr.capability.resource = d.request.resource and
    ca.capability.action = d.request.action and
    cu.capability.audience = d.request.audience and
    TimeOrder/lt[d.request.now, ct.capability.expiry] and
    d.contributors =
      d.resourceContributors +
      d.actionContributors +
      d.audienceContributors +
      d.temporalContributors
}

assert DecisionRequiresSingleEffectiveAtom {
  all d: Decision |
    d.authorized = On implies
      some matchingCapabilities[d.composition, d.request] and
      d.contributors =
        expectedDecisionContributors[d.composition, d.request] and
      some d.contributors
}

assert DecisionDimensionSourcesRemainValid {
  all d: Decision |
    d.resourceContributors =
      resourceSources[d.composition, d.request] and
    d.actionContributors =
      actionSources[d.composition, d.request] and
    d.audienceContributors =
      audienceSources[d.composition, d.request] and
    d.temporalContributors =
      temporalSources[d.composition, d.request]
}

assert CompositionAtomAndProvenanceExact {
  all comp: Composition |
    comp.authority = expectedAuthority[comp] and
    all cap: comp.authority |
      cap.(comp.contributorClaims) =
        expectedContributors[comp, cap] and
      some cap.(comp.contributorClaims)
}

run ValidEffectiveContextWitness
  for 12 but
  4 Resource, 4 Action, 4 Audience, 4 Time,
  8 Capability, 8 Claim, 4 Composition, 4 Request, 4 Decision, 6 Agent

run ContextualLaunderingWitness
  for 12 but
  4 Resource, 4 Action, 4 Audience, 4 Time,
  8 Capability, 8 Claim, 4 Composition, 4 Request, 4 Decision, 6 Agent

check DecisionRequiresSingleEffectiveAtom
  for 12 but
  4 Resource, 4 Action, 4 Audience, 4 Time,
  8 Capability, 8 Claim, 4 Composition, 4 Request, 4 Decision, 6 Agent

check DecisionDimensionSourcesRemainValid
  for 12 but
  4 Resource, 4 Action, 4 Audience, 4 Time,
  8 Capability, 8 Claim, 4 Composition, 4 Request, 4 Decision, 6 Agent

check CompositionAtomAndProvenanceExact
  for 12 but
  4 Resource, 4 Action, 4 Audience, 4 Time,
  8 Capability, 8 Claim, 4 Composition, 4 Request, 4 Decision, 6 Agent
