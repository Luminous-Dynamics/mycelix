module EvidenceAttestationCapabilityAttenuationV1

open util/ordering[Time] as TO

enum Bit { On, Off }

sig Resource {}
sig Action {}
sig Audience {}
sig Time {}

sig Capability {
  resource: one Resource,
  action: one Action,
  audience: one Audience,
  expiry: one Time
}

sig Agent {
  root: one Bit,
  authority: set Capability
}

sig Grant {
  issuer: one Agent,
  grantee: one Agent,
  capability: one Capability,
  active: one Bit,
  revoked: one Bit
}

sig Evidence {
  recorded: one Bit,
  signatureValid: one Bit,
  signerTrusted: one Bit,
  claimAuthorized: one Bit,
  subject: one Agent,
  target: one Agent,
  claimResources: set Resource,
  claimActions: set Action,
  claimAudiences: set Audience,
  claimExpiry: one Time,
  grantsBefore: set Grant,
  grantsAfter: set Grant,
  authorityBefore: Agent -> Capability,
  authorityAfter: Agent -> Capability
}

fun grantDerivedAuthority[gs: set Grant]: Agent -> Capability {
  { a: Agent, c: Capability |
    (a.root = On and c in a.authority) or
    (a.root = Off and some g: gs |
      g.grantee = a and
      g.capability = c and
      g.active = On and
      g.revoked = Off)
  }
}

fact RootAuthority {
  all a: Agent | a.root = On implies a.authority = Capability
}

fact ActiveGrantCurrent {
  all g: Grant | g.active = On implies g.revoked = Off
}

fact GrantCapabilityWithinIssuerAuthority {
  all g: Grant |
    g.active = On implies g.capability in g.issuer.authority
}

fact ChildAuthorityRequiresGrant {
  all a: Agent, c: Capability |
    a.root = Off and c in a.authority implies
      some g: Grant |
        g.active = On and g.revoked = Off and
        g.grantee = a and g.capability = c
}

fact ClaimResourceBounded {
  all e: Evidence, g: Grant |
    e.recorded = On and
    e.signatureValid = On and
    e.signerTrusted = On and
    e.claimAuthorized = On and
    e.subject = e.target and
    g in e.grantsAfter and
    g not in e.grantsBefore implies
      g.capability.resource in e.claimResources
}

fact ClaimActionBounded {
  all e: Evidence, g: Grant |
    e.recorded = On and
    e.signatureValid = On and
    e.signerTrusted = On and
    e.claimAuthorized = On and
    e.subject = e.target and
    g in e.grantsAfter and
    g not in e.grantsBefore implies
      g.capability.action in e.claimActions
}

fact ClaimAudienceBounded {
  all e: Evidence, g: Grant |
    e.recorded = On and
    e.signatureValid = On and
    e.signerTrusted = On and
    e.claimAuthorized = On and
    e.subject = e.target and
    g in e.grantsAfter and
    g not in e.grantsBefore implies
      g.capability.audience in e.claimAudiences
}

fact ClaimExpiryBounded {
  all e: Evidence, g: Grant |
    e.recorded = On and
    e.signatureValid = On and
    e.signerTrusted = On and
    e.claimAuthorized = On and
    e.subject = e.target and
    g in e.grantsAfter and
    g not in e.grantsBefore implies
      TO/lte[g.capability.expiry, e.claimExpiry]
}

fact EvidenceSnapshots {
  all e: Evidence |
    e.recorded = On implies
      e.authorityBefore = grantDerivedAuthority[e.grantsBefore] and
      e.authorityAfter = grantDerivedAuthority[e.grantsAfter]
}

pred ValidCapabilityDeltaWitness {
  some e: Evidence, g: Grant |
    e.recorded = On and
    e.signatureValid = On and
    e.signerTrusted = On and
    e.claimAuthorized = On and
    e.subject = e.target and
    g in e.grantsAfter and
    g not in e.grantsBefore and
    g.active = On and
    g.revoked = Off and
    g.grantee = e.subject and
    g.capability.resource in e.claimResources and
    g.capability.action in e.claimActions and
    g.capability.audience in e.claimAudiences and
    TO/lte[g.capability.expiry, e.claimExpiry] and
    e.authorityBefore != e.authorityAfter
}

pred ResourceExpansionWitness {
  some e: Evidence, g: Grant |
    e.recorded = On and
    e.signatureValid = On and
    e.signerTrusted = On and
    e.claimAuthorized = On and
    e.subject = e.target and
    g in e.grantsAfter and
    g not in e.grantsBefore and
    g.active = On and
    g.revoked = Off and
    g.grantee = e.subject and
    g.capability.resource not in e.claimResources and
    g.capability.action in e.claimActions and
    g.capability.audience in e.claimAudiences and
    TO/lte[g.capability.expiry, e.claimExpiry] and
    e.authorityBefore = grantDerivedAuthority[e.grantsBefore] and
    e.authorityAfter = grantDerivedAuthority[e.grantsAfter] and
    e.authorityBefore != e.authorityAfter
}

pred ActionExpansionWitness {
  some e: Evidence, g: Grant |
    e.recorded = On and
    e.signatureValid = On and
    e.signerTrusted = On and
    e.claimAuthorized = On and
    e.subject = e.target and
    g in e.grantsAfter and
    g not in e.grantsBefore and
    g.active = On and
    g.revoked = Off and
    g.grantee = e.subject and
    g.capability.resource in e.claimResources and
    g.capability.action not in e.claimActions and
    g.capability.audience in e.claimAudiences and
    TO/lte[g.capability.expiry, e.claimExpiry] and
    e.authorityBefore = grantDerivedAuthority[e.grantsBefore] and
    e.authorityAfter = grantDerivedAuthority[e.grantsAfter] and
    e.authorityBefore != e.authorityAfter
}

pred AudienceExpansionWitness {
  some e: Evidence, g: Grant |
    e.recorded = On and
    e.signatureValid = On and
    e.signerTrusted = On and
    e.claimAuthorized = On and
    e.subject = e.target and
    g in e.grantsAfter and
    g not in e.grantsBefore and
    g.active = On and
    g.revoked = Off and
    g.grantee = e.subject and
    g.capability.resource in e.claimResources and
    g.capability.action in e.claimActions and
    g.capability.audience not in e.claimAudiences and
    TO/lte[g.capability.expiry, e.claimExpiry] and
    e.authorityBefore = grantDerivedAuthority[e.grantsBefore] and
    e.authorityAfter = grantDerivedAuthority[e.grantsAfter] and
    e.authorityBefore != e.authorityAfter
}

pred ExpiryExpansionWitness {
  some e: Evidence, g: Grant |
    e.recorded = On and
    e.signatureValid = On and
    e.signerTrusted = On and
    e.claimAuthorized = On and
    e.subject = e.target and
    g in e.grantsAfter and
    g not in e.grantsBefore and
    g.active = On and
    g.revoked = Off and
    g.grantee = e.subject and
    g.capability.resource in e.claimResources and
    g.capability.action in e.claimActions and
    g.capability.audience in e.claimAudiences and
    TO/gt[g.capability.expiry, e.claimExpiry] and
    e.authorityBefore = grantDerivedAuthority[e.grantsBefore] and
    e.authorityAfter = grantDerivedAuthority[e.grantsAfter] and
    e.authorityBefore != e.authorityAfter
}

run ValidCapabilityDeltaWitness
  for 6 but 6 Agent, 8 Capability, 6 Grant, 6 Evidence,
  3 Resource, 3 Action, 3 Audience, 4 Time

run ResourceExpansionWitness
  for 6 but 6 Agent, 8 Capability, 6 Grant, 6 Evidence,
  3 Resource, 3 Action, 3 Audience, 4 Time

run ActionExpansionWitness
  for 6 but 6 Agent, 8 Capability, 6 Grant, 6 Evidence,
  3 Resource, 3 Action, 3 Audience, 4 Time

run AudienceExpansionWitness
  for 6 but 6 Agent, 8 Capability, 6 Grant, 6 Evidence,
  3 Resource, 3 Action, 3 Audience, 4 Time

run ExpiryExpansionWitness
  for 6 but 6 Agent, 8 Capability, 6 Grant, 6 Evidence,
  3 Resource, 3 Action, 3 Audience, 4 Time

assert ResourceScopeNeverExceedsClaim {
  all e: Evidence |
    e.recorded = On and
    e.signatureValid = On and
    e.signerTrusted = On and
    e.claimAuthorized = On and
    e.subject = e.target implies
      all c: e.authorityAfter[e.subject] - e.authorityBefore[e.subject] |
        c.resource in e.claimResources
}

assert ActionScopeNeverExceedsClaim {
  all e: Evidence |
    e.recorded = On and
    e.signatureValid = On and
    e.signerTrusted = On and
    e.claimAuthorized = On and
    e.subject = e.target implies
      all c: e.authorityAfter[e.subject] - e.authorityBefore[e.subject] |
        c.action in e.claimActions
}

assert AudienceScopeNeverExceedsClaim {
  all e: Evidence |
    e.recorded = On and
    e.signatureValid = On and
    e.signerTrusted = On and
    e.claimAuthorized = On and
    e.subject = e.target implies
      all c: e.authorityAfter[e.subject] - e.authorityBefore[e.subject] |
        c.audience in e.claimAudiences
}

assert ExpiryScopeNeverExceedsClaim {
  all e: Evidence |
    e.recorded = On and
    e.signatureValid = On and
    e.signerTrusted = On and
    e.claimAuthorized = On and
    e.subject = e.target implies
      all c: e.authorityAfter[e.subject] - e.authorityBefore[e.subject] |
        TO/lte[c.expiry, e.claimExpiry]
}

check ResourceScopeNeverExceedsClaim
  for 6 but 6 Agent, 8 Capability, 6 Grant, 6 Evidence,
  3 Resource, 3 Action, 3 Audience, 4 Time

check ActionScopeNeverExceedsClaim
  for 6 but 6 Agent, 8 Capability, 6 Grant, 6 Evidence,
  3 Resource, 3 Action, 3 Audience, 4 Time

check AudienceScopeNeverExceedsClaim
  for 6 but 6 Agent, 8 Capability, 6 Grant, 6 Evidence,
  3 Resource, 3 Action, 3 Audience, 4 Time

check ExpiryScopeNeverExceedsClaim
  for 6 but 6 Agent, 8 Capability, 6 Grant, 6 Evidence,
  3 Resource, 3 Action, 3 Audience, 4 Time
