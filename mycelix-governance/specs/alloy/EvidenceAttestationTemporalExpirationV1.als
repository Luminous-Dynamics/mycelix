module EvidenceAttestationTemporalExpirationV1

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
  grant: one Grant
}

sig TemporalState {
  now: one Time,
  effectiveAuthority: Agent -> Capability
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

fact EvidenceRecordedValid {
  all e: Evidence |
    e.recorded = On implies
      e.signatureValid = On and
      e.signerTrusted = On and
      e.claimAuthorized = On and
      e.subject = e.target and
      e.grant.active = On and
      e.grant.revoked = Off
}

fact NonExpiredGrantEffective {
  all s: TemporalState, g: Grant |
    g.active = On and
    g.revoked = Off and
    not TO/lt[g.capability.expiry, s.now] implies
      g.capability in s.effectiveAuthority[g.grantee]
}

fact ExpiredGrantNotEffective {
  all s: TemporalState, g: Grant |
    g.active = On and
    TO/lt[g.capability.expiry, s.now] implies
      g.capability not in s.effectiveAuthority[g.grantee]
}

pred FreshGrantEffectiveWitness {
  some s: TemporalState, g: Grant |
    g.active = On and
    g.revoked = Off and
    not TO/lt[g.capability.expiry, s.now] and
    g.capability in s.effectiveAuthority[g.grantee]
}

pred ExpiredGrantPersistenceWitness {
  some s: TemporalState, e: Evidence, g: Grant |
    e.recorded = On and
    e.signatureValid = On and
    e.signerTrusted = On and
    e.claimAuthorized = On and
    e.subject = e.target and
    e.grant = g and
    g.active = On and
    g.revoked = Off and
    TO/lt[g.capability.expiry, s.now] and
    g.capability in s.effectiveAuthority[g.grantee]
}

assert EffectiveAuthorityMatchesCurrentTime {
  all s: TemporalState, g: Grant |
    g.active = On and
    not TO/lt[g.capability.expiry, s.now] implies
      g.capability in s.effectiveAuthority[g.grantee]
}

check EffectiveAuthorityMatchesCurrentTime
  for 6 but 6 Agent, 8 Capability, 6 Grant, 4 Evidence, 4 TemporalState,
  3 Resource, 3 Action, 3 Audience, 4 Time

check ExpiredGrantNotEffective
  for 6 but 6 Agent, 8 Capability, 6 Grant, 4 Evidence, 4 TemporalState,
  3 Resource, 3 Action, 3 Audience, 4 Time

run FreshGrantEffectiveWitness
  for 6 but 6 Agent, 8 Capability, 6 Grant, 4 Evidence, 4 TemporalState,
  3 Resource, 3 Action, 3 Audience, 4 Time

run ExpiredGrantPersistenceWitness
  for 6 but 6 Agent, 8 Capability, 6 Grant, 4 Evidence, 4 TemporalState,
  3 Resource, 3 Action, 3 Audience, 4 Time
