module EvidenceAttestationScopeAttenuationV1

enum Bit { On, Off }

sig Scope {}
sig Agent {
  root: one Bit,
  authority: set Scope
}

sig Grant {
  issuer: one Agent,
  grantee: one Agent,
  scope: set Scope,
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
  claimScope: set Scope,
  grantsBefore: set Grant,
  grantsAfter: set Grant,
  authorityBefore: Agent -> Scope,
  authorityAfter: Agent -> Scope
}

fun grantDerivedAuthority[gs: set Grant]: Agent -> Scope {
  { a: Agent, s: Scope |
    (a.root = On and s in a.authority) or
    (a.root = Off and some g: gs |
      g.grantee = a and
      s in g.scope and
      g.active = On and
      g.revoked = Off)
  }
}

fact RootAuthority {
  all a: Agent | a.root = On implies #a.authority >= 1
}

fact ActiveGrantCurrent {
  all g: Grant | g.active = On implies g.revoked = Off
}

fact GrantScopeWithinIssuerAuthority {
  all g: Grant |
    g.active = On implies all s: g.scope | s in g.issuer.authority
}

fact ChildAuthorityRequiresGrant {
  all a: Agent, s: Scope |
    a.root = Off and s in a.authority implies
      some g: Grant |
        g.active = On and g.revoked = Off and
        g.grantee = a and s in g.scope
}

fact EvidenceSnapshots {
  all e: Evidence |
    e.recorded = On implies
      e.authorityBefore = grantDerivedAuthority[e.grantsBefore] and
      e.authorityAfter = grantDerivedAuthority[e.grantsAfter]
}

fact ClaimAuthorizationScopeAttenuated {
  all e: Evidence |
    e.recorded = On and
    e.signatureValid = On and
    e.signerTrusted = On and
    e.claimAuthorized = On and
    e.subject = e.target implies
      e.authorityAfter[e.subject] - e.authorityBefore[e.subject] in e.claimScope
}

pred ValidAuthorizedNarrowDeltaWitness {
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
    g.scope in e.claimScope and
    e.authorityBefore != e.authorityAfter
}

pred BroadScopeGrantWitness {
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
    g.scope not in e.claimScope and
    g.scope in e.issuer.authority and
    e.authorityBefore = grantDerivedAuthority[e.grantsBefore] and
    e.authorityAfter = grantDerivedAuthority[e.grantsAfter] and
    e.authorityBefore != e.authorityAfter
}

run ValidAuthorizedNarrowDeltaWitness
  for 6 but 6 Agent, 4 Scope, 6 Grant, 6 Evidence

run BroadScopeGrantWitness
  for 6 but 6 Agent, 4 Scope, 6 Grant, 6 Evidence

assert ScopeNeverExceedsClaimAuthorization {
  all e: Evidence |
    e.recorded = On and
    e.signatureValid = On and
    e.signerTrusted = On and
    e.claimAuthorized = On and
    e.subject = e.target implies
      e.authorityAfter[e.subject] - e.authorityBefore[e.subject] in e.claimScope
}

check ScopeNeverExceedsClaimAuthorization
  for 6 but 6 Agent, 4 Scope, 6 Grant, 6 Evidence expect 0
