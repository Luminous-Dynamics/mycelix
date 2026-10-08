module AgentDelegationAuthorityV1

enum Bit { On, Off }

sig Power {}
sig Agent {
  root: one Bit,
  authority: set Power
}

sig Grant {
  issuer: one Agent,
  grantee: one Agent,
  power: one Power,
  active: one Bit,
  revoked: one Bit,
  parent: lone Grant
}

sig Evidence {
  recorded: one Bit,
  grantsBefore: set Grant,
  grantsAfter: set Grant,
  authorityBefore: Agent -> Power,
  authorityAfter: Agent -> Power
}

sig ProviderFailure {
  authorityBefore: set Power,
  authorityAfter: set Power
}

fun grantDerivedAuthority[gs: set Grant]: Agent -> Power {
  { a: Agent, p: Power |
    (a.root = On and p in a.authority) or
    (a.root = Off and
      some g: gs |
        g.grantee = a and
        g.power = p and
        g.active = On and
        g.revoked = Off)
  }
}

fact RootAuthority {
  all a: Agent | a.root = On implies #a.authority >= 1
}

fact ActiveGrantCurrent {
  all g: Grant |
    g.active = On implies g.revoked = Off
}

fact GrantCannotExceedIssuerAuthority {
  all g: Grant |
    g.active = On implies g.power in g.issuer.authority
}

fact TransitiveDelegationCannotExceedAncestorAuthority {
  all g: Grant |
    g.active = On implies
      all a: g.*parent |
        g.power in a.issuer.authority
}

fact ChildAuthorityRequiresGrant {
  all a: Agent, p: Power |
    a.root = Off and p in a.authority implies
      some g: Grant | g.active = On and g.grantee = a and g.power = p
}

fact GrantParentMatchesIssuer {
  all g: Grant |
    some g.parent implies g.parent.grantee = g.issuer
}

fact RevocationPropagates {
  all g: Grant |
    g.revoked = On implies
      no h: Grant | h.active = On and (h = g or g in h.^parent)
}

fact EvidenceRecordingDoesNotChangeGrants {
  all e: Evidence |
    e.recorded = On implies e.grantsAfter = e.grantsBefore
}

fact EvidenceTransitionSnapshots {
  all e: Evidence |
    e.recorded = On implies
      e.authorityBefore = grantDerivedAuthority[e.grantsBefore] and
      e.authorityAfter = grantDerivedAuthority[e.grantsAfter]
}

fact ProviderFailurePreservesAuthority {
  all f: ProviderFailure |
    f.authorityAfter = f.authorityBefore
}

pred ValidDelegationChain {
  some g: Grant |
    g.active = On and
    g.issuer.root = On and
    g.power in g.issuer.authority and
    g.power in g.grantee.authority
}

pred TransitiveBoundedWitness {
  some child, parent: Grant |
    child.active = On and
    parent.active = On and
    child.parent = parent and
    child.power in parent.issuer.authority
}

pred TransitivePowerExceedsAncestorWitness {
  some child, ancestor: Grant |
    child.active = On and
    ancestor.active = On and
    ancestor in child.^parent and
    child.power not in ancestor.issuer.authority
}

pred RevokedGrantNoActiveDescendant {
  some g: Grant |
    g.revoked = On and
    no h: Grant | h.active = On and (h = g or g in h.^parent)
}

pred CurrentEvidenceWithGrantBackedAuthority {
  some e: Evidence |
    e.recorded = On and
    e.grantsBefore = e.grantsAfter and
    e.authorityBefore = e.authorityAfter
}

pred EvidenceGrantBackedAuthorityDeltaWitness {
  some e: Evidence, g: Grant |
    e.recorded = On and
    g in e.grantsAfter and g not in e.grantsBefore and
    g.active = On and
    g.revoked = Off and
    g.power in g.issuer.authority and
    e.grantsBefore != e.grantsAfter and
    e.authorityBefore = grantDerivedAuthority[e.grantsBefore] and
    e.authorityAfter = grantDerivedAuthority[e.grantsAfter] and
    e.authorityBefore != e.authorityAfter
}

run ValidDelegationChain
  for 6 but 6 Agent, 6 Power, 6 Grant

run TransitiveBoundedWitness
  for 6 but 6 Agent, 6 Power, 6 Grant

run TransitivePowerExceedsAncestorWitness
  for 6 but 6 Agent, 6 Power, 6 Grant

run RevokedGrantNoActiveDescendant
  for 6 but 6 Agent, 6 Power, 6 Grant

run CurrentEvidenceWithGrantBackedAuthority
  for 6 but 6 Agent, 6 Power, 6 Grant, 6 Evidence

run EvidenceGrantBackedAuthorityDeltaWitness
  for 6 but 6 Agent, 6 Power, 6 Grant, 6 Evidence

assert ActiveGrantsNeverExceedIssuerAuthority {
  all g: Grant |
    g.active = On implies g.power in g.issuer.authority
}

assert TransitiveDelegationDoesNotAmplifyAuthority {
  all g: Grant |
    g.active = On implies
      all a: g.*parent |
        g.power in a.issuer.authority
}

assert RevokedGrantsHaveNoActiveDescendants {
  all g: Grant |
    g.revoked = On implies
      no h: Grant | h.active = On and (h = g or g in h.^parent)
}

assert ChildAuthorityComesFromCurrentGrant {
  all a: Agent, p: Power |
    a.root = Off and p in a.authority implies
      some g: Grant | g.active = On and g.grantee = a and g.power = p
}

assert EvidenceCannotMintAuthority {
  all e: Evidence |
    e.recorded = On implies
      e.authorityAfter = e.authorityBefore
}

assert ProviderFailureIsNonAmplifying {
  all f: ProviderFailure |
    f.authorityAfter = f.authorityBefore
}

check ActiveGrantsNeverExceedIssuerAuthority
  for 6 but 6 Agent, 6 Power, 6 Grant expect 0

check TransitiveDelegationDoesNotAmplifyAuthority
  for 6 but 6 Agent, 6 Power, 6 Grant expect 0

check TransitivePowerExceedsAncestorWitness
  for 6 but 6 Agent, 6 Power, 6 Grant expect 0

check RevokedGrantsHaveNoActiveDescendants
  for 6 but 6 Agent, 6 Power, 6 Grant expect 0

check ChildAuthorityComesFromCurrentGrant
  for 6 but 6 Agent, 6 Power, 6 Grant expect 0

check EvidenceCannotMintAuthority
  for 6 but 6 Agent, 6 Power, 6 Grant, 6 Evidence expect 0

check ProviderFailureIsNonAmplifying
  for 6 but 6 Agent, 6 Power, 6 Grant expect 0
