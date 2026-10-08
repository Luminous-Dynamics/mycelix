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
  recorded: one Bit
}

sig ProviderFailure {
  authorityBefore: set Power,
  authorityAfter: set Power
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

fact TransitiveDelegationCannotExceedIssuerAuthority {
  all g: Grant |
    g.active = On implies g.grantee.authority in g.issuer.authority
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

fact EvidenceDoesNotMintAuthority {
  all e: Evidence |
    e.recorded = On implies
      all a: Agent | a.root = Off implies
        all p: a.authority |
          some g: Grant | g.active = On and g.grantee = a and g.power = p
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

pred RevokedGrantNoActiveDescendant {
  some g: Grant |
    g.revoked = On and
    no h: Grant | h.active = On and (h = g or g in h.^parent)
}

pred EvidenceRecordWithoutAuthorityMint {
  some e: Evidence, a: Agent |
    e.recorded = On and a.root = Off and
    some p: a.authority and
    some g: Grant | g.active = On and g.grantee = a and g.power = p
}

run ValidDelegationChain
  for 6 but 6 Agent, 6 Power, 6 Grant

run TransitiveBoundedWitness
  for 6 but 6 Agent, 6 Power, 6 Grant

run RevokedGrantNoActiveDescendant
  for 6 but 6 Agent, 6 Power, 6 Grant

run EvidenceRecordWithoutAuthorityMint
  for 6 but 6 Agent, 6 Power, 6 Grant, 6 Evidence

assert ActiveGrantsNeverExceedIssuerAuthority {
  all g: Grant |
    g.active = On implies g.power in g.issuer.authority
}

assert TransitiveDelegationDoesNotAmplifyAuthority {
  all g: Grant |
    g.active = On implies g.grantee.authority in g.issuer.authority
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
      all a: Agent | a.root = Off implies
        all p: a.authority |
          some g: Grant | g.active = On and g.grantee = a and g.power = p
}

assert ProviderFailureIsNonAmplifying {
  all f: ProviderFailure |
    f.authorityAfter = f.authorityBefore
}

check ActiveGrantsNeverExceedIssuerAuthority
  for 6 but 6 Agent, 6 Power, 6 Grant expect 0

check TransitiveDelegationDoesNotAmplifyAuthority
  for 6 but 6 Agent, 6 Power, 6 Grant expect 0

check RevokedGrantsHaveNoActiveDescendants
  for 6 but 6 Agent, 6 Power, 6 Grant expect 0

check ChildAuthorityComesFromCurrentGrant
  for 6 but 6 Agent, 6 Power, 6 Grant expect 0

check EvidenceCannotMintAuthority
  for 6 but 6 Agent, 6 Power, 6 Grant, 6 Evidence expect 0

check ProviderFailureIsNonAmplifying
  for 6 but 6 Agent, 6 Power, 6 Grant expect 0
