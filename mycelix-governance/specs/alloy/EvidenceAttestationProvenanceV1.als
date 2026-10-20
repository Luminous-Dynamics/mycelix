module EvidenceAttestationProvenanceV1

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
  revoked: one Bit
}

sig Evidence {
  recorded: one Bit,
  signatureValid: one Bit,
  signerTrusted: one Bit,
  subject: one Agent,
  target: one Agent,
  grantsBefore: set Grant,
  grantsAfter: set Grant,
  authorityBefore: Agent -> Power,
  authorityAfter: Agent -> Power
}

fun grantDerivedAuthority[gs: set Grant]: Agent -> Power {
  { a: Agent, p: Power |
    (a.root = On and p in a.authority) or
    (a.root = Off and some g: gs |
      g.grantee = a and g.power = p and
      g.active = On and g.revoked = Off)
  }
}

fact RootAuthority {
  all a: Agent | a.root = On implies #a.authority >= 1
}

fact ActiveGrantCurrent {
  all g: Grant | g.active = On implies g.revoked = Off
}

fact GrantCannotExceedIssuerAuthority {
  all g: Grant |
    g.active = On implies g.power in g.issuer.authority
}

fact ChildAuthorityRequiresGrant {
  all a: Agent, p: Power |
    a.root = Off and p in a.authority implies
      some g: Grant |
        g.active = On and g.revoked = Off and
        g.grantee = a and g.power = p
}

fact EvidenceSnapshots {
  all e: Evidence |
    e.recorded = On implies
      e.authorityBefore = grantDerivedAuthority[e.grantsBefore] and
      e.authorityAfter = grantDerivedAuthority[e.grantsAfter]
}

fact UntrustedAttestationNoAuthorityDelta {
  all e: Evidence |
    e.recorded = On and
    e.signatureValid = On and
    e.signerTrusted = Off implies
      e.authorityAfter = e.authorityBefore
}

fact SubjectMismatchNoAuthorityDelta {
  all e: Evidence |
    e.recorded = On and
    e.signatureValid = On and
    e.signerTrusted = On and
    e.subject != e.target implies
      e.authorityAfter = e.authorityBefore
}

pred ValidTrustedBoundedAuthorityDeltaWitness {
  some e: Evidence, g: Grant |
    e.recorded = On and
    e.signatureValid = On and
    e.signerTrusted = On and
    e.subject = e.target and
    g in e.grantsAfter and
    g not in e.grantsBefore and
    g.active = On and
    g.revoked = Off and
    g.grantee = e.subject and
    g.power in g.issuer.authority and
    e.authorityBefore != e.authorityAfter
}

pred UntrustedGrantBackedAuthorityDeltaWitness {
  some e: Evidence, g: Grant |
    e.recorded = On and
    e.signatureValid = On and
    e.signerTrusted = Off and
    g in e.grantsAfter and
    g not in e.grantsBefore and
    g.active = On and
    g.revoked = Off and
    g.grantee = e.subject and
    g.power in g.issuer.authority and
    e.authorityBefore = grantDerivedAuthority[e.grantsBefore] and
    e.authorityAfter = grantDerivedAuthority[e.grantsAfter] and
    e.authorityBefore != e.authorityAfter
}

pred SubjectMismatchGrantBackedAuthorityDeltaWitness {
  some e: Evidence, g: Grant |
    e.recorded = On and
    e.signatureValid = On and
    e.signerTrusted = On and
    e.subject != e.target and
    g in e.grantsAfter and
    g not in e.grantsBefore and
    g.active = On and
    g.revoked = Off and
    g.grantee = e.subject and
    g.power in g.issuer.authority and
    e.authorityBefore = grantDerivedAuthority[e.grantsBefore] and
    e.authorityAfter = grantDerivedAuthority[e.grantsAfter] and
    e.authorityBefore != e.authorityAfter
}

run ValidTrustedBoundedAuthorityDeltaWitness
  for 6 but 6 Agent, 6 Power, 6 Grant, 6 Evidence

run UntrustedGrantBackedAuthorityDeltaWitness
  for 6 but 6 Agent, 6 Power, 6 Grant, 6 Evidence

run SubjectMismatchGrantBackedAuthorityDeltaWitness
  for 6 but 6 Agent, 6 Power, 6 Grant, 6 Evidence

assert EvidenceAuthenticityCannotMintAuthority {
  all e: Evidence |
    e.recorded = On and
    e.signatureValid = On and
    e.signerTrusted = Off implies
      e.authorityAfter = e.authorityBefore
}

assert SubjectSubstitutionCannotMintAuthority {
  all e: Evidence |
    e.recorded = On and
    e.signatureValid = On and
    e.signerTrusted = On and
    e.subject != e.target implies
      e.authorityAfter = e.authorityBefore
}

check EvidenceAuthenticityCannotMintAuthority
  for 6 but 6 Agent, 6 Power, 6 Grant, 6 Evidence expect 0

check SubjectSubstitutionCannotMintAuthority
  for 6 but 6 Agent, 6 Power, 6 Grant, 6 Evidence expect 0
