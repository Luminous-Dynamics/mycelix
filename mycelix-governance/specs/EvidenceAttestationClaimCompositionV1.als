module EvidenceAttestationClaimCompositionV1

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
  authority: set Capability
}

fun authorizedCapabilities: set Capability {
  { c: Capability |
    some cl: Claim |
      cl.authorized = On and
      cl.grantBacked = On and
      cl.capability = c
  }
}

fact AuthorizedClaimsAreGrantBacked {
  all cl: Claim |
    cl.authorized = On implies cl.grantBacked = On
}

fact CompositionOnlyUsesAtomicAuthorizedCapabilities {
  all comp: Composition |
    comp.authority in authorizedCapabilities
}

pred AtomicCompositionWitness {
  one c1, c2: Claim |
    c1 != c2 and
    c1.authorized = On and
    c1.grantBacked = On and
    c2.authorized = On and
    c2.grantBacked = On and
    c1.capability != c2.capability and
    some comp: Composition |
      comp.authority = c1.capability + c2.capability
}

pred CartesianAmplificationWitness {
  some c1, c2, synthetic: Capability, cl1, cl2: Claim, comp: Composition |
    cl1.authorized = On and
    cl1.grantBacked = On and
    cl2.authorized = On and
    cl2.grantBacked = On and
    cl1 != cl2 and
    cl1.capability = c1 and
    cl2.capability = c2 and
    c1.resource = synthetic.resource and
    c2.action = synthetic.action and
    c1.action != c2.action and
    synthetic != c1 and
    synthetic != c2 and
    synthetic not in authorizedCapabilities and
    comp.authority = c1 + c2 + synthetic
}

run AtomicCompositionWitness
  for 6 but 6 Resource, 6 Action, 8 Capability, 6 Claim, 4 Composition

run CartesianAmplificationWitness
  for 6 but 6 Resource, 6 Action, 8 Capability, 6 Claim, 4 Composition

assert CompositionContainsOnlyAuthorizedAtoms {
  all comp: Composition |
    comp.authority in authorizedCapabilities
}

check CompositionContainsOnlyAuthorizedAtoms
  for 6 but 6 Resource, 6 Action, 8 Capability, 6 Claim, 4 Composition