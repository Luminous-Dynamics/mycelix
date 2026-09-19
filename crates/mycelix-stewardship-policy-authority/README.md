# mycelix-stewardship-policy-authority

STEW-012B is the first executable composition theorem between STEW-003 policy and STEW-008 stewardship admission.

It creates only a **policy-authority candidate** when narrow structural prerequisites are present:

- the admission is labelled `AdmittedUnderProfile`;
- its domain is exactly `AccessPolicyParticipation`;
- the admitted claimant is the asserted policy issuer;
- the admitted claim structurally covers the policy target;
- explicit mandate-scope, authority-profile, mandate evidence, currentness evidence, and binding evidence are present;
- currentness is explicitly asserted as current.

```text
candidate constructed
!= mandate valid
!= admission profile legitimate
!= decider authorized
!= currentness true
!= policy authority established
!= runtime authorization
```

V1 deliberately does not support delegated issuers distinct from the admitted claimant. That requires a separate delegation theorem rather than inferring delegation from a broad claim or represented collective.
