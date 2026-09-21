# mycelix-stewardship-authority-evidence

`mycelix-stewardship-authority-evidence` is a narrow evidence/typing substrate for evaluator/profile authority claims used by later stewardship theorems.

It deliberately does **not** decide whether anyone actually has authority.

```text
well-formed authority evidence
!= evaluator identity proven
!= mandate legitimate
!= scope applicable
!= authority verified
!= runtime authorization
```

The v1 substrate keeps three semantic evidence planes compile-time distinct:

```text
MandateEvidenceRefsV1
AuthorityCurrentnessEvidenceRefsV1
AuthorityBindingEvidenceRefsV1
```

Those planes are carried together with exact opaque subject/profile/scope references and an independent authority-currentness assertion.

The currentness vocabulary is closed:

```text
AssertedCurrent
AssertedRevoked
AssertedSuperseded
AssertedExpired
Indeterminate
```

`AssertedCurrent` is still only an assertion backed by evidence. This crate exposes no `is_authorized`, `permit`, capability, delegation, policy, satisfaction, or execution decision API.

## Bundle reference boundary

`AuthorityEvidenceBundleRefV1` is an opaque provenance/locator reference only. It is **not** semantic identity for the evidence bundle:

```text
same bundle_ref
!= same subject/profile/scope/currentness/evidence
!= same semantic bundle
```

A later authority-binding theorem that requires exact equality must retain/compare the complete typed `AuthorityEvidenceBundleV1`, or consume a separately qualified canonical commitment theorem. It must never treat the opaque bundle reference alone as a content commitment.

Delegation remains a separate STEW-013 theorem. Domain-specific consumers must decide whether a direct or delegated authority claim actually covers their exact target, action, purpose, occurrence, benefit, or other scope.
