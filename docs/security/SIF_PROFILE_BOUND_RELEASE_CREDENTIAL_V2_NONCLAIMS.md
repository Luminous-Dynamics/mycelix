# SIF Credential v2 Non-Claims

Until the dedicated workflow is green, this branch is draft implementation evidence only.

It does **not** claim:

- that Xenia already consumes credential v2;
- that live SIF transfer routing is enabled;
- that profile-bound authorization has been independently audited;
- that credential v2 is wire-compatible with any implementation other than the specified canonical contract;
- that legacy credential v1 implies any SIF protected-transfer profile.

The intended next proof is an independent Xenia verifier that reproduces the v2 canonical statement and rejects any required-profile mismatch before protected Offer construction.
