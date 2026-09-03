# SIF Credential v2 Semantic Qualification Vectors

These are normative semantic cases for the profile-bound credential layer. Byte-for-byte cross-implementation vectors should be generated only after the Rust implementation compiles green.

| Vector | Input | Expected |
|---|---|---|
| V2-001 | valid witnessed v1 statement + nonzero profile P1 | construct v2 |
| V2-002 | same v1 statement + profile P2 | different v2 credential ID and signing message |
| V2-003 | required profile = all zero | reject |
| V2-004 | valid v2 authority signature, unchanged statement/profile | verify |
| V2-005 | valid signature for P1 checked against otherwise-identical P2 | reject |
| V2-006 | valid v1 authority signature attached to v2 statement | reject |
| V2-007 | tampered derived v2 credential ID | reject before signature verification |
| V2-008 | nested statement schema is not exact v1 schema | reject |
| V2-009 | v2 outer schema changed | reject |

The Xenia verifier must reproduce these semantics independently before the profile can be used as an authorization boundary.
