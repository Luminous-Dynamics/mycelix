# Mycelix Public Election Resource-Policy Anchor v1

Status: **ELECT-014 foundation; non-circular resource-policy commitment and package cross-binding**

Parent stack:

- ELECT-011 offline verifier/package contract;
- ELECT-013 bounded verifier resource envelope;
- ADV-003 qualified resource-exhaustion corpus.

Profile:

- `mycelix-public-election-resource-policy-anchor-v1`

## Purpose

ELECT-013 can decide whether a supplied election resource policy is structurally valid and whether a verifier can safely process the workload. That is not enough by itself: a malicious or mistaken operator could supply a different valid policy than the one the frozen election intended.

ELECT-014 adds an explicit commitment chain so the standalone verifier can prove that the resource policy it enforces is the policy committed by the election's certification evidence and named by the package preflight.

## The circularity trap

A naive construction would hash the final election-constitution digest into the resource-policy commitment and then place the resource-policy commitment back into certification evidence whose digest is stored inside that same constitution.

That creates a cycle:

```text
constitution digest
      -> resource-policy digest
      -> certification-policy digest
      -> constitution digest
```

ELECT-014 forbids that construction.

The canonical resource-policy commitment deliberately excludes both:

- `ElectionConstitutionV1.certification_policy_digest`; and
- `ElectionVerificationResourcePolicyV1.election_constitution_digest`.

The second exclusion matters because that field is itself the digest of the final constitution and therefore carries the same downstream dependency indirectly.

## Construction order

The v1 commitment DAG is:

```text
frozen non-certification election context
  election_definition_digest
  jurisdiction_snapshot_digest
  eligibility_rules_digest
  ballot_definition_digest
  trustee_policy_digest
  audit_policy_digest
  dispute_policy_digest
          +
package canonicalization profile
          +
exact ELECT-013 resource limits
          |
          v
SHA-256 resource-policy commitment
          |
          v
certification-policy binding
  election_definition_digest
  resource-policy commitment
  certification-requirements digest
          |
          v
SHA-256 certification-policy digest
          |
          v
final ElectionConstitutionV1
          |
          v
final constitution artifact digest
          |
          +-----------------------------+
          |                             |
          v                             v
resource policy                  package manifest
constitution-digest link         constitution artifact link
                                        |
                                        v
                               certification-policy artifact
                                        |
                                        v
                                bounded package preflight
                              resource-policy commitment
```

There is no backward hash edge from the resource commitment to either downstream constitution digest.

## Canonical commitment encoding

The resource-policy commitment uses domain-separated SHA-256 with a fixed field order.

The input contains:

1. `MYCELIX:PUBLIC-ELECTION:RESOURCE-POLICY-COMMITMENT:V1\0`;
2. seven frozen non-certification election-context digests;
3. package-canonicalization-profile digest; and
4. all ten ELECT-013 resource limits.

Every limit is normalized to unsigned 64-bit big-endian form before hashing. Rust `usize` values are checked before conversion rather than silently truncating.

The certification-policy binding uses a second domain:

`MYCELIX:PUBLIC-ELECTION:CERTIFICATION-POLICY-BINDING:V1\0`

and commits to:

- exact election-definition digest;
- exact resource-policy commitment digest; and
- an opaque nonzero `certification_requirements_digest` for the jurisdiction-specific certification rules.

Changing the requirements therefore changes the constitution's certification-policy digest without feeding that digest backward into the resource commitment.

## Cross-binding theorem

`validate_resource_policy_anchor(...)` requires all of the following:

- the election constitution is structurally valid;
- the ELECT-013 resource policy is structurally valid;
- the ELECT-011 package manifest is structurally valid;
- the package's election-constitution artifact digest equals the manifest's constitution digest;
- the resource policy names that same constitution digest;
- the resource policy and package manifest name the same canonicalization profile;
- the preflight package root equals the manifest package root;
- the preflight resource-policy digest equals the independently recomputed commitment;
- the certification binding names the same election definition and resource-policy commitment;
- the independently recomputed certification-policy binding digest equals the constitution's `certification_policy_digest`; and
- the package's certification-policy artifact has exactly that digest.

A substitution at any one of those links fails closed.

## Anchoring is not capacity

Authenticity and resource capacity are intentionally orthogonal.

A package may carry the correct anchored resource policy and still exceed a verifier's safe resource envelope. In that case ELECT-014 can establish the correct policy identity while ELECT-013 returns a certification-blocking `BlockIndeterminate(...)` resource disposition.

This distinction prevents two bad shortcuts:

```text
policy authentic  != verifier capable
verifier incapable != policy unauthentic
```

## Evidence output

A successful anchor produces `ResourcePolicyAnchorEvidenceV1`, binding:

- package root;
- election-constitution digest;
- election-definition digest;
- resource-policy commitment digest;
- certification-policy digest;
- certification-requirements digest; and
- the ELECT-013 preflight resource disposition.

This evidence is suitable for inclusion in a later standalone-verifier receipt or certification-evidence envelope.

## Qualification tests

The v1 corpus requires:

- a fully valid positive commitment chain;
- explicit proof that downstream certification/constitution digests do not affect the resource commitment;
- proof that frozen election context or resource-limit changes do affect the resource commitment;
- stale preflight resource-digest rejection;
- stale certification-binding resource-digest rejection;
- certification-policy artifact substitution rejection;
- election-constitution artifact substitution rejection;
- policy/constitution digest substitution rejection;
- package-canonicalization substitution rejection;
- certification-requirements commitment sensitivity; and
- correct separation of authentic anchoring from resource-capacity indeterminacy.

## Deliberate non-claims

ELECT-014 does not yet define:

- canonical serialization/hash recomputation for the full `ElectionConstitutionV1` artifact;
- the full jurisdiction-specific certification-requirements schema;
- signatures on the anchor evidence;
- a new mandatory base-verifier stage;
- a lifecycle transition bit requiring anchor evidence;
- archive extraction or parser sandboxing; or
- ballot/tally cryptography.

The theorem assumes the ordinary package-integrity stage has established the digest of the bytes parsed as the election constitution. ELECT-014 then checks every resource/certification link around that established artifact identity.

## Next tranche

After hosted qualification, add **ELECT-015 — certification consumption of resource anchoring**. That tranche should bind a successful resource-anchor receipt into certification evidence and make an independently verified anchor a mandatory precondition for public-election certification without modifying the meaning of the already-qualified cryptographic/protocol gaps.
