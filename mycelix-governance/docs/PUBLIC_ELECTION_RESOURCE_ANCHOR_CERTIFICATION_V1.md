# Mycelix Public Election Resource-Anchor Certification v1

Status: **ELECT-015 foundation; independent resource-anchor evidence becomes an additional certification gate**

Parent stack:

- ELECT-011 offline verifier and independent-verifier agreement;
- ELECT-013 bounded resource envelope;
- ADV-003 qualified resource adversarial corpus;
- ELECT-014 non-circular resource-policy anchoring.

Profile:

- `mycelix-public-election-resource-anchor-certification-v1`

## Purpose

ELECT-014 proves how a resource policy is cryptographically anchored to frozen election/certification evidence. Certification still needs a separate theorem: the election must not be able to move from `ChallengeWindow` to `Certified` merely because one implementation says that anchor passed.

ELECT-015 makes independently verified resource anchoring an additional certification precondition while preserving every existing certification requirement.

## Do not replace the existing certification transition

The existing public-election foundation already requires, among other things:

- no unresolved qualifying challenges;
- an independent verifier quorum; and
- complete certification evidence.

ELECT-015 calls the existing `validate_transition(ChallengeWindow, Certified, ...)` first. It does not reinterpret or bypass those requirements.

It then adds a second mechanically verified layer for the resource anchor.

## Reuse the existing independence theorem

The verifier contract already defines `VerifierAgreementPolicyV1` and rejects non-passing verifiers while distinguishing verifier release, implementation lineage, and builder-control domain.

ELECT-015 reuses that theorem twice:

```text
base offline verifier attestations
        |
        v
existing independent-verifier agreement
        |
        v
resource-anchor verifier receipts
(each bound to one of those same passing base runs)
        |
        v
existing independent-verifier agreement again
        |
        v
resource-anchor certification evidence
```

No second definition of verifier independence is invented.

## Public-election minimum independence floor

The v1 resource-anchor certification profile refuses policies weaker than:

- 3 total verifiers;
- 3 distinct implementation lineages; and
- 2 distinct builder-control domains.

A jurisdiction may require stronger values, but not weaker ones under this profile.

## Resource-anchor verifier receipt

Each `ResourceAnchorVerifierReceiptV1` binds:

- package root;
- election-constitution digest;
- resource-policy commitment digest;
- certification-policy digest;
- exact ELECT-014 anchor-evidence digest;
- verifier lineage;
- verifier release;
- builder-control domain;
- the exact ordinary/base verifier-run receipt digest;
- anchor-verification disposition;
- complete resource-verification disposition; and
- detailed finding digest.

The receipt is domain-separated and SHA-256 committed before it is presented to the existing verifier-agreement theorem.

## Same-verifier binding

A resource-anchor receipt is not accepted merely because its subject digests are correct.

For every anchor receipt, ELECT-015 requires an existing passing base verifier attestation with exactly the same:

```text
package root
verifier lineage
verifier release
builder-control domain
base verifier-run receipt digest
```

This prevents detached anchor attestations from being counted as if they came from the already-qualified ordinary verifier population.

## Anchor evidence digest

ELECT-015 defines a domain-separated digest of the complete `ResourcePolicyAnchorEvidenceV1`, including the ELECT-013 preflight resource disposition.

Consequently:

```text
same policy + Proceed
    !=
same policy + BlockIndeterminate(...)
```

Changing the resource disposition changes the anchor-evidence digest and invalidates stale anchor receipts.

## Capacity must pass for certification

ELECT-014 correctly permits an authentic resource policy to remain authentic even when a verifier cannot safely process it.

Certification is stricter. ELECT-015 requires:

- anchor preflight disposition `Proceed`;
- each anchor receipt's anchor disposition `Pass`; and
- each anchor receipt's complete resource-verification disposition `Pass`.

An authentic-but-over-capacity package therefore remains verifiable as authentic but cannot be certified from an incomplete resource verification.

## Base quorum is revalidated

The existing lifecycle contains the boolean `independent_verifier_quorum_passed`, but ELECT-015 does not trust that boolean by itself.

The actual base `VerifierAgreementAttestationV1` records are supplied to the gate and passed through `validate_independent_verifier_agreement(...)` again. Only after that succeeds are anchor receipts allowed to bind to those base runs.

This produces the stronger implication:

```text
certified with ELECT-015
    => existing certification transition passed
    && concrete base verifier quorum passed
    && each anchor verifier is bound to a passing base run
    && concrete independent anchor quorum passed
    && complete resource verification passed
```

## Qualification tests

The v1 qualification corpus covers:

- successful base + anchor quorum certification;
- rejection of an under-strength quorum policy;
- unresolved challenge preservation;
- revalidation of the concrete base verifier quorum rather than trusting its boolean;
- exact base-run binding for every anchor verifier;
- stale anchor-evidence digest rejection;
- resource-policy subject substitution rejection;
- non-passing anchor verification;
- indeterminate resource verification;
- indeterminate preflight resource gate;
- anchor-evidence digest sensitivity to resource disposition; and
- duplicate verifier-release / independence collapse.

## Deliberate non-claims

ELECT-015 does not yet provide:

- cryptographic signatures over the verifier receipts;
- a canonical digest for the entire base `VerifierRunReceiptV1` object;
- remote attestation of the verifier execution environment;
- proof that distinct implementation lineages share no common dependency bug;
- a concrete archive parser/sandbox;
- ballot/tally cryptography; or
- legal election certification.

`base_verifier_run_receipt_digest` is therefore a binding field whose bytes/digest must be established by the concrete offline-verifier packaging tranche before deployment.

## Next tranche

After qualification, proceed to **ELECT-016 — canonical verifier receipt/attestation envelope**: freeze language-neutral canonical bytes for the complete base verifier run and resource-anchor receipt, bind source/build/toolchain provenance, and make signatures/attestations independently checkable offline.
