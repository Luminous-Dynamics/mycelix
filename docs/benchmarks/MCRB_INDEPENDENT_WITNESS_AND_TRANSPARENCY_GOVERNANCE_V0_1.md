# MCRB Independent Witness and Transparency Governance v0.1

**Status:** Research specification; not implemented or qualified  
**Tracking:** [#6402](https://github.com/Luminous-Dynamics/mycelix/issues/6402)  
**Composes:** #6346, #6357, #6140, #6392, #6335

## 1. Problem statement

MCRB now has a source-level architecture for scenario-bound DSSE/in-toto correctness-gate attestations. The inspected gate verifier accepts exactly one DSSE signature per envelope and one attestation per required gate. The trust policy binds a signing key to a verifier identity, builder identity, allowed gates and a validity interval.

This is a good first signature-verification boundary. It does not establish that the signer was independent of the benchmark runner, another signer, the evidence collector, the build platform, the identity provider or the source of the observation.

A signature proves that a key signed bytes. It does not prove:
- the underlying observation is true;
- the signer used an independently implemented verifier;
- two keys are operated by independent organizations;
- two attestations use independent source evidence;
- the signer did not share a compromised builder, runner, identity provider or administrator;
- the signed result was publicly committed before a later dispute;
- a log has not shown different histories to different verifiers.

The trust boundary therefore needs another explicit layer.

## 2. Design principle: count failure domains, not keys

Do not call two signatures independent merely because the key IDs differ.

Model at least four independent dimensions:

1. **Administrative control domain** — who can provision, revoke, operate or coerce the witness.
2. **Execution domain** — builder, CI provider, cloud tenant, runner fleet, host and verifier binary distribution.
3. **Evidence-source dependency domain** — collector, data feed, sensor, provider, observation endpoint and upstream record source relevant to the proposition.
4. **Cryptographic identity domain** — signer key, key owner, rotation lineage, revocation and validity.

A witness policy must describe known common-mode relationships. It must not interpret an absent relationship as independence.

Use explicit dispositions:
- `independent` — reviewed evidence supports separation for the relevant dimension;
- `known-shared` — a dependency/control domain is shared;
- `unknown` — the relationship has not been established.

Unknown relationships cannot satisfy a required distinct-failure-domain quorum. This is conservative by design.

Do not count a second key, second container, second workflow, mirrored receipt or re-signed copy as an extra witness if its control, execution or evidence-source failure domain is shared.

## 3. Witness trust policy

A versioned policy is provisioned outside the mutable benchmark runner. Its exact digest is pinned by the verifier's protected configuration and included in the run receipt.

Each witness entry binds:

- stable witness ID and signer key ID;
- public key and rotation predecessor/successor;
- accountable organization and administrative-control domain;
- execution platform, builder, runner-fleet and cloud-account domains;
- verifier identity, version, source revision and binary digest;
- allowed gate IDs, scenario classes and builder identities;
- required evidence-source classes and dependency declarations;
- valid-from / expiry times and revocation status;
- witness independence review identity, version and evidence digest.

The trust policy has an explicit owner and review lifecycle. Hashing a policy is not evidence that its owner is legitimate. The process that provisions the expected policy digest remains an external trust-root decision.

## 4. Quorum profiles

Use named, versioned policy profiles. Never hard-code one universal threshold or silently lower a threshold when a witness is unavailable.

| Profile | Purpose | Proposed evidence rule |
|---|---|---|
| Diagnostic | Development and debugging | One valid witness; output is diagnostic-only and cannot produce eligible QOPS |
| Standard | Public benchmark qualification | Initial experiment: 2-of-3 independently administered witnesses, with distinct administrative and execution domains |
| High consequence | Authority, revocation, privacy leakage, recovery and external effects | Stronger policy selected by domain risk analysis; require independent observation where the proposition needs it |

The initial 2-of-3 profile is an experimental policy, not a BFT theorem, legal assurance or production recommendation. Compare 2-of-3, 3-of-4 and 3-of-5 under explicit compromise and correlation models before selecting a policy for any high-consequence workload.

A quorum is computed separately for each gate and exact run context. Every qualifying attestation must bind the same:

- run ID and scenario ID;
- scenario and operation-contract digests;
- source SHA and source-tree digest;
- runtime, DNA/hApp and topology identity;
- evidence proposition and raw artifact digests;
- policy digest and time window.

Attestations from different runs, scenarios, source heads, runtime identities or evidence propositions cannot be combined.

## 5. Per-gate verifier semantics

For each required gate:

1. Derive the required gate set from the frozen scenario manifest.
2. Validate every DSSE/in-toto statement and its context.
3. Verify each signature against the externally pinned witness policy.
4. Reject revoked, expired, unknown, unauthorized or stale witnesses.
5. Evaluate organization, execution and evidence-source relationship constraints.
6. Deduplicate by witness identity and failure domain, not just key ID.
7. Preserve every valid PASS, FAIL and INCOMPLETE attestation.
8. Detect conflicting valid statements for the same gate/context. Do not majority-vote away a conflict.
9. Apply the exact policy quorum and required source-diversity rule.
10. Produce an opaque verified gate-set capability only when the full policy succeeds.

A failed, incomplete, missing or contradictory gate cannot be hidden by extra signatures.

A valid signature over a false sensor observation remains a false assertion. Where the gate concerns an external or physical effect, compose #6140 and #6357's evidence-kind, source-dependency, coverage and outcome semantics. Do not substitute signer count for independent observation.

## 6. Transparency and anti-equivocation

Eligible benchmark results should be committed to an append-only, Merkle-verifiable transparency log.

Each entry commits to:
- canonical DSSE envelope digest(s);
- gate-set digest and run/scenario identity;
- trust-policy digest and witness-policy version;
- source, runtime and topology identity commitments;
- public artifact commitments and verifier/tool versions.

The proof bundle should contain:
- log identity and log-key trust-root reference;
- entry digest, log index and tree size;
- Merkle inclusion proof;
- signed checkpoint / tree head;
- consistency proof from the verifier's previously pinned checkpoint;
- checkpoint observation time and verification result.

Verifiers persist the latest accepted checkpoint. A new checkpoint without a valid consistency proof is not silently accepted. Separate verifiers should gossip checkpoints or otherwise compare them to detect split views and equivocation.

A signed promise of future inclusion is not the same as a verified inclusion proof. A run may be labelled pending-log-inclusion, but it cannot be promoted to transparency-qualified until a valid inclusion proof and required consistency proof are available.

Offline verification must be possible from a receipt bundle. Log availability should not be required to re-verify an already complete inclusion/checkpoint/consistency bundle.

### Privacy boundary

Never publish raw private evidence to a public transparency log. Do not assume that a plain SHA-256 digest hides low-entropy or guessable data. Use access-controlled evidence bundles and, where needed, a salted commitment or another privacy-preserving commitment whose opening is available to authorized reviewers.

A log proves that a commitment was included in a log view. It does not prove the committed proposition is true or that the evidence source was independent.

## 7. Adversarial scenario pack

The companion `MCRB_WITNESS_GOVERNANCE_SCENARIOS_V0_1.json` freezes the following scenario families:

- duplicate key IDs and repeated signatures;
- multiple keys under one organization counted as separate witnesses;
- different organizations sharing a builder, runner, cloud account or identity provider;
- unknown or incomplete common-mode dependency inventories;
- one compromised witness signing a false PASS;
- quorum at threshold, threshold minus one and above threshold;
- contradictory PASS and FAIL attestations for the same exact context;
- same gate/run with a different evidence digest;
- source/runtime/context mismatch and stale replay;
- revoked, expired, rotated and compromised keys;
- trust-policy digest change after run creation;
- missing inclusion proof and stale checkpoint;
- invalid consistency proof, rollback and split-view checkpoints;
- delayed inclusion promise without a completed inclusion proof;
- low-entropy private-evidence disclosure through public commitments;
- transparency service outage and offline receipt verification.

All negative controls must be included in the public test corpus. The test corpus should include mutants that intentionally weaken the independence policy, allow a duplicate witness, skip consistency verification or count a signed promise as final inclusion.

## 8. Metrics

Report independent dimensions; do not collapse them into a scalar trust score.

### Correctness and security

- false-eligible rate under witness compromise;
- independence false-promotion rate;
- duplicate/mirrored witness inflation;
- common-mode compromise acceptance rate;
- conflicting-attestation suppression or concealment rate;
- stale/revoked signer acceptance rate;
- stale-checkpoint, rollback and split-view acceptance rate;
- private evidence leakage through public commitments.

### Availability and cost

- quorum availability during witness outages;
- time to collect a valid quorum;
- signature and policy verification latency;
- inclusion and consistency proof verification latency;
- storage, bandwidth and CPU cost per eligible run;
- log outage behavior and offline verification success;
- independent reviewer reconstruction accuracy and time.

Correctness and availability must be reported separately. Do not lower quorum automatically to improve availability.

## 9. Relationship to existing work

This is a governance and transparency layer, not a replacement for existing semantic owners:

- #6346 — per-gate signature and scenario-context verification;
- #6357 — per-operation outcome evidence;
- #6140 — evidence kind, source dependency, independence and coverage;
- #6392 — pre-dispatch operation proposition and attempt/effect semantics;
- #6335 — raw timing and cross-host measurement provenance;
- #6359 — exact dependency lock closure.

A verifier quorum cannot make a weak operation proposition meaningful. A transparency log cannot make an untrusted observation true. A source-independent witness cannot establish a physical effect if its evidence source is not independently observed.

## 10. Claim ceiling

This specification does not implement a verifier quorum, witness independence registry, transparency log, BFT protocol or production trust-root governance. It does not establish Mycelix benchmark eligibility or runtime qualification. The proposed quorum profiles are experimental inputs for comparative evaluation only.

## References

- DSSE / in-toto attestation: https://github.com/in-toto/attestation
- Rekor transparency log: https://docs.sigstore.dev/logging/overview/
- Rekor inclusion and consistency proofs: https://github.com/sigstore/architecture-docs/blob/main/rekor-spec.md
- SLSA build provenance v1: https://slsa.dev/spec/v1.2/build-provenance
- SLSA artifact verification: https://slsa.dev/spec/v1.2/verifying-artifacts
