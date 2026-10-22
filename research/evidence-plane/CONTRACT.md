# Evidence Plane Contract v0

Status: proposed contract. MUST/MUST NOT/SHOULD/MAY use their usual standards sense. No production service currently enforces this document.

## Identity and immutability

Every receipt includes a unique ID, schema version, UTC creation time, run state, evidence tier, claim ceiling, subject, sources, transforms, benchmark protocol/budget, gates, independent-verification state and limitations. The subject identifies repository, exact commit when applicable, artifact path and SHA-256 when bytes exist. Branch names alone are not immutable identity.

Hashes cover exact bytes. An unavailable or prohibited artifact may have a null digest only with explicit explanation and a lower claim ceiling. V0 hashes exact UTF-8 receipt bytes; any later canonicalization must be versioned and cross-implementation tested. Corrections create new append-only receipts that supersede old IDs rather than overwriting failure history. Signatures bind exact receipt digest and key identity.

## Sources and stewardship

Record canonical URL/DOI, publisher/steward, retrieval time, source temporal/spatial coverage, byte count/hash when retrieved, declared/observed media type, parser/version, units/CRS/time basis, terms/licence, attribution/contact requirements, known gaps/conflicts and whether raw bytes were actually ingested. A landing page, catalog listing, citation or screenshot is not raw-data ingestion.

Metadata disagreements stay explicit with origin and date. Resolution records who decided, why and supporting evidence. Contact states distinguish unknown, required_not_contacted, acknowledged_not_contacted, contacted, permission_confirmed, denied and not_required. A flag acknowledging an instruction does not prove contact. A license does not erase a separate contact condition; a contact request is not itself permission.

## Evidence tiers and claims

S0: synthetic/planned. S1: citation-level transcription/catalog evidence without end-to-end byte verification. S2: source bytes or machine-readable artifact hashed, reuse/stewardship recorded and parser/outputs linked. S3: independent replay against locked inputs and protocol with effective negative controls. S4: separate domain-appropriate external, field or operational validation.

Maximum generic claim mapping: S0 synthetic_only; S1 transcription_only; S2 source_reproducible; S3 independently_reproduced; S4 domain_validated. These ceilings are maxima, not automatic labels. Domain validation applies only to the declared claim and conditions.

## Gates and transforms

Every gate is pass, fail, unknown or not_applicable; missing never equals pass. Record source integrity, stewardship/rights, transform reproducibility, split lock, budget parity, negative controls, exact-head identity, safety/critical-service regression, independent verification and domain validation. Fail or unknown blocks any claim needing that gate. not_applicable needs written justification. CI state is not a gate result; skipped tests remain skipped/unknown.

Each transform records name, exact implementation version/commit, parameter digest, inputs/outputs and unit/CRS/time rules. Pin behavior-changing dependencies or record binary/container digests. Non-deterministic steps record seed/RNG policy, hardware/precision effects and tolerance. No implicit downsampling, extrapolation, interpolation, site allocation, imputation or unit conversion; each is an explicit transform. Missingness/censoring remain visible.

## Benchmark integrity

Before inspecting holdout outcomes, freeze endpoints/units/thresholds, data and source versions, inclusion/exclusion rules, partitions and leakage controls, baseline versions, tuning allowance, information access, hardware/compute/human budgets, precision, timeouts/retries, seed plan, statistical design, uncertainty method and hard gates.

Use matched information/resource/tuning budgets or narrow the claim. Report failures, aborted trials, cost and tails. Do not select the best seed without declaring the policy. Do not retroactively change the primary endpoint after seeing test results.

## Privacy and versioning

Public receipts must not disclose credentials, precise private locations, personal information or restricted contents. Hashes of sensitive/guessable inputs may leak equality. Storage requires access controls, retention/deletion policy and audit logs; digest is not permission to retain or redistribute.

V0 uses JSON Schema Draft 2020-12 and is not a full verifier. Compatible field additions require tests; changes to digest, tier or gate semantics require a major version. Unsupported major versions fail closed.
