# Collective Assurance v0.1

A Rust qualification core for two important gaps in human–AI collaboration research:

1. **AI-off independence:** qualify a challenge phase using signed, nonce-bound isolation claims, complete process/artifact inventories, negative-control probes and explicit unknown states.
2. **Team-level randomization:** assign whole teams to preregistered treatment arms in balanced complete blocks, then independently validate the assignment schedule.

This is an assurance contract and deterministic reference implementation. It is not a sandbox, hypervisor, host-measurement agent, secure recorder, durable replay database, study platform, or human-subject result. The allocation layer uses pinned SHA-256, OS entropy and ChaCha20 crates; this does not by itself solve allocation concealment or authenticate the published commitment.

## Why this matters

A benchmark can report high AI-assisted accuracy without establishing whether a team can perform when AI is removed. Setting ai_enabled=false is not an independence proof: an agent process, inference endpoint, credential, cached answer, model weight, or prompt/session cache may remain reachable.

The validator requires a signed run/phase/nonce envelope, pinned runner/policy/platform digests, inventories, and five negative controls. It returns PASS, FAIL, or INDETERMINATE. Missing evidence never becomes a pass; explicit violations dominate indeterminate states.

## AI-off qualification boundary

Required negative controls: inference endpoint, agent control plane, external network egress, model credential vault, and cache replay canary. Each must be explicitly BLOCKED.

The process inventory must be complete; candidate-agent and model-runtime processes are disallowed. The artifact inventory must be complete; candidate-reachable AI answers, model weights, and prompt/session caches are disallowed. The external connection count must be established and equal zero.

The signature covers a crate-local length-prefixed encoding, not a public canonical JSON format. Production adapters must implement mature signature verification against pinned trust roots, durable atomic consume-once replay protection, trusted process/artifact measurement, and a genuinely isolated execution boundary. Unit-test verifiers are deliberately non-cryptographic.

AI-off means the candidate execution path cannot use AI services, agent processes, or retained AI answer artifacts during the challenge. It cannot erase what human participants remember from earlier collaboration; that retained human knowledge is part of the intervention, not a machine cache leak.

## Independent response-span audit

The `response_audit` module adds a second-stage auditor contract over the rendered answer:
- An independent evaluator annotates material response spans by UTF-8 byte offsets and SHA-256 span digest.
- Every audited claim is classified as supported, contested, unresolved or non-material.
- Supported claims need current, independently verified source evidence.
- Contested claims need supporting and contradicting evidence from disjoint source origins.
- Unresolved claims need a rationale and must not present evidence as decisive.
- Duplicate/overlapping annotations, invalid UTF-8 boundaries, stale or unverified citations, unknown evidence IDs, span tampering and incomplete-coverage attestations fail.

This does not extract claims or decide factual correctness on its own. The independent auditor must enumerate all material claims, and production must authenticate the auditor, audit protocol, evidence catalogue and coverage attestation. A self-audit by the answer-generating model is not independent evidence.

## Response constraint binding

The `constraint_binding` module is a structured reference for GroupAssistBench-style memory use:
- Each available item binds evidence ID, source digest, subject, scope, authorized responder, verification state, temporal/status state and applicability.
- Every required visible item must be explicitly applied, excluded with a rationale, applied with a condition, preserved as a conflict, or marked unresolved.
- Superseded/expired facts cannot be applied as current; disputed facts cannot be silently collapsed; unverified or unknown facts remain unresolved.
- A response cannot silently omit a required item, change its scope/status/digest, or use evidence outside the responder's authorized view.

This is not a natural-language output judge. The validator qualifies the structured evidence-use record only; production evaluation needs a separate independent audit of the actual response against source spans and user-request rubrics.

## Matched benchmark comparison

The `comparison` module checks whether two benchmark runs can support a matched leaderboard claim:
- **Hard-match controls:** benchmark identity/version, dataset/split/task family, evaluation protocol and scorer, context/output/time/token/compute budgets, agent count, memory budget, seed policy and seed.
- **Matched-system mode:** also requires identical model snapshot, tokenizer, prompt, toolset and decoding configuration. The system variant/configuration is the intended treatment difference.
- **Cross-model context mode:** allows model/tokenizer/prompt/tool/decoding differences but explicitly returns `ContextOnly`, never a matched leaderboard comparison.
- Missing, malformed or mismatched controls return `NotComparable`.

This module compares experiment manifests, not result scores or statistical significance. Paired seeds and matched budgets are necessary but not sufficient: benchmark task licenses, model availability, evaluator drift, confidence intervals and external replication must still be audited.

## User-directed assistance policy

The `support_policy` module validates an assistant's declared support choice against the user's explicit request and intent:
- Direct completion, explanation, co-solving, critique/verification, hint/challenge, or defer/pause/handoff are distinct modes.
- An explicitly requested mode cannot be changed without user confirmation.
- Hint/challenge is not the default for task-completion or accessibility requests; it requires a learning/practice intent, explicit request, or user-confirmed switch.
- A required accommodation must remain available.
- A support decision must bind to the exact request identifier.

This is a policy-contract check, not preference inference or enforcement by itself. The recorder must faithfully capture the user's stated intent/choice, and production systems must enforce the chosen mode. No participant should be forced to struggle, and asking for a direct answer must not be scored as failure.

## Response constraint binding

The `constraint_binding` module converts provenance into explicit response-use obligations:
- Every required evidence item visible to a responder must be accounted for; silent omission fails.
- Current, verified and applicable facts must be applied.
- Conditional facts must be applied with an explicit condition.
- Superseded or non-applicable facts must be explicitly excluded with a rationale.
- Disputed facts must be preserved as a conflict rather than collapsed to one answer.
- Unknown applicability, unknown status or unverified evidence must remain unresolved.
- Every response use must match the source digest, subject, scope, status and applicability in the trusted available-evidence record.
- Evidence not authorized for the responder is rejected.

This is a structured use-contract validator inspired by GroupAssistBench's Assistance Constraint Binding. It does not read the natural-language answer or prove that the answer obeys the structured declaration. Production evaluation must independently audit generated text against the response constraints and source spans. The available-evidence records also need to be derived from authenticated ledger receipts, not from model self-report.

## Evidence lineage ledger

The ledger explicitly distinguishes the **information holder** (the actor permitted to respond) from the **source origin** (the underlying record or evidence source). Multiple people repeating one source do not create independent corroboration; one holder disclosing two independently identified source records can. The `source_origin_id` is a pseudonymous provenance reference, not self-authenticating proof of independence: production must resolve it to a trusted source registry, signed source receipt or externally auditable reference.

The `ledger` module records the sequence from information assignment to a decision:

1. **Assignment:** identify the pseudonymous holder, disclosure permission and scoped recipient list.
2. **Inquiry:** record request creation and actual delivery as separate events.
3. **Disclosure:** distinguish granted information from rights-based withholding and non-response.
4. **Verification:** bind an independent verifier to the disclosed payload digest.
5. **Exposure:** record which authorized recipient actually viewed the evidence.
6. **Claim revision:** reference only verified evidence viewed by the claim author; represent supported, disputed and unknown claims separately.
7. **Decision:** reference explicit claim revisions; a decisive outcome cannot cite disputed or unknown premises, and stale claim revisions are rejected.

The ledger uses a domain-separated SHA-256 hash chain. This detects mutation relative to a retained root digest, but it is not a signature and does not establish that an event was true. Production must independently authenticate the recorder and root publication, protect the event store, and reconcile missing or contradictory transport evidence. Its event vocabulary is designed to map to W3C PROV-DM / PROV-O Entity, Activity and Agent concepts without claiming complete PROV conformance.

Rights-based withholding is an explicit valid outcome—not a missing-data failure, disclosure success, or reason to infer that the fact does not exist. The ledger stores pseudonymous references and payload digests, not sensitive payloads.

## Reproducible allocation commitment and reveal

The allocation module adds:
- OS-generated 256-bit seed and independent 256-bit salt through `getrandom`; entropy failure is returned, never replaced by a weak fallback.
- A domain-separated SHA-256 commitment binding the run ID, algorithm version, canonical team roster digest, ordered treatment arms and secret seed/salt.
- ChaCha20-based deterministic complete-block randomization after seed reveal.
- Canonical team sorting before shuffling, so equivalent frozen rosters supplied in different input orders produce the same schedule.
- Fail-closed verification of the revealed seed/salt against the original commitment and current frozen roster/design.

Publish the commitment through an independently timestamped channel before enrollment; keep seed and salt secret until the preregistered enrollment and analysis freeze; then reveal both and independently regenerate the schedule. A hash string stored only by the same operator who controls enrollment is not allocation concealment. CONSORT guidance distinguishes concealment of forthcoming assignments from blinding; the study must also separate recruitment/enrollment from access to the upcoming sequence.

## Team allocation

The allocator randomizes teams, not messages or turns. It uses predeclared strata, rejects incomplete blocks, assigns every arm once per block, uses Fisher–Yates with rejection sampling, and validates supplied schedules against the frozen roster.

The commitment/reveal module supplies OS entropy, domain-separated SHA-256 binding and a pinned ChaCha20 stream. Publish the commitment before enrollment; reveal seed and salt only after the preregistered freeze. This provides reproducibility and tamper-evident schedule checking, not independent timestamping, blinding or allocation concealment by itself. A separate operator/allocator boundary is still required.

## Research comparators

- **HiddenBench (ICML 2026):** 65 distributed-information tasks and 15 frontier models. Its reported 30.1% multi-agent distributed-information result and 80.7% single-model full-profile result are different information conditions, not a matched head-to-head. It highlights latent-holder discovery and premature convergence.
- **GroupMemBench (2026):** 715 questions across six categories; the strongest tested memory system averaged 46.0%, with knowledge updates at 27.1% and term ambiguity at 37.7%. Multi-party memory update remains a difficult problem; recall alone does not show team capability after AI withdrawal.
- **COLLABMEM (EMNLP 2026):** diagnoses collaborative memory operations rather than only answer recall: routing, access control, provenance, conflict resolution and retention. Its 58 hand-authored scenarios include 1,118 core probes, with a larger 174-scenario/3,359-probe split; the paper reports 69.6% for its strongest non-oracle framework. This is a direct comparator for the evidence-custody and role-aware memory parts of #6089, not for human capability after AI withdrawal.
- **GroupAssistBench (November 2026 preprint):** evaluates both recall and open-ended group assistance over 80,299 messages from 50 groups. It finds that memory systems can be close to full-context on recall yet lag substantially on applying distributed or conflicting memories to real assistance requests. This motivates measuring whether retained team information is actually used correctly, not just stored.
- **Empirical information design (November 2026 preprint):** tests ten frontier models in trade, bargaining and auctions and finds that exposing finer-grained information can improve one model's outcomes while degrading another's. Treat information order, disclosure scope and model as interacting design factors; more disclosure is not a universal objective.
- **Common Ground (2026):** evaluates shared understanding, joint action, referential coordination and repair in interactive human–AI puzzles. It complements this project but does not measure post-AI independent capability.
- **Cluster-randomized trial guidance:** the team is the randomization unit; power and inference must account for cluster counts and intra-cluster dependence. Allocation concealment is distinct from blinding.

The competitive target is the full capability lifecycle: independent baseline → AI-supported work → AI-off challenge → delayed retention → held-out transfer → resilience perturbation. Report quality, unique-information preservation, provenance, agency, workload, cost and worst-condition outcomes as a Pareto frontier, not one opaque score.

## Benchmark tournament

See [BENCHMARK_TOURNAMENT_V0.1.md](BENCHMARK_TOURNAMENT_V0.1.md) for the matched comparator matrix, proposed targets, statistical contract, and non-compensatory rights/safety gates across HiddenBench, GroupMemBench, COLLABMEM, GroupAssistBench, Common Ground, information-disclosure studies, and the SympoeticBench human capability-dividend track.

## Proposed qualification targets (not achieved results)

- 100% reproduction of deterministic synthetic known-answer cases.
- Zero false PASS outcomes across explicit negative controls.
- 100% detection of replayed nonces and invalid signatures.
- 100% rejection of duplicate, missing, or imbalanced team assignments.
- 100% preservation of INDETERMINATE for incomplete or unknown evidence.
- For confirmatory trials, preregister practical agency non-inferiority and capability-dividend margins from pilot variance; do not invent universal thresholds before the pilot.
- Match task family, model snapshot, information distribution, time/compute/communication budget and independent scoring before claiming superiority.

## Run

From this directory:

```bash
cargo fmt
cargo clippy --locked --all-targets -- -D warnings
cargo test --locked
```

The standalone workflow at `.github/workflows/collective-assurance.yml` uses pinned Rust 1.96 and the committed `Cargo.lock`. It currently normalizes source formatting in its isolated checkout before Clippy/tests and exports the formatted source artifact; it does not yet assert that the committed Rust files are already rustfmt-clean. The workflow's locked metadata check and `--locked` build/test steps prevent silent dependency re-resolution.

## Ownership and study gates

- [#6597 — team capability after AI withdrawal](https://github.com/Luminous-Dynamics/mycelix/issues/6597)
- [#6616 — randomized information-order deliberation](https://github.com/Luminous-Dynamics/mycelix/issues/6616)
- [#6089 — distributed information discovery and provenance](https://github.com/Luminous-Dynamics/mycelix/issues/6089)

The crate does not authorize recruitment. Human studies require ethics review, accessible consent, compensation, data minimization, withdrawal, preregistration, pilot-informed power analysis, blinded scoring and independent replication before superiority claims.
