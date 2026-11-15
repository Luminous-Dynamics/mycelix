# Authenticated Workflow Research Crosswalk — LISE v1

**Status:** literature/claim crosswalk and benchmark design input; not an independent product evaluation  
**Date:** 2026-11-15  
**Primary follow-on:** [LISE-AUTH-MESH-001 / #6216](https://github.com/Luminous-Dynamics/mycelix/issues/6216)  
**Composes:** [#6189 effect-boundary enforcement](https://github.com/Luminous-Dynamics/mycelix/issues/6189), [#6193 one-shot consistency](https://github.com/Luminous-Dynamics/mycelix/issues/6193), [#6136 exact-operation effectuation](https://github.com/Luminous-Dynamics/mycelix/issues/6136), [#6140 observation provenance](https://github.com/Luminous-Dynamics/mycelix/issues/6140), [#6069 bitemporal reconstruction](https://github.com/Luminous-Dynamics/mycelix/issues/6069), [#4835 independent evaluation](https://github.com/Luminous-Dynamics/mycelix/issues/4835).

## 1. Why this crosswalk exists

The relevant comparison space has expanded beyond conventional policy engines. Authenticated-workflow research treats an agent system as a distributed system crossing prompt, tool, data and context boundaries. It combines policy checks with cryptographic identity, signed invocations, receiver-side verification, context/prompt lineage and workflow attestations.

LISE should compare against this work directly. It should not claim novelty for signatures, least privilege, monotonic policy narrowing, prompt provenance, hash chains, inline PEPs, delegation, replay protection, or policy composition.

The prospective LISE contribution is narrower and compositional: independently qualify whether cryptographic identity and workflow provenance are bound to current owner authority, the exact protected operation, effect-boundary enforcement, independent observation, bitemporal reconstruction and contestability.

## 2. Source record and evidence class

### 2.1 Authenticated Workflows preprint

Rajagopalan & Rao, *Authenticated Workflows: A Systems Approach to Protecting Agentic AI* (arXiv:2602.10465, submitted 11 February 2026): https://arxiv.org/abs/2602.10465

The preprint describes:
- Four workflow control surfaces: prompts, tools, data and context.
- Authenticated invocation records binding arguments and policy references.
- Receiver-side signature verification and policy enforcement.
- Hierarchical policy composition by intersection/monotonic restriction.
- Runtime principal/context resolution and delegated-agent scope.
- Attestation-linked workflow dependencies.
- Independent PEPs at boundary crossings.
- Integrations described for MCP, A2A, OpenAI, Claude, LangChain, CrewAI, AutoGen, LlamaIndex and Haystack.

The paper reports formal properties under its stated model and threat assumptions, plus 11 attack patterns, 18 explicit variants and 174 test cases. It reports 100% recall and 0% false positives on that author-defined test set, and maps protections to 9 of 10 OWASP Top 10 for LLM Applications 2025 risks (with LLM04 partial and LLM09 tangential). It discusses emulations of Atlas-browser and GitHub-MCP attack paths.

**Evidence class:** author-authored arXiv preprint and author-reported experiments. This is valuable prior art and a benchmark target, but not an independent LISE reproduction, production-wide security proof, or external replication. The reported test corpus and full reproducibility package must be obtained and independently assessed before a matched numerical comparison.

### 2.2 Protecting Context and Prompts preprint

Rajagopalan & Rao, *Protecting Context and Prompts: Deterministic Security for Non-Deterministic AI* (arXiv:2602.10481, submitted 11 February 2026): https://arxiv.org/abs/2602.10481

The preprint describes authenticated prompt lineage and tamper-evident context chains, along with a policy algebra intended to narrow permissions as prompts are derived.

**Evidence class:** author-authored arXiv preprint and author-reported evaluation. Its exact implementation correspondence, full datasets and claim boundaries require the same independent review.

### 2.3 Public MACAW benchmark disclosure

MACAW public benchmark page: https://www.macawsecurity.com/benchmarks

The page reports:
- AgentDojo v1.2.2: 0/389 injected attacks took effect with GPT-4o plus MACAW, versus 184/389 for the stated undefended GPT-4o condition.
- A further no-model condition reports 0/949 injected attacks across four suites and 97/97 tasks completed.
- τ²-bench v2.0.0: 36/36 rule-breaking calls refused; the page says τ²'s own tools would run 19/36. It reports 163/164 tasks in expected state, with the single refused airline task violating the airline's written rule.
- The page describes 5 ms policy decision latency for its displayed AgentDojo configuration and labels results early.

The page says full results/method are available on request. It is a vendor-published benchmark disclosure, not an independent audit. The exact harness changes, configuration, full traces, hidden/held-out status, repeated-run design and adjudication must be inspected before comparing results.

**Do not quote 0/949 as a universal attack probability or claim it proves all model-independent attacks are blocked.** It is a finite result on the named suite and configuration.

### 2.4 MACAW architecture and trust assumptions

Public documentation:
- https://www.macawsecurity.com/docs
- https://www.macawsecurity.com/docs/concepts/primitives
- https://www.macawsecurity.com/faq/how-it-works
- https://www.macawsecurity.com/faq/compliance-privacy

The documentation describes endpoint-embedded PEPs, signed invocations, a control plane for identity/registry/policy/logging, local policy caches, receiver-side signature verification and session/context binding. Its compliance FAQ describes local decisions against cached policies and a configurable offline grace period (described as about one hour by default) before fail-closed behavior.

The Authenticated Workflows preprint explicitly states:
- cryptographic hardness as a trust assumption;
- a trusted control plane (including registry, policy store, logging and routing) with cryptographic integrity and administrative controls;
- PEP enforcement integrity at framework boundaries;
- a deployment assumption that operations cannot bypass instrumented framework APIs or corrupt PEP memory;
- system-level debugger/memory-corruption attacks are out of scope.

These assumptions do not invalidate the paper's model results. They define the boundary of those results. LISE should test the assumptions, the recovery behavior if they are weakened, and whether external product claims preserve the same scope.

## 3. Critical semantic distinctions

| Cryptographic or workflow evidence | What it can support | What it does not establish by itself |
|---|---|---|
| Valid signature | Byte/invocation integrity relative to a key | Current authority, issuer competence, truthful content or benign intent |
| Authenticated principal | Identity binding under the selected trust root | Whether the principal has legitimate authority for this operation |
| Signed prompt lineage | Origin/derivation and tamper evidence under the lineage model | That a signed instruction is authorized or safe |
| Context hash chain | Tamper evidence over the chain supplied | Completeness of captured events, trusted genesis or absence of omitted context |
| Policy-bound invocation | The supplied policy reference was bound to the signed invocation | That the policy is the correct/current owner policy or that every path is mediated |
| Receiver-side PEP decision | The named PEP evaluated the received request | That all routes to the protected resource traverse that PEP |
| Workflow attestation | A named issuer asserts a prerequisite/result | That the prerequisite was independently observed or the physical effect occurred |
| Provider-signed response | The provider made an authenticated assertion | Independent physical effect observation or complete outcome coverage |
| Append-only/signed audit record | Record integrity and attribution within the key/trust model | Complete event capture, non-repudiation under key compromise or institutional legitimacy |

## 4. LISE benchmark mapping

| Authenticated-workflow surface | LISE owner / test dimension | Adversarial question |
|---|---|---|
| Prompt | #6216 prompt/data lineage | Can signed but untrusted content be mistaken for owner-authorized instruction? |
| Tool | #6189 effect interposition; #6136 exact-operation effectuation | Does the exact operation traverse the required PEP and one-shot capability boundary? |
| Data | #6140 provenance and dependency closure | Is the source authentic, competent for this claim, purpose-authorized and independent? |
| Context | #6216 session/context integrity; #6069 bitemporal reconstruction | Can cross-session replay, omitted events, late knowledge or history forks alter the decision context? |
| Delegation | #6179 reference; #6189 enforcement; #6193 consistency | Does every child/grandchild remain within current parent authority at the effect boundary? |
| Workflow attestations | #6216 plus #6140 | Does each attestation bind exact actor, operation, resource, policy epoch and observation proposition? |
| PEP and control plane | #6216 trust-anchor benchmark | What happens under registry equivocation, key revocation, policy rollback, stale caches or a compromised PEP? |
| Effect | #6136, #6194 PETC, #6140 | Is a permit, signature or acknowledgement being promoted into realized independent effect? |
| History and challenge | #6069, #4835 | Can an external reviewer reconstruct what was known and challenge a specific evidence/authority edge? |

## 5. Required experiment design

Use two complementary benchmark planes.

### Plane A — boundary conformance

A deterministic finite corpus tests:
- signature subject/operation/policy binding;
- key and registry currentness;
- policy epoch and cache freshness;
- prompt/context ancestry, integrity and completeness;
- session/tenant separation;
- delegation and attestation scope;
- PEP identity and exact route coverage;
- receiver/resource-side outcome correspondence.

Keep positive controls and explicit indeterminate states. A cryptographically valid but unauthorized or stale request must not be counted as safe merely because the signature verifies.

### Plane B — realistic agent trajectories

Use matched subsets of AgentDojo, SABench, τ²-bench and ToolSandbox where licenses, harness versions and semantics permit. Preserve separate oracles for:
- task completion;
- policy/consent correctness;
- unauthorized calls;
- realized protected-state effects;
- collateral effects;
- evidence completeness and independent observation;
- repeated-run reliability.

Do not compare raw aggregate percentages from different task populations. Pin exact versions, agent/model, tools, prompts, policy, environment, attack seeds, compute budget and observation oracle.

Run a repeated-trial design. Report pass@k only for “succeeds at least once” questions; use strict pass-all-k / pass^k style reliability when a control must hold on every run. Report false denials and benign utility separately from unauthorized-effect failures.

## 6. Formal and empirical cross-checks

- Keep the LISE executable reference independent from candidate policy/PEP code.
- Model trust roots, registry snapshots, key epochs, policy epochs, PEP identity, session lineage and attestation sequence separately.
- Check both nominal assumptions and one-assumption-at-a-time weakening: control-plane partition, stale registry, revoked key, stale policy cache, PEP rollback and alternate route.
- Reproduce counterexamples across a second implementation and at least one formal model checker where practical.
- Pin state-space bounds, assumptions, key algorithms, clock/freshness rules, cache grace periods and failure model.
- Do not call a bounded test corpus a proof of arbitrary deployments.

## 7. Industry positioning

**Cedar** is a strong policy-language comparator with a formal Lean specification and differential randomized testing infrastructure. It answers policy authorization questions over principal/action/resource/context; the caller must provide relevant entity data. LISE should not claim to replace Cedar's evaluator. Test the integration boundary between a Cedar decision and current authority, PEP coverage, operation effect and evidence reconciliation.

Sources:
- https://docs.cedarpolicy.com/auth/authorization.html
- https://github.com/cedar-policy/cedar-spec

**OPA** is a strong policy-decision and operational decision-logging comparator. Its documentation exposes explicit counters for dropped decision logs due to rate and buffer limits. Missing log events therefore cannot prove that no decision/effect occurred.

Sources:
- https://www.openpolicyagent.org/docs/management-decision-logs
- https://www.openpolicyagent.org/docs/monitoring

**Zanzibar** demonstrates globally consistent relationship authorization at enormous scale. Its published system reports p95 below 10 ms and availability above 99.999% over the reported production period. It is a comparator for authorization data consistency and scale, not a claim of provider-side effect observation or institutional legitimacy.

Source:
- https://www.usenix.org/conference/atc19/presentation/pang

**MACAW / Authenticated Workflows** is a particularly important comparator for distributed PEPs, signatures, prompt/context lineage and policy-bound workflow attestations. Compare the disclosed threat model, assumptions, evidence coverage and exact observed outcomes—not only headline attack rates.

## 8. Standards and risk-framework alignment

- OWASP Top 10 for Agentic Applications 2026: https://genai.owasp.org/resource/owasp-top-10-for-agentic-applications-for-2026/
- NIST AI RMF Profile on Trustworthy AI in Critical Infrastructure (project status: ongoing in the official project page): https://www.nist.gov/programs-projects/concept-note-ai-rmf-profile-trustworthy-ai-critical-infrastructure

Use these as evolving risk/control crosswalks, not certification claims. The 2026 NIST critical-infrastructure profile is under development; pin the exact release/date of any future draft before making a crosswalk claim.

## 9. Research priorities and targets

For #6216, begin with at least 48 frozen known-answer cases and expand to 100+ development cases plus separately adjudicated holdout.

Prospective non-compensatory targets:
- zero acceptance of invalid, wrong-subject, wrong-tenant, replayed or revoked credentials in the frozen panel;
- zero stale policy/registry state accepted beyond declared freshness and grace rules;
- zero reachable route bypass accepted as complete mediation;
- zero delegation or attestation scope amplification;
- zero signed-but-unauthorized prompt/context provenance promoted into authority;
- zero attestation for a different actor, operation, resource or policy epoch accepted as a workflow prerequisite;
- zero provider acknowledgement or PEP receipt promoted into independent effect observation;
- 100% explicit indeterminate classification when route, registry, session or observation inventories are incomplete;
- exact frozen-oracle agreement across independent implementations.

Performance targets must be measured only after semantics are matched. Report p50/p95/p99 latency, throughput, cache-hit behavior, outage availability, recovery/reconciliation time, false-denial burden and operator effort. Do not trade away hard authorization/effect safety gates to improve performance.

## 10. Claim ceiling

This crosswalk is literature and benchmark design work. It does not reproduce MACAW, independently verify its reported results, prove any product secure/insecure, or establish production deployment safety. It does not claim that LISE outperforms any comparator.

A future LISE result may claim only the exact properties tested under named source versions, trust assumptions, scenario corpus, evaluator, route inventory, provider profile and observation boundary. Production and external replication remain separate qualifications.

## 11. OWASP Agent Control Standard (ACS) — a first-class interoperability comparator

Official sources:
- ACS overview: https://genai.owasp.org/resource/agent-control-standard-acs/
- Specification and current release: https://owaspacs.org/
- Normative wire-format overview: https://owaspacs.org/docs/acs/
- FAQ and claim boundaries: https://owaspacs.org/docs/topics/faq/
- Source repository: https://github.com/GenAI-Security-Project/agent-control-standard

As of 15 November 2026, the project site lists release v0.1.4, specification namespace v0.1.0 and 44 published schemas. The project describes ACS as a wire-format specification for an observed agent to send runtime hooks to a separate Guardian, receive an action disposition, and retain an observable audit trail. The project describes a v0.2.0 target of March 2027; pin the exact schema/release used for any conformance claim.

### What ACS specifies

ACS-Core includes:
- a capability-negotiation handshake and JSON-RPC envelope;
- a minimum hook set including session start, user message or agent trigger, tool-call request/result, agent response and session end, plus sub-agent start where applicable;
- runtime dispositions: allow, deny, ask and defer as mandatory, with modify recommended;
- session context and rolling SHA-256 chain hash;
- optional immutable intent;
- replay protection and a baseline HMAC-SHA256 signature;
- decision-honoring requirements and a declared liveness mechanism;
- wrapped MCP when sessions involve MCP.

Optional profiles include:
- **Trace:** OpenTelemetry semantic conventions and OCSF event mapping.
- **Inspect / Inspect-Dynamic:** agent bill of materials (AgBOM), snapshots and changes using CycloneDX, SPDX and SWID derivations.
- **Provenance:** field-level origin/source/derived-from references and an optional trust field with monotonicity semantics.
- **Crypto:** crypto-agile signatures and key algorithms.
- **Audit:** request hashes that commit context-chain entries to request content.

### Explicit claim boundary

ACS documentation says ACS-Core authenticates the channel and binds the agent to Guardian decisions, but does not by itself make a deployment secure or its policies strict. The project says tamper-evidence against a compromised Guardian requires the Crypto and Audit profiles rather than Core alone. Its FAQ also distinguishes runtime gateability/observability from preventing prompt injection itself.

LISE must therefore assess ACS conformance without promoting it to claims it does not make:
- Hook presence does not establish that every effect-capable route is instrumented.
- An observed-agent hook does not establish resource-side enforcement against a compromised host/application.
- Guardian authentication does not establish that the Guardian's policy is legitimate, current or complete.
- A rolling hash chain does not establish that all relevant events entered the chain.
- An AgBOM snapshot does not prove dependency completeness or effective control independence.
- A provenance field does not establish source competence, authority or truth.
- A signed audit event does not establish an independently observed provider/physical effect.
- An allow disposition is not proof of actual execution; ask or defer must not be coerced into permission by the host.

### LISE-to-ACS composition

| ACS element | LISE qualification question |
|---|---|
| ACS-Core hook coverage | Are all effect-capable tool, data, memory, sub-agent and alternate routes represented, and is the inventory complete? |
| Guardian decision and disposition | Is the decision bound to current owner authority, exact operation digest, purpose, resource, actor and authority epoch? |
| Session context and replay protection | Are session, tenant, key epoch, bitemporal knowledge and delegation lineages preserved across reconnect, failover and restore? |
| Trace / OCSF / OpenTelemetry | Do traces distinguish policy decision, PEP enforcement, dispatch, provider acknowledgement, protected-state effect and independent observation? |
| Inspect / AgBOM | Are models, MCP/A2A peers, tools, knowledge sources and memory stores complete, current and dependency-aware? |
| Provenance | Does source provenance preserve origin and transformations without confusing authenticity with competence, consent or authority? |
| Crypto / Audit | Are registry/key lifecycle, Guardian compromise, key rotation, hash-chain genesis, omissions, forks and request-content binding independently challenged? |

ACS should be an explicit comparator arm in #6216, not a competing LISE wire format. A read-only adapter should consume ACS schemas/events and report unsupported hooks/profiles rather than inventing missing values. Where a Guardian has only Core, report Crypto/Audit/Inspect/Trace properties as unsupported or unqualified—not as inherited guarantees.

### First ACS-specific challenge panel

Add ACS-focused cases to the #6216 development panel for:
1. Missing effect-capable hook or unwrapped MCP path.
2. Ask / defer incorrectly treated as allow.
3. Host ignores a valid deny.
4. Guardian session/key replay across tenants.
5. Guardian compromise with Core but without Crypto/Audit profiles.
6. Hash-chain genesis controlled by untrusted host.
7. Chain is internally valid but drops a hook event.
8. AgBOM omits a dynamic tool or memory store.
9. Provenance trust field increases on a derived artifact.
10. A signed Guardian approval is stale against owner authority epoch.
11. Tool-call result is present but no protected-resource effect is observed.
12. Agent/Guardian implementation reports conformance to a profile it does not implement.

### Industry position

ACS fills a useful interoperability gap around hook-based runtime control, trace, inspection and provenance. LISE should build a read-only conformance/evidence adapter rather than reimplement the protocol. The differentiation remains owner-authorized exact-effect assurance, temporal authority reconstruction, complete mediation evidence, independent effect observation, trust-dependency qualification and remedies.

The ACS project describes an AGT Guardian interoperability objective; where its public conformance harness and reference implementation are available, use that as an additional matched implementation baseline. Do not infer production enforcement or universal prompt-injection prevention from a schema/conformance pass.
### Normative v0.1.4 distinctions for conformance work

A closer reading of the current normative specification sharpens the conformance rule. ACS-Core's minimum hook set is not the same as the entire hook taxonomy:
- Mandatory core hooks include `sessionStart`, `userMessage` or `agentTrigger`, `toolCallRequest`, `toolCallResult`, `agentResponse` and `sessionEnd`; `subagentStart` is required for clients that support subagents.
- `subagentStop` is SHOULD-emit and audit-only.
- Additional hooks such as `turnStart`/`turnEnd`, `knowledgeRetrieval`, `memoryContextRetrieval`, `memoryStore`, `preCompact`/`postCompact`, and `skillRegister`/`skillLoad`/`skillUnload` are normatively defined and SHOULD be implemented when the host can observe the corresponding event. If the AgBOM includes skill components, the spec says skill lifecycle hooks SHOULD be emitted.
- Therefore, assess mandatory Core conformance and additional observable-hook coverage as separate axes. Do not call every SHOULD hook a universal MUST, and do not treat a missing hook for an unobservable framework surface as the same as a known bypass.

The six optional profiles are declared during handshake: ClientHello lists `profiles_supported`; ServerHello lists `profiles_accepted`. The profile declaration is explicitly a self-declared claim, not a verified fact. A read-only evaluator must check actual evidence for each profile rather than inherit conformance from the handshake label.

`MODIFY` is SHOULD-support, unlike the four MUST-support dispositions ALLOW, DENY, ASK and DEFER. A client that supports MODIFY must follow the specified composition and execute the modified operation. If a client does not support MODIFY, the Guardian substitutes DENY with the `modify_unsupported` reason code and an audit event; the host must not silently execute the original arguments.

A2A wrapping is **not part of ACS v0.1.4**. The `protocols/A2A/*` namespace is reserved; the detailed A2A method mapping is deferred to v0.2.0. MCP wrapping is specified in v0.1. A v0.1.4 conformance report must not claim A2A wrapping merely because the deployment uses A2A.

These distinctions are covered by the ACS extension seed in #6218: additional hooks, self-declared profile versus observed implementation, MODIFY behavior, and v0.1.4 A2A scope.

Sources (normative, version-pinned to ACS v0.1.4 / schema namespace v0.1.0):
- Conformance profiles: https://owaspacs.org/docs/spec/conformance/
- Instrument specification and hook taxonomy: https://owaspacs.org/docs/spec/instrument/specification/
- FAQ/version and roadmap: https://owaspacs.org/docs/topics/faq/