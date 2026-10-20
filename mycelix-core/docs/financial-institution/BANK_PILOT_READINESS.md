# Financial-Institution Pilot Readiness Playbook

**Status:** proposed buyer/pilot protocol, not a certification, assurance report, or production-readiness claim.  
**Reviewed:** 2026-10-20  
**Intended audience:** bank operations, operational risk, technology risk, third-party risk, model risk, information security, procurement, internal audit, and prospective design partners.

## Executive positioning

**Start as a read-only, evidence-first operational-resilience and dependency-analysis pilot alongside existing bank systems. Do not ask a bank to replace its core banking, payments, custody, identity, or risk platforms.**

The first product should help a bank answer a bounded question:

> For one named critical service, what people, processes, applications, infrastructure, facilities, data, and third parties does it depend on; what breaks under a specified disruption; what evidence supports the assessment; and which mitigations are worth investigating?

The deliverable is a **reviewable evidence bundle and a human-reviewed scenario comparison**, not an autonomous control plane, regulatory opinion, outage-probability oracle, or guaranteed loss forecast.

### Current maturity and claims boundary

The [Mycelix repository README](https://github.com/Luminous-Dynamics/mycelix/blob/main/README.md) describes the project as pre-alpha research, says most clusters are not deployment-ready, and notes that the repository has not been independently threat-modeled or externally audited. The Finance cluster is described there as not production-ready and without a dedicated institution UI. This playbook does not upgrade those statuses.

Related issues and proposals establish direction—not completed integration or bank qualification:

- [Mycelix financial control plane and adoption layer (#4465)](https://github.com/Luminous-Dynamics/mycelix/issues/4465)
- [Mycelix financial-institution console (#4478)](https://github.com/Luminous-Dynamics/mycelix/issues/4478)
- [Mycelix composed-system incremental-value benchmark (#5046)](https://github.com/Luminous-Dynamics/mycelix/issues/5046)
- [Symthaea evidence-bounded capital-assessment adapter (#8002)](https://github.com/Luminous-Dynamics/symthaea/issues/8002)
- [Sol-Atlas asset-assessment export contract (#162)](https://github.com/Luminous-Dynamics/sol-atlas-leptos/issues/162)
- [Symtropy evidence-qualified transport benchmark portfolio (#1667)](https://github.com/Luminous-Dynamics/symtropy/issues/1667)

The four projects must not be described as integrated, independently validated, or production-ready until the exact implementation, tests, deployment profile, and evidence support those claims.

## 1. The first buyer and workflow

### Buyer problem

Start with a bank-owned critical operation and one question that the current tooling makes expensive to answer. Suitable candidates include a payment-operations service, treasury reporting workflow, branch service, or another operation the bank itself classifies as critical.

The pilot should improve the bank's ability to:

1. Assemble a scoped dependency map from source records rather than informal assumptions.
2. Identify missing, stale, conflicting, or weakly supported dependency claims.
3. Explore severe-but-plausible scenarios such as a supplier outage, identity-provider failure, cloud-region disruption, telecommunications loss, facility/power event, or a shared-service bottleneck.
4. Compare mitigation options, recovery assumptions, substitutability, and known evidence gaps.
5. Reconstruct why an assessment was produced, using its exact source manifest, scenario, model/configuration version, and run receipt.

Do not lead with tokens, generalized claims about decentralization, autonomous agents, or a vision of replacing banking infrastructure. Translate capabilities into the bank's existing operational-risk, business-continuity, third-party-risk, and change-management workflows.

### Industry baseline and differentiation hypothesis

We are entering an established market, not inventing operational resilience software from zero. Public product documentation shows that:

- [ServiceNow Operational Resilience Management](https://www.servicenow.com/products/operational-resilience.html) already describes business-service risk, metrics/dependencies/tolerances, and continuous risk/control context. Its [Third-Party Risk Management product](https://www.servicenow.com/products/third-party-risk-management.html) covers lifecycle assessments, monitoring, remediation workflows, and audit trails.
- [IBM OpenPages Third-Party Risk Management](https://www.ibm.com/products/openpages/third-party-risk) describes vendor/engagement inventories, criticality tiers, business-process relationships, questionnaires, evidence, remediation, key risk indicators, and ongoing monitoring.
- [Fusion Third-Party Risk Management](https://www.fusionrm.com/solutions/third-party-risk-management/) describes connecting suppliers and nth-party relationships to business services, processes, applications, locations, and teams, including service impact and continuity planning.

This is a feature-level scan of public product descriptions, not an independent product bake-off. It does not establish relative implementation quality, customer outcomes, pricing, performance, or what a particular bank has licensed. Before claiming superiority, run a buyer-relevant comparison against the institution's actual incumbent configuration and at least one representative alternative.

**Consequences for our product strategy:**

1. Do not differentiate on a risk dashboard, questionnaires, vendor inventory, dependency visualization, or “AI-powered” wording alone; incumbents already advertise those capabilities.
2. Position as a reversible, interoperable **evidence and scenario qualification layer** alongside incumbent GRC/CMDB/business-continuity tools.
3. Try to win on auditable links from each material conclusion to permitted source evidence, explicit observed/asserted/inferred/simulated distinctions, fail-closed handling of missing/conflicting data, independent scenario oracles, exact run replay, open exports, and measured incremental value.
4. Treat these as differentiation hypotheses, not unique capabilities, until a documented competitor evaluation demonstrates the gap.
5. Preserve deployment choice and customer control. A bank should be able to evaluate the workflow without accepting Mycelix-native finance, tokens, a DHT migration, or an autonomous decision system.

### Initial markets and regulatory portability

Use **South Africa as the first evidence and design-partner context**, while designing jurisdiction as a versioned profile, not as hardcoded legal assumptions. Keep separate requirements maps and customer-control ownership for South Africa, the US, the EU, and the UK. A global standards map is not a claim of global compliance; the pilot sponsor's legal/compliance owners decide which requirements apply to the exact entity, data, service, contract, and deployment.

### Proposed project roles

| Project | Proposed role in the pilot | Boundary |
|---|---|---|
| **Sol-Atlas** | Spatial and infrastructure context, source coverage, asset/service relationships, and resource-capacity evidence where the available data supports it. | Geography or topology does not establish live service availability, capacity, probability, or actual delivery. |
| **Symtropy** | Reproducible scenario simulation and constrained comparison of candidate mitigations. | A simulation is conditional on its model and assumptions; it is not proof of real-world behavior. |
| **Symthaea** | Bounded candidate generation, analysis of evidence gaps, sensitivity/uncertainty analysis, and explicit abstention when support is inadequate. | Model output is a proposal, not authoritative evidence, policy approval, or permission to execute. |
| **Mycelix** | Evidence lineage, policy/authority context, reconciliation cases, and reviewable decision receipts where implemented and qualified. | A hash, signature, attestation, vote, or recorded statement proves only the properties established by its protocol; it does not make an underlying claim true. |

These are intended responsibility boundaries. Each cross-project connection needs a versioned contract, golden fixtures, and independent acceptance tests before it is represented as implemented.

## 2. Pilot deployment and data boundaries

Use the least-privilege deployment the bank will approve.

### Phase A — offline evaluation

- Use synthetic fixtures and public information with reviewed terms of use.
- No production connectivity, bank credentials, customer data, personal data, or live financial effects.
- Demonstrate import/export boundaries, scenario behavior, evidence lineage, failure behavior, and result replay.
- Keep synthetic, public-source, bank-supplied, simulated, observed, and independently verified information as distinct evidence classes.

### Phase B — bank-controlled shadow mode

Proceed only after the bank approves architecture, information classification, privacy/security review, legal terms, and access controls.

- Prefer execution inside a bank-controlled environment or another specifically approved isolated environment.
- Start with sanitized or minimized configuration-management / service-inventory data and a named critical service.
- Keep source data and raw operational details inside the approved boundary unless written authorization explicitly permits export.
- Use read-only credentials and bounded exports; do not connect to payment execution, customer-account mutation, trading, custody, credit-decision, or production-control paths.
- Every output is advisory and reviewed by named bank personnel.
- Provide an immediate disable/rollback path and a documented way to delete or return pilot data.

No statement about on-premises, air-gapped, single-tenant, or bank-cloud support should be made until that exact deployment mode has been built and exercised.

### Phase C — controlled extension

Only after Phase B has met its pre-agreed exit criteria should the bank consider broader data, a second service, or additional workflow integration. Production use is a separate decision with its own security, legal, operational, resilience, model-risk, and procurement approvals.

## 3. Benchmark protocol: measure useful outcomes, not demo activity

Freeze the protocol before confirmatory runs. A successful build, convincing visualization, or one favorable scenario is not evidence of economic benefit.

### Compare fair alternatives

At minimum, compare under the same inputs, time horizon, information access, constraints, and fully loaded resource budget:

1. The bank's existing workflow or best available human/rule-based baseline.
2. A simple dependency-graph or rules-only baseline.
3. The strongest qualified single-project baseline.
4. The composed stack.
5. Leave-one-project-out ablations to measure the contribution of each component.

The benchmark should include both cases where the stack helps and cases where a simpler baseline is sufficient. Evaluator-owned ground truth must not leak into candidate inputs.

### Report separate measures

- **Coverage:** declared sources, assets, relationships, time windows, and required fields represented; missingness and rejected records.
- **Dependency quality:** precision/recall against an independently held evaluator set where such a set exists; distinguish known, inferred, disputed, missing, and indeterminate edges.
- **Scenario quality:** correctness against independent toy oracles for synthetic cases; held-out agreement with source-supported observations for real cases.
- **Decision usefulness:** whether the result changes a mitigation decision, investigation priority, or recovery plan; record when it does not.
- **Human effort:** time to assemble, verify, explain, and update the evidence pack versus the existing process.
- **Fully loaded cost:** implementation and integration effort, data preparation, compute, support, expert review, recovery, and ongoing maintenance.
- **Resilience:** outcomes under specified severe-but-plausible shocks, recovery assumptions, common-cause failures, supplier substitutions, and worst-case/tail outcomes.
- **Evidence integrity:** percentage of claims with source lineage; reproducible output rate where determinism is claimed; number of stale, conflicting, or unsupported claims incorrectly treated as verified.
- **Operational safety:** unauthorized side effects, boundary violations, incomplete run records, silent drops, and failures of the kill/disable path.

Choose a primary outcome, practical-effect threshold, uncertainty method, exclusions, and holdout before seeing confirmatory results. Preserve invalid, failed, missing, non-converged, unfavorable, and indeterminate runs in the published result census. Do not use one aggregate score to hide a critical-service floor violation or a material tail-risk regression.

### Minimum acceptance properties

A candidate pilot should not pass unless:

- Every material output binds the exact input/source manifest, scenario, policy/profile, producer version, and result identity.
- All imported source artifacts have known origin, license/usage decision, retrieval or observation time, transformations, and digest where technically applicable.
- Unknown or stale inputs remain unknown/indeterminate or widen the stated bounds; they are not silently treated as safe or available.
- Re-running a run declared deterministic reproduces its declared deterministic fields.
- Independent negative controls demonstrate that altered digests, incompatible units/time bases, missing evidence, duplicate claims, stale sources, conflicting edges, and evaluator-label leakage are detected.
- No financial or operational effect can be initiated by a model output or a frontend state change.
- The output differentiates observation, assertion, inference, simulation, approval, and independently verified outcome.
- The bank can reconstruct, challenge, and reject the assessment without requiring trust in a vendor-hosted black box.

A synthetic pass qualifies only the specified synthetic benchmark. A shadow-mode result is not a causal impact estimate. A measured improvement must be reported with its uncertainty, scope, costs, limitations, and independent review status.

### Proposed quantitative go/no-go targets (hypotheses to preregister, not achieved results)

Set the final thresholds with the pilot sponsor before the confirmatory run. The first qualification corpus should target:

- **100% run accounting:** every scheduled run has a recorded disposition; zero silent dropping of failed, invalid, missing, non-converged, unfavorable, or indeterminate runs.
- **100% declared deterministic replay:** all fields explicitly marked deterministic reproduce for identical pinned inputs, software, schema and configuration.
- **100% material-claim traceability:** every finding used to justify a mitigation has an explicit source/assumption lineage, evidence class, time scope and claim limit—or remains indeterminate.
- **Zero false-safe results in the frozen negative-control suite:** no deliberately unavailable, unknown, stale or conflicting dependency is silently promoted to safe/available. This only qualifies the tested corpus, not zero risk in deployment.
- **Exact expected outcomes on deterministic synthetic fixtures:** the candidate must match the independent oracle for fixture invariants and hard constraints. Unspecified outcomes remain unscored or indeterminate rather than being improvised.
- **Shadow-pilot value hypothesis:** seek at least 20% reduction in fully loaded analyst-hours for the scoped evidence-assembly/review task, with no material degradation against the declared baseline in dependency-detection quality and no hard-guardrail regression. This is a proposal, not a standard or promise; reset it before evaluation if the use case's economics justify another threshold.
- **Honest differentiation:** report paired outcomes against the bank's actual workflow and a comparable incumbent product/configuration where access permits. A feature checklist or one synthetic result is not enough to claim superiority.

If the workflow fails these gates, document the failures and narrow/rework the product. Never weaken a test or alter the primary metric after seeing confirmatory results to manufacture a pass.

## 4. Bank diligence and assurance package

Prepare the following artifacts before asking for a production procurement decision. If an artifact does not exist, label it **not yet available** with an owner and plan; do not substitute a promise for evidence.

### Security and architecture

- System context and deployment diagrams, trust boundaries, data-flow and egress maps.
- Threat model, abuse cases, attack surface, secrets/key-management design, tenant and legal-entity isolation design.
- Authentication/authorization model, separation of duties, privileged access, break-glass controls, and reviewable access logs.
- Software bill of materials (SBOM), dependency and license inventory, vulnerability scanning, secure build/release process, artifact provenance, and patch policy.
- Independent penetration-test scope and results when performed; unresolved findings with severity, owner, mitigation, and due date.
- Data classification, minimization, retention/deletion, backup, residency, encryption, and key-rotation evidence for the exact deployment.
- Incident response and notification process, service-level/support proposal, business continuity, recovery objectives, tested restoration, and documented exit strategy.
- Subprocessor/supplier inventory, concentration and fourth-party dependencies, change notification, right-to-audit provisions, and contractual responsibility boundaries.

### Model, data, and decision governance

- Component/model inventory, intended use, prohibited use, versioning, known limitations, and accountable owner.
- Test protocol, baseline comparison, independent validation, outcome monitoring, drift/change handling, and escalation/abstention policy.
- Source register, rights/consent and permitted-use decisions, lineage, quality limitations, and retention rules.
- Human approval and appeal path; clear distinction between recommendation, policy decision, authorized action, and settled/reconciled result.
- A mapping from applicable requirements to control objectives and concrete evidence for the exact scope. A generic framework table is not a legal opinion or evidence of compliance.

The **FSB's 15 October 2026 final sound practices for responsible AI adoption**, Basel operational-resilience principles, the applicable EU DORA regime, and the relevant jurisdiction's model-risk and outsourcing guidance are useful scoping inputs. They do not make one universal checklist applicable to every institution or deployment. The bank's legal and control owners must determine applicability.

### Assurance language

Do not say “SOC 2 certified,” “ISO 27001 certified,” “DORA compliant,” “GDPR compliant,” “production-ready,” “bank-grade,” “audited,” or “regulator-approved” unless an appropriate, current, independently verifiable artifact supports the exact claim and scope.

Do not use unmethoded compliance percentages. Report each control as one of: **evidence verified for this scope**, **partially evidenced**, **not implemented**, **not tested**, **not applicable with rationale**, or **indeterminate**. Name the evidence artifact, version/date, reviewer, and limitations.

## 5. Commercial and licensing gates

- Confirm the exact repository, directory, component, and dependency licenses used in the proposed deployment. The Mycelix repository documents AGPL-3.0-or-later for many clusters and separate licensing boundaries; procurement must review the actual component set and any proposed commercial license before implementation.
- Offer no implied support, uptime SLA, recovery guarantee, indemnity, or roadmap commitment until it is staffed and contractually defined.
- State what the vendor can inspect, operate, update, or access. Minimize vendor privileges and avoid unnecessary central collection of bank data.
- Define portability, exit, data return/deletion, key custody, audit access, and continuity if the vendor or a collaborating node becomes unavailable.
- Keep initial evaluation reversible and inexpensive. No migration project should be required to test the thesis.

## 6. Suggested pilot plan

Planning estimate only; duration depends on the bank's approvals and data readiness.

| Stage | Work | Exit gate |
|---|---|---|
| **0 — qualify** | Pick one operational owner, one critical service, one decision, one baseline, data boundary, and sponsor in risk/technology. | Written pilot charter, named control owners, data/rights approval, scope and stop conditions. |
| **1 — offline benchmark** | Synthetic/public fixtures, simple baseline, negative controls, source manifest, scenario replay, and reviewer walkthrough. | Reproducible result bundle; all gaps labelled; no unsupported claims; benchmark protocol frozen. |
| **2 — shadow evaluation** | Bank-approved minimized data in an approved environment; compare the analysis to the bank's existing process and expert review. | Bank reviewers accept the evidence trail and identify measurable incremental value; no unauthorized effects. |
| **3 — independent review** | Security, privacy, model-risk, architecture, legal, procurement, and operational-resilience review against the exact build and scope. | Written approval, accepted residual-risk record, remediation plan, tested recovery/exit, and explicit production decision. |

Stop or narrow the pilot if value cannot be measured, required data rights are unavailable, uncertainty is being hidden, the bank cannot independently reconstruct results, or the integration cost overwhelms the expected benefit.

## 7. Owners and implementation sequence

1. **Mycelix:** define the minimum control/evidence packet and a bank-specific profile on top of existing event, authority, and reconciliation semantics; finish the institution console only against real workflows and authorization invariants.
2. **Sol-Atlas:** expose the existing evidence-bounded service/resource assessment contracts through a versioned read-only export; do not duplicate its physical/resource models.
3. **Symtropy:** provide reproducible disruption and mitigation comparisons with explicit model limits and an effects-disabled boundary.
4. **Symthaea:** produce bounded proposals and sensitivity/abstention evidence; keep evaluator truth isolated and require independent validation for claimed performance.
5. **Cross-project:** freeze one golden end-to-end fixture, manifests and schema versions; then build a shadow-mode evidence bundle and measure it against the incumbent baseline.

Use the existing workstreams rather than creating a second bank ontology or duplicate simulation/evidence store. The [CIV-IMPACT-001 benchmark](https://github.com/Luminous-Dynamics/mycelix/issues/5046) already calls for ablations, full-cost accounting, held-out scenarios, adverse results, and a stepwise evidence ladder; make that the common evaluation discipline.

## External references

- [Financial Stability Board — Sound Practices for Responsible Adoption of AI (15 October 2026)](https://www.fsb.org/2026/10/fsb-finalises-sound-practices-for-the-responsible-adoption-of-artificial-intelligence-ai/)
- [Basel Committee — Principles for Operational Resilience](https://www.bis.org/bcbs/publ/d509.pdf)
- [EU Digital Operational Resilience Act — Regulation (EU) 2022/2554](https://eur-lex.europa.eu/eli/reg/2022/2554/oj)
- [OCC — Revised Model Risk Management Guidance, Bulletin 2026-13](https://www.occ.gov/news-issuances/bulletins/2026/bulletin-2026-13.html)
- [NIST AI Risk Management Framework Core](https://airc.nist.gov/airmf-resources/airmf/5-sec-core/)
- [NIST AI RMF Playbook](https://www.nist.gov/itl/ai-risk-management-framework/nist-ai-rmf-playbook)

---

**Non-claim:** This document proposes a path to qualify a narrow design-partner pilot. It does not establish implementation status, interoperability, regulatory compliance, security certification, economic benefit, or readiness for production banking.
