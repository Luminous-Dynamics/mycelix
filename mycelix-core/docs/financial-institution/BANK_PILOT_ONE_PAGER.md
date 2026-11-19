# Operational Resilience Evidence Pilot

**A low-risk way to test whether evidence-bounded dependency analysis improves a bank's resilience workflow.**

## The problem

A bank's critical services depend on combinations of applications, processes, data, facilities, infrastructure and direct or indirect providers. When those relationships are recorded in different inventories and documents, teams can struggle to answer a practical question quickly:

**If a critical dependency fails or degrades, what service is affected, what evidence supports that conclusion, and which mitigations remain feasible?**

The goal is not to replace the bank's existing GRC, CMDB, business-continuity or third-party-risk systems. It is to test whether a reproducible analysis layer makes their evidence more useful for a named decision.

## The proposed pilot

Select **one bank-defined critical operation** and compare the bank's current workflow with a read-only evidence and scenario-analysis workflow.

The pilot starts with synthetic fixtures and permitted public sources. It moves to minimized bank-supplied records only after the bank approves the data, security, legal, privacy and procurement boundaries. Shadow-mode outputs do not execute actions or alter production systems.

### What the bank receives

- **A scoped dependency map** that separates observed facts, institution assertions, model inferences and unknown relationships.
- **A source and gap register** showing provenance, time scope, conflicting or stale records, permitted-use decisions and missing evidence.
- **Severe-but-plausible scenario comparisons** for supplier outages, common-cause failures, partial degradation, recovery-path failures and constrained substitutes.
- **A human-reviewed mitigation comparison** with assumptions, service-floor constraints, sensitivity and explicit abstentions.
- **A reproducible evidence bundle** binding input/source identities, scenario definitions, software/configuration version, outputs, failures and claim limitations.
- **An outcome report** comparing the workflow with the bank's existing process, including reviewer effort and fully loaded cost.

## How the four projects fit

| Project | Pilot contribution | Boundary |
|---|---|---|
| **Sol-Atlas** | Qualified spatial, site and lifeline context where exact source data supports it. | Map geometry is not proof of a bank's dependency or live service capacity. |
| **Symtropy** | Reproducible disruption scenarios and constrained recovery/mitigation comparisons. | Simulated outcomes remain conditional on explicit model assumptions. |
| **Symthaea** | Bounded candidate generation, sensitivity analysis, evidence-acquisition suggestions and abstention. | A model output never becomes evidence, policy approval or execution authority by itself. |
| **Mycelix** | Versioned evidence lineage, review/authority context and result receipts where implemented and qualified. | A signature or hash proves only the integrity/attestation properties actually established. |

These are planned roles. The four-project integration must be separately qualified; this one-pager does not claim it already exists.

## Success is measured against the current workflow

Before confirmatory evaluation, the bank and project team agree the primary outcome, baseline, practical-effect threshold and exclusions. Proposed initial qualification gates:

- Every scheduled run has an explicit disposition; no silent dropping of failed or unfavorable results.
- Every material finding has source/assumption lineage or remains indeterminate.
- Deterministic outputs replay exactly for pinned inputs and software.
- The frozen negative-control suite produces no false-safe result from deliberately missing, stale, unavailable or conflicting dependencies.
- Synthetic fixture outputs match an independent oracle on specified invariants.
- In shadow mode, test a **20% reduction in fully loaded analyst-hours** for the chosen evidence-assembly/review task, without material degradation in dependency-detection quality or hard-guardrail violations.

The 20% figure is a hypothesis to register before evaluation, not a demonstrated saving, promise, regulatory threshold or universal target. If the incumbent is already better or the integrated workflow adds no net value, report that and narrow or stop.

## Controls and boundaries

- No core-banking, payment execution, trading, custody, lending or account-mutation integration in the first pilot.
- No production writes; human reviewers remain responsible for decisions.
- Bank data stays within the bank-approved environment unless explicit written permission permits another flow.
- Synthetic, public, bank-supplied, simulated, observed and independently verified evidence remain distinct.
- Data minimization, retention, deletion/return, exit, access control, threat modeling, incident response and deployment topology require approval for the exact pilot.
- No claim of regulatory compliance, security certification, independent audit, production readiness or superiority without the appropriate evidence.

## What a design partner needs to bring

1. An operational service owner and a risk/technology reviewer.
2. One scoped critical service and its current dependency evidence/workflow.
3. A data-security boundary and approved source-use/retention rules.
4. An incumbent baseline, review time estimate, and a small set of decision-relevant disruption scenarios.
5. Agreement on the measurement protocol and who can independently challenge the result.

## Why evaluate now?

Basel operational-resilience principles emphasize mapping critical-operation dependencies and testing continuity under severe-but-plausible disruptions. The UK FCA and South African Prudential Authority also publish operational-resilience expectations relevant to the jurisdictions in which their rules apply. This pilot turns that broad objective into a bounded, auditable experiment. It does not substitute for the bank's own legal and control-owner assessment.

## References and status

- [Basel Committee — Principles for Operational Resilience](https://www.bis.org/bcbs/publ/d509.pdf)
- [South African Prudential Authority — D4/2023, Directive on Operational Resilience](https://www.resbank.co.za/content/dam/sarb/publications/prudential-authority/pa-deposit-takers/banks-directives/2023/D4-2023%20-%20Directive%20on%20operational%20resilience.pdf)
- [UK FCA — Operational resilience: insights and observations one year on (March 2026)](https://www.fca.org.uk/publications/good-and-poor-practice/operational-resilience-insights-observations-one-year)
- [Mycelix BANK-RES-001 pilot work item](https://github.com/Luminous-Dynamics/mycelix/issues/5302)

**Status:** proposed pilot brief. The products remain subject to project-specific maturity, integration, security, licensing and validation gates. This document does not claim a bank customer, completed pilot, achieved savings, certification, or production readiness.
