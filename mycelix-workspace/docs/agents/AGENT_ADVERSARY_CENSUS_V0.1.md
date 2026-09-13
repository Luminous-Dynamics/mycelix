# Mycelix Agent Adversary Census v0.1

Status: **threat-accountability contract**

AGENT-001 is the immediate child of AGENT-000. It freezes the minimum adversary/threat
surface that later executable agent work must address explicitly.

AGENT-001 does **not** claim these threats are mitigated. It classifies each required
threat into exactly one accountability bucket:

- `future_executable_attack` — a later qualification tranche must construct an
  executable adversarial case;
- `protocol_dependent_blocker` — qualification depends on a concrete external protocol
  adapter/profile that does not yet exist; or
- `residual_research_gap` — the risk remains a named research/assurance gap that must
  not disappear from deployment claims.

The machine-readable source of truth is
`agent-threat-census-v0.1.json`.

## Governing theorem

> Every required AGENT-001 threat appears exactly once, names the AGENT-000 invariants
> it pressures, and remains routed to a concrete future attack, protocol blocker, or
> residual research gap. Passing AGENT-001 proves threat accountability only.

The following claim must remain mechanically blocked:

```text
AGENT-001 PASS != full agent security
```

## Adversary posture

The authority architecture must assume that any model, sub-agent, peer, tool server,
external provider, protocol adapter, runtime, broker, network path, storage view, or
verifier can be buggy or malicious unless its exact semantics are independently
qualified.

A compliant agent model is not part of the trusted computing base merely because it was
instructed to behave.

## Required threat census

| ID | Class | Threat | Coverage | Next |
|---|---|---|---|---|
| AT-001 | identity | Agent principal impersonation | `future_executable_attack` | AGENT-002 |
| AT-002 | identity | Controller or mandator substitution | `future_executable_attack` | AGENT-002 |
| AT-003 | identity | Runtime instance substitution | `future_executable_attack` | AGENT-002/003 |
| AT-004 | identity | Model substitution | `future_executable_attack` | AGENT-003 |
| AT-005 | identity | Credential or key theft | `future_executable_attack` | AGENT-002/008 |
| AT-006 | identity | Identity or key rotation race | `future_executable_attack` | AGENT-002/006 |
| AT-007 | intent | Mission-to-intent semantic overreach | `future_executable_attack` | AGENT-004 |
| AT-008 | intent | Prompt-injection intent confusion | `future_executable_attack` | AGENT-004 |
| AT-009 | intent | Approved-intent substitution | `future_executable_attack` | AGENT-004/007 |
| AT-010 | intent | Deceptive or spoofed approval | `residual_research_gap` | AGENT-004 |
| AT-011 | intent | Consent UI semantic mismatch | `residual_research_gap` | AGENT-004/007 |
| AT-012 | authority | Confused deputy | `future_executable_attack` | AGENT-006/008 |
| AT-013 | authority | Capability scope expansion | `future_executable_attack` | AGENT-005/007 |
| AT-014 | authority | Consumable-budget fan-out duplication | `future_executable_attack` | AGENT-005 |
| AT-015 | authority | Unauthorized re-delegation | `future_executable_attack` | AGENT-005 |
| AT-016 | authority | Delegation cycle or depth exhaustion | `future_executable_attack` | AGENT-005/010 |
| AT-017 | authority | Stale authority replay | `future_executable_attack` | AGENT-006 |
| AT-018 | authority | Revoked authority continued use | `future_executable_attack` | AGENT-006/008 |
| AT-019 | authority | Audience or provider substitution | `future_executable_attack` | AGENT-007/008 |
| AT-020 | authority | Policy or rulebook substitution | `future_executable_attack` | AGENT-006/007 |
| AT-021 | authority | Currentness TOCTOU race | `future_executable_attack` | AGENT-006/009 |
| AT-022 | runtime | Runtime-attestation downgrade | `future_executable_attack` | AGENT-003 |
| AT-023 | runtime | Compromised runtime or model | `residual_research_gap` | AGENT-003/007 |
| AT-024 | runtime | Software supply-chain compromise | `residual_research_gap` | AGENT-003 |
| AT-025 | runtime | Authority broker compromise | `residual_research_gap` | AGENT-008/009 |
| AT-026 | action | Tool-argument escape | `future_executable_attack` | AGENT-007 |
| AT-027 | action | Canonical-action ambiguity | `future_executable_attack` | AGENT-007/010 |
| AT-028 | action | Model self-certification bypass | `future_executable_attack` | AGENT-007 |
| AT-029 | action | Cross-action replay | `future_executable_attack` | AGENT-007/009 |
| AT-030 | credentials | Long-lived secret exfiltration | `future_executable_attack` | AGENT-008 |
| AT-031 | credentials | Capability-handle replay | `future_executable_attack` | AGENT-006/008 |
| AT-032 | protocol | Protocol translation authority widening | `protocol_dependent_blocker` | AGENT-011+ |
| AT-033 | protocol | Malicious A2A peer or Agent Card | `protocol_dependent_blocker` | AGENT-011 |
| AT-034 | protocol | Malicious MCP server or tool metadata | `protocol_dependent_blocker` | AGENT-012 |
| AT-035 | protocol | External authorization semantic mismatch | `protocol_dependent_blocker` | AGENT-013 |
| AT-036 | protocol | Commerce mandate mismatch | `protocol_dependent_blocker` | AGENT-015 |
| AT-037 | effect | Duplicate effect by retry | `future_executable_attack` | AGENT-009 |
| AT-038 | effect | Ambiguous commit exploitation | `future_executable_attack` | AGENT-009 |
| AT-039 | effect | Partial multi-effect completion | `future_executable_attack` | AGENT-009 |
| AT-040 | effect | External provider equivocation | `residual_research_gap` | AGENT-009/010 |
| AT-041 | evidence | Receipt replay or substitution | `future_executable_attack` | AGENT-009/010 |
| AT-042 | evidence | Receipt equivocation or tampering | `future_executable_attack` | AGENT-009/010 |
| AT-043 | evidence | Evidence withholding or incomplete view | `residual_research_gap` | AGENT-010 |
| AT-044 | evidence | Malicious independent verifier | `residual_research_gap` | AGENT-010 |
| AT-045 | privacy | Over-disclosure and cross-context correlation | `residual_research_gap` | AGENT-014 |
| AT-046 | system | Colluding sub-agents | `residual_research_gap` | AGENT-005/017 |
| AT-047 | system | Network partition or delayed revocation | `residual_research_gap` | AGENT-006/017 |
| AT-048 | system | Resource exhaustion | `future_executable_attack` | AGENT-017 |
| AT-049 | system | Malicious or ambiguous Mycelix network view | `residual_research_gap` | AGENT-006/010 |
| AT-050 | safety | Valid authority but substantively bad outcome | `residual_research_gap` | cross-domain safety policy |

## Coverage interpretation

`future_executable_attack` is a requirement, not evidence that the attack already
exists or that the threat is mitigated.

`protocol_dependent_blocker` means Mycelix must not guess the eventual adapter theorem
before the target protocol profile is implemented and frozen.

`residual_research_gap` means later implementation must keep the risk explicit even if
neighboring tests are green.

No coverage label is equivalent to `mitigated`, `resolved`, `safe`, or
`production-ready`.

## Cross-cutting requirements

The census deliberately includes attacks against:

- principal/controller/agent/runtime identity;
- Mission -> Intent interpretation and approval;
- delegation, re-delegation, currentness, and conserved budgets;
- deterministic action binding and tool arguments;
- credential isolation and broker boundaries;
- external agent/tool/payment protocol translation;
- effect ambiguity, retry, partial completion, and provider equivocation;
- receipt/evidence replay, withholding, tampering, and malicious verification;
- selective disclosure and cross-context privacy;
- colluding agents, partitions, resource exhaustion, and ambiguous network views; and
- the case where authority is valid but the resulting action is still substantively
  wrong or harmful.

The last case is intentional:

```text
Authorized != Correct
Authorized != Wise
Authorized != SafeOutcome
```

Agent authority qualification must not become a general correctness or moral oracle.

## Qualification meaning

A green AGENT-001 check proves only that the required threat census is complete under
this exact v0.1 vocabulary and that no threat silently disappears from the assurance
program.

It does not prove:

- any attack is already executable;
- any listed threat is mitigated;
- a protocol adapter is safe;
- a model is aligned or robust;
- a broker/runtime/verifier is uncompromised;
- network observations are globally complete;
- external effects are safe; or
- Mycelix agents are deployment-ready.

## Next

The implementation sequence should close threats by theorem ownership rather than by
deleting them from this census. A later threat may move from a residual/blocker into an
executable adversarial test only through a reviewed versioned change that retains the
same threat identity or explicitly supersedes it.
