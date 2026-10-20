# AC-061: Structured Economic Model Provenance

## Purpose

AC-061 makes the provenance of an economic reasoning model machine-readable so the Economic OS can distinguish genuine implementation identity from deployment aliases and opaque labels.

## Provenance identity

`EconomicModelProvenance` records:
- stable model identity and explicit version;
- optional model-family identity;
- optional provider/maintainer identity;
- optional implementation/code fingerprint;
- optional deployment identity;
- training/evaluation dataset identity references where disclosable;
- provenance declaration time.

## Conservative independence

An implementation fingerprint is required before a comparable provenance key can be derived. Deployment IDs, provider names, or model-family labels do not independently prove that two analyses are independent.

When implementation provenance is unavailable, comparison returns `None` and the decision boundary can report the analysis as unresolved rather than fabricating diversity.

Independence remains distinct from correctness: independent implementations can share assumptions, data, or errors.

For systemic-risk measurement, provenance is therefore an observability primitive: it makes correlated model usage visible without turning the provenance layer into a score of model quality.

## Analysis integration

`EconomicPolicyAnalysis` retains the existing `model_ref` for explicit migration compatibility and adds optional structured provenance.

When present, the structured provenance must carry the same stable `model_ref`. The analysis fingerprint now includes the structured provenance, making provenance changes observable and tamper-evident.

## Governance integration

AC-060 can conservatively count distinct analysis provenance groups using model provenance + observation snapshot + scenario identity. It also reports unresolved provenance separately.

Policy profiles remain responsible for deciding whether a particular decision requires independent analyses. The kernel does not impose a universal model-count rule.

## Why now

BIS identifies widespread use of similar AI models, data, and decision rules as a potential source of correlated behaviour, procyclicality, contagion, and third-party dependency. OECD likewise documents structural concentration across layers of the AI value chain. This makes provenance and concentration observability a system-resilience concern, not only an audit concern.

## Boundary

Model provenance does not prove model correctness, safety, authorship, legal authority, or regulatory compliance.

## Qualification

Repository CI is the qualification source. No local cargo test pass is claimed from an environment without the repository checkout.
