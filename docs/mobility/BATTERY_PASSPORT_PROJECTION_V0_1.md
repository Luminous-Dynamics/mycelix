# Battery Passport projection compiler v0.1

## Purpose

This work defines a **regulatory projection compiler** over a durable, evidence-qualified battery lifecycle record. It must not become the lifecycle record itself, a passport registry, a compliance certifier or a source of battery measurements.

The architecture deliberately separates:

1. **Engineering/lifecycle record** — physical asset identity, exact configuration, component lineage, observations, estimates, tests, repairs, verified service completion, revalidation and custody.
2. **Regulatory projection** — a versioned interpretation of which source facts map to which regulatory data points, for which battery category, effective date and access class.
3. **Registry operation** — registration or update by the responsible economic operator, or a party acting under explicit written authorisation.

The responsible economic operator retains ultimate legal responsibility. The projection compiler must never register, publish or sign on its behalf.

## Regulatory source baseline

The initial mapping is based on the European Commission's *Guidance Document: Digital Batteries Passport – data points by category*, Version 3.2, 19 October 2026:

https://single-market-economy.ec.europa.eu/document/download/cd1e5e6c-4a4a-4b99-995a-49eb6916187e_en?filename=Digital+Batteries+Passport+-+data+point+by+category.pdf

Commission Battery Passport FAQ:

https://single-market-economy.ec.europa.eu/single-market/digital-product-passport/eu-digital-product-passport-faq-batteries_en

Regulation (EU) 2023/1542:

https://eur-lex.europa.eu/legal-content/EN/TXT/?uri=CELEX:32023R1542

The Commission guidance documents 71 data points and differentiates EV, LMT and industrial applicability, information access and February 2027 fill/display status. It explicitly states that it is guidance, can evolve, is not authoritative interpretation and does not add legal requirements. Production mappings must be reconciled against the Regulation and any then-applicable delegated/implementing acts. This v0.1 registry is an engineering transcription of selected points, not a legal opinion.

The mapping effective date for relevant battery passports is 18 February 2027. The registry-launch date (20 July 2026) is not the same as the battery-passport obligation date.

## Paradigm shift: compile evidence into purpose-bound projections

Do not use a flat JSON form where empty, null and absent all mean the same thing.

Each data point has independent dimensions:

- **Applicability**: mandatory, optional-if-available, conditional, not-to-be-filled/displayed at the effective date, or applicable from first use.
- **Disclosure class**: public, legitimate-interest restricted, or authority-only.
- **Evidence state**: measured, reference-tested, derived, predicted, simulated, vendor-claimed, document-backed or unknown.
- **Publication disposition**: publish, hold for evidence, withhold by access policy, not required at this date, not applicable, or require operator/legal review.

No dimension can be inferred from another. A field may be legally required and restricted; it may be technically measurable but not publishable to the public. An estimate cannot silently become a measurement. Not-to-be-filled/displayed is not missing evidence. A hash or signature establishes byte identity/issuer binding, not physical truth or regulatory compliance.

## Selected v0.1 mapping

See `registries/eu_battery_passport_guidance_v3_2_selected_points.json` for the machine-readable selected-point matrix.

High-value distinctions from Guidance v3.2 include:

| Point | Data point | EV | LMT | Industrial |
|---:|---|---|---|---|
| 1–4, 6–15 | Core identity/manufacturer/category/technical descriptors | Public, generally mandatory | Public, generally mandatory | Public, generally mandatory |
| 5 | Manufacturer web/email | Optional if available | Optional if available | Optional if available |
| 17–18 | Carbon footprint declaration/label | Not to be filled/displayed as of Feb 2027 | Same | Same |
| 19 | Responsible-sourcing due-diligence information | Not to be filled/displayed as of Feb 2027; Article 48 timing applies later | Same | Same |
| 47–49 | Spare parts, dismantling and safety information | Restricted to legitimate interests and designated authorities; mandatory | Same | Same |
| 50 | Conformity test-report results | Restricted to notified bodies, Member State authorities and Commission | Same | Same |
| 51–60 | Individual battery capacity, fade, power, resistance, lifetime/efficiency parameters | Restricted; many fields apply to rechargeable batteries | Similar category rules | Conditional for some non-rechargeable/industrial cases |
| 61 | State of certified energy (SOCE) | Restricted; mandatory | Not to be filled/displayed | Not to be filled/displayed |
| 62 | Remaining capacity | Not to be filled/displayed | Restricted; mandatory | Restricted; only stationary BESS with BMS |
| 63 | Remaining power capability | Not to be filled/displayed | Restricted; mandatory | Restricted; only stationary BESS with BMS |
| 64–66 | Remaining round-trip efficiency, self-discharge, ohmic resistance | Not to be filled/displayed | Restricted; mandatory | Restricted; only stationary BESS with BMS |
| 67 | Battery status (original/repurposed/re-used/remanufactured/waste) | Restricted; mandatory | Same | Same |
| 68–71 | Cycles, negative events, environmental-condition histogram, SOC histogram | Restricted; applicable from first use | Same | Same |

The summary table is deliberately compressed. The JSON registry carries the individual statuses and conditions. Never use this prose table as a production legal rules engine.

### Important lifecycle distinctions

- A simple repair does not by itself create a new passport.
- Remanufacturing or repurposing requires a new passport linked to the prior passport.
- The passport status lifecycle (original, repurposed, re-used, remanufactured, waste) is not interchangeable with an internal service-workflow state.
- Dynamic information should be updated when battery status changes or is being considered, with update frequency proportionate to the data and purpose.
- The current economic operator remains responsible for passport contents even when another operator is authorised to maintain them.

## Projection input contract

A projection input must bind:

- exact physical battery reference and battery category;
- current pack/configuration digest and predecessor passport reference where applicable;
- target jurisdiction and effective date;
- registry guidance identifier, version, publication date, source URL and source document digest;
- data-point registry digest;
- current lifecycle status and first-use date;
- source evidence for every populated field;
- authorised submitting/responsible operator and the scope/validity of any written delegation;
- access-control context for the requesting audience.

A source record also carries its own evidence class, measurement/protocol conditions, source timestamp/window, units, method/model version, uncertainty or explicit unknown, issuer/authority and digest where available.

## Projection outcomes

The compiler returns explicit per-data-point outcomes:

- `READY_TO_PUBLISH` — applicable, source-supported and allowed for the requested audience.
- `MISSING_REQUIRED_EVIDENCE` — required by the selected mapping, but no admissible evidence is present.
- `BLOCKED_UNQUALIFIED_SOURCE` — a value exists but evidence class/domain/identity is not sufficient for this use.
- `WITHHELD_BY_ACCESS_POLICY` — source exists but audience does not have access.
- `NOT_REQUIRED_AT_EFFECTIVE_DATE` — guidance marks it not to be filled/displayed yet.
- `NOT_APPLICABLE` — the declared category/condition does not apply.
- `CONDITIONAL_REVIEW_REQUIRED` — the condition cannot be established from admitted evidence.
- `OPERATOR_AUTHORIZATION_REQUIRED` — a human/legal operator must authorise a registry action.

The compiler does not output a Boolean `compliant` claim. A readiness report is a projection-quality and exception report, not a legal determination.

## Frozen benchmark and adversarial tests

The v0.1 fixtures should test:

1. EV, LMT and stationary industrial BESS category projections.
2. EV SOCE versus LMT/BESS remaining-capacity/power distinctions.
3. Restricted data requested by a public audience.
4. Public manufacturer/core identity fields.
5. Missing mandatory source evidence.
6. Model-predicted SOCE presented as a reference-test result.
7. A vendor claim presented as a direct measurement.
8. Guidance version change after a projection was generated.
9. First-use dynamic data not yet applicable versus missing after first use.
10. Battery changed to repurposed/remanufactured without predecessor linkage.
11. Simple repair incorrectly triggering a new passport.
12. Delegated operator with expired/insufficient written authorisation.
13. Duplicate component/battery identity and configuration drift.
14. Unknown or unsupported category.
15. Not-to-be-displayed fields incorrectly treated as missing obligations.
16. Access-policy change after a previously generated public export.

Acceptance targets:
- 100% of outputs bind exact source registry version, category, effective date and configuration.
- Zero restricted values in a public projection.
- Zero unsupported values in a publishable disposition.
- Zero not-to-be-displayed fields counted as missing obligations.
- Zero new passport lineage gaps in remanufacture/repurpose fixtures.
- 100% deterministic replay and input-order permutation invariance.
- All unresolved authority, access, applicability and evidence states remain explicit.

Benchmark against a manual spreadsheet workflow and static template generator on the same fixtures. Report applicability accuracy, disclosure violations, source-evidence coverage, exception detection, stale-data detection, lineage completeness, regeneration/reconciliation effort and portability. Do not claim superiority without independent adjudication.

## Ownership integration

- Mycelix Mobility Commons preserves identity, configuration, component and lifecycle lineage.
- Symthaea produces advisory measurements/inferences, estimator qualifications and decision reports; it cannot self-authorise values as regulatory facts.
- Symtropy owns physical integration, architecture, repairability, test and validation evidence.
- Sol-Atlas supplies spatial/manufacturing context where useful.
- The responsible economic operator controls legally required content and registry actions.

## Claim ceiling

v0.1 is an engineering projection/applicability prototype. It is not legal advice, a notified-body decision, a conformity assessment, proof of compliance, a Battery Passport Registry client, authority to publish restricted information, or proof that any battery is safe or fit for a service.
