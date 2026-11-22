# PCOE Energy-Attribute and Renewable-Claims Addendum v0.1

**Status:** proposed interoperability/assurance profile; not an accredited registry, certificate, legal approval or claim of production readiness.  
**Date:** 22 November 2026.

## 1. The missing distinction: electrons, attributes, and claims are different objects

A metered quantity of electricity, a contractual delivery, an energy-attribute certificate (EAC), and a consumer's renewable-energy claim are not the same asset or event.

PCOE should model at least four linked but separate records:

1. **Physical generation observation:** what a qualified meter reports at a declared generation boundary and interval.
2. **Contractual delivery/consumption:** what a buyer received or consumed at the agreed delivery boundary, net of applicable losses and settlement adjustments.
3. **Energy attribute certificate:** the transferable environmental attribute issued for a unit of qualifying generation under a specific accredited product code and registry.
4. **Attribute claim:** the end-user claim authorized by ownership and final retirement/redemption/cancellation of the corresponding certificate under the relevant programme rules.

A producer can generate electricity without exporting all of it. A buyer can consume electricity without owning the renewable attributes associated with it. A certificate transfer does not physically route electrons. A transparency receipt does not create a certificate. A smart meter's signed reading does not itself establish certificate ownership or retirement.

**The core invariant:** no renewable-generation claim can exceed verified eligible generation, and no attribute unit can be owned, transferred or retired more than once.

## 2. Reuse accredited Energy Attribute Certificate systems

The International Tracking Standard Foundation's accredited electricity product code describes the issuance, transfer and redemption of I-REC(E) certificates. I-REC(E) represents information about a unit of renewable electricity production, including location, facility capacity and energy source. I-TRACK's public product information identifies the certificate unit as one MWh and describes accredited issuers, registry operators, platform operators and participants.

The accredited registry is the authoritative record of EAC issuance, legal ownership, transfers and redemptions under its product code. PCOE must not become a shadow registry or claim ownership based only on its own ledger.

**Integration profile:**

- The production device must be registered with the issuer authorized for the relevant country/product.
- Registry, product-code version, issuer, production-device identity, generation interval, quantity, source attributes and exact registry transaction identifiers are pinned.
- Issuance is bounded by qualifying measured generation after applicable programme rules, losses, exclusions and correction policies.
- Transfer changes the registry owner; it does not change the physical electricity path.
- Redemption/retirement/cancellation is terminal for the corresponding attribute units. A retired unit cannot be transferred or redeemed again.
- The local PCOE projection is a reproducible evidence cache. It must reconcile to the accredited registry and never supersede it.
- Cross-registry or cross-product issuance must be checked against the same underlying generation intervals to prevent duplicate attributes.

For products with a one-MWh denomination, issue in integer multiples of 1,000,000 Wh. Keep denomination and product-code version explicit; do not hardcode all future programmes to a single unit or assume every registry has identical rules.

## 3. Generation quantity is not grid export

The meter boundary is a contract and programme choice. Keep distinct:

- gross generator/inverter output;
- behind-the-meter self-consumption;
- battery charging and discharging;
- site load;
- grid import;
- grid export;
- network delivery after losses.

Do not issue attributes against an export figure when the programme defines eligibility by gross generation; conversely, do not substitute gross generation when a contract explicitly settles export. A battery must not create a second set of renewable attributes merely by discharging energy previously counted at charging time. For mixed-source charging, preserve the provenance and apply the registry's approved accounting method.

PCOE should represent the boundary and programme rule as explicit versioned inputs. A number without its boundary, interval, unit and source method is not a settlement quantity.

## 4. Measurement and verification references

- **IEC 62052-11:2020** defines general requirements and type tests for AC/DC electricity meters, including measurement uncertainty/repeatability and timekeeping. Its published scope explicitly says it does not cover detection and prevention of fraudulent tampering; tamper detection therefore remains a separate security/control requirement.
- **OIML R 46 (2012)** addresses active electrical energy meters under legal metrological control, including type approval, verification, re-verification and relevant modifications. Applicability depends on jurisdiction and meter use.
- **IPMVP Core Concepts 2022** provides a framework for measurement and verification of energy and water efficiency impacts. Savings are not directly measured as a physical meter register; they are calculated by comparing baseline and reporting-period measurements with appropriate adjustments. Its measurement boundary and M&V plan must be pinned.

The current Rust meter function validates arithmetic plausibility and declared fields. It does not perform IEC type testing, OIML legal verification, calibration certificate authentication, IPMVP savings modelling, tamper certification or independent field inspection.

## 5. Attribute-certificate state model

A local reference projection may model:

`VerifiedGeneration → Issued → Held(owner) → Transferred(owner') → Retired(claim)`

Terminal / exception states should include:

- `Indeterminate`: registry or measurement status unavailable or conflicting;
- `Suspended`: a competent issuer/registry has suspended the device/certificate;
- `Cancelled`: a registry-authorized cancellation or correction event, linked to its reason and prior record;
- `Disputed`: ownership, generation, interval or eligibility is contested.

Never rewrite history when a certificate is cancelled or corrected. Append a registry event and reconcile every dependent claim. A partial redemption must be represented in the exact registry unit model—e.g. one certificate per 1 MWh where fractional retirement is unsupported, or a registry-authorized split if supported.

## 5.1 Temporal and geographic matching profiles

The GHG Protocol Scope 2 market-based-method revision is active as of its 16 October 2026 update. The prior 2015 guidance is under review. The public consultation ran from 20 October 2025 to 31 January 2026; the October 2026 update describes continued Technical Working Group and Independent Standards Board deliberation. Hourly matching and geographic deliverability are central proposals, while stakeholders also debate market accessibility and annual matching approaches.

PCOE should not hard-code one proposal as if it were already final. Represent at least:

- `AnnualRegionalMatching`;
- `HourlyRegionalMatching`;
- `HourlyDeliverableGridMatching`;
- `IncrementalityWindow(profile_version)` where a declared policy requires it.

Each profile must pin its standard/version, reporting period, geographic boundary/deliverability method, certificate vintage, generation interval, consumption interval, registry product code, and effective date. A result under one profile must never silently be relabelled as compliant under another.

**Research benchmark:** run identical consumption and EAC portfolios through annual, hourly and deliverable-region policies. Report eligible MWh, shortfalls, unmatched consumption, storage adjustments, residual mix treatment and administrative cost. Keep these as policy-comparison outputs, not assertions that one option is currently mandatory.

Reference: GHG Protocol, *Scope 2 Market-based Method Update: From 2025-2026 Public Consultation to Current Status* (16 October 2026): https://ghgprotocol.org/blog/scope-2-market-based-method-update-2025-2026-public-consultation-current-status

## 6. Adversarial scenarios

The companion `pcoe-energy-attribute-scenarios.v0.1.json` adds 12 scenarios. They target over-issuance, duplicate interval issuance, registry disagreement, double transfer/retirement, consumption/generation confusion, boundary substitution, storage provenance laundering, certificate split/conservation, expiry/cancellation, and unregistered devices.

A test only passes if the certificate/claim state and the no-side-effect quantity deltas are asserted. Logging an exception is not sufficient.

## 7. Metrics and proposed targets

| Metric | Proposed target |
|---|---:|
| Attribute issuance above eligible verified generation | 0 Wh |
| Duplicate certificates for overlapping generation interval | 0 |
| Certificate double transfer or double retirement | 0 |
| Attribute claim without final registry retirement | 0 |
| EAC-to-metered-generation reconciliation | 100% of issued units |
| Certificate-to-registry ownership mismatch | 0 |
| Unknown or disputed registry status promoted to valid | 0 |
| Storage provenance double-counting | 0 Wh |
| Audit reproduction of issuance/transfer/retirement | 100% of published test bundles |

Publish denomination, measurement boundary, generator identity, geography, generation interval, registry/product code, exclusions, uncertainty and correction history with every result. Zero in a synthetic suite is only zero in that suite.

## 8. Competitive strategy

Do not compete with accredited EAC registries by recreating the registry. Interoperate with their accredited issuance and redemption lifecycle. Compete on the cross-domain proof chain that connects the physical meter, verifier, contract, financing obligation, registry certificate, final retirement, and payment.

This gives PCOE a differentiated metric: **cost per independently reconciled renewable attribute unit**, alongside cost per accepted obligation. Measure registry reconciliation latency, manual exceptions, certificate-to-meter mismatch rate, double-counting incidents, and the cost to prove a consumer claim.

## 9. References

- I-TRACK Foundation, Product Code for Electricity (I-REC(E)): https://www.trackingstandard.org/product-code/electricity/
- I-TRACK Foundation, International Attribute Tracking Standard: https://www.irecstandard.org/the-standard/
- I-TRACK Foundation, Product Code FAQ: https://www.irecstandard.org/faq/
- U.S. EPA, How to Avoid Double Counting: https://www.epa.gov/green-power-markets/double-counting
- U.S. EPA, Energy Attribute Certificates: https://www.epa.gov/system/files/documents/2024-02/energy_attribute_certificates.pdf
- IEC 62052-11:2020, Electricity metering equipment — general requirements, tests and test conditions: https://webstore.iec.ch/en/publication/28212
- OIML R 46, Active electrical energy meters: https://www.oiml.org/en/publications/recommendations/46/
- Efficiency Valuation Organization, IPMVP Core Concepts 2022: https://evo-world.org/en/products-services-mainmenu-en/protocols/ipmvp

---

**Interpretation rule:** an EAC is an environmental-attribute instrument, not a physical electricity delivery receipt. The registry is authoritative for certificate ownership and retirement. Meter evidence is authoritative only within the metering device, boundary, method and legal-metrology scope actually qualified.
