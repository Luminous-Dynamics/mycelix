# EV Battery Passport Projection Audit v0.1

**Status:** Offline Rust readiness/projection auditor with synthetic event and policy fixtures. Not a DPP Registry, legal compliance determination or safety qualification.
**Owner:** Mycelix [EV-BATTERY-PASSPORT-001 #6033](https://github.com/Luminous-Dynamics/mycelix/issues/6033).
**Related generic contract:** [MOBILITY-COMMONS-002 #3709](https://github.com/Luminous-Dynamics/mycelix/issues/3709).
**Related lifecycle/repair benchmark:** Symtropy [EV-BATTERY-CIRCULARITY-001 #2155](https://github.com/Luminous-Dynamics/symtropy/issues/2155).

## Architecture boundary

This auditor consumes:
- an existing economic-operator / authorized-host actor reference and identity commitment;
- an opaque finished-battery identity and pack configuration references supplied by Mobility Commons;
- a pinned Commission rulebook and data-model version;
- event evidence references and source digests.

It does not make Holochain ActionHash or EntryHash into engineering asset identities. Those hashes may be used as record locators, not as the physical or configuration identity. It does not add an identity zome or a new general-purpose evidence authority.

The report contains field value commitments and source-evidence references, not the full private battery telemetry payload. The full passport remains hosted by the responsible economic operator or its authorized host. The Registry packet is deliberately registration metadata only.

## Regulatory snapshot

The rulebook metadata pins the Commission's current Battery Passport guidance snapshot dated 19 October 2026:
https://single-market-economy.ec.europa.eu/single-market/digital-product-passport/batteries_en

Official FAQ:
https://single-market-economy.ec.europa.eu/single-market/digital-product-passport/eu-digital-product-passport-faq-batteries_en

Relevant design distinctions:
- Battery Passport subject is the finished, safely functioning battery; the BMS and multiple packs under a common BMS may be part of one battery subject. Independent finished batteries remain separate subjects.
- The responsible economic operator remains accountable even when passport tasks are delegated in writing to another operator.
- The full passport is hosted by the economic operator or authorized host; the Registry primarily stores registration/metadata and enables authority access.
- Some legitimate-interest access permissions are pending an implementing act. This v0.1 uses versioned policy metadata and denies those projected fields while the policy status is pending.
- Non-applicable and not-yet-applicable fields are omitted from the active semantic/data-model projection, not filled with null placeholders.
- Simple repair keeps the passport identity; repurpose/remanufacture and qualifying waste preparation for reuse create a linked successor passport; completed recycling closes the passport.

The auditor does not embed a hand-authored “71 fields are final” catalog. It requires the producer to pin an applicability catalog digest and validates only the declared data-point inventory. A separate source-import/reconciliation process must compare that catalog against the current Commission semantic repository and legally binding acts before a legal readiness claim is made.

## Contract

### Finished-battery boundary

The manifest identifies the finished battery, category, nominal energy, chemistry, pack membership, BMS and rationale/evidence for the chosen functional boundary. It explicitly distinguishes:
- one pack with its BMS;
- multiple packs sharing one common BMS and functioning as one finished battery;
- one independent finished battery pack.

This is a source-declared boundary with evidence, not a legal classification automatically inferred by software. A multi-pack/common-BMS positive fixture ensures that the passport is attached to the functioning battery boundary, not mechanically one per pack. A separate negative fixture rejects two independently functioning packs inside one passport subject.

### Responsibility and delegation

The economic_operator_ref remains the accountable operator. If the passport is hosted by an authorized_operator, the manifest must bind a written mandate from the economic operator to that host, with scope, start/end time, active/revoked/expired state and digest. The audit checks that the mandate covers passport maintenance and projection publication at evaluation time and at each delegated event time.

Delegation does not transfer legal responsibility. A waste-transfer event changes operator lineage only from its event time forward; it cannot rewrite the actor identity that authored earlier events.

### Append-only lifecycle

Events are strictly chronological and preserve:
- event actor and role;
- evidence digest and class;
- passport identity before/after;
- configuration digest before/after;
- responsible operator before/after.

State rules:
- placed_on_market establishes the initial passport;
- simple_repair, inspection, test and ordinary maintenance retain passport identity;
- remanufacture, repurpose and waste_prepared_for_reuse create a distinct successor passport linked to the predecessor;
- waste_transfer preserves passport/configuration identity but explicitly changes responsible-operator lineage;
- recycling_completed is terminal and changes the passport state to ceased.

A lifecycle event is not proof that the associated service improved capability or passed a safety test. Evidence class and source references remain explicit.

### Applicability and access

Every projected field must:
- exist in the pinned applicability catalog;
- use the pinned data-model version;
- carry a value digest, source-evidence digest and evidence class;
- be applicable and effective at the projection time;
- pass the audience's access tier and the pinned access-policy state.

Required-at-launch fields are mandatory; conditional-required fields are mandatory only where the stated condition is met. Deferred and not-applicable fields must be absent. No placeholder null is emitted.

Public, authority-only, restricted and legitimate-interest-pending tiers are distinct. While legitimate-interest access policy is pending, those fields cannot be projected to a legitimate-interest audience.

### Registry boundary

The registry packet must declare registration_metadata_only, must not contain the full passport payload, and must match the passport, responsible operator, host and rulebook identities.

### Claim ceiling

The audit schema explicitly forbids a legal-compliance claim and a safety-qualification claim. A successful report means only that the declared projection/event/authority manifest passed its structural and policy checks; it does not prove the declared legal applicability, operator status, source evidence or finished-battery classification.

## Frozen synthetic corpus

Twenty-two scenarios: seven accepted and fifteen rejected. Positive cases cover launch-ready projection, repair without passport replacement, linked successor after repurpose, a currently authorized passport host, multiple packs under a common BMS as one finished battery, waste transfer that changes accountable operator at the correct event time, and one independent finished pack with its own passport.

Negative controls cover missing required fields, deferred/not-applicable field leakage, expired/revoked delegation, repair incorrectly creating a new passport, unlinked repurpose, Registry full-payload leakage, pending legitimate-interest disclosure, recycling state errors, transfer of accountability through delegation, event-time disorder, missing field evidence, unknown field IDs, and independent packs incorrectly combined into one passport.

These fixtures validate the software contract only and do not establish passport compliance.

## Run

cargo fmt --manifest-path mycelix-commons/tools/ev-battery-passport-audit/Cargo.toml -- --check

cargo test --manifest-path mycelix-commons/tools/ev-battery-passport-audit/Cargo.toml

cargo run --manifest-path mycelix-commons/tools/ev-battery-passport-audit/Cargo.toml -- --schema mycelix-commons/schemas/ev_battery_passport_projection_v0_1.schema.json --corpus mycelix-commons/fixtures/ev_battery/passport_projection_scenarios_v0_1.json
