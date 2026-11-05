# Jurisdiction Pack Qualification Dossier v1

**Status:** Synthetic schema and lifecycle contract; not legal advice, tax policy or a qualified jurisdiction pack.
**Tracking:** [FIN-JURISDICTION-PACK-001 / #5834](https://github.com/Luminous-Dynamics/mycelix/issues/5834)
**Composes:** #5811, #5826, #5792, #5785, #5793, #5802, #5812, #5799.

## 1. Design thesis

A jurisdiction is not supported because a country code, URL or sample schema exists. Qualification is a bounded evidence claim for an exact combination of entity, territory, period, obligation/profile, source revision, validator, locale and reviewer approvals.

This dossier records source and interpretation lineage. It does not itself contain executable law. Candidate interpretations remain inert until converted into a constrained typed rule artifact, independently reviewed, tested and admitted by the separate bitemporal policy engine (#5792).

## 2. Independent scope axes

A pack can include country and subnational levels: state/province, region, municipality, territory, special economic zone or another legally relevant unit. Codes are tuples of code system, value and parent scope; display name is never the identity key.

Keep separate:
- issuing/legal jurisdiction;
- entity domicile, registration and establishment;
- transaction roles and place-of-supply facts;
- accounting-framework adoption and entity eligibility;
- tax type, tax authority and registration;
- statutory reporting/filing profile;
- reporting taxonomy;
- e-invoice scheme, syntax, network and country-qualified validator;
- locale, language/script, calendar/timezone and source translation.

A pack may cover only a named portion of these dimensions. No parent-country, neighbor-country, currency, language or vendor default is inherited without an explicit authority-backed applicability record.

## 3. Source record and archive contract

For every document, retain:
- source role and issuer/publisher metadata;
- canonical URI and document identity;
- exact media type and language;
- original bytes, SHA-256, byte length, content type, ETag/Last-Modified and acquisition timestamp when archived;
- publication, promulgation, effective and expiry/supersession times separately;
- capture time and knowledge frontier separately;
- source passages/locators for every derived interpretation;
- supersession and conflict relationships.

A URL is not a source snapshot. A retrieved search snippet is not an archived document. A translated copy is not automatically authoritative. If bytes have not actually been acquired, the record stays not_archived with null digest/length. If unavailable or restricted, state that; do not synthesize a digest.

Authority metadata is descriptive and source-specific. This schema does not impose a universal legal hierarchy across all countries; local reviewers must identify which issuer/document controls a particular obligation.

## 4. Interpretations and reviewer decisions

Interpretations are candidate statements linked to exact source records and passage locators. They include purpose, applicability context, disagreements, reviewer role/competence, conflict disclosure and decision time.

The system must preserve dissent and conflicted state. Model-generated extraction/translation can propose a draft, but cannot approve legal interpretation or become an executable rule by itself. Each approval is bounded by:
- exact source bytes/digests;
- exact profile and jurisdiction scope;
- reporting/event period and knowledge frontier;
- reviewer competence and declared conflict state;
- dependencies and tests;
- stated limitations.

## 5. Qualification lifecycle

- candidate: initial metadata only;
- research_only: references identified but no approved interpretation;
- review_required: source snapshot, applicability, or interpretation still needs qualified review;
- qualified_for_declared_scope: exact source snapshots, applicability facts, typed rules, required local reviews and test evidence are complete for a named scope/time;
- suspended: a material amendment, conflict, revocation, security incident or missed change watch invalidates the prior operational claim;
- retired: superseded; retained for historical replay.

Synthetic packs can never enter qualified_for_declared_scope. They may exercise schema, resolver and mutation behavior only.

## 6. Material change and requalification

Create a new immutable revision for:
- source content/digest change;
- amendment, promulgation, repeal or effective-date change;
- territorial/entity applicability change;
- tax authority or reporting schema changes;
- validator/codelist/version updates;
- language/translation correction;
- parent/dependency profile revision;
- reviewer decision change or newly discovered conflict.

Do not edit a prior snapshot in place. Preserve old decisions at their original known_at frontier. Suspend only the affected profile intersection and dependent conclusions; do not silently overwrite history or broaden a profile to a neighboring territory.

## 7. Benchmark and controls

Hard gates:
- zero qualified source records without archived bytes and verified digest;
- zero profile uses outside explicit territory/entity/transaction/period scope;
- zero stale source use after a material amendment is known;
- zero silent rewriting of historical decisions;
- zero machine translation or model summary promoted to controlling authority without required review;
- zero candidate rule execution before typed compilation and approval;
- zero synthetic fixtures represented as real jurisdictional coverage.

Quality measures:
- source identity and snapshot integrity;
- source revision/effective/knowledge-time selection;
- scope/applicability precision and recall by territory and entity type;
- stale-source and supersession detection;
- review agreement, disagreement preservation and reviewer minutes;
- source-field/translation preservation;
- profile-intersection selection and correct abstention;
- fully loaded pack maintenance cost and change-detection latency.

Compare a manual checklist, model-assisted extraction with qualified human review, and dossier plus deterministic validator using the same sources and scope. Do not use an LLM judge as the primary judge of law or source authenticity.

## 8. Current synthetic fixture

The fixture at benchmarks/jurisdiction-pack-v1/fixtures/synthetic_jurisdiction_pack_v1.0.0.json deliberately contains:
- synthetic country and province scopes;
- a tax-profile candidate and a separate e-invoice-profile candidate;
- three metadata-only source records with not_archived state and null byte digests;
- candidate interpretations with execution_enabled false;
- unrun scope and archive tests;
- explicit change-watch/requalification triggers.

This fixture is deliberately not a real-world profile and carries no legal rule. It demonstrates that absent source evidence stays absent rather than being fabricated.

## 9. Integration boundaries

- #5811 defines global profile axes and profile-intersection catalogue.
- #5826/#5829 selects exact approved intersections; it does not interpret law.
- #5792 compiles/evaluates approved bitemporal typed rules.
- #5785/#5792 own South African tax source and policy work.
- #5793 validates e-invoices; #5802 compiles XBRL; #5812 ingests CAMT bank evidence.
- #5799 owns isolated evaluation/run manifests.

No duplicate currency registry, ledger, rule engine, invoice validator, taxonomy compiler or run receipt is created.

## 10. Claim ceiling

Schema hardening: a qualified pack is invalid unless every source record is archived with a non-null SHA-256, positive byte length, acquisition time, content type and archive reference; all profile candidates are qualified; at least one interpretation is approved for declared scope; at least one test is run_qualified and reviewed; and at least one pack-level approval is recorded. Synthetic packs are schema-prohibited from using the qualified maturity state.

Passing synthetic dossier tests establishes only source metadata, schema and lifecycle behavior. It does not prove a legal proposition, tax treatment, accounting-framework adoption, regulator filing acceptance, e-invoice compliance, audit readiness or correctness in any real country. Real packs require primary-source byte archives, local professional review and exact-scope test evidence.