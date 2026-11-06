# South African Fraud-Signal Exchange Profile v1

**Status:** synthetic reference contract only. No live data exchange, legal approval, SARB endorsement or automated adverse decision is enabled.

## Strategic context

SARB's PEM programme describes an emerging fraud framework, insights capability and a Fraud & Intelligence Information Sharing Forum. FATF's 2026 global report identifies public-private information-sharing partnerships and emphasises legal basis, governance and data-protection safeguards. This is context for interoperability, not a mandate for Mycelix to build a competing central database.

## Architectural thesis

Exchange bounded evidence, not a person's global risk score.

A sender signature establishes only the sender and integrity claim under the relevant cryptographic profile. It does not prove allegation truth, legal basis, recipient authority, identity, guilt or whether an action is lawful.

Two distinct planes are defined:
1. **Strategic aggregates:** typologies, trends and attack patterns with denominators, suppression and re-identification review. No person-linked subject reference.
2. **Operational case signals:** one case and purpose, one authorised audience scope, pairwise/case-scoped opaque reference, source commitment, epistemic state, expiry, correction and recipient-use receipt.

A token or hash is not automatically anonymous. No raw identifiers or transaction histories belong in the wire projection.

## Legal and governance boundary

FSR Act section 251 is scoped to information handling and sharing by the SARB/financial-sector regulators in their statutory functions and to associated safeguarded designated-authority exchanges. It is not a general private-sector safe harbour. POPIA lawful-purpose, purpose-specification and retention requirements must be assessed for the exact parties, purpose, recipient, transfer and data class. The synthetic field review_state=verified means an external authority/counsel process supplied a verification reference; the Rust evaluator does not determine legal sufficiency.

A deployment must separately establish responsible-party/operator roles, lawful basis, notices, subject rights, recipient membership, onward-disclosure rules, retention/deletion, cross-border safeguards, security and independent oversight.

## Signal semantics

Signature valid is not the same as assertion true, allegation adjudicated or recipient use permitted.

The allowed v1 recipient actions are limited to triage, request more evidence, human review and aggregate analysis. A signal cannot itself decline a payment, close an account, score creditworthiness, deny an essential service or establish legal guilt.

Disputed/corrected/withdrawn/expired signals remain immutable historical evidence but are quarantined from fresh operational use. Corrections and withdrawals require per-recipient acknowledgement or a propagation-incomplete quarantine. Conflicting assertions remain separate; do not average them into one trust score.

## Qualification

The 32-case synthetic corpus covers aggregation suppression, legal-authority scope, sender role, source evidence, token scope, prohibited use, retention, expiry, disputes and correction propagation. The independent Rust evaluator checks schema and seeded semantic invariants. It does not verify the underlying source evidence, cryptographic signature, legal basis, fraud allegation or recipient's real-world governance.

## Claim ceiling

A passing run establishes only that this frozen reference contract rejects the modeled data-governance and signal-lifecycle errors. It does not establish POPIA compliance, a private-sector safe harbour, anonymisation, fraud-detection accuracy, SARB endorsement, production integration or customer protection.