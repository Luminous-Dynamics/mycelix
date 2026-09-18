# MYC-CAP-002D3A — Custody event-chain tip currentness v1

Status: executable candidate over hosted-qualified D3; local preflight only until exact-head hosted qualification succeeds.

## Purpose

Separate custody **state at a supplied D3 event-chain tip** from proof that this exact tip is the currently designated custody snapshot.

```text
valid D3 state at tip != latest designated tip
latest designated tip != healthy custody
CURRENT tip currentness != D3 custody_state CURRENT
```

D3A never reinterprets D3's operational state. It proves only currentness of one exact qualified D3 snapshot.

## Exact qualified input

D3 subject:

`1fefe7a313d1cc83f3559d1d9f3223e12b2df019`

D3 receipt semantic SHA-256:

`26f9f23e24d6a120c3cf210e920e6111fdefbd1355fb0069052bdfb5a68f01e1`

D3 profile semantic SHA-256:

`99478af7e37e6d878c42051bb88657feaf083fe9d55afa6726aa8fa78e8fca3a`

D3 event-history SHA-256:

`5ec741fe98ae16872cd1a6a954b22fa7ea679ff48075cca96a622ed8ee864807`

D3 event-chain tip SHA-256:

`82b13fad92801eee7038b237b0a5e47aebb198ef4ec0e63d3f4ed139dfe28744`

The qualification workflow proves that the D3 receipt embedded in this fixture is identical to the canonical receipt inherited from the exact D3 Git parent.

## Frozen profile

Profile semantic SHA-256:

`81f0139157d99d74fb2cef0cf320bcfd9fc020faf2970776ed69361cc492ada4`

Designation semantic SHA-256:

`2801f37d665a6ae6a989bfb29d03c6a4539bddc073e0f756fba6a592633594dc`

Registry:

`registry:custody-tip-currentness`

The canonical designation is ACTIVE at registry epoch 1.

## State machine

Currentness states:

```text
CURRENT
STALE
PENDING
REVOKED
```

Precedence:

`REVOKED > PENDING > STALE > CURRENT`.

Closed event vocabulary:

```text
SupersedeTip
PendingReconciliation
MaterialInvalidation
RevokeEvidence
```

No local wall-clock time creates authority.

## Orthogonality

D3A echoes D3's `custody_state` but does not interpret it as health.

A future D3 snapshot with a different custody state requires its own qualified D3 subject/receipt. This v1 theorem is intentionally fixed to the exact qualified canonical D3 subject above.

H4 later composes:

```text
D3 operational state
+
D3A exact-tip currentness
```

as separate dimensions.

## Canonical commitments

Case file SHA-256: `4193803616f5e10b3a1842f435854f06b3b9f63388f451d757eb86766ba95a4d`

Receipt file SHA-256: `fd367546317b03d8ba9458b535fc112b3e59f509dca1c8d6d45acf4db505cbca`

Receipt semantic SHA-256: `173eb570d4c5cb7ad9a768cc3af76a95dcdaed7a5bcf31695ec5b644eaadee10`

Canonical empty event-history SHA-256: `4f53cda18c2baa0c0354bb5f9a3ecbe5ed12ab4d8e11ba873c2f11161202b945`

## Local preflight

The stdlib suite passes **31/31** locally, covering exact qualified-source binding, stale subject/receipt/profile/history/tip designation, pending/revoked precedence, invalidation events, event-chain integrity, substitution attacks, wall-clock/currentness contamination, legal-authority contamination, closed source schema, and deterministic replay.

Local PASS is not hosted qualification.

## Authority boundary

Every receipt fixes:

```text
custody_health_established = false
legal_title_transition_established = false
constitutional_stewardship_transition_established = false
execution_authority_established = false
uses_local_wall_clock = false
```

## Nonclaims

Even hosted PASS would establish only that the exact qualified D3 receipt/event-chain tip is the currently designated custody snapshot under this frozen registry lineage. It would not establish custody health, legal title, constitutional stewardship legitimacy, service quality, handback readiness, cybersecurity certification, democratic legitimacy, external authority authenticity, or execution authority.
