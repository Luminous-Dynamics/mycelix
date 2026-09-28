# Integral Economic OS Adversarial Corpus v1

Corpus ID: INTEGRAL-ECO-OS-CONF-001

Purpose: ensure the Economic Fabric cannot accidentally flatten Integral's five-system semantics into generic transactions, currency, or authority.

## Negative cases

| ID | Mutation | Required result |
|---|---|---|
| IECO-N-001 | CDS recommendation treated as decision | Rejected |
| IECO-N-002 | CDS decision treated as authorization | Rejected |
| IECO-N-003 | OAD certification treated as production | Rejected |
| IECO-N-004 | certified design treated as consumed material | Rejected |
| IECO-N-005 | COS planned labor treated as observed labor | Rejected |
| IECO-N-006 | COS labor observation directly mints ITC | Rejected |
| IECO-N-007 | ITC treated as generic Mycelix currency | Rejected |
| IECO-N-008 | FRS recommendation directly authorizes settlement | Rejected |
| IECO-N-009 | FRS prediction written as observation | Rejected |
| IECO-N-010 | stale COS evidence satisfies current ITC requirement | Rejected |
| IECO-N-011 | foreign ITC recognition rewrites source origin | Rejected |
| IECO-N-012 | economic transport acknowledgment treated as settlement | Rejected |
| IECO-N-013 | duplicate settlement accepted as a second settlement | DuplicateIdempotent |
| IECO-N-014 | correction mutates historical event | Rejected |
| IECO-N-015 | Valueflows Intent promoted to EconomicEvent | Rejected |
| IECO-N-016 | Valueflows Commitment promoted to Settlement | Rejected |
| IECO-N-017 | accounting projection treated as physical evidence | Rejected |
| IECO-N-018 | one productive success treated as general capability | Rejected |
| IECO-N-019 | useful output treated as safety qualification | Rejected |
| IECO-N-020 | external dependency treated as node failure | Rejected |

## Positive controls

- verified COS observation may become an economic event while retaining source provenance;
- an explicit ITC policy may derive an entitlement from eligible evidence;
- an explicit authority reference may authorize a settlement;
- a valid settlement may be retried idempotently;
- a correction creates new lineage rather than mutating history;
- a foreign instrument may be recognized without local issuance;
- a Valueflows EconomicEvent may be represented without forcing it into Mycelix's source ontology;
- an FRS recommendation may be accepted, rejected, modified, or escalated by an authorized decision process.

## Claim ceiling

Passing this corpus demonstrates semantic boundary conformance only. It does not prove Integral policy correctness, economic fairness, accounting compliance, physical production, legal validity, or settlement finality.