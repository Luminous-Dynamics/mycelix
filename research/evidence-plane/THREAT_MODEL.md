# Evidence Plane threat model v0

Assets: source bytes/metadata, stewardship decisions, normalized data, transform code/configuration, hidden holdouts, run outputs, signing keys and restricted artifacts. Goals: integrity, provenance, reproducibility, fair comparisons, access control and claims bounded by evidence. Out of scope: proving real-world truth of sources or making DHT consensus equal scientific truth.

| Threat | Defense | Negative control |
|---|---|---|
| Synthetic promotion | S0 conditional and explicit claim ceiling | Raise tier/ceiling while status remains planned |
| HTML mistaken for source data | MIME sniffing, bytes and file-level provenance | Replace expected file with HTML |
| Metadata conflict erased | Keep each assertion, source and date | Delete one side of a known conflict |
| Stewardship bypass | Separate rights and contact states | Claim contact when only acknowledgment exists |
| Byte/result tampering | Recompute hashes and verify lineage | Alter result but retain prior digest |
| Parser/transform drift | Pin code/config/dependencies/units/CRS | Change only parser or unit label |
| Split leakage | Locked group/time holdouts | Add duplicate/correlated holdout row |
| Benchmark favoritism | Per-arm budget/information receipt | Give candidate extra compute/data |
| Evaluator capture | Independent replay and immutable logs | Feed evaluator a known-wrong baseline |
| Aggregate hides service failure | Hard gates and tail reporting | Inject critical-service shortfall |
| Spatial overreach | Carry support geometry, CRS and aggregation | Put region aggregate on station point |
| Reanalysis called measurement | Type measured/reported/reanalysis/simulated/synthetic | Relabel a grid estimate as site measurement |
| Supply-chain mismatch | Pin build/toolchain and commit provenance | Swap commit/artifact digest |
| Privacy leak | Secret scan, disclosure review and access tiers | Seed fake token and assert publication block |
| Store loss/corruption | Atomic writes, independent backup and digest recheck | Truncate object or remove parent |
| Hash mistaken for trust | Separate integrity, truth, provenance and rights | Integrity check passes on an untrusted source |

Trust boundaries: source web to acquisition (untrusted bytes, redirect/host policy, size limits and safe extraction); raw cache to normalized data (verify input hash, preserve raw object); candidate to evaluator (evaluator controls test data); local receipt to public view (privacy/rights/secret review); signer/ledger (key rotation, revocation and recovery).

If source digest, exact commit, stewardship, lineage, split, budget parity, independent check or safety gate is missing/unknown/failing, block stronger claims. Preserve failed receipt/reason and supersede rather than erase. A green mutation suite only proves its declared controls were caught.
