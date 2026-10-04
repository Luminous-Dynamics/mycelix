# Mycelix Security-Domain Enclave Profile v0.1

**Status:** Reference architecture only  
**Claim ceiling:** `ReferenceModelOnly`

This profile defines the security semantics that a future Mycelix enclave implementation must satisfy before any government-facing assurance claim.

It deliberately does **not** certify or authorize classified processing, CUI processing, export-controlled technology handling, SIPRNet/NIPRNet connectivity, CMMC, NISPOM, RMF, CDS, or FIPS validation.

## Design law

The protected authorization decision is multidimensional:

```
subject identity
AND device posture
AND workload identity
AND security domain
AND policy version
AND purpose
AND releasability
AND export-control policy
AND freshness
AND delegation
```

Missing or stale required evidence produces **DENY or explicit INDETERMINATE** according to the profile. It never becomes implicit authorization.

## Security domains

The reference profile separates:

- **E0** — public/development
- **E1** — CUI/export-controlled
- **E2** — classified Secret reference architecture
- **E3** — higher-assurance/compartmented reference architecture

These are separate security domains, not labels on one shared network.

A protected implementation must not use a shared DHT, shared trust root, or shared authorization cache across independent domains.

## Non-equivalences

The following are architectural laws:

```
DID control                 != host integrity
identity                    != authorization
capability                  != clearance
reputation                  != authorization
network membership          != resource authorization
attestation                 != permanent authorization
qualification evidence      != runtime authority
classification label        != enforcement
```

The identity layer therefore consumes external posture/attestation evidence rather than manufacturing security claims that belong to hardware, facilities, personnel systems, or independent assessors.

## Hardware/workload admission

The target chain is:

```
hardware root
 -> secure/measured boot
 -> signed OS/image
 -> measured workload
 -> workload identity
 -> attestation evidence
 -> enclave policy
 -> resource authorization
```

Attestations are bound to freshness, nonce/audience, workload measurements, verifier profile, and trust anchors.

A valid attestation from another security domain is not local authorization.

## Controlled release

Cross-domain movement is a separate state transition:

```
source object
 -> source authorization
 -> release decision
 -> approved transformation/filter
 -> destination admission
 -> transfer receipt
```

The Mycelix application layer owns the **semantic release/admission contract and evidence**.

It does **not** claim to be a Cross Domain Solution.

Any actual classified cross-domain transfer must use an independently governed and qualified CDS or equivalent boundary.

## Cryptography

The profile requires:

- algorithm agility;
- hybrid classical/PQ migration;
- separate signing, encryption, and attestation keys;
- hardware-backed key custody where required;
- explicit trust-anchor/key rotation;
- secure zeroization;
- deterministic qualification vectors.

Federal cryptographic validation remains a separate boundary. Implementing ML-KEM, ML-DSA, or another approved algorithm does not itself establish FIPS 140-3 validation.

## Supply chain

Protected builds target:

- isolated or tightly allow-listed dependency acquisition;
- content-pinned dependencies;
- signed artifacts;
- SBOM/provenance;
- independent rebuildability;
- dual control for high-impact release roots;
- qualification infrastructure that is not controlled by candidate code.

Public GitHub Actions may provide development/qualification evidence but must not be treated as the classified production build environment.

## Evidence

Security evidence is separated into:

```
security events
access decisions
policy decisions
device/workload posture
credential/key lifecycle
data movement/release
administrative actions
configuration changes
artifact provenance
incident response
```

Receipts should bind the exact enclave/profile, actor/workload, policy version, relevant object digest, authorization basis, trusted time source, outcome, predecessor state, and software/build identity.

Evidence is never itself runtime authority.

## E1 first proving ground

The first practical implementation should target **CUI/export-controlled workloads**, with a machine-readable control/evidence matrix against:

- NIST SP 800-171 Rev. 3;
- NIST SP 800-171A Rev. 3;
- NIST SP 800-172 Rev. 3;
- NIST SP 800-172A Rev. 3;
- applicable CMMC contractual requirements.

This remains an engineering mapping, not a compliance certification.

## Comparative-security methodology

Do not compare Mycelix to SIPRNet/NIPRNet by product name alone.

A future comparative claim must bind:

```
threat model
+ exact deployment profile
+ adversary capabilities
+ security boundary
+ artifact identity
+ measurement methodology
+ independent evidence
```

Preferred measurements include:

- compromise blast radius;
- lateral-movement paths;
- standing privilege;
- revocation latency;
- stale-policy acceptance;
- cross-domain exposure;
- provenance completeness;
- recovery correctness;
- supply-chain integrity;
- audit completeness.

Never collapse these into a single security score.

## Reference basis

- NIST SP 800-171 Rev. 3: https://csrc.nist.gov/pubs/sp/800/171/r3/final
- NIST SP 800-171A Rev. 3: https://csrc.nist.gov/pubs/sp/800/171/a/r3/final
- NIST SP 800-172 Rev. 3: https://csrc.nist.gov/pubs/sp/800/172/r3/final
- NIST SP 800-172A Rev. 3: https://csrc.nist.gov/pubs/sp/800/172/a/r3/final
- NIST SP 800-207: https://csrc.nist.gov/pubs/sp/800/207/final
- FIPS 140-3: https://csrc.nist.gov/pubs/fips/140-3/final
- FIPS 203 / ML-KEM: https://csrc.nist.gov/pubs/fips/203/final
- FIPS 204 / ML-DSA: https://csrc.nist.gov/pubs/fips/204/final
- NSA Zero Trust Implementation Guidelines: https://www.nsa.gov/Press-Room/Press-Releases-Statements/Press-Release-View/Article/4496862/
- DFARS 252.204-7021 / CMMC: https://www.acquisition.gov/dfars/part-252-solicitation-provisions-and-contract-clauses
- DFARS 252.204-7012: https://www.acquisition.gov/dfars/252.204-7012-safeguarding-covered-defense-information-and-cyber-incident-reporting.

## Qualification ceiling

A future green qualification can establish only the exact semantics and implementation evidence of the selected profile.

It cannot by itself establish:

- classified authorization;
- facility/personnel clearance;
- NISPOM/RMF approval;
- CMMC certification/status;
- CDS certification;
- NSA approval;
- FIPS 140-3 validation;
- BIS/EAR legal compliance;
- superiority over SIPRNet/NIPRNet.
