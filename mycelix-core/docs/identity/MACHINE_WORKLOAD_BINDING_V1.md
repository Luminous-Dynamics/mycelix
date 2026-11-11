# Machine–Workload Binding Contract v1

**Status:** proposed canonical Mycelix identity contract for NixOS, Nixward, Sovereign Boot and platform consumers.  
**Owner:** Mycelix owns the durable subject, controller, key-epoch, enrollment, revocation and re-enrollment lifecycle.  
**Non-goal:** this binding is not a Nixward action permit, a deployment authorization, or a boot promotion decision.

## 1. Why this is a separate identity layer

A machine subject must survive ordinary NixOS rebuilds. Its key may rotate, its installed instance may be re-imaged, its current generation may change, and its workload credentials may be renewed or revoked. These are separate lifecycles.

The contract distinguishes:
- durable machine subject and its enrollment method/trust domain;
- controller relationship, explicit scope, validity and revocation evidence;
- public-key epoch and key lifecycle;
- installed physical/virtual instance;
- exact NixOS software closure, configuration revision and boot session;
- short-lived workload identity;
- independently verified hardware attestation;
- a fresh, audience-bound signed binding statement.

No single identifier or signature collapses those subjects into one trust score.

## 2. Identity and authority are distinct

**A valid binding signature proves only that the stated key signed these canonical binding bytes.** The verifier must additionally resolve the signer key, validate its relationship to the controller/enrollment authority receipt, enforce key epoch and revocation state, evaluate trust policy, validate freshness and audience, and independently validate hardware evidence if the status claims it.

The binding does not authorize a NixOS activation, service mutation, generation promotion, payment or other consequential operation. Nixward still requires a separately verified exact ChangePlan permit bound to the target, operation, plan digest, policy epoch, audience, expiry and replay state. A workload SVID is an authenticated process identity, not owner authorization.

## 3. Canonical signature profile

For v1, the binding payload is the complete JSON object with `binding_signature` omitted. Serialize it with RFC 8785 JSON Canonicalization Scheme (JCS), then compute SHA-256 over those bytes. The detached Ed25519 signature signs:

`UTF8("org.mycelix.machine-binding.v1") || 0x00 || SHA256(JCS(payload_without_binding_signature))`

The signature object declares the context, encoding, signer key ID/role, algorithm, payload digest, base64 signature and signing time. Verifiers must:
1. reject duplicate JSON object keys before materializing a JSON value;
2. validate the schema and canonical encoding profile;
3. recompute and compare the payload digest;
4. resolve the signer key from an independently trusted enrollment/controller chain;
5. verify Ed25519 over the domain-separated digest;
6. validate key epoch, controller scope/validity/revocation, binding freshness, nonce replay state and audience;
7. validate workload and hardware-attestation evidence independently.

A syntactically valid signature is not an attestation. A signed self-report of TPM or Secure Boot state does not become verified hardware evidence.

## 4. Lifecycle rules

- **Rebuild:** subject ID and controller relationship remain stable; closure digest and generation change.
- **Key rotation:** increment key epoch with an explicit continuity receipt; old keys remain historically attributable and revoked keys fail new verification.
- **Re-image/re-enrollment:** create a new instance ID or an explicit successor link; do not silently clone controller or key state.
- **Workload issue/renew/revoke:** use short-lived SPIFFE/SPIRE SVIDs where the deployment has a trust domain and workload API; require audience, expiry and revocation/freshness checks.
- **Decommission:** revoke controller/key/instance bindings under a durable lifecycle receipt; do not recycle subject IDs.
- **Offline boot:** locally cached, bounded trust material may be checked, but Mycelix/DHT/network availability must never be a normal boot prerequisite. Missing or stale remote evidence cannot authorize LKG promotion.

## 5. Privacy and anti-cloning

Do not publish serial numbers, raw DMI/PCI identifiers, private keys, recovery secrets, passwords, user email/phone, public IPs or raw device measurements in this binding. Use opaque references to separately controlled evidence artifacts.

A disk image or VM clone may copy a software-held key. The schema does not claim physical uniqueness. A verifier may report a clone suspicion only when supported by independent evidence; otherwise the correct result is unresolved/unknown. Hardware uniqueness requires an actual independent hardware-backed enrollment/attestation system and explicit threat model.

## 6. Consumer mappings

**NixOS:** emits read-only observations for flake lock digest, configuration revision, current system closure, booted generation and boot ID. It must not mint a controller relationship or hardware-attestation claim itself.

**Nixward:** references machine subject, key epoch and closure/boot identity inside its target/evidence envelope. The exact plan authorization remains a distinct authority object. Machine identity alone never releases a payload or admits a mutation.

**Sovereign Boot:** binds the boot target, boot ID, actually booted generation, local trust-bundle version and health receipt. Its local boot-safe path must remain available when Mycelix is offline.

**SPIFFE/SPIRE:** use the standard X.509-SVID or JWT-SVID for short-lived process identity when available. Avoid a proprietary workload-credential system unless a concrete interoperability gap is documented.

## 7. Benchmark corpus and hard gates

Freeze positive and negative fixture sets, schema version, trust-policy digest, verifier build and raw results.

- **MACHINE-STABILITY:** rebuild, kernel update, rollback and generation switch preserve subject ID while instance/closure/boot fields update appropriately.
- **KEY-ROTATION:** accept current key after continuity proof; reject unknown, stale, revoked, compromised and wrong-epoch keys.
- **CONTROLLER-SCOPE:** reject expired/revoked controller, wrong relationship, out-of-scope controller action and missing authority receipt.
- **WORKLOAD-BOUND:** reject wrong trust domain, wrong audience, expired/revoked SVID, stale trust bundle and replay.
- **INSTANCE-CLONE:** identify independently evidenced clone conflicts; classify software-only clone ambiguity as unknown.
- **PLAN-BIND:** reject substituted machine, target closure, plan digest, policy epoch, audience or permit even when the binding signature is valid.
- **BOOT-OFFLINE:** Mycelix unavailable does not block local boot; stale identity cannot authorize LKG promotion.
- **ATTESTATION-TRUTH:** reject verified status without an independent verifier, measurement digest and evidence reference; signed self-report remains unverified.
- **PRIVACY:** no secrets/PII/raw hardware IDs; quantify cross-context linkability.

Hard gates: zero acceptance of frozen unauthorized/cross-target/replay negative controls; zero false hardware-attestation claims; zero workload credentials accepted outside their trust domain or audience; zero boot promotion based on unavailable evidence. Report unknown/unresolved separately from detected conflict and valid identity.

Performance metrics: enrollment and rotation latency; signature and policy verification p50/p95/p99; trust-bundle fetch/cache latency; revocation propagation; SVID issue/renew/revoke time; offline boot availability; CPU/RSS/bytes; evidence-chain completeness. Set performance thresholds only after a reproducible baseline on pinned hardware and trust policy.

## 8. Claim ceiling

The JSON Schema validates structure, not cryptographic truth, controller authority, anti-cloning, hardware uniqueness, or real SPIFFE issuance. The example is a shape fixture with illustrative signature bytes and does not verify against a production trust root. Production enrollment and verifier implementations require separate qualification.
