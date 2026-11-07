# Preferential-flow evidence triangulation audit v0.1

This Rust CLI records concordance/disagreement between sensor-level velocity-threshold and trajectory-threshold proxies, preserves the profile-level NSR signature, and separately records independent tracer/geophysical/water-table/lysimeter references. It deliberately does not calculate a majority vote or a weighted consensus across proxy methods.

## Run

    cargo test --manifest-path research/terra-preta/tools/pf-triangulation-audit/Cargo.toml
    cargo run --manifest-path research/terra-preta/tools/pf-triangulation-audit/Cargo.toml -- \
      research/terra-preta/tools/pf-triangulation-audit/fixtures/synthetic-method-evidence.json \
      /tmp/terra-preta-triangulation-report.json

The report binds to the SHA-256 of exact input bytes and retains the source digest plus the output digests/evidence references for the upstream method reports.

## Evidence levels and aggregation boundary

- Velocity threshold and trajectory threshold are sensor-level proxy methods. Their agreement/disagreement is recorded per sensor at matched depths.
- NSR is profile-level. Its signature is carried through unchanged rather than forced into a sensor-level voting scheme.
- Independent references can be conservative tracer, stable-isotope tracer, dye tracer, ERT, GPR, water table, lysimeter boundary flux or another declared method. Each record carries its own outcome, evidence reference and detection limit/spatial-temporal resolution.
- Independent-reference outcomes and review metadata are recorded as claims. This v0.1 tool does not verify reviewer identity, blinding, signatures or a Mycelix attestation, so **no reference is eligible to adjudicate a physical claim**. Physical claim status remains not adjudicated until a trusted external verifier supplies a cryptographically verified attestation.

This tool records method artifact SHA-256 strings as supplied; it does not open or independently verify the referenced reports. It also does not authenticate reviewer identity or blinding claims. A nonblank reviewer reference and a `claims_independent_review_blinded` value are not proof of independent review. All reference outcomes and quality statuses remain unverified claims in this version. A future verifier must validate a signed receipt against a separately trusted issuer-key registry before any reference can become eligible.

## Interpretation

Sensor-level proxy outcomes are counted as positive agreement, negative agreement, disagreement, one method only, neither method evaluable, or indeterminate. No consensus classification is produced. The report never calls a proxy-majority result a confirmed preferential pathway.

The synthetic fixture includes intentional VT/trajectory disagreement and one self-declared tracer-positive reference claim. It is intentionally **not eligible** for physical adjudication. All values are software-test data only, not measured soil observations.

Implementation tracker: [PF-EVIDENCE-001](https://github.com/Luminous-Dynamics/mycelix/issues/5543). Independent references are required to match the input event/profile IDs; a record from a different event or profile fails closed.

## Review-attestation boundary

A future attestation verifier must accept a canonical review payload containing the event/profile/source digest, method and reference outcome; verify a detached signature against an issuer key that is trusted independently of the payload; enforce issuer role, key validity/revocation, blinding protocol and scope; and emit a verifier receipt binding the reviewed artifact digest and trust-policy version. Merely embedding a public key or a boolean `verified=true` in the input is not sufficient. Issue #5538 tracks this follow-on.
