# Holochain Countersigning Feature Build Experiment v0.1

**Purpose:** prove that the exact locked Holochain/HDK/HDI source combination can compile the optional countersigning API when explicitly enabled.  
**Status:** opt-in compile/build experiment only. It does not enable countersigning in the default Finance environment or DNA.

## 1. Why this is separate

The locked default Finance build does not request countersigning:
- pinned Holonix builds the default conductor package with an empty `cargoExtraArgs`;
- the default conductor feature set does not include `unstable-countersigning` or `unstable-functions`;
- the Finance Payments/TEND workspace dependencies use plain HDK 0.6.1 / HDI 0.7.1 without the required unstable features.

Holochain 0.6.1 source and HDK 0.6.1 expose explicit feature flags:
- conductor: `unstable-countersigning`, `unstable-functions`;
- HDK: `unstable-countersigning`, `unstable-functions`;
- HDI: `unstable-functions`.

The opt-in Holonix override follows its documented custom-build pattern:
```nix
holochainPackages.holochain.override {
  cargoExtraArgs = "--features unstable-countersigning,unstable-functions";
}
```

## 2. What this PR adds

- `packages.holochain-countersigning-experimental` in the Finance flake, available only when explicitly built.
- `tools/countersigning-feature-probe`, a separate workspace crate with the required HDK/HDI feature flags.
- One zome extern that calls the feature-gated `accept_countersigning_preflight_request` host API.
- An exact-head Nix workflow that builds the custom conductor and compiles the probe to `wasm32-unknown-unknown`.
- Canonical and workspace Cargo manifests, lockfile and probe source parity.

The default `nix develop .#ci`, `nix develop .#test`, default development shell and `packages.zomes` remain on the unmodified default Holonix conductor and do not include the probe in the Finance DNA.

## 3. Qualification boundaries

A successful run proves:
1. the locked Holonix source can build a custom Holochain binary with the two requested conductor features;
2. the locked HDK 0.6.1 / HDI 0.7.1 graph can compile the countersigning preflight API in a WASM zome;
3. the exact conductor version/build-info and binary digest, plus probe WASM digest, are reproducible for that CI subject.

It does **not** prove:
- two or more agents can complete a session;
- mandatory signer semantics or optional-witness strict-majority semantics;
- enzyme identity and ordering;
- session timeout, cancellation, offline participants, or source-chain lock release;
- retry, replay, duplicate session or crash recovery;
- witness equivocation/refusal or a global double-spend theorem;
- integration into the Finance DNA or production spendability.

These remain explicit future multi-agent qualification work under FIN-SAFE-021 / #6576 and the witness policy roadmap.

## 4. Safety posture

No production dependency is changed to request unstable features. No Finance DNA is rebuilt by default with countersigning. The custom conductor is a separate opt-in Nix output and the compile probe is not a zome in the packaged Finance DNA.

A compile-only success is not a production recommendation. Countersigning remains an unstable Holochain feature with documented operational hazards; use only in a disposable test environment until session-level qualification and failure recovery are complete.
