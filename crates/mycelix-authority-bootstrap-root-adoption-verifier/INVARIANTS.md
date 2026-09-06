# Bootstrap Root Adoption Verifier v0.1 — Normative Invariants

Status: **pure qualification candidate; no runtime verifier provisioned**

## 1. Proof verification is not adoption semantics

The external proof-verifier boundary may return only `VerifiedBootstrapRootAdoptionProof`.

It MUST NOT directly return #111's `VerifiedBootstrapRootAdoption` or a qualified bootstrap root.

The proof receipt says only:

> this verifier authenticated this exact adoption claim/proof instance.

The pure kernel remains responsible for deciding what exact claim those proof bytes are allowed to mean.

## 2. The adoption claim is reconstructed locally

`BootstrapRootAdoptionClaim` is built from:

- the exact current `ConstitutionStatement`; and
- the exact candidate `AuthorityStateBootstrapRootManifest`.

The runtime must not accept a caller/provider-selected adoption claim as semantic authority.

The claim commits the exact:

- current constitutional statement digest/profile;
- rulebook ID/version/digest/profile;
- root-manifest digest/profile;
- adoption authority;
- adoption reference; and
- adoption proof reference.

Because root-manifest identity commits root epoch, control-plane namespace, exact root source identity and exact root coverage/context policy semantics, changing any of those facts changes the adoption claim.

## 3. Current constitution and rulebook are exact

Before building a claim, the kernel recomputes the current constitutional statement digest and requires the root manifest to exactly match:

- network;
- institution;
- constitution ID/version;
- statement digest/profile;
- rulebook ID/version/digest/profile.

A proof for a historically valid old rulebook/root cannot be rebound to a later constitutional epoch.

## 4. Proof binding is exact

A positive result requires the proof receipt to bind the exact locally computed adoption-claim digest/profile and exact:

- adoption authority;
- adoption reference; and
- adoption proof reference.

A cryptographically valid proof over another root, rulebook, constitution or authority denies.

## 5. Positive adoption is constructed only by the kernel

Only `qualify_bootstrap_root_adoption` may project `VerifiedBootstrapRootAdoption` for #111.

`QualifiedBootstrapRootAdoption` derives `Serialize` but not `Deserialize`.

The projected #111 receipt is revalidated before it is returned.

## 6. Stable semantics and dynamic proof evidence are separate

The stable qualification digest commits the exact semantic adoption claim.

A separate evidence digest commits the stable qualification plus proof-verifier identity/reference and verification window.

Re-verifying the same exact adoption may refresh evidence without minting a different semantic root-adoption identity.

Changing constitution, rulebook, root manifest, authority, adoption reference or proof reference changes semantic identity.

## 7. Validity can only narrow

Projected adoption validity is the minimum of:

- proof-verifier validity; and
- root-manifest validity.

#111 later narrows this again against current-constitution and embedded root-policy horizons.

No layer may widen a proof or authority lease.

## 8. No Holochain or execution authority

This crate is pure Rust. It contains no HDK calls, DHT lookup, persistence, policy discovery, lifecycle mutation or external effects.

It does not decide current constitutional state; the binding constitutional plane supplies that exact statement to the runtime before this kernel is called.

## 9. Remaining runtime boundary

A production adapter still needs to implement the role-specific proof verifier that can cryptographically/institutionally authenticate the exact adoption claim under the current rulebook's authorized adoption mechanism.

That adapter should receive the locally constructed claim, not invent it, and return only `VerifiedBootstrapRootAdoptionProof`.

Missing verifier, proof failure, stale verification or decode failure must deny.
