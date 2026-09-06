# Current Authority Runtime Adapter v0.1 — Normative Contract

Status: **contract only; no verifier zome is provisioned by this tranche**

This contract defines the first HDK boundary allowed to project the pure authority stack into a live Holochain current-freshness receipt.

The final positive pure boundary is #126 `QualifiedCompleteDeploymentAuthority`.

The runtime adapter MUST collect evidence; it MUST NOT invent authority semantics.

## 1. External request surface is target-only

The externally callable authorization request MUST contain only the exact target `AuthoritySubjectRef` plus future non-authoritative request correlation metadata if needed.

The caller MUST NOT authoritatively provide:

- expected/local DNA hash;
- current time;
- current constitution;
- bootstrap root/adoption;
- context/coverage policy identities;
- control-plane subject set;
- probe nonce/entropy/proof ref;
- source head/generation;
- witness identities/trust domains;
- transition set/current state;
- verification refs; or
- freshness leases.

Any informational copy of those fields is ignored for authority and MUST be recomputed or obtained from the designated local/verified provider boundary.

## 2. Local DNA identity comes from the Holochain host

The runtime MUST obtain the expected live DNA identity from the local cell context with the HDK DNA-info host primitive (Holochain v0.6 equivalent of `dna_info()?.hash`).

The resulting exact hash is the only value allowed to populate #123 `VerifiedLocalDnaContext.dna_hash`.

It MUST NOT be copied from `VerifiedCurrentConstitutionReceipt.dna_hash` or from caller input.

If local DNA identity cannot be obtained, qualification denies.

## 3. Current time comes from the host

`now_ms` MUST be derived from the Holochain host clock (`sys_time`) for the current invocation.

The caller cannot select the verification time used for freshness/lease checks.

## 4. Current constitution is an explicit verifier boundary

The runtime MUST obtain `VerifiedCurrentConstitutionReceipt` from the DNA-bound constitution/transition verifier.

That provider must independently prove the exact current constitutional statement/lineage and must not re-enable the quarantined legacy mutable constitution plane.

The runtime then relies on #123 to require:

`current_constitution.dna_hash == host-derived local DNA hash`.

Provider outage, decode failure, stale receipt, legacy-authority indication, or DNA mismatch denies.

## 5. Bootstrap manifest and adoption are explicit verifier boundaries

The exact `AuthorityStateBootstrapRootManifest` and `VerifiedBootstrapRootAdoption` MUST come from the constitution/rulebook bootstrap-root boundary.

The runtime MUST NOT infer root adoption from record existence, author identity, a policy ID, or the root manifest's own claims.

#126 re-runs #111 over those evidence-shaped inputs.

## 6. Operational semantic policy evidence is independent of currentness

For the target subject, the runtime obtains the exact semantic:

- `VerifiedAuthorityCoveragePolicy`; and
- `VerifiedCoverageTrustContextPolicy`

from their semantic policy-verification boundary.

Semantic validity is not currentness. The runtime must still prove current control-plane generations through the covered #115 path.

## 7. Required control-plane subjects are deterministic, not provider-selected authority

From the current bootstrap root plus exact operational semantic policies, the runtime constructs candidate evidence for the deterministic current-policy set:

DirectSource:

1. exact `AuthorityCoveragePolicy` in `root.control_plane_namespace`;
2. exact `CoverageTrustContextPolicy` in `root.control_plane_namespace`.

WitnessQuorum additionally requires:

3. exact `WitnessTrustPolicy` in `root.control_plane_namespace`.

The operational policy's covered namespace is NOT the policy-state registry namespace.

A provider may return candidate evidence bundles, but it cannot broaden the required subject set. #116/#120/#126 reconstruct and re-check the exact set.

## 8. Probe issuance is evidence collection, not authority

For every control-plane subject and the target operational subject, the runtime may obtain a fresh #114 probe.

Probe generation:

- uses host entropy;
- cannot accept caller nonce/entropy;
- does not require an issuer AuthorityGrant merely to collect read-only evidence; and
- grants no current authority.

The runtime must verify the probe's randomness/provenance before using it as #96 challenge evidence.

## 9. Source-head verification is mandatory

For every challenged subject, the runtime obtains one exact `VerifiedAuthoritySourceHead` from the designated source-head verifier.

The verifier must bind:

- exact subject;
- exact challenge;
- exact authoritative source identity;
- exact head generation/digest/status ref; and
- bounded verification window.

A local DHT query, highest observed generation, newest timestamp, or absence of a newer record MUST NOT substitute for this receipt.

## 10. Witness/trust evidence is mode-specific and independently verified

DirectSource mode requires no witness list and no witness-trust bindings beyond what its policy defines.

WitnessQuorum mode requires the exact witness observations and trust bindings accepted by #96, including:

- exact challenge/source-head binding;
- exact observer identity;
- exact trust-domain identity;
- exact witness-trust policy/verifier context;
- trust-domain diversity; and
- per-domain contribution caps.

Provider list order has no authority meaning.

## 11. Transition evidence is independently verified and complete

For every challenged subject, the runtime obtains `VerifiedAuthorityStateTransition` candidates from the transition-verification boundary.

The runtime MUST NOT select a current transition itself.

#91 must receive the full verified chain and covered source head and must reject:

- missing root;
- orphan/gap;
- fork;
- skipped generation;
- wrong parent digest;
- causal regression;
- endpoint mismatch; and
- omitted later revocation.

## 12. Positive construction occurs only through #126

After collecting all candidate evidence, the runtime assembles `CurrentOperationalAuthorityEvidence`, constructs host-derived `VerifiedLocalDnaContext`, and calls #126 `qualify_complete_deployment_authority`.

No wire receipt may be assembled directly from provider fields.

The runtime MUST NOT treat the existence/deserialization of any `Verified*` or historical `Qualified*` bytes as sufficient authority.

## 13. Runtime output exposes three different identities

A successful runtime response SHOULD expose at least:

- semantic current-authority digest/profile;
- deployment-bound current-authority digest/profile; and
- complete dynamic evidence digest/profile.

These answer different questions and MUST NOT be collapsed:

- semantic identity: what authority exists institutionally;
- deployment identity: is that authority live in this exact DNA;
- evidence identity: which exact fresh proof instances established current use.

## 14. Current freshness projection is lease-bounded

The wire `VerifiedAuthorityFreshness` returned to downstream consumers MUST come from #126's positive object and inherit its narrowed verification ref/window.

A runtime cache may reuse a positive result only within that exact lease and only if all cache keys include at least:

- exact target subject identity;
- exact deployment qualification identity; and
- exact complete evidence identity or equivalent proof-generation key.

No cache may extend a lease or survive DNA/root/generation change by textual ID alone.

## 15. Provider failure is denial

For every required provider boundary:

- zome/function missing;
- `Unauthorized`;
- network/provider error;
- decode failure;
- empty required result;
- stale result;
- profile mismatch; or
- contradictory evidence

means **DENY / unavailable**, never fallback to legacy voting, mutable timelock/proposal state, Phi, reputation, caller assertions, or cached stale authority.

## 16. Status endpoints are non-authoritative

A runtime status endpoint MUST NOT probe providers with synthetic subjects to infer operational authority.

Code presence and provider reachability are diagnostics only.

`operational=true` MUST NOT mean a particular subject is authorized; authorization remains subject-specific and request-time.

## 17. No production provisioning yet

The first implementation of this runtime MUST be workspace-buildable but absent from the binding governance `dna.yaml` until:

- #100/#111/#114/#115/#116/#117/#120/#123/#126 have executable green qualification;
- required provider contracts are implemented;
- multi-agent adversarial tests pass; and
- the lifecycle/effect path consumes the deployment/evidence-qualified receipt rather than a weaker legacy freshness receipt.

## 18. Adversarial acceptance suite

Before provisioning, test at minimum:

- caller-supplied fake DNA hash is ignored; host DNA wins;
- correct statement on wrong DNA denies;
- hidden later revocation denies;
- valid prefix without covered head denies;
- source refusal/partition denies;
- stale/rotated bootstrap root denies;
- coverage policy revoked during probe denies;
- context policy rotated during probe denies;
- witness trust reclassification denies stale proof;
- wrong trust domain/proof rebinding denies;
- transition fork/gap denies;
- historical receipt replay denies;
- provider outage/decode failure denies;
- same stable authority with refreshed proofs preserves semantic/deployment digests while changing complete evidence digest; and
- no external effect can occur merely because this verifier returns a positive current-freshness receipt.

## 19. No effect authority

This runtime verifier proves current authority state only.

It does not itself:

- authorize threshold signing;
- designate an executor;
- satisfy effect-safety policy;
- create a lifecycle claim; or
- perform an external effect.

Those remain later explicit boundaries.

Related: #126 #124 #123 #121 #120 #117 #116 #115 #114 #111 #109 #73.
