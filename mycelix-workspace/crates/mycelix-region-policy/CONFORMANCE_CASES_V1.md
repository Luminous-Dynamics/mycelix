# Regional Policy Kernel — Conformance Case Matrix v1

**Status:** Draft fixture specification. The test names identify source-level cases; a green run on the exact commit is still required before any case can be claimed as passing.

## Purpose

This matrix is the shared test contract for any native Rust, WASM, Holochain coordinator, storage adapter, or external-service implementation of the regional policy decision. It is designed to prevent each region from reinventing the policy engine and to make policy changes reviewable through stable expected outcomes. The current kernel contains 35 source-level tests, including the cases mapped below; exact-head CI must still compile and execute them before any is called passed.

The test clock is an explicit deterministic input. Production adapters must obtain it from a trusted runtime clock, not from an untrusted request. Holochain integrity validation must remain deterministic across peers and time; current profile/consent freshness checks belong at the runtime action boundary.

## Positive controls

| Case ID | Scenario | Expected result | Source-level test anchor |
|---|---|---|---|
| RPK-ALLOW-001 | Source and destination both authorize the exact data class, action, source/destination scopes, purpose, recipient and active consent | Allow | `permits_only_explicitly_authorized_bilateral_route` |
| RPK-ALLOW-002 | Reordered set-like actions/scopes in otherwise equivalent profile inputs | Same canonical profile payload | `canonical_payload_is_independent_of_set_order` |
| RPK-AGREE-001 | One verified agreement binds the exact source/destination profile IDs and digests, allowed class/action/purpose/recipient, and active validity window | Allow only when agreement and both profile policies jointly allow the action | `permits_only_explicitly_authorized_bilateral_route` |
| RPK-AGREE-002 | Source and destination signers approve the same exact agreement digest using distinct role-bound messages | Messages are domain-separated to prevent source/destination signature substitution | `source_and_destination_approval_messages_are_domain_separated` |
| RPK-AUDIT-001 | Valid decision has agreement ID/version/digest, source/destination profile IDs/versions/digests, action, data class, content digest and timestamp | Audit context is bound; no raw purpose, recipient, or resource ID in the result object | `binds_policy_decision_to_profile_versions_and_non_sensitive_audit_fields` |
| RPK-MATRIX-001 | All four combinations of source permission × destination permission | Allow iff both sides permit | `bilateral_route_truth_table_only_allows_mutual_permission` |

## Negative controls

| Case ID | Mutation or failure | Required denial/rejection | Source-level test anchor |
|---|---|---|---|
| RPK-DENY-001 | Source allows destination but destination does not authorize source | `SourceNotAllowedByDestination` | `denies_asymmetric_destination_policy` |
| RPK-DENY-002 | Route otherwise valid but consent is absent | `ConsentRequired` | `denies_missing_consent_even_when_route_is_allowed` |
| RPK-DENY-003 | Consent names a different recipient and purpose | Both mismatch reason codes | `denies_consent_bound_to_another_recipient_or_purpose` |
| RPK-DENY-004 | Consent/revocation evidence expires at the evaluation instant | `ConsentEvidenceStale` | `denies_when_revocation_evidence_is_stale` |
| RPK-DENY-005 | Current profile version has expired | Source and destination inactivity reasons | `denies_after_profile_expires` |
| RPK-DENY-006 | Profile verifier rejects the signature | Verified profile construction fails | `rejects_profile_that_trusted_adapter_does_not_verify` |
| RPK-DENY-007 | Unknown schema version | `UnsupportedSchemaVersion` | `rejects_unsupported_profile_schema_before_verification` |
| RPK-DENY-008 | Duplicate rule class or repeated scope ID | Profile shape rejected | `rejects_duplicate_rule_classes_and_duplicate_scopes` |
| RPK-DENY-009 | Malformed profile digest or missing signature bytes | Profile shape rejected | `rejects_malformed_digest_and_empty_signature` |
| RPK-DENY-010 | Required purpose and recipient are missing | Required-field and exact consent-binding reasons | `missing_purpose_and_recipient_are_denied` |
| RPK-DENY-011 | Claimed resource ID differs from verified classification subject | `ClassificationSubjectMismatch` | `denies_transfer_of_a_different_object_than_the_classification_claim` |
| RPK-DENY-012 | Object content digest changed after classification evidence was verified | `ClassificationContentMismatch` | `denies_changed_content_after_classification` |
| RPK-DENY-013 | Classification evidence freshness expires | `ClassificationEvidenceStale` | `denies_when_classification_evidence_is_stale` |
| RPK-DENY-014 | Profile trust-root evidence is stale at construction time | Verified profile construction fails | `denies_profile_when_trust_root_evidence_deadline_is_stale` |
| RPK-DENY-015 | Profile document is still within its own validity window but signer-trust evidence expires | `SourceProfileEvidenceStale` | `expires_profile_trust_evidence_even_when_profile_document_is_still_active` |
| RPK-DENY-016 | Caller supplies an evaluation timestamp before the source/destination/classification evidence was verified | Backdated decision rejected | `rejects_backdated_evaluation_after_profile_was_verified` |
| RPK-DENY-017 | Caller supplies an evaluation timestamp before consent was verified | `ConsentVerifiedAfterDecisionTime` | `rejects_backdated_evaluation_after_consent_was_verified` |
| RPK-DENY-018 | Agreement binds a different source profile ID/version | `AgreementSourceProfileMismatch` | `rejects_agreement_bound_to_another_profile_version` |
| RPK-DENY-019 | Agreement source profile digest differs even though the ID matches | `AgreementSourceProfileMismatch` | `rejects_agreement_when_source_profile_digest_does_not_match` |
| RPK-DENY-020 | Action is permitted by both profiles but absent from the bilateral agreement | `AgreementActionNotAllowed` | `denies_action_not_in_agreement_even_if_both_profiles_allow_route` |
| RPK-DENY-021 | Purpose is absent from the exact agreement allowlist | `AgreementPurposeNotAllowed` | `denies_purpose_not_explicitly_authorized_by_agreement` |
| RPK-DENY-022 | Recipient is absent from the exact agreement allowlist | `AgreementRecipientNotAllowed` | `denies_recipient_not_explicitly_authorized_by_agreement` |
| RPK-DENY-023 | Agreement allowlist order changes but members do not | Canonical bytes remain identical | `agreement_canonical_payload_is_independent_of_allowlist_order` |
| RPK-DENY-024 | Agreement expiry is reached while both region profiles remain active | `AgreementExpired` | `denies_expired_agreement_even_if_source_and_destination_profiles_remain_active` |
| RPK-DENY-025 | Agreement signer/trust evidence freshness deadline is reached | `AgreementEvidenceStale` | `denies_when_agreement_trust_evidence_becomes_stale` |
| RPK-DENY-026 | Source-party approval verification fails | Agreement construction rejected | `rejects_agreement_when_source_approval_verification_fails` |
| RPK-DENY-027 | Destination-party approval verification fails | Agreement construction rejected | `rejects_agreement_when_destination_approval_verification_fails` |
| RPK-DENY-028 | Consent is replayed for another resource or content digest | Resource and content mismatch reasons | `denies_consent_reused_for_another_resource_or_content_version` |
| RPK-DENY-029 | Consent is replayed across subjects, data classes, or actions | Subject, data-class and action mismatch reasons | `denies_consent_for_another_subject_data_class_or_action` |


## Mandatory extension corpus before production

The current source-level unit cases do not yet prove the production verifier or the storage/replication path. Implement these additions with a real cryptographic and runtime adapter:

- **Profile integrity:** recompute BLAKE3-256 over canonical bytes; flip every identity-bearing field and confirm digest/signature verification fails; test wrong signing key ID, wrong trust-root, revoked key, key rotation, unknown algorithm and downgrade attempts.
- **Canonical encoding:** test length boundaries, arbitrary UTF-8, control characters, duplicate data-class rules, duplicate actions/scopes, rule ordering and serialization-version changes. Maintain fixed canonical payload and digest test vectors across Rust and WASM implementations.
- **Consent:** test grant issuer, subject/agent binding where required, exact recipient, purpose, action, data class, issuance and expiry; stale/revoked/unavailable status evidence; revocation racing with action execution; clock rollback; duplicate request IDs.
- **Classification:** bind the attestation to exact byte digest, resource identifier, schema, source, data class and validity period. An untrusted caller must not be able to relabel sensitive data as public or reuse a valid claim after content changes.
- **Replay and time:** backdated timestamps; future timestamps; expired trust-root freshness; stale source and destination profile; replay across region, resource, purpose and action; retry after reconnect; monotonic time rollback and large jumps.
- **Bilateral policy:** exhaustively enumerate allowed/denied source/destination actions, classes, scopes, retention, recipient and purpose combinations. Test the exact reason code and assert no decision is allowed when any hard constraint fails.
- **Lifecycle:** revoke a profile, signer, grant, integration permission and federation agreement; roll forward, roll back and recover; show that old decisions remain interpretable against pinned profile digests without authorizing new actions.
- **I/O boundary:** instrument writes/reads to DHT/gossip, queues, caches, indexes, backups, crash reports, telemetry, support exports and connectors. Seed restricted canary records and verify they never appear in a prohibited destination, including failure/retry/recovery paths.

## Evidence bundle per run

Each qualified run should store:

1. Full 40-character Git commit SHA and clean source-tree identifier.
2. Rust version, target triple, OS/kernel, CPU/device and relevant feature flags.
3. Conformance schema version and cryptographic digest of the exact fixture corpus.
4. Test command, run ID, individual case pass/fail/skip counts and captured stable reason codes.
5. For integrated tests, instrumented sink coverage showing every attempted output path was observed.
6. Clear scope statement: policy-kernel unit qualification is not signature security, data residency, legal compliance, or production readiness.

**Release policy:** no queued, cancelled, skipped, or stale-head run counts as a pass. Any confirmed unauthorized restricted-data egress blocks release of that data class until fixed and re-qualified.