# SSF Source-Owned Operation Material v0.1

This crate materializes the exact operation payload for one capability-time revalidated possible-effect lineage **inside an independently trusted operation provider**. The SSF API accepts no caller-supplied payload bytes.

The provider receives only an exact `SourceOwnedOperationMaterializationSubjectV1` and returns an opaque provider-issued operation handle plus payload, encoding/schema, and receipt commitments bound to the exact admitted `OperationCarrierCommitment`.

Provider descriptor identity/policy/generation is independently expected and checked before and after materialization. A fresh uncertainty-aware trusted-time token is required again, and material validity is capped by the capability-time effect ceiling, provider generation, materialization receipt, and trusted-time receipt.

The resulting token still is not a move-only effect capability. It proves only that source-owned immutable operation material exists under an exact provider lineage and can be considered by a later capability-issuance boundary.
