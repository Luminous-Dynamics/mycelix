# Response Attempt Store v0.1 — Normative Invariants

Status: **crash-durable reservation only; no effect-start transition and no execution authority**

1. **Stable effect key, not authority generation.** The final journal filename is derived only from the exact response effect identity produced by `mycelix-response-attempt-identity`. Refreshing authority/currentness must not create another effect namespace.

2. **Store-owned winning attempt identity.** The native store generates the nonce from `/dev/urandom` and derives the final attempt reference from the effect key + nonce. An unpersisted provisional candidate has no durable meaning.

3. **One final record per effect key.** Publication uses a no-overwrite hard link from a private temporary file. Existing final records are never truncated, replaced, or silently superseded.

4. **Journal before any future effect.** Record bytes are written and `fsync`ed before publication. The containing directory is `fsync`ed after publication/removal of the temporary name.

5. **Ambiguity fails closed.** Malformed JSON, invalid protocol/profile, digest mismatch, unsafe file type, unsafe permissions, oversized records, or a different existing attempt identity all deny.

6. **Same-attempt reconciliation only.** If a record exists, the exact stored nonce/ref is re-run through `prepare_response_attempt` against the supplied current qualifications. The attempt identity and both current-authority/safety qualification identities must remain exactly equal. Otherwise a fresh attempt is forbidden and explicit reconciliation is required.

7. **Private filesystem objects.** The store root and record/lock files must be owned by the effective UID and inaccessible to group/other. Record and lock opens use `O_NOFOLLOW`.

8. **Thread + process serialization.** Reservation uses both an in-process `Mutex` and `flock(LOCK_EX)` on the store lock file.

9. **No effect-start state in v0.1.** The only durable state is `Reserved`. There is no public transition to `EffectStarted`, no actuator call, no Holochain call, and no external network effect.

10. **Durability is not update atomicity.** A durably reserved response attempt still does not prove coordinator code cannot change after observation. #298/#387 stability/provenance and a future coordinator-update exclusion theorem remain separate requirements.

11. **Store configuration provenance remains external.** `TrustedResponseAttemptStore::open` validates the resulting filesystem object, ownership and permissions. The application still must source the store path from trusted configuration rather than request payloads.

12. **No automatic retry from uncertainty.** A crash that leaves a valid reservation causes the same stored attempt to be reconciled. It never licenses minting a replacement nonce for that effect key.
