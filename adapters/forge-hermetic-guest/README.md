# FORGE-004D3B2C2C — Concrete Hermetic Guest

This crate executes the frozen Forge M0 guest workflow. It deliberately adds no new repository authority semantics; those remain in the guest plan, transcript, tool-map, trust, and envelope contracts.

It builds two binaries:

- `forge-hermetic-guest` — executes the canonical guest plan;
- `forge-isolation-probe` — runs the destructive inside-isolation probe in a disposable subprocess.

## Execution order

The guest executes exactly:

1. load the fixed-path typed plan/capsule artifacts;
2. verify their semantic commitments;
3. verify the 32-byte run challenge;
4. spawn the exact `forge-isolation-probe` path from the guest tool map;
5. replay the self-contained Git bundle into a fresh bare `/work/repository.git` **without running gittuf**;
6. reject ambient Git object sources, shallow/promisor state, and configured remotes;
7. execute the frozen strict `git --no-replace-objects fsck --full --strict --no-reflogs --no-progress` policy;
8. collect the complete gittuf policy trust inventory;
9. derive `LocalEmbeddedKeysV1` qualification;
10. derive `GittufInvocation::from_request(...)` from the typed plan request and embedded policy state;
11. run the exact cache-free gittuf adapter and require the expected replay receipt;
12. construct and self-qualify the canonical guest transcript;
13. atomically write and re-read the transcript;
14. construct and self-qualify the raw guest evidence envelope;
15. atomically write and re-read the envelope.

The existing `OfflineBundleAdapter::replay_bundle` is intentionally **not** used inside the guest because it performs gittuf verification during replay, which would violate the frozen phase order.

## Exact process environment

Every directly launched subprocess uses `env_clear()` and restores only the same exact environment entries used by the FORGE-004D2B1 isolation qualification contract. Git and gittuf executables are always absolute paths supplied by `GuestToolMapV1`; the guest never searches `$PATH` or scans `/nix/store` by package name.

## Process boundary

The guest emits only `GuestEvidenceEnvelopeV1` raw evidence. Its guest-side qualification is a consistency check, not authority. The host must independently:

- qualify the guest tool map against the exact ExecutionSpec and NAR-qualified closure;
- re-qualify the guest envelope;
- combine raw inside evidence with parent-side process/namespace evidence;
- establish pidfd same-process binding;
- perform post-run NAR qualification;
- qualify the same-run transcript;
- call the final M0 composition theorem.

## Pending composition refinement

`/inputs/guest-tool-map.json` and `/work/guest-evidence-envelope.json` are fixed v1 runner paths. The later D3A refinement must add the exact guest-tool-map JSON bytes to the execution input/mount set and bind the qualified guest envelope/transcript before `OfflineEvidence` is considered complete.
