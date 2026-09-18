# mycelix-forge-gittuf-adapter

FORGE-004B adapter from gittuf 0.16.0 into the provider-neutral Mycelix Forge repository-verification contract.

## Security boundary

This crate does not implement repository policy itself. It invokes a separately supplied `gittuf` executable and consumes only the exit status of `gittuf verify-ref <ref>`, then derives Forge evidence from exact Git ref identities read with `git rev-parse`.

The adapter:

- requires `gittuf version` to report exactly `0.16.0`;
- never passes `--latest-only` or `--from-entry` on the M0 path;
- forces `GITTUF_DEV=0` and `GITTUF_DEBUG=0`;
- rejects `refs/local/gittuf/persistent-cache`, so `FullHistory` cannot silently resume from a local cached verification point;
- snapshots the protected ref, RSL, policy, and optional attestations refs before and after verification and rejects any change;
- binds the Forge `RepositoryPolicyState` to the exact `refs/gittuf/policy` tip;
- stores the full translated repository-policy state inside receipts and revalidates it on deserialization;
- requires the exact Forge verification request again before a receipt can become Forge evidence;
- produces path-independent canonical local receipts;
- deliberately does not claim `OfflineEvidence`.

The supplied executable's binary provenance is not established by this crate. A later packaging/qualification tranche must pin the executable by Nix/store identity or artifact digest.

## Validation

```bash
cargo fmt --all -- --check
cargo clippy --all-targets --all-features -- -D warnings
cargo test --all-features
```

FORGE-004C will add replayable offline Git/gittuf evidence bundles. FORGE-005 remains the Xenia/Mycelix principal-authentication bridge.
