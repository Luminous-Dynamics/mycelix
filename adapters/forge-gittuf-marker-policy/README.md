# forge-gittuf-marker-policy

FORGE-009H provides a concrete, read-only inspection of the exact active gittuf v0.16 policy tip for the durable protected-merge consumption-marker namespace.

It consumes the positive FORGE-009F transaction plan and the verified `GittufLocalReceipt`. It then:

- verifies the live `refs/gittuf/policy` tip still equals the receipt's policy tip;
- recomputes the external repository-policy subject from that exact tip;
- reads the policy metadata directly from the exact Git object rather than parsing CLI presentation;
- mirrors the relevant gittuf delegation traversal for the exact `git:<marker-ref>` target;
- records every applicable authorization path and the union of authorized principal IDs;
- requires `block-force-pushes` coverage for the marker namespace.

The positive `GittufMarkerNamespacePolicyEvidenceV1` result does **not** claim create-only marker immutability. gittuf's block-force-push rule permits descendant/fast-forward updates, so a later constrained-executor theorem must restrict every exposed principal to the exact FORGE-009F transaction profile.

This adapter also fails closed on policy features it does not reproduce exactly, including multi-repository controller policy and unsupported fnmatch syntax.

## Qualification

Pinned Rust 1.96.0:

- `cargo fmt -- --check`
- all-target/all-feature Clippy with warnings denied
- all-feature tests

No repository mutation is performed by this crate.
