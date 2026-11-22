# Mycelix Directory Fabric: owner epochs and post-barrier route admission v0.1

**Profile:** MYC-DIR-OWNER-EPOCH-ROUTE-ADMISSION-0.1  
**Status:** source-present reference contract and model; not compiled or runtime-qualified  
**Tracking:** [MYC-DIR-REVOCATION-BARRIER-001 / #6476](https://github.com/Luminous-Dynamics/mycelix/issues/6476)  
**Composes with:** [Global Revocation Barrier PR #6478](https://github.com/Luminous-Dynamics/mycelix/pull/6478)

## 1. The lifecycle gap

A global revocation barrier can be correct at time T and still fail later if:
- an effect owner restores a stale database backup;
- a former primary reopens with a previously acknowledged generation;
- an owner epoch is reused after multiple restore/failover cycles;
- a newly added worker, direct API or scheduler route becomes executable before it inherits the current fence;
- a route is admitted against an old inventory manifest or route-set digest;
- a fence generation is present in memory but has not committed and been read back durably.

The safety claim therefore has to persist beyond the initial acknowledgement. It is a lifecycle invariant over recovery and route admission, not only a one-time barrier operation.

## 2. Fencing tokens are established practice

The existence of fencing tokens or sequencers is not a Mycelix novelty claim.

Google's Chubby lock service describes sequencers containing lock identity, mode and lock generation. The recipient resource server is expected to validate the sequencer and reject requests from an obsolete lock generation. See the [Chubby OSDI 2006 paper](https://research.google.com/archive/chubby-osdi06.pdf).

Mycelix's research contribution is the larger composition around that established primitive:
- one exact resource/action scope;
- a complete, independently verified effect-route inventory;
- owner-local atomic generation checks;
- worker-claim and decision-receipt binding;
- owner epoch plus generation recovery coordinates;
- durable owner commit and read-back evidence;
- exact owner-set barrier acknowledgement;
- post-barrier route admission against the current inventory;
- independent reconstruction of effect histories.

A token carried in a message is not useful if the final resource can be reached through a route that does not check it.

## 3. Owner fencing coordinate

Represent the mutation fencing coordinate as:

`(owner_epoch, resource_generation)`

The position is lexicographically ordered by owner epoch first, then generation. Every position component is nonzero.

- Generation advances monotonically within an owner epoch.
- A failover or restore recovery receives a strictly newer owner epoch.
- Recovery may not lower the generation below either the restored candidate's observed generation or the independently durable high-water generation.
- A prior token from an earlier owner epoch is rejected even if its generation number happens to match.
- A restored candidate is closed to writes until owner epoch and generation reconciliation completes and the resulting state is durably read back.
- The owner must never reopen with an owner epoch or generation below the last acknowledged high-water position.

The high-water record must live outside the rollback domain whose restore it is intended to detect, or be recoverable from an independently governed durable source. Copying the high-water value only inside the same stale backup is not recovery protection.

A production owner must additionally fence stale primaries, not merely label them with an epoch. A former primary that can still accept a write remains an effect owner until its write authority has actually been disabled.

## 4. New routes are staged, not immediately executable

A new route follows:

1. **Stage:** record route identity, owner, enforcement mode and expected inventory epoch/manifest/route-set digests. The route cannot issue protected mutations.
2. **Adopt owner fence:** authenticate the owner identity and current owner position; ensure the owner has recovered and is ready.
3. **Durability gate:** require the owner fence position to be committed and durably read back.
4. **Inventory gate:** require exact equality of inventory epoch, manifest digest and route-set digest. A same-number epoch with a substituted manifest is not equivalent.
5. **Activate:** only an AtomicOwnerFence route may enter the executable state.
6. **Revalidate each effect:** every mutation still checks the current owner epoch and generation atomically at the mutation boundary.

CAEP-only, token-expiry-only and best-effort invalidation routes cannot be admitted as strict effect routes. They can exist as signal/attenuation routes, but they must not count toward a strong revocation claim.

An active route may retain an old capability after a later fence install. That does not permit a mutation: the owner must reject the stale position. The route's continued existence is not itself continued authorization.

## 5. Typed Rust reference contract

Adds `tools/myc-dir-fabric-reference/src/owner_epoch.rs`, exported by the crate.

The module introduces:
- `OwnerFencePosition`: lexicographic owner-epoch/generation coordinate;
- `OwnerHighWaterMark`: external receipt commitment for the highest acknowledged owner position;
- `OwnerEpochState`: bootstrapped, unverified-restore and reconciled recovery states;
- `OwnerEpochFenceAck`: exact previous/current positions plus revocation and inventory binding, commit digest and post-commit read-back digest;
- `OwnerEpochFenceTarget`: the exact transition each owner must acknowledge;
- `assess_owner_epoch_fence_acks`: exact owner-set and position matching;
- `canonical_owner_epoch_ack_set_bytes`: deterministic owner-sorted receipt-set bytes;
- `GlobalRevocationBarrierReceipt`: coordinator receipt constructible only from a complete owner-ack rollup;
- `FencedRouteAdmission`: staged route activation requiring current durable owner position and exact inventory identity.

The reference layer does not hash, sign, authenticate, verify a trust root, make storage durable or enforce a live database lock. Digests are external commitments. In production, the verifier and owner must validate these records and the actual mutation must still use an atomic resource-local fence.

## 6. Formal model

The lifecycle model is [DirectoryFabricPostBarrierRouteAdmission.tla](../../mycelix-workspace/mycelix-pulse/docs/security/DirectoryFabricPostBarrierRouteAdmission.tla).

It covers:
- an initial barrier over an exact initial owner set;
- post-ack route staging, fence adoption, read-back and activation;
- owner restore to a stale candidate with routes paused;
- recovery to a newer owner epoch and durable high-water generation;
- safe and intentionally broken behaviors.

Configurations:
- [safe profile](../../mycelix-workspace/mycelix-pulse/docs/security/DirectoryFabricPostBarrierRouteAdmission.cfg);
- [new-route fence bypass negative control](../../mycelix-workspace/mycelix-pulse/docs/security/DirectoryFabricPostBarrierRouteAdmission_no_fence.cfg);
- [stale-restore high-water bypass negative control](../../mycelix-workspace/mycelix-pulse/docs/security/DirectoryFabricPostBarrierRouteAdmission_stale_restore.cfg).

The stale-restore high-water invariant requires both the recovered owner epoch and the recovered generation to be at least their independently durable high-water values; advancing the epoch cannot mask a lower restored generation. The stale-restore negative control intentionally violates that rule.

This is a finite abstraction. It does not model cryptographic verification, a real consensus/lease service, database WAL, actual split-brain network fencing, broker behavior or arbitrary multi-route-per-owner topology. The existing queue-worker model and global-barrier model cover other slices. All models are source-present only until TLC runs and negative controls produce the expected counterexamples.

## 7. Qualification requirements

### Owner recovery

- Restore a stale backup after an acknowledged fence.
- Confirm all routes are paused or fail closed while recovery is incomplete.
- Recover with an owner epoch strictly above both the candidate and durable high-water epoch.
- Preserve generation at least as high as both observed and acknowledged generation.
- Reopen only after durable read-back and exact recovery evidence.
- Repeat failover/restore cycles to prove no owner-epoch reuse.
- Attempt stale-primary writes from the former owner and verify they are physically rejected.

### Route admission

- Attempt execution from the staged route before owner fence adoption: reject.
- Attempt activation with correct owner generation but wrong owner epoch: reject.
- Attempt activation with correct position but stale inventory epoch, manifest digest or route-set digest: reject.
- Attempt activation after in-memory fence update but before commit/read-back: reject.
- Attempt CAEP-only or best-effort route activation into strict execution: reject.
- Introduce a new route after global acknowledgement and prove it cannot execute until staged, fenced, read back and inventoried.

### Independent oracle

The history checker must correlate route admission, owner epoch/generation, owner fence commit, durable read-back, owner acknowledgement, global acknowledgement and physical effects. It must verify that a mutation's exact owner position was current at its linearization point and that every executable route was covered by the current verified inventory.

A test count with zero observed violations is not a mathematical proof of universal safety. Model checking, transaction-level races and independent history reconstruction are complementary evidence sources; each must be reported separately.

## 8. Evidence boundary

The Rust tests in this module are authored, not compiled or executed in this environment. TLC configurations are source-present, not model-checked. No PostgreSQL, failover, restore, broker, route-admission or performance result is claimed. The benchmark companion corpus is preregistered and explicitly not executed.

The next execution gates are the pinned TLC runs, the PostgreSQL two-session/failover harness, real broker adapters, and an independent multi-owner history oracle.


## Additional hardening: proposed versus durable owner position

The 18-test reference increment now distinguishes a proposed in-memory fence from a durably committed and read-back position. `assess_position` returns `FenceNotDurablyReadBack` while the position is proposed but not yet backed by commit/read-back evidence. It returns `StaleGeneration` for an older route capability only after the newer position is durably read back. The owner-ack rollup also retains the exact canonical sorted owner-ack bytes, and the global receipt embeds those bytes alongside the externally computed digest. This preserves the evidence being summarized while leaving cryptographic hashing and verification outside the dependency-free reference crate.
