# EVID-TIME-001A — Structural Time Evidence

Status: **source candidate / structural only / not qualified**

This crate is the reusable semantic root for time-sensitive evidence in Mycelix. It intentionally does not read a system clock, contact a time server, verify a signature, or decide freshness/currentness.

## Core distinction

```text
time value observed
!= observation authentic
!= source trusted
!= source synchronized
!= policy-trusted time
!= observation fresh
!= domain state current
```

## Closed intervals, not mandatory exact points

Time evidence is represented as:

```text
TimeIntervalV1 {
    earliest: UnixTimePointV1,
    latest:   UnixTimePointV1,
}
```

A source may report a zero-width interval, but zero width does not itself prove exact universal time. Uncertainty is theorem-bearing and changes observation identity.

`UnixTimePointV1` is structural Unix-seconds + nanoseconds only. The exact UTC/leap-second/time-scale interpretation belongs to the named `time_scale_profile` and its future adapter qualification.

## Source identity

Every raw observation binds:

- source namespace;
- source instance;
- source trust domain/operator domain;
- source profile ID + version;
- time-scale profile;
- exact closed time interval;
- optional source sequence;
- exact transcript digest;
- exact source-receipt digest;
- optional request-binding digest.

The trust domain is separate from instance identity so future multi-source policy cannot mistake two keys or servers from one operator for two independent time authorities.

## Structural admission policy

`TimeSourceAdmissionPolicyV1` binds the exact expected source/profile identities plus:

- exact verifier-profile ID + version;
- maximum admitted interval width;
- whether source sequence is required;
- whether request binding is required.

Naming a verifier profile is configuration only:

```text
verifier profile configured
!= verifier executed
```

## Strongest v1 positive

`StructurallyCompatibleTimeObservationV1` is serializable but not caller-deserializable. It establishes only that the raw observation structurally satisfies the exact admission profile.

It explicitly remains false for:

```text
source cryptographically verified
verifier execution
source trusted under policy
replay resistance
synchronized clock
policy-trusted time
freshness
currentness
legal timestamp
application authority
```

## No ambient clock

EVID-TIME-001A must remain pure structural semantics. `SystemTime::now`, `Instant::now`, Chrono `Utc::now`, NTP sockets, HTTP clients, Holochain runtime calls, Xenia provider calls, or hardware-clock APIs do not belong here.

## Future adapters

Later independent adapters may consume this vocabulary for:

- RFC 8915 NTS;
- Roughtime Experimental profiles;
- signed/provider time receipts;
- TPM/TEE/hardware-backed time evidence;
- Xenia provider attestations.

Those adapters must emit provider-relative verification evidence, not mutate this structural crate into a protocol implementation.

## Future policy composition

EVID-TIME-001C should join provider-verified evidence under explicit trust policy and reason over interval intersection/conflict. Independence should be based on exact trust domains, not key/server count.

EVID-TIME-001D should consume a policy-trusted **now interval** plus a target observation interval and explicit freshness profile to derive outcomes such as definitely fresh, definitely stale, ambiguous, or future-dated. It must never call the system clock internally.

## Source corpus

The committed tests cover invalid nanoseconds, reversed intervals, zero-width intervals, cross-second interval arithmetic, exact structural positive/non-authority, excessive uncertainty, trust-domain/profile substitution, missing required sequence/request binding, uncertainty changing observation identity, and raw observation serde round-trip.

## Standards context

RFC 8915 NTS provides cryptographic protections for NTP synchronization including peer identity/authentication, replay prevention, and request-response consistency. Those are source-verification properties, not a universal application-currentness theorem.

Current IETF Roughtime work is Experimental and intentionally models rough secure time/inconsistency evidence. It should therefore enter as an adapter profile rather than a built-in trusted-time enum.
