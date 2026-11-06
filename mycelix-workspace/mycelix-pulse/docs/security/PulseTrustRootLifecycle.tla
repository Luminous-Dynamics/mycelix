---- MODULE PulseTrustRootLifecycle ----
EXTENDS Naturals, FiniteSets, TLC

(*
Bounded trust-policy root and key-lifecycle model.

A role-authenticated DSSE statement is only as trustworthy as the policy that
authorizes its key and role. A root-keyset identifier in the bundle is not an
authentication mechanism.

The safe transition:
  pinned root-A signs policy-2, whose parent digest is current policy-1;
  policy epoch increases monotonically;
  key-A is revoked and key-B becomes active.

Three deliberately unsafe switches model:
  UseUnpinnedRoot       — a self-asserted root can install a trust policy;
  AllowPolicyRollback   — an older signed policy can replace a newer policy;
  IgnoreKeyRevocation   — the verifier accepts a currently revoked key.

This models *current policy verification*. Historical signature validation is
a distinct policy query and must not be conflated with current acceptance.
*)

CONSTANTS
    PinnedRoot, SelfClaimedRoot,
    KeyA, KeyB, NoKey,
    UseUnpinnedRoot,
    AllowPolicyRollback,
    IgnoreKeyRevocation

Keys == {KeyA, KeyB}

VARIABLES
    trustedRoot,
    policyRoot,
    policyEpoch,
    previousPolicyEpoch,
    policyDigest,
    keyStatus,
    lastVerifyKey,
    lastVerifyDecision,
    lastVerifyEpoch,
    installDisposition

vars ==
    << trustedRoot, policyRoot, policyEpoch, previousPolicyEpoch,
       policyDigest, keyStatus, lastVerifyKey, lastVerifyDecision,
       lastVerifyEpoch, installDisposition >>

Init ==
    /\ trustedRoot = PinnedRoot
    /\ policyRoot = PinnedRoot
    /\ policyEpoch = 1
    /\ previousPolicyEpoch = 1
    /\ policyDigest = "policy-1"
    /\ keyStatus = [KeyA |-> "active", KeyB |-> "inactive"]
    /\ lastVerifyKey = NoKey
    /\ lastVerifyDecision = "none"
    /\ lastVerifyEpoch = 1
    /\ installDisposition = "initial"

InstallRootSignedPolicy2 ==
    /\ policyEpoch = 1
    /\ PinnedRoot = trustedRoot
    /\ policyRoot = trustedRoot
    /\ previousPolicyEpoch' = policyEpoch
    /\ policyRoot' = trustedRoot
    /\ policyEpoch' = 2
    /\ policyDigest' = "policy-2"
    /\ keyStatus' = [KeyA |-> "revoked", KeyB |-> "active"]
    /\ installDisposition' = "root-signed-child-policy"
    /\ UNCHANGED
        << trustedRoot, lastVerifyKey, lastVerifyDecision, lastVerifyEpoch >>

InstallSelfAssertedRoot ==
    /\ UseUnpinnedRoot
    /\ policyEpoch = 1
    /\ SelfClaimedRoot # trustedRoot
    /\ previousPolicyEpoch' = policyEpoch
    /\ policyRoot' = SelfClaimedRoot
    /\ policyEpoch' = 2
    /\ policyDigest' = "policy-self"
    /\ keyStatus' = [KeyA |-> "active", KeyB |-> "active"]
    /\ installDisposition' = "self-asserted-root"
    /\ UNCHANGED
        << trustedRoot, lastVerifyKey, lastVerifyDecision, lastVerifyEpoch >>

ReplayPolicy1 ==
    /\ AllowPolicyRollback
    /\ policyEpoch = 2
    /\ previousPolicyEpoch' = policyEpoch
    /\ policyRoot' = PinnedRoot
    /\ policyEpoch' = 1
    /\ policyDigest' = "policy-1"
    /\ keyStatus' = [KeyA |-> "active", KeyB |-> "inactive"]
    /\ installDisposition' = "replayed-old-policy"
    /\ UNCHANGED
        << trustedRoot, lastVerifyKey, lastVerifyDecision, lastVerifyEpoch >>

VerifyCurrentKeyA ==
    /\ policyEpoch = 2
    /\ keyStatus[KeyA] = "revoked"
    /\ lastVerifyKey' = KeyA
    /\ lastVerifyDecision' =
        IF IgnoreKeyRevocation
        THEN "accepted"
        ELSE "rejected"
    /\ lastVerifyEpoch' = policyEpoch
    /\ UNCHANGED
        << trustedRoot, policyRoot, policyEpoch, previousPolicyEpoch,
           policyDigest, keyStatus, installDisposition >>

VerifyCurrentKeyB ==
    /\ policyEpoch = 2
    /\ keyStatus[KeyB] = "active"
    /\ lastVerifyKey' = KeyB
    /\ lastVerifyDecision' = "accepted"
    /\ lastVerifyEpoch' = policyEpoch
    /\ UNCHANGED
        << trustedRoot, policyRoot, policyEpoch, previousPolicyEpoch,
           policyDigest, keyStatus, installDisposition >>

Next ==
    \/ InstallRootSignedPolicy2
    \/ InstallSelfAssertedRoot
    \/ ReplayPolicy1
    \/ VerifyCurrentKeyA
    \/ VerifyCurrentKeyB

Spec == Init /\ [][Next]_vars

TypeOK ==
    /\ trustedRoot \in {PinnedRoot, SelfClaimedRoot}
    /\ policyRoot \in {PinnedRoot, SelfClaimedRoot}
    /\ policyEpoch \in 1..2
    /\ previousPolicyEpoch \in 1..2
    /\ policyDigest \in {"policy-1", "policy-2", "policy-self"}
    /\ keyStatus \in [Keys -> {"active", "revoked", "inactive"}]
    /\ lastVerifyKey \in Keys \cup {NoKey}
    /\ lastVerifyDecision \in {"none", "accepted", "rejected"}
    /\ lastVerifyEpoch \in 1..2
    /\ installDisposition \in
        {"initial", "root-signed-child-policy", "self-asserted-root", "replayed-old-policy"}

PolicyEpochNeverRollsBack ==
    policyEpoch >= previousPolicyEpoch

OnlyPinnedRootCanAuthorizePolicy ==
    policyRoot = trustedRoot

CurrentPolicyDigestMatchesEpoch ==
    /\ (policyEpoch = 1 => policyDigest = "policy-1")
    /\ (policyEpoch = 2 => policyDigest \in {"policy-2", "policy-self"})

NoRevokedKeyAcceptedUnderCurrentPolicy ==
    lastVerifyDecision = "accepted"
    => keyStatus[lastVerifyKey] = "active"

RevocationIsMonotonicWithinPolicy2 ==
    policyEpoch = 2 /\ installDisposition = "root-signed-child-policy"
    => /\ keyStatus[KeyA] = "revoked"
       /\ keyStatus[KeyB] = "active"

====
