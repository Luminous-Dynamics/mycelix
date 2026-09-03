# SIF v0.1 Evidence Trust Policy

Status: design contract for the next verifier hardening tranche.

Role presence alone is not enough for high-assurance evidence. Two proof references can claim different roles while being controlled by the same compromised key, service, administrator, or trust domain. SIF therefore distinguishes **what a proof establishes** from **who independently vouches for it**.

## Required direction

Future `AttestationRef` evolution should make evidence authority explicit rather than overloading `scheme` or `verifier_profile` as identity. The verifier needs stable, commitment-safe metadata for at least:

- proof role;
- verifier/key identity;
- trust domain or administrative authority;
- proof scheme/profile;
- shared statement digest;
- proof digest.

A deployment policy can then express constraints such as:

- at least one verified `ExecutionBinding` from the authenticated Xenia authority;
- at least one verified `ComputationProof` from the approved computation authority;
- delayed/high-impact disclosure additionally requires one `ExternalWitness` outside the requester trust domain;
- N-of-M witnesses must contain at least K distinct verifier keys and D distinct trust domains;
- one proof artifact cannot satisfy multiple logically independent roles unless the policy explicitly permits it.

## Anti-collusion principle

Distinct strings are not independence. Independence is a property of verifiable authority roots and operational control. A future production profile should therefore bind trust-domain membership to signed configuration, credentials, or governance state rather than accepting caller-provided labels.

## Failure cases to cover

- same verifier key submits both execution and witness evidence;
- two verifier keys are controlled by one declared trust domain when policy requires two domains;
- proof reference changes its role without changing proof bytes;
- duplicate proof digest is replayed under multiple verifier profiles;
- retired/revoked verifier key is used for a fresh disclosure;
- witness is cryptographically valid but not independent of requester organization;
- optional invalid evidence attempts to hitchhike beside sufficient valid evidence;
- threshold is met numerically but not by the required role/domain composition.

This is intentionally staged after canonical/CI qualification so evidence-authority metadata does not destabilize the v0.1 commitment boundary while it is still being validated.
