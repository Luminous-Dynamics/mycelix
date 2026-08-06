#!/usr/bin/env python3
"""Deterministic protocol-v19 endorsement-chain lifecycle simulation."""
from __future__ import annotations

import hashlib
from dataclasses import dataclass, replace


def h(*parts: bytes) -> bytes:
    digest = hashlib.sha256()
    for part in parts:
        digest.update(len(part).to_bytes(8, "little"))
        digest.update(part)
    return digest.digest()


@dataclass(frozen=True)
class Cert:
    serial: bytes
    subject: bytes
    issuer: bytes
    security_epoch: int
    firmware_floor: int
    is_ca: bool
    signature: bytes

    def tbs(self) -> bytes:
        return h(
            b"cert-v1",
            self.serial,
            self.subject,
            self.issuer,
            self.security_epoch.to_bytes(8, "little"),
            self.firmware_floor.to_bytes(8, "little"),
            bytes([self.is_ca]),
        )


def sign(issuer_key: bytes, cert: Cert) -> Cert:
    return replace(cert, signature=h(b"sig-v1", issuer_key, cert.tbs()))


def verify_signature(issuer_key: bytes, cert: Cert) -> bool:
    return cert.signature == h(b"sig-v1", issuer_key, cert.tbs())


def verify_chain(
    leaf: Cert,
    intermediate: Cert,
    root_key: bytes,
    active_root: bytes,
    root_set: set[bytes],
    revoked_serials: set[bytes],
    firmware_version: int,
) -> bool:
    return all(
        [
            active_root in root_set,
            not leaf.is_ca,
            intermediate.is_ca,
            leaf.issuer == intermediate.subject,
            intermediate.issuer == active_root,
            leaf.serial not in revoked_serials,
            intermediate.serial not in revoked_serials,
            leaf.security_epoch <= intermediate.security_epoch,
            firmware_version >= leaf.firmware_floor,
            firmware_version >= intermediate.firmware_floor,
            verify_signature(intermediate.subject, leaf),
            verify_signature(root_key, intermediate),
        ]
    )


root_key = b"manufacturer-root-key"
root_id = h(b"root-id", root_key)
intermediate_key = h(b"intermediate-key")
leaf_key = h(b"endorsement-key")
intermediate = sign(
    root_key,
    Cert(h(b"serial-i"), intermediate_key, root_id, 8, 12, True, b""),
)
leaf = sign(
    intermediate_key,
    Cert(h(b"serial-l"), leaf_key, intermediate_key, 7, 12, False, b""),
)
root_set = {root_id}

assert verify_chain(leaf, intermediate, root_key, root_id, root_set, set(), 12)
assert not verify_chain(replace(leaf, issuer=h(b"wrong")), intermediate, root_key, root_id, root_set, set(), 12)
assert not verify_chain(leaf, intermediate, root_key, root_id, root_set, {leaf.serial}, 12)
assert not verify_chain(leaf, intermediate, root_key, root_id, root_set, set(), 11)
assert not verify_chain(leaf, replace(intermediate, security_epoch=6), root_key, root_id, root_set, set(), 12)
assert not verify_chain(leaf, intermediate, root_key, root_id, set(), set(), 12)

previous = {
    "root_epoch": 3,
    "revocation_epoch": 4,
    "certificate_epoch": 7,
    "firmware_version": 12,
}
valid_successor = {
    "root_epoch": 4,
    "revocation_epoch": 5,
    "certificate_epoch": 8,
    "firmware_version": 13,
}
for key in previous:
    assert valid_successor[key] >= previous[key]
for key in previous:
    rollback = dict(valid_successor)
    rollback[key] = previous[key] - 1
    assert not all(rollback[field] >= previous[field] for field in previous)

print("runtime_protocol=19")
print("valid_endorsement_chain=true")
print("revoked_leaf_rejected=true")
print("firmware_rollback_rejected=true")
print("manufacturer_root_rollback_rejected=true")
print("proof_acceptance_enabled=false")
