#!/usr/bin/env python3
# Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
# SPDX-License-Identifier: AGPL-3.0-or-later
"""GOVSYS-003C-V predecessor-owned constitutional transition verifier."""

from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import struct
from pathlib import Path
from typing import Any

ROOT_ORACLE_PATH = Path(__file__).with_name('govsys_constitutional_trust_root_identity_v0_1.py')
VECTOR_PATH = Path(__file__).with_name('govsys_003c_transition_verifier_vector_v1.json')

TRANSITION_PROFILE = 'mycelix-constitutional-root-transition-v1-sha256-framed-semantic'
TRANSITION_DOMAIN = b'mycelix/public-institution/constitutional-root-transition/v1'
MATERIAL_PROFILE = 'mycelix-constitutional-root-rotation-authority-ed25519-single-v1'
MATERIAL_DOMAIN = b'mycelix/public-institution/constitutional-root-rotation-authority-material/ed25519-single/v1'
MATERIAL_SUITE = 'ed25519-spki-v1'
PROOF_SUITE = 'ed25519-signature-v1'
NORMAL_ROTATION_PROFILE = 'constitutional-root-rotation-v1'
MAX_NONCE_BYTES = 64
ED25519_SPKI_PREFIX = bytes.fromhex('302a300506032b6570032100')

# Ed25519 constants for a dependency-free conformance verifier.
Q = 2**255 - 19
L = 2**252 + 27742317777372353535851937790883648493
D = (-121665 * pow(121666, Q - 2, Q)) % Q
I = pow(2, (Q - 1) // 4, Q)
IDENTITY = (0, 1)


class ContractError(ValueError):
    pass


def _load_root_oracle():
    spec = importlib.util.spec_from_file_location('govsys003a_root', ROOT_ORACLE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError('cannot load exact Root-A oracle')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


ROOT = _load_root_oracle()


def _frame(raw: bytes) -> bytes:
    return struct.pack('<Q', len(raw)) + raw


def _text(value: str) -> bytes:
    return _frame(value.encode('utf-8'))


def _u64(value: int) -> bytes:
    return _frame(struct.pack('<Q', value))


def _profiled(profile: str, digest_hex: str) -> bytes:
    return _text(profile) + _frame(bytes.fromhex(digest_hex))


def _optional_profiled(profile: str | None, digest_hex: str | None) -> bytes:
    if profile is None or digest_hex is None:
        if profile is not None or digest_hex is not None:
            raise ContractError('optional profiled identity must be fully present or absent')
        return _frame(b'\x00')
    return _frame(b'\x01') + _profiled(profile, digest_hex)


def _hex32(value: Any, field: str) -> str:
    if not isinstance(value, str) or len(value) != 64:
        raise ContractError(f'{field} must be 32-byte hex')
    try:
        raw = bytes.fromhex(value)
    except ValueError as exc:
        raise ContractError(f'{field} must be hex') from exc
    if len(raw) != 32 or raw == bytes(32):
        raise ContractError(f'{field} must be non-zero 32-byte hex')
    return raw.hex()


def _u64_value(value: Any, field: str) -> int:
    if not isinstance(value, int) or isinstance(value, bool) or not (0 <= value <= 0xFFFFFFFFFFFFFFFF):
        raise ContractError(f'{field} must be a u64')
    return value


def _validate_profiles(profiles: Any) -> None:
    expected = {
        'root_identity': ROOT.IDENTITY_PROFILE,
        'source_descriptor': ROOT.SOURCE_DESCRIPTOR_PROFILE,
        'rotation_authority': ROOT.ROTATION_AUTHORITY_PROFILE,
        'authorization_material': MATERIAL_PROFILE,
        'authorization_suite': MATERIAL_SUITE,
        'authorization_proof_suite': PROOF_SUITE,
        'transition': TRANSITION_PROFILE,
    }
    if not isinstance(profiles, dict) or profiles != expected:
        raise ContractError('profile registry does not exact-match the frozen verifier profile')


def _authorization_signature(proof: Any) -> bytes:
    if not isinstance(proof, dict) or set(proof) != {'suite', 'signature_hex'}:
        raise ContractError('invalid authorization proof shape')
    if proof['suite'] != PROOF_SUITE:
        raise ContractError('unsupported authorization proof suite')
    try:
        signature = bytes.fromhex(proof['signature_hex'])
    except (TypeError, ValueError) as exc:
        raise ContractError('signature_hex must be hex') from exc
    if len(signature) != 64:
        raise ContractError('Ed25519 signature must be 64 bytes')
    return signature


def _material_commitment(material: dict[str, Any]) -> str:
    if not isinstance(material, dict) or set(material) != {'profile', 'suite', 'public_key_spki_der_hex'}:
        raise ContractError('invalid authorization material shape')
    if material['profile'] != MATERIAL_PROFILE or material['suite'] != MATERIAL_SUITE:
        raise ContractError('unsupported authorization material profile')
    try:
        der = bytes.fromhex(material['public_key_spki_der_hex'])
    except (TypeError, ValueError) as exc:
        raise ContractError('public_key_spki_der_hex must be hex') from exc
    if len(der) != len(ED25519_SPKI_PREFIX) + 32 or not der.startswith(ED25519_SPKI_PREFIX):
        raise ContractError('public key is not canonical Ed25519 SPKI DER')
    transcript = bytearray(MATERIAL_DOMAIN)
    transcript += _text(MATERIAL_PROFILE)
    transcript += _text(MATERIAL_SUITE)
    transcript += _frame(der)
    return hashlib.sha256(transcript).hexdigest()


def _candidate_bytes(candidate: dict[str, Any]) -> bytes:
    required = {
        'profile',
        'predecessor_root_identity_profile', 'predecessor_root_digest_hex', 'predecessor_generation',
        'predecessor_source_descriptor_profile', 'predecessor_source_descriptor_digest_hex',
        'predecessor_rotation_authority_profile', 'predecessor_rotation_authority_digest_hex',
        'successor_root_identity_profile', 'successor_root_digest_hex', 'successor_generation',
        'successor_source_descriptor_profile', 'successor_source_descriptor_digest_hex',
        'successor_rotation_authority_profile', 'successor_rotation_authority_digest_hex',
        'authorized_at_ms', 'effective_at_ms', 'replay_nonce_hex',
    }
    if not isinstance(candidate, dict) or set(candidate) != required:
        raise ContractError('invalid transition candidate shape')
    if candidate['profile'] != TRANSITION_PROFILE:
        raise ContractError('wrong transition profile')
    for field in (
        'predecessor_root_digest_hex', 'predecessor_source_descriptor_digest_hex',
        'predecessor_rotation_authority_digest_hex', 'successor_root_digest_hex',
        'successor_source_descriptor_digest_hex',
    ):
        _hex32(candidate[field], field)
    if candidate['successor_rotation_authority_profile'] is None:
        if candidate['successor_rotation_authority_digest_hex'] is not None:
            raise ContractError('successor rotation identity must be fully absent')
    else:
        _hex32(candidate['successor_rotation_authority_digest_hex'], 'successor_rotation_authority_digest_hex')
    pred_gen = _u64_value(candidate['predecessor_generation'], 'predecessor_generation')
    succ_gen = _u64_value(candidate['successor_generation'], 'successor_generation')
    authorized = _u64_value(candidate['authorized_at_ms'], 'authorized_at_ms')
    effective = _u64_value(candidate['effective_at_ms'], 'effective_at_ms')
    try:
        nonce = bytes.fromhex(candidate['replay_nonce_hex'])
    except (TypeError, ValueError) as exc:
        raise ContractError('replay_nonce_hex must be hex') from exc
    if not nonce or len(nonce) > MAX_NONCE_BYTES or nonce == bytes(len(nonce)):
        raise ContractError('replay nonce must be non-zero and bounded')

    out = bytearray(TRANSITION_DOMAIN)
    out += _text(TRANSITION_PROFILE)
    out += _profiled(candidate['predecessor_root_identity_profile'], candidate['predecessor_root_digest_hex'])
    out += _u64(pred_gen)
    out += _profiled(candidate['predecessor_source_descriptor_profile'], candidate['predecessor_source_descriptor_digest_hex'])
    out += _profiled(candidate['predecessor_rotation_authority_profile'], candidate['predecessor_rotation_authority_digest_hex'])
    out += _profiled(candidate['successor_root_identity_profile'], candidate['successor_root_digest_hex'])
    out += _u64(succ_gen)
    out += _profiled(candidate['successor_source_descriptor_profile'], candidate['successor_source_descriptor_digest_hex'])
    out += _optional_profiled(candidate['successor_rotation_authority_profile'], candidate['successor_rotation_authority_digest_hex'])
    out += _u64(authorized)
    out += _u64(effective)
    out += _frame(nonce)
    return bytes(out)


def _candidate_digest(candidate: dict[str, Any]) -> str:
    return hashlib.sha256(_candidate_bytes(candidate)).hexdigest()


def _xrecover(y: int) -> int:
    xx = (y * y - 1) * pow(D * y * y + 1, Q - 2, Q) % Q
    x = pow(xx, (Q + 3) // 8, Q)
    if (x * x - xx) % Q != 0:
        x = x * I % Q
    if (x * x - xx) % Q != 0:
        raise ContractError('invalid Ed25519 point')
    return x


def _decode_point(raw: bytes) -> tuple[int, int]:
    if len(raw) != 32:
        raise ContractError('Ed25519 point must be 32 bytes')
    encoded = int.from_bytes(raw, 'little')
    sign = encoded >> 255
    y = encoded & ((1 << 255) - 1)
    if y >= Q:
        raise ContractError('non-canonical Ed25519 y coordinate')
    x = _xrecover(y)
    if (x & 1) != sign:
        x = Q - x
    if x == 0 and sign:
        raise ContractError('non-canonical Ed25519 x sign')
    point = (x, y)
    if _scalar_mult(point, L) != IDENTITY:
        raise ContractError('Ed25519 point is not in the prime-order subgroup')
    return point


def _point_add(p: tuple[int, int], q: tuple[int, int]) -> tuple[int, int]:
    x1, y1 = p
    x2, y2 = q
    prod = D * x1 * x2 * y1 * y2 % Q
    x3 = (x1 * y2 + x2 * y1) * pow((1 + prod) % Q, Q - 2, Q) % Q
    y3 = (y1 * y2 + x1 * x2) * pow((1 - prod) % Q, Q - 2, Q) % Q
    return x3, y3


def _scalar_mult(point: tuple[int, int], scalar: int) -> tuple[int, int]:
    result = IDENTITY
    addend = point
    while scalar:
        if scalar & 1:
            result = _point_add(result, addend)
        addend = _point_add(addend, addend)
        scalar >>= 1
    return result


BASE = (_xrecover(4 * pow(5, Q - 2, Q) % Q), 4 * pow(5, Q - 2, Q) % Q)
if BASE[0] & 1:
    BASE = (Q - BASE[0], BASE[1])


def _verify_ed25519(spki_der: bytes, message: bytes, signature: bytes) -> None:
    if len(spki_der) != len(ED25519_SPKI_PREFIX) + 32 or not spki_der.startswith(ED25519_SPKI_PREFIX):
        raise ContractError('invalid Ed25519 SPKI DER')
    if len(signature) != 64:
        raise ContractError('Ed25519 signature must be 64 bytes')
    public = spki_der[len(ED25519_SPKI_PREFIX):]
    r_raw = signature[:32]
    s = int.from_bytes(signature[32:], 'little')
    if s >= L:
        raise ContractError('non-canonical Ed25519 scalar')
    a = _decode_point(public)
    r = _decode_point(r_raw)
    h = int.from_bytes(hashlib.sha512(r_raw + public + message).digest(), 'little') % L
    if _scalar_mult(BASE, s) != _point_add(r, _scalar_mult(a, h)):
        raise ContractError('Ed25519 authorization verification failed')


def _assert_semantic_relation(pred: dict[str, Any], succ: dict[str, Any], candidate: dict[str, Any]) -> dict[str, str | None]:
    ROOT.validate_root(pred)
    ROOT.validate_root(succ)
    if pred['rotation_mode'] != 'predecessor-authorized' or pred['rotation_profile'] != NORMAL_ROTATION_PROFILE:
        raise ContractError('predecessor does not authorize normal rotation')
    if pred['generation'] == 0xFFFFFFFFFFFFFFFF or succ['generation'] != pred['generation'] + 1:
        raise ContractError('successor generation must be predecessor + 1')
    pred_id = ROOT.identity_hex(pred)
    if succ['predecessor_root_digest_hex'].lower() != pred_id:
        raise ContractError('successor predecessor digest mismatch')

    preserved = (
        'protocol_version', 'institution_id', 'jurisdiction_id', 'bootstrap_mode', 'bootstrap_profile',
        'authoritative_root_source_ref', 'root_coverage_profile', 'root_source_verification_profile',
    )
    for field in preserved:
        if succ[field] != pred[field]:
            raise ContractError(f'normal rotation cannot change {field}')

    authorized = candidate['authorized_at_ms']
    effective = candidate['effective_at_ms']
    if authorized < pred['valid_from_ms']:
        raise ContractError('authorization predates predecessor validity')
    if effective < authorized:
        raise ContractError('effective time predates authorization')
    if pred['expires_at_ms'] is not None and (authorized >= pred['expires_at_ms'] or effective >= pred['expires_at_ms']):
        raise ContractError('transition is outside predecessor validity')
    if succ['valid_from_ms'] != effective:
        raise ContractError('successor valid_from must equal transition effective time')

    pred_source = ROOT.source_descriptor_hex(pred)
    succ_source = ROOT.source_descriptor_hex(succ)
    pred_rotation = ROOT.rotation_authority_hex(pred)
    succ_rotation = ROOT.rotation_authority_hex(succ)
    if pred_rotation is None:
        raise ContractError('predecessor rotation identity unexpectedly absent')

    expected = {
        'profile': TRANSITION_PROFILE,
        'predecessor_root_identity_profile': ROOT.IDENTITY_PROFILE,
        'predecessor_root_digest_hex': pred_id,
        'predecessor_generation': pred['generation'],
        'predecessor_source_descriptor_profile': ROOT.SOURCE_DESCRIPTOR_PROFILE,
        'predecessor_source_descriptor_digest_hex': pred_source,
        'predecessor_rotation_authority_profile': ROOT.ROTATION_AUTHORITY_PROFILE,
        'predecessor_rotation_authority_digest_hex': pred_rotation,
        'successor_root_identity_profile': ROOT.IDENTITY_PROFILE,
        'successor_root_digest_hex': ROOT.identity_hex(succ),
        'successor_generation': succ['generation'],
        'successor_source_descriptor_profile': ROOT.SOURCE_DESCRIPTOR_PROFILE,
        'successor_source_descriptor_digest_hex': succ_source,
        'successor_rotation_authority_profile': None if succ_rotation is None else ROOT.ROTATION_AUTHORITY_PROFILE,
        'successor_rotation_authority_digest_hex': succ_rotation,
        'authorized_at_ms': authorized,
        'effective_at_ms': effective,
        'replay_nonce_hex': candidate['replay_nonce_hex'].lower(),
    }
    normalized = copy.deepcopy(candidate)
    for field in (
        'predecessor_root_digest_hex', 'predecessor_source_descriptor_digest_hex',
        'predecessor_rotation_authority_digest_hex', 'successor_root_digest_hex',
        'successor_source_descriptor_digest_hex', 'successor_rotation_authority_digest_hex',
        'replay_nonce_hex',
    ):
        if normalized[field] is not None:
            normalized[field] = normalized[field].lower()
    if normalized != expected:
        raise ContractError('candidate does not exactly rebind locally recomputed Root-A semantics')
    return {'pred_id': pred_id, 'pred_source': pred_source, 'pred_rotation': pred_rotation,
            'succ_id': expected['successor_root_digest_hex'], 'succ_source': succ_source, 'succ_rotation': succ_rotation}


def qualify_transition(bundle: dict[str, Any]) -> dict[str, Any]:
    required = {
        'profiles',
        'predecessor_root',
        'successor_root',
        'authorization_material',
        'authorization_proof',
        'candidate',
    }
    if not isinstance(bundle, dict) or set(bundle) != required:
        raise ContractError('invalid transition qualification input shape')
    _validate_profiles(bundle['profiles'])

    pred = bundle['predecessor_root']
    succ = bundle['successor_root']
    candidate = bundle['candidate']
    ids = _assert_semantic_relation(pred, succ, candidate)

    material = bundle['authorization_material']
    commitment = _material_commitment(material)
    if commitment != pred['rotation_authority_anchor_digest_hex'].lower():
        raise ContractError('authorization material does not match predecessor anchor')

    message = _candidate_bytes(candidate)
    signature = _authorization_signature(bundle['authorization_proof'])
    der = bytes.fromhex(material['public_key_spki_der_hex'])
    _verify_ed25519(der, message, signature)

    return {
        'transition_identity_profile': TRANSITION_PROFILE,
        'transition_identity_digest_hex': hashlib.sha256(message).hexdigest(),
        'predecessor_root_digest_hex': ids['pred_id'],
        'predecessor_source_descriptor_digest_hex': ids['pred_source'],
        'predecessor_rotation_authority_digest_hex': ids['pred_rotation'],
        'successor_root_digest_hex': ids['succ_id'],
        'successor_source_descriptor_digest_hex': ids['succ_source'],
        'successor_rotation_authority_digest_hex': ids['succ_rotation'],
        'authorization_material_commitment_hex': commitment,
        'authorization_proof_digest_hex': hashlib.sha256(signature).hexdigest(),
        'authorized_at_ms': candidate['authorized_at_ms'],
        'effective_at_ms': candidate['effective_at_ms'],
        'replay_nonce_hex': candidate['replay_nonce_hex'].lower(),
        'grants_currentness': False,
        'grants_effect_authority': False,
    }


def assert_no_replay_conflicts(evidence: list[dict[str, Any]]) -> None:
    seen: dict[tuple[str, str], str] = {}
    for item in evidence:
        key = (item['predecessor_root_digest_hex'], item['replay_nonce_hex'])
        identity = item['transition_identity_digest_hex']
        prior = seen.get(key)
        if prior is not None and prior != identity:
            raise ContractError('replay nonce reused across distinct transition identity')
        seen[key] = identity


def _expect_error(bundle: dict[str, Any]) -> None:
    try:
        qualify_transition(bundle)
    except (ContractError, ValueError):
        return
    raise AssertionError('expected transition qualification rejection')


def self_test() -> None:
    vector = json.loads(VECTOR_PATH.read_text(encoding='utf-8'))
    wrapper_required = {
        'profiles', 'predecessor_root', 'successor_root', 'authorization_material',
        'authorization_proof', 'candidate', 'successor_authorization_material', 'expected',
    }
    if not isinstance(vector, dict) or set(vector) != wrapper_required:
        raise AssertionError('transition conformance vector wrapper has unexpected shape')

    qualify_input = {
        key: copy.deepcopy(vector[key])
        for key in (
            'profiles', 'predecessor_root', 'successor_root',
            'authorization_material', 'authorization_proof', 'candidate',
        )
    }
    evidence = qualify_transition(qualify_input)
    expected = vector['expected']
    assert evidence['authorization_material_commitment_hex'] == expected['authorization_material_commitment_hex']
    assert evidence['transition_identity_digest_hex'] == expected['candidate_digest_hex']
    assert _candidate_bytes(vector['candidate']).hex() == expected['candidate_canonical_bytes_hex']
    assert _material_commitment(vector['successor_authorization_material']) == expected['successor_material_commitment_hex']
    assert not evidence['grants_currentness'] and not evidence['grants_effect_authority']

    bad = copy.deepcopy(qualify_input)
    bad['authorization_material'] = copy.deepcopy(vector['successor_authorization_material'])
    _expect_error(bad)

    bad = copy.deepcopy(qualify_input)
    sig = bytearray.fromhex(bad['authorization_proof']['signature_hex'])
    sig[0] ^= 1
    bad['authorization_proof']['signature_hex'] = sig.hex()
    _expect_error(bad)

    bad = copy.deepcopy(qualify_input)
    bad['authorization_proof']['suite'] = 'other-signature-suite'
    _expect_error(bad)

    bad = copy.deepcopy(qualify_input)
    bad['profiles']['authorization_material'] = 'other-material-profile'
    _expect_error(bad)

    bad = copy.deepcopy(qualify_input)
    bad['candidate']['predecessor_root_digest_hex'] = '77' * 32
    _expect_error(bad)

    bad = copy.deepcopy(qualify_input)
    bad['candidate']['successor_root_digest_hex'] = '77' * 32
    _expect_error(bad)

    bad = copy.deepcopy(qualify_input)
    bad['successor_root']['generation'] = 2
    _expect_error(bad)

    bad = copy.deepcopy(qualify_input)
    bad['successor_root']['predecessor_root_digest_hex'] = '77' * 32
    _expect_error(bad)

    bad = copy.deepcopy(qualify_input)
    bad['successor_root']['authoritative_root_source_ref'] = 'registry:constitutional-root:other-city'
    _expect_error(bad)

    bad = copy.deepcopy(qualify_input)
    bad['successor_root']['root_coverage_profile'] = 'other-coverage-v1'
    _expect_error(bad)

    bad = copy.deepcopy(qualify_input)
    bad['successor_root']['root_source_verification_profile'] = 'other-source-verifier-v1'
    _expect_error(bad)

    # Source verification anchor rotation is allowed when bound by the exact successor identity.
    assert qualify_transition(copy.deepcopy(qualify_input))['successor_source_descriptor_digest_hex'] != evidence['predecessor_source_descriptor_digest_hex']

    bad = copy.deepcopy(qualify_input)
    bad['candidate']['authorized_at_ms'] = bad['predecessor_root']['valid_from_ms'] - 1
    _expect_error(bad)

    bad = copy.deepcopy(qualify_input)
    bad['candidate']['effective_at_ms'] = bad['candidate']['authorized_at_ms'] - 1
    _expect_error(bad)

    bad = copy.deepcopy(qualify_input)
    bad['candidate']['authorized_at_ms'] = bad['predecessor_root']['expires_at_ms']
    _expect_error(bad)

    bad = copy.deepcopy(qualify_input)
    bad['candidate']['effective_at_ms'] = bad['predecessor_root']['expires_at_ms']
    _expect_error(bad)

    bad = copy.deepcopy(qualify_input)
    bad['successor_root']['valid_from_ms'] += 1
    _expect_error(bad)

    bad = copy.deepcopy(qualify_input)
    bad['candidate']['replay_nonce_hex'] = '00' * 32
    _expect_error(bad)

    bad = copy.deepcopy(qualify_input)
    bad['predecessor_root']['rotation_profile'] = 'constitutional-root-rotation-v2'
    _expect_error(bad)

    bad = copy.deepcopy(qualify_input)
    bad['predecessor_root']['rotation_authority_anchor_digest_hex'] = bad['predecessor_root']['root_source_anchor_digest_hex']
    _expect_error(bad)

    duplicate = copy.deepcopy(evidence)
    assert_no_replay_conflicts([evidence, duplicate])
    conflict = copy.deepcopy(evidence)
    conflict['transition_identity_digest_hex'] = '77' * 32
    try:
        assert_no_replay_conflicts([evidence, conflict])
    except ContractError:
        pass
    else:
        raise AssertionError('expected replay nonce conflict')

    print('GOVSYS-003C-V transition verifier PASS:', evidence['transition_identity_digest_hex'])


if __name__ == '__main__':
    self_test()
