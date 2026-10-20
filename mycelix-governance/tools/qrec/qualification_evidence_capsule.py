#!/usr/bin/env python3
"""Deterministic QREC evidence/authentication capsule tooling.

Structural artifact verification only. No GitHub, Sigstore, Xenia, signer,
qualification, provider, or runtime authority is established by this tool.
"""

from __future__ import annotations

import argparse
import hashlib
import io
import json
import os
from pathlib import Path
import re
import stat
import sys
import tarfile
import tempfile
from typing import Mapping, Sequence

EVIDENCE_PROFILE = "mycelix-governance-qrec-evidence-payload-v1"
AUTH_PROFILE = "mycelix-governance-qrec-auth-capsule-v1"
MANIFEST_SCHEMA = "mycelix.qrec.authentication-capsule-manifest.v1"

EVIDENCE_NAME = "evidence.payload.tar"
RECEIPT_NAME = "qualification-receipt.json"
MANIFEST_NAME = "authentication-manifest.json"
AUTH_MEMBER_NAMES = (MANIFEST_NAME, EVIDENCE_NAME, RECEIPT_NAME)
RESERVED_EVIDENCE_NAMES = frozenset(AUTH_MEMBER_NAMES)

MAX_MEMBERS = 256
MAX_NAME_BYTES = 100
MAX_MEMBER_BYTES = 64 * 1024 * 1024
MAX_TOTAL_BYTES = 512 * 1024 * 1024
MAX_RECEIPT_BYTES = 1024 * 1024
_SAFE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._/-]*$")


class CapsuleError(ValueError):
    pass


def sha256_tagged(data: bytes) -> str:
    return f"sha256:{hashlib.sha256(data).hexdigest()}"


def canonical_json_bytes(value: object) -> bytes:
    try:
        return (
            json.dumps(
                value,
                ensure_ascii=False,
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            ).encode("utf-8")
            + b"\n"
        )
    except (TypeError, ValueError) as exc:
        raise CapsuleError(f"value is not canonical-JSON encodable: {exc}") from exc


def validate_member_name(name: str, *, evidence_member: bool) -> None:
    encoded = name.encode("utf-8")
    if not name or len(encoded) > MAX_NAME_BYTES:
        raise CapsuleError(f"member name must be 1..={MAX_NAME_BYTES} UTF-8 bytes")
    if not _SAFE_NAME.fullmatch(name):
        raise CapsuleError(f"member name is outside the portable ASCII profile: {name!r}")
    if name.startswith("/") or name.startswith("./") or name.endswith("/"):
        raise CapsuleError(f"non-canonical member name: {name!r}")
    parts = name.split("/")
    if any(part in ("", ".", "..") for part in parts):
        raise CapsuleError(f"unsafe member name: {name!r}")
    if evidence_member and name in RESERVED_EVIDENCE_NAMES:
        raise CapsuleError(f"reserved authentication member name: {name!r}")


def validate_member_set(members: Sequence[tuple[str, bytes]], *, evidence: bool) -> None:
    if not members or len(members) > MAX_MEMBERS:
        raise CapsuleError(f"member count must be 1..={MAX_MEMBERS}")
    seen: set[str] = set()
    total = 0
    for name, body in members:
        validate_member_name(name, evidence_member=evidence)
        if name in seen:
            raise CapsuleError(f"duplicate member name: {name}")
        seen.add(name)
        if len(body) > MAX_MEMBER_BYTES:
            raise CapsuleError(f"member too large: {name}")
        total += len(body)
        if total > MAX_TOTAL_BYTES:
            raise CapsuleError("total member bytes exceed profile limit")


def make_tar(members: Sequence[tuple[str, bytes]]) -> bytes:
    buf = io.BytesIO()
    try:
        with tarfile.open(fileobj=buf, mode="w", format=tarfile.USTAR_FORMAT) as tf:
            for name, body in sorted(members, key=lambda item: item[0].encode("ascii")):
                info = tarfile.TarInfo(name)
                info.size = len(body)
                info.mtime = 0
                info.uid = 0
                info.gid = 0
                info.uname = ""
                info.gname = ""
                info.mode = 0o644
                info.type = tarfile.REGTYPE
                info.pax_headers = {}
                tf.addfile(info, io.BytesIO(body))
    except (tarfile.TarError, ValueError, OSError) as exc:
        raise CapsuleError(f"could not build canonical USTAR: {exc}") from exc
    return buf.getvalue()


def read_tar_exact(
    data: bytes, *, expected_names: Sequence[str] | None = None
) -> list[tuple[str, bytes]]:
    if not data:
        raise CapsuleError("archive is empty")
    if len(data) > MAX_TOTAL_BYTES + (MAX_MEMBERS + 32) * 10240:
        raise CapsuleError("encoded archive exceeds profile limit")
    members: list[tuple[str, bytes]] = []
    seen: set[str] = set()
    try:
        with tarfile.open(fileobj=io.BytesIO(data), mode="r:") as tf:
            infos = tf.getmembers()
            if not infos or len(infos) > MAX_MEMBERS:
                raise CapsuleError("archive member count is out of bounds")
            for info in infos:
                name = info.name
                validate_member_name(name, evidence_member=False)
                if name in seen:
                    raise CapsuleError(f"duplicate archive member: {name}")
                seen.add(name)
                if not info.isfile() or info.type != tarfile.REGTYPE:
                    raise CapsuleError(f"non-regular archive member: {name}")
                if (
                    info.mtime != 0
                    or info.uid != 0
                    or info.gid != 0
                    or info.uname != ""
                    or info.gname != ""
                    or (info.mode & 0o7777) != 0o644
                    or info.pax_headers
                ):
                    raise CapsuleError(f"non-canonical USTAR metadata: {name}")
                if info.size < 0 or info.size > MAX_MEMBER_BYTES:
                    raise CapsuleError(f"archive member size out of bounds: {name}")
                reader = tf.extractfile(info)
                if reader is None:
                    raise CapsuleError(f"archive member body unavailable: {name}")
                body = reader.read(MAX_MEMBER_BYTES + 1)
                if len(body) != info.size:
                    raise CapsuleError(f"archive member size mismatch: {name}")
                members.append((name, body))
    except CapsuleError:
        raise
    except (tarfile.TarError, OSError) as exc:
        raise CapsuleError(f"invalid USTAR archive: {exc}") from exc

    names = [name for name, _ in members]
    canonical_names = sorted(names, key=lambda name: name.encode("ascii"))
    if names != canonical_names:
        raise CapsuleError("archive member order is not canonical")
    if expected_names is not None:
        expected = sorted(expected_names, key=lambda name: name.encode("ascii"))
        if names != expected:
            raise CapsuleError(
                f"archive member census mismatch: expected {expected}, got {names}"
            )
    if make_tar(members) != data:
        raise CapsuleError("archive bytes are not the canonical USTAR projection")
    return members


def pack_evidence_bytes(members: Sequence[tuple[str, bytes]]) -> bytes:
    validate_member_set(members, evidence=True)
    return make_tar(members)


def verify_evidence_bytes(data: bytes) -> dict[str, object]:
    members = read_tar_exact(data)
    validate_member_set(members, evidence=True)
    return {
        "profile": EVIDENCE_PROFILE,
        "byte_length": len(data),
        "sha256": sha256_tagged(data),
        "member_count": len(members),
        "members": [
            {"name": name, "byte_length": len(body), "sha256": sha256_tagged(body)}
            for name, body in members
        ],
    }


def _reject_duplicate_pairs(pairs: list[tuple[str, object]]) -> dict[str, object]:
    out: dict[str, object] = {}
    for key, value in pairs:
        if key in out:
            raise CapsuleError(f"duplicate JSON key: {key}")
        out[key] = value
    return out


def parse_receipt(receipt_bytes: bytes) -> Mapping[str, object]:
    if not receipt_bytes or len(receipt_bytes) > MAX_RECEIPT_BYTES:
        raise CapsuleError("receipt bytes must be non-empty and bounded")
    try:
        value = json.loads(
            receipt_bytes.decode("utf-8"), object_pairs_hook=_reject_duplicate_pairs
        )
    except CapsuleError:
        raise
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise CapsuleError(f"receipt is not valid UTF-8 JSON: {exc}") from exc
    if not isinstance(value, dict):
        raise CapsuleError("receipt JSON must be an object")
    required = {
        "schema_version",
        "dependency_id",
        "issuer_id",
        "semantic_head",
        "verifier_head",
        "run_id",
        "run_attempt",
        "artifact_digest",
        "receipt_commitment",
        "navigation",
    }
    if set(value) != required:
        raise CapsuleError("receipt top-level field census mismatch")
    if value["schema_version"] != 1:
        raise CapsuleError("unsupported QREC schema_version")
    if not isinstance(value["artifact_digest"], str):
        raise CapsuleError("receipt artifact_digest must be a string")
    commitment = value["receipt_commitment"]
    if (
        not isinstance(commitment, str)
        or not commitment.startswith("blake3-256:")
        or len(commitment) != len("blake3-256:") + 64
        or any(ch not in "0123456789abcdef" for ch in commitment[len("blake3-256:") :])
    ):
        raise CapsuleError("receipt_commitment must be canonical blake3-256")
    if not isinstance(value["navigation"], dict):
        raise CapsuleError("receipt navigation must be an object")
    return value


def auth_manifest(
    evidence_bytes: bytes, receipt_bytes: bytes, receipt: Mapping[str, object]
) -> dict[str, object]:
    return {
        "schema": MANIFEST_SCHEMA,
        "profile": AUTH_PROFILE,
        "evidence_payload": {
            "name": EVIDENCE_NAME,
            "byte_length": len(evidence_bytes),
            "sha256": sha256_tagged(evidence_bytes),
        },
        "qualification_receipt": {
            "name": RECEIPT_NAME,
            "byte_length": len(receipt_bytes),
            "sha256": sha256_tagged(receipt_bytes),
            "receipt_commitment": receipt["receipt_commitment"],
        },
    }


def pack_auth_bytes(evidence_bytes: bytes, receipt_bytes: bytes) -> bytes:
    verify_evidence_bytes(evidence_bytes)
    receipt = parse_receipt(receipt_bytes)
    evidence_digest = sha256_tagged(evidence_bytes)
    if receipt["artifact_digest"] != evidence_digest:
        raise CapsuleError("receipt artifact_digest does not bind exact E bytes")
    manifest_bytes = canonical_json_bytes(
        auth_manifest(evidence_bytes, receipt_bytes, receipt)
    )
    members = [
        (MANIFEST_NAME, manifest_bytes),
        (EVIDENCE_NAME, evidence_bytes),
        (RECEIPT_NAME, receipt_bytes),
    ]
    validate_member_set(members, evidence=False)
    return make_tar(members)


def verify_auth_bytes(data: bytes) -> dict[str, object]:
    by_name = dict(read_tar_exact(data, expected_names=AUTH_MEMBER_NAMES))
    manifest_bytes = by_name[MANIFEST_NAME]
    try:
        manifest = json.loads(
            manifest_bytes.decode("utf-8"), object_pairs_hook=_reject_duplicate_pairs
        )
    except CapsuleError:
        raise
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise CapsuleError(f"manifest is invalid UTF-8 JSON: {exc}") from exc
    if not isinstance(manifest, dict) or canonical_json_bytes(manifest) != manifest_bytes:
        raise CapsuleError("manifest must be canonical JSON object")

    if set(manifest) != {"schema", "profile", "evidence_payload", "qualification_receipt"}:
        raise CapsuleError("manifest top-level field census mismatch")
    if manifest["schema"] != MANIFEST_SCHEMA or manifest["profile"] != AUTH_PROFILE:
        raise CapsuleError("manifest schema/profile mismatch")

    evidence_info = manifest["evidence_payload"]
    receipt_info = manifest["qualification_receipt"]
    if not isinstance(evidence_info, dict) or set(evidence_info) != {
        "name",
        "byte_length",
        "sha256",
    }:
        raise CapsuleError("manifest evidence_payload shape mismatch")
    if not isinstance(receipt_info, dict) or set(receipt_info) != {
        "name",
        "byte_length",
        "sha256",
        "receipt_commitment",
    }:
        raise CapsuleError("manifest qualification_receipt shape mismatch")
    if evidence_info["name"] != EVIDENCE_NAME or receipt_info["name"] != RECEIPT_NAME:
        raise CapsuleError("manifest member name mismatch")

    evidence_bytes = by_name[EVIDENCE_NAME]
    receipt_bytes = by_name[RECEIPT_NAME]
    if (
        evidence_info["byte_length"] != len(evidence_bytes)
        or evidence_info["sha256"] != sha256_tagged(evidence_bytes)
    ):
        raise CapsuleError("E length/digest mismatch")
    if (
        receipt_info["byte_length"] != len(receipt_bytes)
        or receipt_info["sha256"] != sha256_tagged(receipt_bytes)
    ):
        raise CapsuleError("R length/digest mismatch")

    evidence_summary = verify_evidence_bytes(evidence_bytes)
    receipt = parse_receipt(receipt_bytes)
    if receipt["artifact_digest"] != evidence_info["sha256"]:
        raise CapsuleError("R does not bind exact E")
    if receipt["receipt_commitment"] != receipt_info["receipt_commitment"]:
        raise CapsuleError("R commitment does not match manifest")
    if manifest != auth_manifest(evidence_bytes, receipt_bytes, receipt):
        raise CapsuleError("manifest differs from independent reconstruction")

    return {
        "profile": AUTH_PROFILE,
        "byte_length": len(data),
        "sha256": sha256_tagged(data),
        "evidence": evidence_summary,
        "receipt_commitment": receipt["receipt_commitment"],
    }


def read_regular_file(path: Path, max_bytes: int) -> bytes:
    try:
        st = path.lstat()
    except OSError as exc:
        raise CapsuleError(f"cannot stat {path}: {exc}") from exc
    if not stat.S_ISREG(st.st_mode):
        raise CapsuleError(f"source is not an ordinary file: {path}")
    if st.st_size < 0 or st.st_size > max_bytes:
        raise CapsuleError(f"source exceeds {max_bytes} bytes: {path}")
    try:
        with path.open("rb") as handle:
            data = handle.read(max_bytes + 1)
    except OSError as exc:
        raise CapsuleError(f"cannot read {path}: {exc}") from exc
    if len(data) > max_bytes:
        raise CapsuleError(f"source exceeds {max_bytes} bytes: {path}")
    return data


def parse_member_arg(raw: str) -> tuple[str, Path]:
    if "=" not in raw:
        raise CapsuleError("--member requires NAME=PATH")
    name, source = raw.split("=", 1)
    validate_member_name(name, evidence_member=True)
    if not source:
        raise CapsuleError("member source path must be non-empty")
    return name, Path(source)


def atomic_write(path: Path, data: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(f".{path.name}.tmp-{os.getpid()}")
    try:
        with tmp.open("wb") as handle:
            handle.write(data)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(tmp, path)
    finally:
        try:
            tmp.unlink()
        except FileNotFoundError:
            pass


def sample_receipt(evidence_digest: str) -> bytes:
    return canonical_json_bytes(
        {
            "schema_version": 1,
            "dependency_id": "test:qualification",
            "issuer_id": "github-repository:1176351975",
            "semantic_head": "1" * 40,
            "verifier_head": "2" * 40,
            "run_id": 123,
            "run_attempt": 1,
            "artifact_digest": evidence_digest,
            "receipt_commitment": "blake3-256:" + "3" * 64,
            "navigation": {},
        }
    )


def self_test() -> None:
    logical = [("b/result.txt", b"PASS\n"), ("a/source.txt", b"alpha\n")]
    e1 = pack_evidence_bytes(logical)
    e2 = pack_evidence_bytes(list(reversed(logical)))
    assert e1 == e2
    assert verify_evidence_bytes(e1)["member_count"] == 2

    changed = pack_evidence_bytes(
        [("b/result.txt", b"FAIL\n"), ("a/source.txt", b"alpha\n")]
    )
    assert changed != e1

    receipt = sample_receipt(sha256_tagged(e1))
    a1 = pack_auth_bytes(e1, receipt)
    a2 = pack_auth_bytes(e1, receipt)
    assert a1 == a2
    assert verify_auth_bytes(a1)["receipt_commitment"] == "blake3-256:" + "3" * 64

    stale = json.loads(receipt)
    stale["artifact_digest"] = sha256_tagged(changed)
    try:
        pack_auth_bytes(e1, canonical_json_bytes(stale))
    except CapsuleError:
        pass
    else:
        raise AssertionError("stale receipt accepted")

    for bad_name in (RECEIPT_NAME, "../escape", "/absolute", "a\\b", "a//b"):
        try:
            pack_evidence_bytes([(bad_name, b"x")])
        except CapsuleError:
            pass
        else:
            raise AssertionError(f"unsafe/reserved evidence name accepted: {bad_name}")

    extra = make_tar(read_tar_exact(a1) + [("unexpected.txt", b"x")])
    try:
        verify_auth_bytes(extra)
    except CapsuleError:
        pass
    else:
        raise AssertionError("extra A member accepted")

    parts = dict(read_tar_exact(a1))
    mutated_e = bytearray(parts[EVIDENCE_NAME])
    mutated_e[0] ^= 1
    mutated_a = make_tar(
        [
            (MANIFEST_NAME, parts[MANIFEST_NAME]),
            (EVIDENCE_NAME, bytes(mutated_e)),
            (RECEIPT_NAME, parts[RECEIPT_NAME]),
        ]
    )
    try:
        verify_auth_bytes(mutated_a)
    except CapsuleError:
        pass
    else:
        raise AssertionError("mutated E accepted")

    duplicate_receipt = receipt.replace(
        b'"artifact_digest":', b'"artifact_digest":"sha256:' + b"0" * 64 + b'","artifact_digest":', 1
    )
    try:
        parse_receipt(duplicate_receipt)
    except CapsuleError:
        pass
    else:
        raise AssertionError("duplicate JSON key accepted")

    with tempfile.TemporaryDirectory() as tmpdir:
        target = Path(tmpdir) / "target"
        target.write_bytes(b"x")
        link = Path(tmpdir) / "link"
        try:
            link.symlink_to(target)
            try:
                read_regular_file(link, 16)
            except CapsuleError:
                pass
            else:
                raise AssertionError("symlink accepted")
        except (OSError, NotImplementedError):
            pass

    print("QREC-002A deterministic capsule self-test: PASS")


def cmd_pack_evidence(args: argparse.Namespace) -> None:
    specs = [parse_member_arg(raw) for raw in args.member]
    if len({name for name, _ in specs}) != len(specs):
        raise CapsuleError("duplicate --member name")
    members = [(name, read_regular_file(path, MAX_MEMBER_BYTES)) for name, path in specs]
    data = pack_evidence_bytes(members)
    atomic_write(Path(args.output), data)
    print(json.dumps(verify_evidence_bytes(data), sort_keys=True))


def cmd_pack_auth(args: argparse.Namespace) -> None:
    evidence = read_regular_file(Path(args.evidence), MAX_TOTAL_BYTES)
    receipt = read_regular_file(Path(args.receipt), MAX_RECEIPT_BYTES)
    data = pack_auth_bytes(evidence, receipt)
    atomic_write(Path(args.output), data)
    print(json.dumps(verify_auth_bytes(data), sort_keys=True))


def cmd_verify_auth(args: argparse.Namespace) -> None:
    data = read_regular_file(
        Path(args.input), MAX_TOTAL_BYTES + (MAX_MEMBERS + 32) * 10240
    )
    print(json.dumps(verify_auth_bytes(data), sort_keys=True))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("pack-evidence")
    p.add_argument("--output", required=True)
    p.add_argument("--member", action="append", required=True, help="NAME=PATH")
    p.set_defaults(func=cmd_pack_evidence)

    p = sub.add_parser("pack-auth")
    p.add_argument("--evidence", required=True)
    p.add_argument("--receipt", required=True)
    p.add_argument("--output", required=True)
    p.set_defaults(func=cmd_pack_auth)

    p = sub.add_parser("verify-auth")
    p.add_argument("--input", required=True)
    p.set_defaults(func=cmd_verify_auth)

    p = sub.add_parser("self-test")
    p.set_defaults(func=lambda _args: self_test())
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        args.func(args)
    except CapsuleError as exc:
        print(f"QREC capsule error: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
