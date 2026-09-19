#!/usr/bin/env python3
"""Verify the frozen JIT-1B VM candidate payload and isolation contract."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import tomllib

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
LOCK = tomllib.loads((HERE / "JIT1B.lock").read_text())
DOMAIN = b"MYCELIX-JIT-QUAL/V0.1/JIT1B-PAYLOAD"

EXPECTED_PARENT = "4f54129e03228c01662d201fb433bce763205921"
EXPECTED_PARENT_TREE = "707b8ef28fa0bd588e29120ca9614558379dd165"
EXPECTED_JIT1A_MANIFEST = "a8646d121a73bdaf7c9c1b022f6db27dc75514e86ed29f7dc411092470cd6762"
EXPECTED_NIXPKGS_REV = "9ae611a455b90cf061d8f332b977e387bda8e1ca"
EXPECTED_NIXPKGS_NAR = "sha256-md8WlXOlfnIeHeOScMTTHFyf2d6iaTwPl2apR5EQ3P4="
EXPECTED_FILES = {
    "flake.lock",
    "flake.nix",
    "vm/README.md",
    "vm/check_lock.py",
    "vm/jit-1b-vm.nix",
}


def git_blob_oid(data: bytes) -> str:
    return hashlib.sha1(f"blob {len(data)}\0".encode("ascii") + data).hexdigest()


def fail(message: str) -> None:
    raise SystemExit(message)


if LOCK.get("protocol") != "MYCELIX-JIT-QUAL/JIT1B":
    fail("unexpected JIT-1B protocol")
if LOCK.get("version") != 1:
    fail("unexpected JIT-1B lock version")
if LOCK.get("status") != "nix-execution-pending":
    fail("unexpected JIT-1B status")
if LOCK.get("payload_domain") != DOMAIN.decode("ascii"):
    fail("unexpected JIT-1B payload domain")
if LOCK.get("parent_jit1a_commit") != EXPECTED_PARENT:
    fail("JIT-1A parent mismatch")
if LOCK.get("parent_jit1a_tree") != EXPECTED_PARENT_TREE:
    fail("JIT-1A parent tree mismatch")
if LOCK.get("jit1a_payload_manifest_sha256") != EXPECTED_JIT1A_MANIFEST:
    fail("JIT-1A payload manifest mismatch")
if LOCK.get("nixpkgs_rev") != EXPECTED_NIXPKGS_REV:
    fail("JIT-1B lock nixpkgs revision mismatch")
if LOCK.get("nixpkgs_nar_hash") != EXPECTED_NIXPKGS_NAR:
    fail("JIT-1B lock nixpkgs NAR hash mismatch")

files = LOCK.get("files")
if not isinstance(files, dict):
    fail("missing JIT-1B payload files")
if set(files) != EXPECTED_FILES:
    fail("unexpected JIT-1B payload file set")

actual: list[tuple[str, str]] = []
for rel, expected_oid in sorted(files.items()):
    path = ROOT / rel
    if not path.is_file():
        fail(f"missing JIT-1B payload file: {rel}")
    oid = git_blob_oid(path.read_bytes())
    if oid != expected_oid:
        fail(f"Git blob mismatch: {rel}")
    actual.append((rel, oid))

h = hashlib.sha256()
h.update(DOMAIN)
for rel, oid in actual:
    name = rel.encode("utf-8")
    h.update(len(name).to_bytes(4, "big"))
    h.update(name)
    h.update(bytes.fromhex(oid))
manifest = h.hexdigest()
if manifest != LOCK.get("payload_manifest_sha256"):
    fail("JIT-1B payload manifest SHA-256 mismatch")

flake_lock = json.loads((ROOT / "flake.lock").read_text())
nixpkgs = flake_lock.get("nodes", {}).get("nixpkgs", {}).get("locked", {})
if nixpkgs.get("rev") != EXPECTED_NIXPKGS_REV:
    fail("unexpected nixpkgs revision")
if nixpkgs.get("narHash") != EXPECTED_NIXPKGS_NAR:
    fail("unexpected nixpkgs NAR hash")
if flake_lock.get("root") != "root" or flake_lock.get("version") != 7:
    fail("unexpected flake.lock root/version")

flake = (ROOT / "flake.nix").read_text()
for required in [
    'system = "x86_64-linux";',
    "checks.${system}.jit-1b-vm",
    "./vm/jit-1b-vm.nix",
    "jitRoot = ./.;",
]:
    if required not in flake:
        fail(f"required JIT-1B flake contract text missing: {required}")

vm = (HERE / "jit-1b-vm.nix").read_text()
for required in [
    "pkgs.testers.nixosTest",
    "diskImage = null;",
    "mountHostNixStore = false;",
    "useNixStoreImage = true;",
    "writableStore = false;",
    "sharedDirectories = lib.mkForce { };",
    "vlans = [ ];",
    "restrictNetwork = true;",
    "services.openssh.enable = false;",
    "users.users.jitqual",
    "! mountpoint -q /tmp/shared",
    "! mountpoint -q /tmp/xchg",
    "! findmnt -rn -t 9p,virtiofs",
    "--property=User=jitqual",
    "--property=NoNewPrivileges=yes",
    "--property=CapabilityBoundingSet=",
    "--property=AmbientCapabilities=",
    "--property=PrivateDevices=yes",
    "--property=PrivateTmp=yes",
    "--property=PrivateMounts=yes",
    "--property=ProtectSystem=strict",
    "--property=ProtectKernelTunables=yes",
    "--property=ProtectKernelModules=yes",
    "--property=ProtectKernelLogs=yes",
    "--property=ProtectControlGroups=yes",
    "--property=ProtectHostname=yes",
    "--property=ProtectClock=yes",
    "--property=RestrictNamespaces=yes",
    "--property=RestrictSUIDSGID=yes",
    "--property=LockPersonality=yes",
    "--property=RemoveIPC=yes",
    "--property=SystemCallArchitectures=native",
    "--property=RestrictAddressFamilies=AF_UNIX",
    "--property=ReadWritePaths=/run/jit-1b",
    "--property=UMask=0077",
    "CapEff CapBnd CapAmb",
    "socket.AF_INET",
    "socket.AF_INET6",
    "JIT-1B_SANDBOX_SELFTEST_PASS",
    "JIT-1B_REUSE_BLOCKED",
    "exit 23",
    "harness/check_lock.py",
    "harness/check_harness.py",
    "harness/check_policy_guard.py",
    'machine.fail(subject_command("jit-fixture-reuse"))',
    "ExecMainStatus",
    "jit-1b-persistence-sentinel",
    "machine.shutdown()",
    "machine.wait_for_shutdown()",
    "machine.start()",
    "second_boot != first_boot",
]:
    if required not in vm:
        fail(f"required VM-isolation contract text missing: {required}")

for forbidden in [
    "copy_from_host(",
    "copy_from_machine(",
    'sharedDirectories = { };',
    'umount "$p"',
]:
    if forbidden in vm:
        fail(f"forbidden host-exchange pattern present: {forbidden}")

print(f"jit1b_payload_manifest_sha256={manifest}")
print(f"jit1b_payload_file_count={len(actual)}")
print(f"jit1b_nixpkgs_rev={EXPECTED_NIXPKGS_REV}")
