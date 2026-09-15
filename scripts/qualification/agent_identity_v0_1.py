#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import json
import struct
import subprocess
import tomllib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "mycelix-workspace/docs/agents/agent-identity-v0.1.json"
DOC = ROOT / "mycelix-workspace/docs/agents/AGENT_IDENTITY_V0.1.md"
SOURCE = ROOT / "crates/mycelix-agent-identity/src/lib.rs"
CARGO_MANIFEST = ROOT / "crates/mycelix-agent-identity/Cargo.toml"
PARENT_RECEIPT = ROOT / "mycelix-workspace/docs/agents/evidence/agent-identity-v0.1/parent-convergence-receipt.json"
QUAL_LOCK = ROOT / "mycelix-workspace/docs/agents/evidence/agent-identity-v0.1/agent-identity-Cargo.lock"

EXPECTED_PARENT = "6163a80c8398fac01fbd506515c11f60cee48320"
EXPECTED_PARENT_RUN = 34881344813
EXPECTED_PARENT_ATTEMPT = 1
EXPECTED_PARENT_RECEIPT_SHA = "ed74e503a5cc7f42fe354ecae1b645d885596add0d7b98987aa079bd7fc7113b"
EXPECTED_PARENT_LOCK_SHA = "38889f5045c6e5f6ea3b9427e056429b316021b244cdec6fda4fbd1afb49888d"
EXPECTED_QUAL_LOCK_SHA = "17721b17f3f8fec578465b21ca504ee3e00b98e20c6a3a53b8f5f49efc9879a8"
EXPECTED_PROFILE = "mycelix-agent-runtime-instance-v1-blake3-framed-semantic"
EXPECTED_PROTOCOL = "mycelix-agent-identity-v0.1"
EXPECTED_DOMAIN = b"mycelix/agent/runtime-instance/v1"
EXPECTED_VECTOR_AGENT = b"did:example:agent-alpha"
EXPECTED_VECTOR_RUNTIME = b"runtime:host-a:proc-7"
EXPECTED_VECTOR_DIGEST = "466beee862b306834aa76be027eb9c9f9629e6e550e1bdd5f033a7fec1398a9e"
EXPECTED_CHANGED_PATHS = {
    ".github/workflows/agent-identity-v0.1.yml",
    "crates/mycelix-agent-identity/Cargo.toml",
    "crates/mycelix-agent-identity/INVARIANTS.md",
    "crates/mycelix-agent-identity/src/lib.rs",
    "mycelix-workspace/docs/agents/AGENT_IDENTITY_V0.1.md",
    "mycelix-workspace/docs/agents/agent-identity-v0.1.json",
    "mycelix-workspace/docs/agents/evidence/agent-identity-v0.1/agent-identity-Cargo.lock",
    "mycelix-workspace/docs/agents/evidence/agent-identity-v0.1/parent-convergence-receipt.json",
    "scripts/qualification/agent_identity_v0_1.py",
}
EXPECTED_ROOT_TREES = {
    "crates/mycelix-institutional-core": "6602af340acaa660ffd7e4d46a2f84e67009f396",
    "crates/mycelix-authority-identity": "ab98e976ee7acc67e7b5d2af7fc0d16a476f2710",
    "crates/mycelix-authority-freshness": "76ec24222cfbd7a7d528996eb27df2f33dcc3c5a",
    "crates/mycelix-authority-delegation-policy": "9c41ee69990e0fbb78a58dd82ed693df0889c862",
    "crates/mycelix-authority-delegation": "665db95344b9787a377a4e06c74e434914535389",
}
EXPECTED_MUTATION_DIGESTS = {
    "profile": "2f6efb042af7be9d49af6591de7ac8b81bccbe36409210df2f73e372c695321b",
    "protocol": "663c586481a514a491d1cd4ed677286301a8a9a3aabec0c9fb98308663d42a12",
    "domain": "420ffa88fc8d137869ab595735105ab1d99f71f9128dda124ebc225f18243a03",
    "field_order": "ea0ae15b3a7fa7352cf409d3425221f04f7e322a6d1742ac8f253866ff04878a",
}

# Minimal independent unkeyed BLAKE3 reference used only for the frozen vectors.
# It is deliberately separate from the production Rust implementation.
_IV = [
    0x6A09E667,
    0xBB67AE85,
    0x3C6EF372,
    0xA54FF53A,
    0x510E527F,
    0x9B05688C,
    0x1F83D9AB,
    0x5BE0CD19,
]
_PERM = [2, 6, 3, 10, 7, 0, 4, 13, 1, 11, 12, 5, 9, 14, 15, 8]
_CHUNK_START = 1
_CHUNK_END = 2
_PARENT = 4
_ROOT = 8
_MASK = 0xFFFFFFFF


def require(condition: bool, message: str) -> None:
    if not condition:
        raise SystemExit(message)


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True).strip()


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def frame(value: bytes) -> bytes:
    return len(value).to_bytes(8, "little") + value


def _rotr(value: int, count: int) -> int:
    return ((value >> count) | (value << (32 - count))) & _MASK


def _g(state: list[int], a: int, b: int, c: int, d: int, x: int, y: int) -> None:
    state[a] = (state[a] + state[b] + x) & _MASK
    state[d] = _rotr(state[d] ^ state[a], 16)
    state[c] = (state[c] + state[d]) & _MASK
    state[b] = _rotr(state[b] ^ state[c], 12)
    state[a] = (state[a] + state[b] + y) & _MASK
    state[d] = _rotr(state[d] ^ state[a], 8)
    state[c] = (state[c] + state[d]) & _MASK
    state[b] = _rotr(state[b] ^ state[c], 7)


def _round(state: list[int], message: list[int]) -> None:
    _g(state, 0, 4, 8, 12, message[0], message[1])
    _g(state, 1, 5, 9, 13, message[2], message[3])
    _g(state, 2, 6, 10, 14, message[4], message[5])
    _g(state, 3, 7, 11, 15, message[6], message[7])
    _g(state, 0, 5, 10, 15, message[8], message[9])
    _g(state, 1, 6, 11, 12, message[10], message[11])
    _g(state, 2, 7, 8, 13, message[12], message[13])
    _g(state, 3, 4, 9, 14, message[14], message[15])


def _compress(
    chaining_value: list[int],
    block_words: list[int],
    counter: int,
    block_len: int,
    flags: int,
) -> list[int]:
    state = list(chaining_value) + _IV[:4] + [
        counter & _MASK,
        (counter >> 32) & _MASK,
        block_len,
        flags,
    ]
    message = list(block_words)
    for round_index in range(7):
        _round(state, message)
        if round_index != 6:
            message = [message[index] for index in _PERM]
    result = [0] * 16
    for index in range(8):
        result[index] = (state[index] ^ state[index + 8]) & _MASK
        result[index + 8] = (state[index + 8] ^ chaining_value[index]) & _MASK
    return result


def _words(block: bytes) -> list[int]:
    require(len(block) <= 64, "BLAKE3 reference block overflow")
    return list(struct.unpack("<16I", block + b"\0" * (64 - len(block))))


class _Output:
    def __init__(
        self,
        chaining_value: list[int],
        block_words: list[int],
        counter: int,
        block_len: int,
        flags: int,
    ) -> None:
        self.chaining_value_input = chaining_value
        self.block_words = block_words
        self.counter = counter
        self.block_len = block_len
        self.flags = flags

    def chaining_value(self) -> list[int]:
        return _compress(
            self.chaining_value_input,
            self.block_words,
            self.counter,
            self.block_len,
            self.flags,
        )[:8]

    def root_bytes(self, length: int) -> bytes:
        output = bytearray()
        output_counter = 0
        while len(output) < length:
            words = _compress(
                self.chaining_value_input,
                self.block_words,
                output_counter,
                self.block_len,
                self.flags | _ROOT,
            )
            output.extend(struct.pack("<16I", *words))
            output_counter += 1
        return bytes(output[:length])


class _ChunkState:
    def __init__(self, key: list[int], counter: int, flags: int) -> None:
        self.chaining_value = list(key)
        self.counter = counter
        self.block = bytearray()
        self.blocks_compressed = 0
        self.flags = flags

    def length(self) -> int:
        return 64 * self.blocks_compressed + len(self.block)

    def start_flag(self) -> int:
        return _CHUNK_START if self.blocks_compressed == 0 else 0

    def update(self, data: bytes) -> None:
        offset = 0
        while offset < len(data):
            if len(self.block) == 64:
                self.chaining_value = _compress(
                    self.chaining_value,
                    _words(bytes(self.block)),
                    self.counter,
                    64,
                    self.flags | self.start_flag(),
                )[:8]
                self.blocks_compressed += 1
                self.block.clear()
            take = min(64 - len(self.block), len(data) - offset)
            self.block.extend(data[offset : offset + take])
            offset += take

    def output(self) -> _Output:
        return _Output(
            self.chaining_value,
            _words(bytes(self.block)),
            self.counter,
            len(self.block),
            self.flags | self.start_flag() | _CHUNK_END,
        )


def _parent_output(left: list[int], right: list[int], key: list[int], flags: int) -> _Output:
    return _Output(list(key), list(left) + list(right), 0, 64, flags | _PARENT)


def _parent_cv(left: list[int], right: list[int], key: list[int], flags: int) -> list[int]:
    return _parent_output(left, right, key, flags).chaining_value()


def blake3(data: bytes) -> bytes:
    key = _IV
    flags = 0
    chunk = _ChunkState(key, 0, flags)
    stack: list[list[int]] = []
    offset = 0
    while offset < len(data):
        if chunk.length() == 1024:
            current = chunk.output().chaining_value()
            total_chunks = chunk.counter + 1
            while total_chunks & 1 == 0:
                current = _parent_cv(stack.pop(), current, key, flags)
                total_chunks >>= 1
            stack.append(current)
            chunk = _ChunkState(key, chunk.counter + 1, flags)
        take = min(1024 - chunk.length(), len(data) - offset)
        chunk.update(data[offset : offset + take])
        offset += take
    output = chunk.output()
    while stack:
        output = _parent_output(stack.pop(), output.chaining_value(), key, flags)
    return output.root_bytes(32)


def main() -> None:
    manifest = json.loads(MANIFEST.read_text())
    require(manifest.get("schema") == "mycelix.agent.identity.v0.1", "wrong AGENT-002 schema")
    require(manifest.get("parent_convergence_head") == EXPECTED_PARENT, "wrong convergence parent")
    require(manifest.get("parent_convergence_run") == EXPECTED_PARENT_RUN, "wrong convergence run")
    require(manifest.get("parent_convergence_attempt") == EXPECTED_PARENT_ATTEMPT, "wrong convergence attempt")
    require(
        manifest.get("parent_convergence_receipt_sha256") == EXPECTED_PARENT_RECEIPT_SHA,
        "wrong parent convergence receipt digest",
    )
    require(manifest.get("parent_authority_lock_sha256") == EXPECTED_PARENT_LOCK_SHA, "wrong parent authority lock digest")
    require(manifest.get("qualification_lock_sha256") == EXPECTED_QUAL_LOCK_SHA, "wrong AGENT-002 lock digest")
    require(manifest.get("protocol_version") == EXPECTED_PROTOCOL, "protocol drift")
    require(manifest.get("runtime_identity_profile") == EXPECTED_PROFILE, "profile drift")
    require(manifest.get("runtime_identity_domain") == EXPECTED_DOMAIN.decode(), "domain drift")
    require(manifest.get("max_runtime_instance_id_bytes") == 512, "runtime id byte bound drift")

    require(git("rev-parse", "HEAD^") == EXPECTED_PARENT, "AGENT-002 is not the direct child of qualified #806")
    require(len(git("rev-list", "--parents", "-n1", "HEAD").split()) == 2, "AGENT-002 head must have exactly one parent")

    changed = set(git("diff", "--name-only", EXPECTED_PARENT, "HEAD").splitlines())
    require(changed == EXPECTED_CHANGED_PATHS, f"AGENT-002 exact path census drift: {sorted(changed ^ EXPECTED_CHANGED_PATHS)}")

    require(sha256(PARENT_RECEIPT) == EXPECTED_PARENT_RECEIPT_SHA, "parent receipt bytes changed")
    parent_receipt = json.loads(PARENT_RECEIPT.read_text())
    require(parent_receipt.get("qualification_pass") is True, "parent convergence receipt is not PASS")
    require(parent_receipt.get("subject_head") == EXPECTED_PARENT, "parent receipt subject mismatch")
    require(parent_receipt.get("github_run_id") == EXPECTED_PARENT_RUN, "parent receipt run mismatch")
    require(parent_receipt.get("github_run_attempt") == EXPECTED_PARENT_ATTEMPT, "parent receipt attempt mismatch")
    require(parent_receipt.get("authority_parent_qualification_lock_sha256") == EXPECTED_PARENT_LOCK_SHA, "parent lock commitment mismatch")
    require(parent_receipt.get("agent_002_identity_claim_blocked") is True, "parent receipt must block AGENT-002 claim")
    for root, expected_tree in EXPECTED_ROOT_TREES.items():
        require(parent_receipt["admitted_root_trees"].get(root) == expected_tree, f"parent receipt tree mismatch: {root}")
        require(git("rev-parse", f"{EXPECTED_PARENT}:{root}") == expected_tree, f"parent tree mismatch: {root}")
        require(git("rev-parse", f"HEAD:{root}") == expected_tree, f"AGENT-002 changed inherited authority root: {root}")

    require(sha256(QUAL_LOCK) == EXPECTED_QUAL_LOCK_SHA, "AGENT-002 qualification lock changed")

    cargo = tomllib.loads(CARGO_MANIFEST.read_text())
    dependencies = cargo.get("dependencies", {})
    require(set(dependencies) == {"blake3", "mycelix-institutional-core", "serde"}, "AGENT-002 direct dependency surface drift")
    require(dependencies.get("blake3") == "1", "blake3 dependency drift")
    require(
        dependencies.get("mycelix-institutional-core") == {"path": "../mycelix-institutional-core"},
        "institutional-core dependency drift",
    )
    require(
        dependencies.get("serde") == {"version": "1", "features": ["derive"]},
        "serde dependency drift",
    )
    require(cargo.get("dev-dependencies") == {"serde_json": "1"}, "AGENT-002 dev-dependency surface drift")

    source = SOURCE.read_text()
    require("#![forbid(unsafe_code)]" in source, "AGENT-002 crate must forbid unsafe code")
    require('pub const AGENT_IDENTITY_PROTOCOL_VERSION: &str = "mycelix-agent-identity-v0.1";' in source, "missing protocol constant")
    require('"mycelix-agent-runtime-instance-v1-blake3-framed-semantic"' in source, "missing runtime profile")
    require('b"mycelix/agent/runtime-instance/v1"' in source, "missing runtime domain")
    require("pub struct RuntimeInstanceId(String);" in source, "RuntimeInstanceId must keep a private field")
    require("pub struct QualifiedAgentRuntimeIdentityV1" in source, "missing qualified identity type")
    require("let agent = PrincipalId::new(claim.agent.as_str().to_owned())" in source, "legacy PrincipalId is not revalidated into the retained result")
    require("let digest = canonical_runtime_instance_digest(&agent, &claim.instance_id);" in source, "canonical digest does not use revalidated principal")
    require("Self::new(value).map_err(D::Error::custom)" in source, "runtime ID deserializer bypasses constructor")
    require("AgentPrincipalId" not in source, "parallel agent principal universe introduced")
    require("AgentControllerBinding" not in source, "parallel controller graph introduced")
    require(
        "#[derive(Clone, Debug, PartialEq, Eq)]\npub struct QualifiedAgentRuntimeIdentityV1" in source,
        "qualified identity derive surface drift",
    )
    require("impl Serialize for QualifiedAgentRuntimeIdentityV1" not in source, "positive qualified type implements Serialize")
    require("Deserialize<'de> for QualifiedAgentRuntimeIdentityV1" not in source, "positive qualified type implements Deserialize")

    require(
        blake3(b"").hex() == "af1349b9f5f9a1a6a0404dea36dcc9499bcb25c9adc112b7cc9a93cae41f3262",
        "independent BLAKE3 reference self-test failed",
    )
    preimage = (
        EXPECTED_DOMAIN
        + frame(EXPECTED_PROFILE.encode())
        + frame(EXPECTED_PROTOCOL.encode())
        + frame(EXPECTED_VECTOR_AGENT)
        + frame(EXPECTED_VECTOR_RUNTIME)
    )
    vector = manifest.get("independent_vector", {})
    require(vector.get("agent") == EXPECTED_VECTOR_AGENT.decode(), "vector agent drift")
    require(vector.get("runtime_instance_id") == EXPECTED_VECTOR_RUNTIME.decode(), "vector runtime drift")
    require(vector.get("preimage_hex") == preimage.hex(), "vector preimage drift")
    require(vector.get("digest_hex") == EXPECTED_VECTOR_DIGEST, "vector digest manifest drift")
    require(blake3(preimage).hex() == EXPECTED_VECTOR_DIGEST, "independent vector digest mismatch")

    mutation_preimages = {
        "profile": (
            EXPECTED_DOMAIN
            + frame((EXPECTED_PROFILE + "-mutated").encode())
            + frame(EXPECTED_PROTOCOL.encode())
            + frame(EXPECTED_VECTOR_AGENT)
            + frame(EXPECTED_VECTOR_RUNTIME)
        ),
        "protocol": (
            EXPECTED_DOMAIN
            + frame(EXPECTED_PROFILE.encode())
            + frame(b"mycelix-agent-identity-v0.2")
            + frame(EXPECTED_VECTOR_AGENT)
            + frame(EXPECTED_VECTOR_RUNTIME)
        ),
        "domain": (
            b"mycelix/agent/runtime-instance/v2"
            + frame(EXPECTED_PROFILE.encode())
            + frame(EXPECTED_PROTOCOL.encode())
            + frame(EXPECTED_VECTOR_AGENT)
            + frame(EXPECTED_VECTOR_RUNTIME)
        ),
        "field_order": (
            EXPECTED_DOMAIN
            + frame(EXPECTED_PROFILE.encode())
            + frame(EXPECTED_PROTOCOL.encode())
            + frame(EXPECTED_VECTOR_RUNTIME)
            + frame(EXPECTED_VECTOR_AGENT)
        ),
    }
    for name, mutated_preimage in mutation_preimages.items():
        mutated_digest = blake3(mutated_preimage).hex()
        require(mutated_digest == EXPECTED_MUTATION_DIGESTS[name], f"{name} mutation vector drift")
        require(mutated_digest != EXPECTED_VECTOR_DIGEST, f"{name} substitution did not change identity")

    for key in (
        "controller_binding_present",
        "parallel_agent_principal_present",
        "runtime_authenticity_claimed",
        "runtime_attestation_claimed",
        "current_agent_authority_claimed",
        "effect_authority_claimed",
        "full_agent_security_claimed",
    ):
        require(manifest.get(key) is False, f"forbidden positive claim: {key}")

    doc = DOC.read_text()
    for claim in (
        "AGENT-002 PASS != runtime authenticity",
        "AGENT-002 PASS != runtime attestation",
        "AGENT-002 PASS != current agent authority",
        "AGENT-002 PASS != effect authority",
        "AGENT-002 PASS != full agent security",
    ):
        require(claim in doc, f"missing negative claim: {claim}")

    print("AGENT-002 identity structural/evidence/vector contract: PASS")


if __name__ == "__main__":
    main()
