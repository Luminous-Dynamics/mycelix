#!/usr/bin/env python3
"""Fail-closed capability invariant for Pulse V2 remote wake reception.

The realtime receiver is intentionally non-authoritative, but Holochain still
requires a capability grant before a remote peer may invoke
`recv_remote_signal`.  This guard permits the current no-grant baseline while
freezing the only future grant shape that may be introduced: one unrestricted,
listed grant for the current zome's `recv_remote_signal` function and no app
state writes in `init`.
"""

from __future__ import annotations

from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
COORDINATOR = ROOT / "mycelix-workspace/mycelix-pulse/holochain/zomes/messages/coordinator/src/lib.rs"


class BoundaryViolation(RuntimeError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise BoundaryViolation(message)


def extract_function(source: str, name: str) -> str:
    marker = re.search(rf"\bpub\s+fn\s+{re.escape(name)}\s*\(", source)
    require(marker is not None, f"missing function {name}")
    assert marker is not None

    brace = source.find("{", marker.start())
    require(brace >= 0, f"missing body for {name}")

    depth = 0
    for index in range(brace, len(source)):
        char = source[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return source[marker.start() : index + 1]

    raise BoundaryViolation(f"unterminated body for {name}")


def validate_init(init: str) -> str:
    broad_tokens = (
        "GrantedFunctions::All",
        "CapAccess::Transferable",
        "CapAccess::Assigned",
    )
    for token in broad_tokens:
        require(token not in init, f"remote-signal capability may not use broader/different authority: {token}")

    domain_writes = (
        "create_entry(",
        "update_entry(",
        "delete_entry(",
        "create_link(",
        "delete_link(",
    )
    for token in domain_writes:
        require(token not in init, f"init must not create app-domain state while enabling realtime reception: {token}")

    grant_count = init.count("create_cap_grant(")
    require(grant_count <= 1, "init may create at most one capability grant")
    if grant_count == 0:
        require(
            "recv_remote_signal" not in init or "remote signals" in init,
            "init mentions recv_remote_signal without an explicit minimal grant",
        )
        return "no remote-signal capability grant yet; implementation tranche remains pending"

    required = {
        "let mut functions": "grant must build an explicit listed function set",
        "zome_info()?.name": "grant must bind to the current coordinator zome",
        '"recv_remote_signal"': "grant must expose exactly the remote-signal receiver",
        "ZomeCallCapGrant": "grant must use Holochain's typed capability record",
        'tag: "pulse-v2-recv-remote-signal".into()': "grant must carry the canonical audit tag",
        "access: CapAccess::Unrestricted": "remote peers need an unrestricted receiver grant",
        "functions: GrantedFunctions::Listed(functions)": "grant must remain function-listed, never all-functions",
    }
    for token, meaning in required.items():
        require(token in init, f"{meaning}: missing {token}")

    inserts = re.findall(
        r"functions\.insert\s*\(\s*\(\s*[^,]+,\s*\"([^\"]+)\"\.into\(\)\s*\)\s*\)\s*;",
        init,
        re.S,
    )
    require(inserts == ["recv_remote_signal"], f"capability function set must be exactly recv_remote_signal, got {inserts!r}")
    require(
        init.count('"recv_remote_signal"') == 1,
        "recv_remote_signal capability name must appear exactly once in init",
    )
    require(
        init.find("functions.insert") < init.find("create_cap_grant("),
        "listed receiver function must be fixed before the grant is committed",
    )

    return "remote-signal capability is current-zome-only, receiver-only, listed, and domain-state-free"


def self_test() -> None:
    baseline = """pub fn init(_: ()) -> ExternResult<InitCallbackResult> {
        // remote signals remain disabled in this baseline
        Ok(InitCallbackResult::Pass)
    }"""
    validate_init(baseline)

    good = """pub fn init(_: ()) -> ExternResult<InitCallbackResult> {
        let mut functions = HashSet::new();
        functions.insert((zome_info()?.name, "recv_remote_signal".into()));
        create_cap_grant(ZomeCallCapGrant {
            tag: "pulse-v2-recv-remote-signal".into(),
            access: CapAccess::Unrestricted,
            functions: GrantedFunctions::Listed(functions),
        })?;
        Ok(InitCallbackResult::Pass)
    }"""
    validate_init(good)

    mutations = {
        "all functions": good.replace("GrantedFunctions::Listed(functions)", "GrantedFunctions::All"),
        "extra function": good.replace(
            'functions.insert((zome_info()?.name, "recv_remote_signal".into()));',
            'functions.insert((zome_info()?.name, "recv_remote_signal".into()));\n        functions.insert((zome_info()?.name, "send_email_v2".into()));',
        ),
        "domain write": good.replace(
            "create_cap_grant(ZomeCallCapGrant {",
            "create_entry(foo)?;\n        create_cap_grant(ZomeCallCapGrant {",
        ),
        "wrong tag": good.replace("pulse-v2-recv-remote-signal", "remote-everything"),
        "assigned": good.replace("CapAccess::Unrestricted", "CapAccess::Assigned { assignees: set() }"),
    }
    for name, mutated in mutations.items():
        try:
            validate_init(mutated)
        except BoundaryViolation:
            continue
        raise BoundaryViolation(f"self-test: {name} mutation was not rejected")


def main() -> None:
    try:
        self_test()
        source = COORDINATOR.read_text(encoding="utf-8")
        init = extract_function(source, "init")
        result = validate_init(init)
        prefix = "PASS" if "receiver-only" in result else "INFO"
        print(f"remote-capability-boundary: {prefix}: {result}")
    except BoundaryViolation as error:
        print(f"remote-capability-boundary: FAIL: {error}", file=sys.stderr)
        raise SystemExit(1) from error


if __name__ == "__main__":
    main()
