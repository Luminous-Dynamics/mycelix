#!/usr/bin/env python3
"""Fail-closed source invariant for Pulse V2 remote ingress authority.

Remote signaling is intentionally treated as untrusted scheduling input. Once
`recv_remote_signal` is capability-opened, the callback must be V2-only:
legacy MailSignal bytes may not be decoded remotely because the callback has no
sender provenance and an unrestricted capability would otherwise make rich
legacy semantic payloads forgeable.

The only permitted remote grant is an unrestricted, listed-function grant for
`recv_remote_signal` itself. The initializer may perform no other source-chain,
network, scheduling, or local-signal provisioning. A capability-open receiver
may perform no DHT reads/writes or remote calls; its sole successful side effect
is forwarding one qualified information-poor V2 hint to the local UI. Malformed
V2 and legacy bytes are terminal errors.
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


def validate_init(init: str) -> tuple[bool, str]:
    forbidden_side_effects = (
        "create_entry(",
        "create_link(",
        "update_entry(",
        "delete_entry(",
        "create_cap_claim(",
        "delete_cap_grant(",
        "send_remote_signal(",
        "call_remote(",
        "emit_signal(",
        "schedule(",
    )
    for token in forbidden_side_effects:
        require(
            token not in init,
            f"init may do nothing except install the one remote-ingress grant: {token}",
        )

    if "create_cap_grant(" not in init:
        require(
            "CapGrantEntry" not in init and "GrantedFunctions::" not in init,
            "partial capability configuration is forbidden without create_cap_grant",
        )
        return False, "remote ingress capability is not opened yet"

    require(init.count("create_cap_grant(") == 1, "init must create exactly one capability grant")
    require(init.count("CapGrantEntry") == 1, "init must construct exactly one CapGrantEntry")
    require("GrantedFunctions::All" not in init, "remote ingress may not use GrantedFunctions::All")
    require(
        "let mut fns = HashSet::new();" in init,
        "remote ingress grant must build an explicit function allowlist",
    )
    require(
        init.count("fns.insert(") == 1,
        "remote ingress grant must list exactly one remotely callable function",
    )
    require(
        re.search(
            r'fns\.insert\(\(zome_info\(\)\?\.name,\s*"recv_remote_signal"\.into\(\)\)\);',
            init,
            re.S,
        )
        is not None,
        "the sole remotely granted function must be recv_remote_signal in this zome",
    )
    require(
        init.count("GrantedFunctions::Listed(fns)") == 1,
        "remote ingress grant must use exactly one GrantedFunctions::Listed allowlist",
    )
    require(
        "let functions = GrantedFunctions::Listed(fns);" in init,
        "remote ingress grant must bind the explicit function allowlist",
    )
    require(
        'tag: "pulse-v2-remote-ingress-v1".into()' in init,
        "remote ingress capability must carry the canonical audit tag",
    )
    require(
        "access: ().into()" in init,
        "remote ingress capability must explicitly declare unrestricted access",
    )
    require(
        re.search(r"create_cap_grant\(CapGrantEntry\s*\{[\s\S]*?functions,[\s\S]*?\}\)\?;", init)
        is not None,
        "remote ingress capability entry must bind the listed functions",
    )

    return True, "init grants only unrestricted recv_remote_signal and performs no other side effects"


def validate_recv(recv: str, capability_open: bool) -> str:
    if not capability_open:
        return "receiver remains pre-capability baseline; strict V2-only ingress is not active yet"

    required = (
        "admit_extern_io(&signal)",
        "RemoteSignalAdmission::PulseV2(Ok(hint))",
        "RemoteSignalAdmission::PulseV2(Err(",
        "RemoteSignalAdmission::Legacy(_)",
        "emit_signal(hint)",
    )
    for token in required:
        require(token in recv, f"capability-open receiver is missing strict V2 ingress token: {token}")

    require(
        "MailSignal" not in recv,
        "capability-open recv_remote_signal may not interpret rich legacy MailSignal payloads",
    )
    require(
        "signal.decode" not in recv,
        "capability-open recv_remote_signal may not perform legacy deserialization",
    )

    forbidden_receiver_side_effects = (
        "create_entry(",
        "create_link(",
        "update_entry(",
        "delete_entry(",
        "get(",
        "get_links(",
        "must_get",
        "send_remote_signal(",
        "call_remote(",
        "call(",
        "schedule(",
    )
    for token in forbidden_receiver_side_effects:
        require(
            token not in recv,
            f"capability-open receiver may only classify/reject or emit one local wake: {token}",
        )

    require(
        recv.count("emit_signal(") == 1,
        "capability-open receiver must have exactly one local signal side effect",
    )

    admission_pos = recv.find("admit_extern_io(&signal)")
    ok_pos = recv.find("RemoteSignalAdmission::PulseV2(Ok(hint))")
    err_pos = recv.find("RemoteSignalAdmission::PulseV2(Err(")
    legacy_pos = recv.find("RemoteSignalAdmission::Legacy(_)")
    emit_pos = recv.find("emit_signal(hint)")
    require(
        admission_pos < min(ok_pos, err_pos, legacy_pos),
        "raw ExternIO admission must occur before all protocol-family branches",
    )
    require(
        ok_pos < emit_pos,
        "the sole local wake may occur only inside the admitted V2-success branch",
    )

    require(
        re.search(
            r"RemoteSignalAdmission::PulseV2\(Err\([^)]*\)\)\s*=>\s*\{\s*return\s+Err\(",
            recv,
            re.S,
        )
        is not None,
        "malformed V2-family traffic must terminate explicitly as an error",
    )
    require(
        re.search(
            r"RemoteSignalAdmission::Legacy\(_\)\s*=>\s*\{\s*return\s+Err\(",
            recv,
            re.S,
        )
        is not None,
        "legacy remote traffic must terminate explicitly as an error",
    )

    return "capability-open receiver is V2-only, side-effect-minimal, and cannot express legacy remote semantics"


def self_test() -> None:
    baseline_init = """pub fn init(_: ()) -> ExternResult<InitCallbackResult> {
        Ok(InitCallbackResult::Pass)
    }"""
    opened, _ = validate_init(baseline_init)
    require(not opened, "self-test: no-grant baseline unexpectedly opened capability")

    good_init = """pub fn init(_: ()) -> ExternResult<InitCallbackResult> {
        let mut fns = HashSet::new();
        fns.insert((zome_info()?.name, "recv_remote_signal".into()));
        let functions = GrantedFunctions::Listed(fns);
        create_cap_grant(CapGrantEntry {
            tag: "pulse-v2-remote-ingress-v1".into(),
            access: ().into(),
            functions,
        })?;
        Ok(InitCallbackResult::Pass)
    }"""
    opened, _ = validate_init(good_init)
    require(opened, "self-test: exact grant was not recognized as open")

    for bad_init in (
        good_init.replace("GrantedFunctions::Listed(fns)", "GrantedFunctions::All"),
        good_init.replace(
            'fns.insert((zome_info()?.name, "recv_remote_signal".into()));',
            'fns.insert((zome_info()?.name, "recv_remote_signal".into()));\n        fns.insert((zome_info()?.name, "send_email_v2".into()));',
        ),
        good_init.replace(
            "let mut fns = HashSet::new();",
            "create_entry(x)?;\n        let mut fns = HashSet::new();",
        ),
        good_init.replace(
            "let mut fns = HashSet::new();",
            "emit_signal(x)?;\n        let mut fns = HashSet::new();",
        ),
    ):
        try:
            validate_init(bad_init)
        except BoundaryViolation:
            pass
        else:
            raise BoundaryViolation("self-test: overbroad/side-effecting capability init was not rejected")

    good_recv = """pub fn recv_remote_signal(signal: ExternIO) -> ExternResult<()> {
        match admit_extern_io(&signal) {
            RemoteSignalAdmission::PulseV2(Ok(hint)) => { emit_signal(hint)?; }
            RemoteSignalAdmission::PulseV2(Err(error)) => { return Err(make_error(error)); }
            RemoteSignalAdmission::Legacy(_) => { return Err(legacy_disabled()); }
        }
        Ok(())
    }"""
    validate_recv(good_recv, True)

    bad_cases = (
        good_recv.replace(
            "RemoteSignalAdmission::Legacy(_) => { return Err(legacy_disabled()); }",
            "RemoteSignalAdmission::Legacy(_) => { let mail: MailSignal = signal.decode()?; emit_signal(mail)?; }",
        ),
        good_recv.replace(
            "match admit_extern_io(&signal) {",
            "let _ = get(x)?;\n        match admit_extern_io(&signal) {",
        ),
        good_recv.replace(
            "RemoteSignalAdmission::Legacy(_) => { return Err(legacy_disabled()); }",
            "RemoteSignalAdmission::Legacy(_) => { emit_signal(PulseRealtimeHintV1::inbox_changed())?; }",
        ),
    )
    for bad_recv in bad_cases:
        try:
            validate_recv(bad_recv, True)
        except BoundaryViolation:
            pass
        else:
            raise BoundaryViolation("self-test: authority-bearing or side-effecting remote ingress was not rejected")


def main() -> None:
    try:
        self_test()
        source = COORDINATOR.read_text(encoding="utf-8")
        init = extract_function(source, "init")
        recv = extract_function(source, "recv_remote_signal")
        capability_open, init_result = validate_init(init)
        print(f"remote-ingress-boundary: PASS: {init_result}")
        recv_result = validate_recv(recv, capability_open)
        prefix = "PASS" if capability_open else "INFO"
        print(f"remote-ingress-boundary: {prefix}: {recv_result}")
    except BoundaryViolation as error:
        print(f"remote-ingress-boundary: FAIL: {error}", file=sys.stderr)
        raise SystemExit(1) from error


if __name__ == "__main__":
    main()
