#!/usr/bin/env python3
"""Fail-closed source invariant for Pulse V2 realtime producer authority.

The durable V2 coordinator may emit no wake yet. Once a wake is implemented it
must use the qualified PulseV2RemoteSignal wire wrapper directly, occur only
after the durable V2 inbox link has been committed, target only the recipient,
and remain best-effort.

The receive side must classify raw ExternIO bytes before protocol-specific
interpretation. V2-family failures are terminal. The explicit Legacy branch may
either preserve the pre-capability compatibility decoder or reject legacy bytes
terminally; it may never be reached by V2-family failure. The separate remote-
ingress theorem requires the terminal V2-only form once recv_remote_signal is
capability-opened.
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


def validate_send_v2(send_v2: str) -> str:
    forbidden = {
        "MailSignal": "legacy realtime envelope type",
        "EmailReceived": "legacy message-arrival authority shape",
        "DeliveryConfirmed": "legacy delivery authority shape",
        "ReadReceiptReceived": "legacy read-receipt authority shape",
        "encrypted_subject": "message content metadata",
    }
    for token, meaning in forbidden.items():
        require(token not in send_v2, f"send_email_v2 contains {meaning}: {token}")

    if "send_remote_signal(" not in send_v2:
        return "send_email_v2 emits no realtime wake yet"

    require(
        "PulseV2RemoteSignal::inbox_changed_v2()" in send_v2,
        "V2 remote wake must use the qualified typed wire wrapper",
    )
    require(
        "ExternIO::encode" not in send_v2,
        "V2 typed wire wrapper must be passed directly to send_remote_signal; explicit ExternIO::encode would double-encode",
    )

    recipient_only = re.search(
        r"send_remote_signal\s*\(\s*PulseV2RemoteSignal::inbox_changed_v2\(\)\s*,\s*vec!\s*\[\s*input\.recipient(?:\.clone\(\))?\s*\]",
        send_v2,
        re.S,
    )
    require(recipient_only is not None, "V2 wake must target only input.recipient")

    require(
        "let _ = send_remote_signal" in send_v2 or "if let Err(" in send_v2,
        "V2 wake must be best-effort and must not determine durable send success",
    )

    wake_pos = send_v2.find("send_remote_signal(")
    entry_pos = send_v2.find("create_entry(")
    sent_link_pos = send_v2.find("LinkTypes::AgentToSentV2")
    inbox_link_pos = send_v2.find("LinkTypes::AgentToInboxV2")
    require(entry_pos >= 0, "send_email_v2 must commit the V2 entry before waking")
    require(sent_link_pos >= 0, "send_email_v2 must commit the V2 sent link before waking")
    require(inbox_link_pos >= 0, "send_email_v2 must commit the V2 inbox link before waking")
    require(
        max(entry_pos, sent_link_pos, inbox_link_pos) < wake_pos,
        "V2 wake must occur only after entry, sent-link, and inbox-link durable operations",
    )

    return "send_email_v2 wake is typed, recipient-only, post-durable, and best-effort"


def validate_recv(recv: str) -> str:
    if "admit_extern_io" not in recv:
        require(
            "PulseV2RemoteSignal" not in recv and "RemoteSignalAdmission" not in recv,
            "recv_remote_signal mentions V2 wire types without the qualified ExternIO admission seam",
        )
        return "recv_remote_signal is still legacy-only; implementation tranche remains pending"

    admission_pos = recv.find("admit_extern_io")
    legacy_pos = recv.find("RemoteSignalAdmission::Legacy")
    v2_ok_pos = recv.find("RemoteSignalAdmission::PulseV2(Ok(hint))")
    v2_err_pos = recv.find("RemoteSignalAdmission::PulseV2(Err(")
    decode_pos = recv.find("signal.decode")
    mail_signal_pos = recv.find("MailSignal")

    require(v2_ok_pos >= 0, "V2 receive path must explicitly handle admitted V2 hints")
    require(v2_err_pos >= 0, "V2 receive path must explicitly handle terminal V2 failures")
    require(legacy_pos >= 0, "V2 receive path must retain an explicit legacy-family branch")
    require("emit_signal(hint)" in recv, "admitted V2 hint must be forwarded only as a local wake")
    require(
        admission_pos < min(v2_ok_pos, v2_err_pos, legacy_pos),
        "raw ExternIO admission must occur before protocol branches",
    )
    require(
        v2_err_pos < legacy_pos and "return Err(" in recv[v2_err_pos:legacy_pos],
        "V2-family decode failure must terminate before the legacy branch",
    )

    has_decode = decode_pos >= 0
    has_mail_signal = mail_signal_pos >= 0
    require(
        has_decode == has_mail_signal,
        "legacy compatibility must not partially retain a decoder or MailSignal interpretation",
    )

    if has_decode:
        require(
            decode_pos > legacy_pos,
            "legacy signal.decode must occur only after RemoteSignalAdmission::Legacy",
        )
        require(
            mail_signal_pos > legacy_pos,
            "MailSignal interpretation must occur only inside the explicit legacy branch",
        )
        return "recv_remote_signal classifies raw bytes first; V2 failures are terminal; legacy decode is branch-confined"

    legacy_branch = re.search(
        r"RemoteSignalAdmission::Legacy\s*\([^)]*\)\s*=>\s*\{[\s\S]*?return\s+Err\s*\(",
        recv,
    )
    require(
        legacy_branch is not None,
        "V2-only receiver must terminally reject the explicit legacy-family branch",
    )
    return "recv_remote_signal classifies raw bytes first; V2 failures and legacy family are terminal"


def self_test() -> None:
    baseline_send = "pub fn send_email_v2() { create_entry(x)?; LinkTypes::AgentToSentV2; LinkTypes::AgentToInboxV2; Ok(hash) }"
    validate_send_v2(baseline_send)

    good_send = """pub fn send_email_v2() {
        create_entry(x)?;
        LinkTypes::AgentToSentV2;
        LinkTypes::AgentToInboxV2;
        let _ = send_remote_signal(
            PulseV2RemoteSignal::inbox_changed_v2(),
            vec![input.recipient.clone()],
        );
        Ok(hash)
    }"""
    validate_send_v2(good_send)

    bad_double_encode = good_send.replace(
        "let _ = send_remote_signal(",
        "let _ = ExternIO::encode(PulseV2RemoteSignal::inbox_changed_v2()); let _ = send_remote_signal(",
    )
    try:
        validate_send_v2(bad_double_encode)
    except BoundaryViolation:
        pass
    else:
        raise BoundaryViolation("self-test: double-encoded V2 wake was not rejected")

    bad_order = good_send.replace(
        "LinkTypes::AgentToInboxV2;\n        let _ = send_remote_signal(",
        "let _ = send_remote_signal(",
    ) + " LinkTypes::AgentToInboxV2;"
    try:
        validate_send_v2(bad_order)
    except BoundaryViolation:
        pass
    else:
        raise BoundaryViolation("self-test: pre-inbox-link wake was not rejected")

    compatibility_recv = """pub fn recv_remote_signal(signal: ExternIO) {
        match admit_extern_io(&signal) {
            RemoteSignalAdmission::PulseV2(Ok(hint)) => { emit_signal(hint)?; }
            RemoteSignalAdmission::PulseV2(Err(error)) => { return Err(make_error(error)); }
            RemoteSignalAdmission::Legacy(_) => {
                let mail_signal: MailSignal = signal.decode()?;
                emit_signal(mail_signal)?;
            }
        }
    }"""
    validate_recv(compatibility_recv)

    v2_only_recv = """pub fn recv_remote_signal(signal: ExternIO) {
        match admit_extern_io(&signal) {
            RemoteSignalAdmission::PulseV2(Ok(hint)) => { emit_signal(hint)?; }
            RemoteSignalAdmission::PulseV2(Err(error)) => { return Err(make_error(error)); }
            RemoteSignalAdmission::Legacy(_) => { return Err(legacy_disabled()); }
        }
    }"""
    validate_recv(v2_only_recv)

    bad_fallback = compatibility_recv.replace(
        "RemoteSignalAdmission::PulseV2(Err(error)) => { return Err(make_error(error)); }",
        "RemoteSignalAdmission::PulseV2(Err(_error)) => {}",
    )
    try:
        validate_recv(bad_fallback)
    except BoundaryViolation:
        pass
    else:
        raise BoundaryViolation("self-test: V2-to-legacy fallthrough was not rejected")

    silent_legacy = v2_only_recv.replace(
        "RemoteSignalAdmission::Legacy(_) => { return Err(legacy_disabled()); }",
        "RemoteSignalAdmission::Legacy(_) => {}",
    )
    try:
        validate_recv(silent_legacy)
    except BoundaryViolation:
        pass
    else:
        raise BoundaryViolation("self-test: silent legacy acceptance in V2-only mode was not rejected")

    partial_legacy = v2_only_recv.replace(
        "RemoteSignalAdmission::Legacy(_) => { return Err(legacy_disabled()); }",
        "RemoteSignalAdmission::Legacy(_) => { let _ = signal.decode::<u8>(); return Err(legacy_disabled()); }",
    )
    try:
        validate_recv(partial_legacy)
    except BoundaryViolation:
        pass
    else:
        raise BoundaryViolation("self-test: partial legacy decoder retention was not rejected")


def main() -> None:
    try:
        self_test()
        source = COORDINATOR.read_text(encoding="utf-8")
        send_v2 = extract_function(source, "send_email_v2")
        recv = extract_function(source, "recv_remote_signal")
        print(f"producer-boundary: PASS: {validate_send_v2(send_v2)}")
        recv_result = validate_recv(recv)
        prefix = "INFO" if "legacy-only" in recv_result else "PASS"
        print(f"producer-boundary: {prefix}: {recv_result}")
    except BoundaryViolation as error:
        print(f"producer-boundary: FAIL: {error}", file=sys.stderr)
        raise SystemExit(1) from error


if __name__ == "__main__":
    main()
