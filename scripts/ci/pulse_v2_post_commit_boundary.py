#!/usr/bin/env python3
"""Fail-closed source invariant for true post-commit Pulse V2 wake authority.

The V2 send extern is durable-state-only: it may create the encrypted V2 entry
and its V2 links, but it may not perform the realtime network side effect.
Once realtime delivery is implemented, only `post_commit` may emit the V2 wake,
and only a committed `AgentToInboxV2` link with the exact `inbox-v2` tag may
authorize it. The recipient must be recovered from that committed link's base
agent hash. The wake remains information-poor and best-effort.
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


def extract_function(source: str, name: str) -> str | None:
    marker = re.search(rf"\bpub\s+fn\s+{re.escape(name)}\s*\(", source)
    if marker is None:
        return None

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
    require(
        "PulseV2RemoteSignal" not in send_v2,
        "send_email_v2 must remain network-silent; V2 wakes belong in post_commit",
    )
    require(
        "send_remote_signal(" not in send_v2,
        "send_email_v2 may not perform a remote signal before the source-chain transaction commits",
    )
    require("create_entry(" in send_v2, "send_email_v2 must commit the V2 entry")
    require(
        "LinkTypes::AgentToSentV2" in send_v2,
        "send_email_v2 must retain the V2 sent link",
    )
    require(
        "LinkTypes::AgentToInboxV2" in send_v2,
        "send_email_v2 must retain the V2 inbox link",
    )
    require(
        'LinkTag::new("inbox-v2")' in send_v2,
        "send_email_v2 V2 inbox link must retain the exact inbox-v2 tag",
    )
    return "send_email_v2 is durable-state-only and network-silent"


def validate_post_commit(post_commit: str | None) -> str:
    if post_commit is None:
        return "no post_commit V2 wake yet; implementation tranche remains pending"

    v2_tokens = (
        "PulseV2RemoteSignal",
        "AgentToInboxV2",
        "inbox-v2",
    )
    if not any(token in post_commit for token in v2_tokens):
        return "post_commit exists but has no V2 wake yet"

    required = {
        "Vec<SignedActionHashed>": "post_commit must consume the committed action batch",
        ".action()": "post_commit must inspect committed actions directly",
        "Action::CreateLink": "V2 wake authority must originate from a committed CreateLink action",
        "ScopedLinkType": "committed link coordinates must be reconstructed explicitly",
        "LinkTypes::try_from": "committed link coordinates must be interpreted through the generated LinkTypes contract",
        "LinkTypes::AgentToInboxV2": "only the V2 inbox-link type may authorize a wake",
        'LinkTag::new("inbox-v2")': "only the exact V2 inbox tag may authorize a wake",
        "base_address": "recipient authority must come from the committed link base",
        "into_agent_pub_key()": "the committed link base must decode as an agent key",
        "PulseV2RemoteSignal::inbox_changed_v2()": "post_commit must emit only the qualified information-poor V2 wake",
    }
    for token, meaning in required.items():
        require(token in post_commit, f"{meaning}: missing {token}")

    require(
        "ExternIO::encode" not in post_commit,
        "typed V2 wake must be passed directly to send_remote_signal; explicit ExternIO::encode would double-encode",
    )
    require(
        "let _ = send_remote_signal" in post_commit,
        "post_commit V2 signaling must be best-effort and non-authoritative",
    )
    require(
        re.search(
            r"send_remote_signal\s*\(\s*PulseV2RemoteSignal::inbox_changed_v2\(\)\s*,\s*vec!\s*\[\s*recipient(?:\.clone\(\))?\s*\]",
            post_commit,
            re.S,
        )
        is not None,
        "post_commit V2 wake must target only the recipient recovered from the committed inbox-link base",
    )
    require(
        re.search(r"target_address[\s\S]{0,160}into_agent_pub_key\(\)", post_commit) is None,
        "recipient may not be derived from the link target; the V2 inbox-link base is the recipient authority",
    )

    forbidden_authority = (
        "EncryptedEmailV2",
        "ciphertext",
        "message_id",
        "encrypted_subject",
        "DeliveryConfirmed",
        "ReadReceiptReceived",
    )
    for token in forbidden_authority:
        require(
            token not in post_commit,
            f"post_commit V2 wake path must not inspect or transport message authority: {token}",
        )

    return "post_commit wake authority is committed-link-only, recipient-only, minimal, and best-effort"


def self_test() -> None:
    good_send = """pub fn send_email_v2() {
        create_entry(x)?;
        LinkTypes::AgentToSentV2;
        LinkTypes::AgentToInboxV2;
        LinkTag::new("inbox-v2");
        Ok(hash)
    }"""
    validate_send_v2(good_send)

    try:
        validate_send_v2(
            good_send.replace(
                "Ok(hash)",
                "let _ = send_remote_signal(PulseV2RemoteSignal::inbox_changed_v2(), vec![input.recipient]); Ok(hash)",
            )
        )
    except BoundaryViolation:
        pass
    else:
        raise BoundaryViolation("self-test: pre-commit V2 wake inside send_email_v2 was not rejected")

    good_post_commit = """pub fn post_commit(actions: Vec<SignedActionHashed>) {
        for signed in actions {
            if let Action::CreateLink(create_link) = signed.action() {
                let scoped = ScopedLinkType { zome_index: create_link.zome_index, zome_type: create_link.link_type };
                if LinkTypes::try_from(scoped) != Ok(LinkTypes::AgentToInboxV2) { continue; }
                if create_link.tag != LinkTag::new("inbox-v2") { continue; }
                let base_address = create_link.base_address.clone();
                let Some(recipient) = base_address.into_agent_pub_key() else { continue; };
                let _ = send_remote_signal(
                    PulseV2RemoteSignal::inbox_changed_v2(),
                    vec![recipient],
                );
            }
        }
    }"""
    validate_post_commit(good_post_commit)

    bad_double_encode = good_post_commit.replace(
        "let _ = send_remote_signal(",
        "let _ = ExternIO::encode(PulseV2RemoteSignal::inbox_changed_v2()); let _ = send_remote_signal(",
    )
    try:
        validate_post_commit(bad_double_encode)
    except BoundaryViolation:
        pass
    else:
        raise BoundaryViolation("self-test: double-encoded post_commit wake was not rejected")

    bad_target = good_post_commit.replace(
        "let base_address = create_link.base_address.clone();\n                let Some(recipient) = base_address.into_agent_pub_key()",
        "let target_address = create_link.target_address.clone();\n                let Some(recipient) = target_address.into_agent_pub_key()",
    )
    try:
        validate_post_commit(bad_target)
    except BoundaryViolation:
        pass
    else:
        raise BoundaryViolation("self-test: target-derived recipient was not rejected")

    bad_link_type = good_post_commit.replace("LinkTypes::AgentToInboxV2", "LinkTypes::AgentToSentV2")
    try:
        validate_post_commit(bad_link_type)
    except BoundaryViolation:
        pass
    else:
        raise BoundaryViolation("self-test: non-inbox V2 link authority was not rejected")

    bad_tag = good_post_commit.replace('LinkTag::new("inbox-v2")', 'LinkTag::new("sent-v2")')
    try:
        validate_post_commit(bad_tag)
    except BoundaryViolation:
        pass
    else:
        raise BoundaryViolation("self-test: wrong committed link tag was not rejected")


def main() -> None:
    try:
        self_test()
        source = COORDINATOR.read_text(encoding="utf-8")
        send_v2 = extract_function(source, "send_email_v2")
        require(send_v2 is not None, "missing function send_email_v2")
        assert send_v2 is not None
        print(f"post-commit-boundary: PASS: {validate_send_v2(send_v2)}")
        post_result = validate_post_commit(extract_function(source, "post_commit"))
        prefix = "PASS" if "committed-link-only" in post_result else "INFO"
        print(f"post-commit-boundary: {prefix}: {post_result}")
    except BoundaryViolation as error:
        print(f"post-commit-boundary: FAIL: {error}", file=sys.stderr)
        raise SystemExit(1) from error


if __name__ == "__main__":
    main()
