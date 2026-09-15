#!/usr/bin/env python3
"""Install the qualified Pulse V2 runtime by exact baseline replacement.

This helper is intentionally temporary. It refuses to write anything unless
all expected theorem-only source fragments occur exactly once. The persistent
runtime qualification workflow and source guards remain the authority after
this installer is removed.
"""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "mycelix-workspace/mycelix-pulse/holochain/zomes/messages/coordinator/Cargo.toml"
SOURCE = ROOT / "mycelix-workspace/mycelix-pulse/holochain/zomes/messages/coordinator/src/lib.rs"


def replace_exact(text: str, old: str, new: str, label: str) -> str:
    count = text.count(old)
    if count != 1:
        raise SystemExit(f"{label}: expected exactly one baseline fragment, found {count}")
    return text.replace(old, new, 1)


manifest = MANIFEST.read_text(encoding="utf-8")
source = SOURCE.read_text(encoding="utf-8")

manifest = replace_exact(
    manifest,
    '''serde = { version = "1.0", features = ["derive"] }
mail_messages_integrity = { path = "../integrity" }
mail-leptos-types = { path = "../../../../crates/mail-leptos-types" }
''',
    '''serde = { version = "1.0", features = ["derive"] }
mail_messages_integrity = { path = "../integrity" }
mail-leptos-types = { path = "../../../../crates/mail-leptos-types" }
pulse-realtime-hdk-adapter = { path = "../../../../crates/pulse-realtime-hdk-adapter" }
pulse-realtime-wire = { path = "../../../../crates/pulse-realtime-wire" }
''',
    "coordinator manifest dependency seam",
)

source = replace_exact(
    source,
    '''use hdk::prelude::*;
use mail_messages_integrity::*;
''',
    '''use hdk::prelude::*;
use mail_messages_integrity::*;
use pulse_realtime_hdk_adapter::admit_extern_io;
use pulse_realtime_wire::{PulseV2RemoteSignal, RemoteSignalAdmission};
use std::collections::HashSet;
''',
    "coordinator import seam",
)

source = replace_exact(
    source,
    '''    Ok(hash)
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct EmailV2Wire {
''',
    '''    Ok(hash)
}

/// Emit an information-poor V2 inbox wake only after Holochain has committed
/// the source-chain transaction that created the recipient's V2 inbox link.
#[hdk_extern(infallible)]
pub fn post_commit(actions: Vec<SignedActionHashed>) {
    for signed in actions {
        let Action::CreateLink(create_link) = signed.action() else {
            continue;
        };
        let scoped = ScopedLinkType {
            zome_index: create_link.zome_index,
            zome_type: create_link.link_type,
        };
        if LinkTypes::try_from(scoped) != Ok(LinkTypes::AgentToInboxV2) {
            continue;
        }
        if create_link.tag != LinkTag::new("inbox-v2") {
            continue;
        }
        let base_address = create_link.base_address.clone();
        let Some(recipient) = base_address.into_agent_pub_key() else {
            continue;
        };
        let _ = send_remote_signal(
            PulseV2RemoteSignal::inbox_changed_v2(),
            vec![recipient],
        );
    }
}

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct EmailV2Wire {
''',
    "post-commit insertion seam",
)

source = replace_exact(
    source,
    '''/// Signal handler for incoming signals
#[hdk_extern]
pub fn recv_remote_signal(signal: ExternIO) -> ExternResult<()> {
    let mail_signal: MailSignal = signal.decode().map_err(|e| {
        wasm_error!(WasmErrorInner::Guest(format!(
            "Failed to decode signal: {}",
            e
        )))
    })?;

    // Forward to UI
    emit_signal(mail_signal)?;

    Ok(())
}
''',
    '''/// V2-only remote wake ingress.
///
/// Remote signals are untrusted scheduling input. They may trigger a local
/// reconciliation wake, but they can never carry authoritative mail semantics.
#[hdk_extern]
pub fn recv_remote_signal(signal: ExternIO) -> ExternResult<()> {
    match admit_extern_io(&signal) {
        RemoteSignalAdmission::PulseV2(Ok(hint)) => {
            emit_signal(hint)?;
        }
        RemoteSignalAdmission::PulseV2(Err(error)) => {
            return Err(wasm_error!(WasmErrorInner::Guest(format!(
                "Rejected Pulse V2 realtime frame: {error:?}"
            ))));
        }
        RemoteSignalAdmission::Legacy(_) => {
            return Err(wasm_error!(WasmErrorInner::Guest(
                "Legacy remote ingress is disabled".to_string()
            )));
        }
    }

    Ok(())
}
''',
    "V2-only receiver seam",
)

source = replace_exact(
    source,
    '''/// Initialize zome - create system folders
#[hdk_extern]
pub fn init(_: ()) -> ExternResult<InitCallbackResult> {
    // Installation must remain side-effect free. The previous initializer
    // committed seven legacy folders, seven links, and a capability grant for
    // every agent before the app could start. That made cold two-conductor
    // startup nondeterministic and none of those writes are required by the
    // V2 encrypted-message lifecycle. Legacy folder provisioning and remote
    // signals are outside the restricted alpha artifact and must not be
    // implied by successful alpha installation.
    Ok(InitCallbackResult::Pass)
}
''',
    '''/// Install only the capability needed for information-poor V2 remote wakes.
#[hdk_extern]
pub fn init(_: ()) -> ExternResult<InitCallbackResult> {
    let mut fns = HashSet::new();
    fns.insert((zome_info()?.name, "recv_remote_signal".into()));
    let functions = GrantedFunctions::Listed(fns);
    create_cap_grant(CapGrantEntry {
        tag: "pulse-v2-remote-ingress-v1".into(),
        access: ().into(),
        functions,
    })?;

    Ok(InitCallbackResult::Pass)
}
''',
    "single-function capability seam",
)

MANIFEST.write_text(manifest, encoding="utf-8")
SOURCE.write_text(source, encoding="utf-8")
print("pulse-v2-runtime-installer: applied exact theorem-to-runtime transformation")
