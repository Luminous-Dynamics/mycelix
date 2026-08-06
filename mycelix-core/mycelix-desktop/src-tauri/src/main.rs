// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
// Prevents additional console window on Windows in release, DO NOT REMOVE!!
#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use tauri::{Manager, State};
use holochain_client::{AdminWebsocket, AllowedOrigins, IssueAppAuthenticationTokenPayload};
use std::sync::Mutex;
use std::process::{Child, Command, Stdio};
use std::fs;
use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};
use serde::{Deserialize, Serialize};
use tokio_tungstenite::{connect_async, tungstenite::Message};
use futures_util::{StreamExt, SinkExt};
use tokio::sync::Mutex as AsyncMutex;
use log;

// Data structures for Holochain admin API responses
#[derive(Debug, Serialize, Deserialize)]
pub struct AppInfo {
    pub installed_app_id: String,
    pub cell_info: Vec<CellInfo>,
    pub status: String,
}

#[derive(Debug, Serialize, Deserialize)]
pub struct CellInfo {
    pub cell_id: String,
    pub dna_hash: String,
    pub agent_pub_key: String,
}

// Configuration for app interface
#[derive(Debug, Clone)]
pub struct AppConfig {
    pub dna_hash: String,
    pub agent_key: String,
}

// Application state
pub struct AppState {
    pub status: Mutex<String>,
    pub holochain_process: Mutex<Option<Child>>,
    pub admin_port: Mutex<u16>,
    pub config: AsyncMutex<AppConfig>,
}

// Basic Tauri command - test the bridge
#[tauri::command]
fn greet(name: &str) -> String {
    format!("Hello, {}! Welcome to Mycelix Network 🍄", name)
}

// Get app status
#[tauri::command]
fn get_status(state: State<AppState>) -> String {
    let status = state.status.lock().expect("status mutex poisoned");
    status.clone()
}

// Update app status
#[tauri::command]
fn set_status(state: State<AppState>, new_status: String) -> Result<String, String> {
    let mut status = state.status.lock().expect("status mutex poisoned");
    *status = new_status.clone();
    Ok(format!("Status updated to: {}", new_status))
}

/// Write a conductor config into the app data directory if one is not there.
///
/// The schema below is the CANONICAL Holochain 0.6.1 shape, taken verbatim from
/// `holochain --create-config` run against the bundled 0.6.1 binary -- not
/// hand-authored, and not carried over from an older release.
///
/// This matters: the config previously checked in at the repo root is a 0.5.6
/// document and Holochain 0.6.1 REFUSES TO PARSE IT --
///     unknown field `device_seed_lair_tag`, expected one of `tracing_override`,
///     `data_root_path`, `keystore`, `admin_interfaces`, `network`,
///     `db_sync_strategy`, `db_max_readers`,
///     `incoming_request_concurrency_limit`, `tuning_params`, `tracing_scope`
/// -- because 0.6 dropped the whole `dpki` block and the device-seed fields, and
/// added `db_max_readers`, `incoming_request_concurrency_limit`, and several
/// network fields.
///
/// Note there is deliberately no `app_interfaces` section. The 0.6 default does
/// not have one; the app interface is attached at runtime over the admin API
/// (see `connect_to_network`), which is also what lets the port be chosen
/// rather than baked in.
///
/// `holochain --create-config` is NOT used directly at runtime, despite existing:
/// it ignores the `-c` path, invents a random sandbox directory, and drops a
/// `.hc` file in the process CWD. Those are developer-sandbox behaviours, not
/// what an installed application should do to a user's filesystem.
fn ensure_conductor_config(data_dir: &Path) -> Result<PathBuf, String> {
    let config_path = data_dir.join("conductor-config.yaml");
    if config_path.exists() {
        return Ok(config_path);
    }

    let holochain_root = data_dir.join("holochain");
    let keystore_root = holochain_root.join("ks");

    // lair-keystore binds a Unix domain socket at <lair_root>/socket, and those
    // are limited to SUN_LEN (108 bytes incl. NUL on Linux, 104 on macOS). Over
    // that, the conductor dies with a bare panic:
    //     Failed to spawn Lair keystore in process
    //     err={"error":"InvalidInput","message":"path must be shorter than SUN_LEN"}
    // which says nothing about paths being too long. Reproduced against the real
    // 0.6.1 binary. Fail with something actionable instead.
    //
    // A typical install is well under the limit (~/.local/share/<id>/holochain/ks
    // is ~50 bytes), so this is a guard, not an expected path. The robust fix is
    // Kangaroo's: symlink the keystore from a short directory. Not done here --
    // it is Unix-only and Windows uses named pipes, so it needs its own design.
    let socket_path = keystore_root.join("socket");
    const SUN_LEN_LIMIT: usize = 104;
    if socket_path.as_os_str().len() >= SUN_LEN_LIMIT {
        return Err(format!(
            "Keystore socket path is too long for a Unix domain socket \
             ({} bytes, limit {SUN_LEN_LIMIT}): {}. \
             Set a shorter app data directory.",
            socket_path.as_os_str().len(),
            socket_path.display()
        ));
    }
    fs::create_dir_all(&keystore_root)
        .map_err(|e| format!("Failed to create {}: {e}", keystore_root.display()))?;

    let yaml = format!(
        "tracing_override: null\n\
         data_root_path: {data}\n\
         keystore:\n\
         \x20 type: lair_server_in_proc\n\
         \x20 lair_root: {ks}\n\
         admin_interfaces:\n\
         - driver:\n\
         \x20   type: websocket\n\
         \x20   port: 0\n\
         \x20   danger_bind_addr: null\n\
         \x20   allowed_origins: '*'\n\
         network:\n\
         \x20 base64_auth_material_bootstrap: null\n\
         \x20 base64_auth_material_relay: null\n\
         \x20 bootstrap_url: https://dev-test-bootstrap2.holochain.org/\n\
         \x20 signal_url: wss://dev-test-bootstrap2.holochain.org/\n\
         \x20 relay_url: https://use1-1.relay.n0.iroh-canary.iroh.link./\n\
         \x20 request_timeout_s: 60\n\
         \x20 webrtc_config: null\n\
         \x20 target_arc_factor: 1\n\
         \x20 report: none\n\
         \x20 advanced: null\n\
         db_sync_strategy: Resilient\n\
         db_max_readers: 24\n\
         incoming_request_concurrency_limit: 21\n\
         tuning_params: null\n\
         tracing_scope: null\n",
        data = holochain_root.display(),
        ks = keystore_root.display(),
    );

    fs::write(&config_path, yaml)
        .map_err(|e| format!("Failed to write {}: {e}", config_path.display()))?;
    log::info!("Generated conductor config at {}", config_path.display());
    Ok(config_path)
}

/// Resolve a binary that ships with the app as a Tauri `externalBin` sidecar.
///
/// A packaged app must NOT search `PATH` for a conductor -- it cannot assume the
/// user has Holochain installed, and picking up a stray system binary of the
/// wrong generation is worse than failing. Tauri installs sidecars alongside the
/// main executable (verified: the built .deb places `usr/bin/holochain` next to
/// `usr/bin/mycelix-desktop`), so resolve relative to `current_exe()`.
///
/// Falls back to the bare name for `nix develop`, where the binary is on PATH and
/// no bundle exists. That fallback is a DEV convenience, not the shipping path.
fn resolve_sidecar(name: &str) -> PathBuf {
    let file_name = if cfg!(target_os = "windows") {
        format!("{name}.exe")
    } else {
        name.to_string()
    };

    if let Ok(exe) = std::env::current_exe() {
        if let Some(dir) = exe.parent() {
            let bundled = dir.join(&file_name);
            if bundled.is_file() {
                return bundled;
            }
        }
    }

    PathBuf::from(file_name)
}

/// Spawn the bundled conductor and return the child plus the admin port it
/// announced.
///
/// Deliberately free of Tauri types. Everything interesting about first run --
/// config generation, sidecar resolution, passphrase piping, port discovery --
/// used to live inside a `#[tauri::command]`, and Tauri commands are invoked
/// *from the frontend*. That made this logic unreachable without a working
/// webview, and the webview cannot start in a headless/CI environment
/// (WebKitGTK aborts with EGL_BAD_PARAMETER). So none of it could be tested at
/// all. Keeping it here, as a plain function, is what makes `--self-test` and
/// therefore CI possible.
fn spawn_conductor(data_dir: &Path) -> Result<(Child, u16), String> {
    fs::create_dir_all(data_dir)
        .map_err(|e| format!("Failed to create {}: {e}", data_dir.display()))?;

    let config_path = ensure_conductor_config(data_dir)?;

    // Bundled sidecar (scripts/fetch-sidecars.sh + tauri.conf.json externalBin),
    // NOT a PATH lookup.
    let holochain_cmd = resolve_sidecar("holochain");

    let mut child = Command::new(&holochain_cmd)
        .arg("--piped")
        .arg("-c")
        .arg(&config_path)
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|e| {
            format!(
                "Failed to start conductor at {}: {e}",
                holochain_cmd.display()
            )
        })?;

    // --piped reads the keystore passphrase from stdin.
    if let Some(mut stdin) = child.stdin.take() {
        use std::io::Write;
        let _ = stdin.write_all(b"\n");
    }

    // The config requests port 0, so the OS assigns and the conductor announces
    // the result as `###ADMIN_PORT:<n>###`. Read it rather than assuming: 8888 is
    // both Holochain's dev default and, on the build host, the live
    // mail-conductor's app interface.
    let stdout = child
        .stdout
        .take()
        .ok_or_else(|| "conductor stdout unavailable".to_string())?;

    let mut discovered: Option<u16> = None;
    let mut reader = BufReader::new(stdout);
    let mut line = String::new();
    for _ in 0..200 {
        line.clear();
        match reader.read_line(&mut line) {
            Ok(0) => break, // EOF: conductor exited
            Ok(_) => {
                if let Some(rest) = line.split("###ADMIN_PORT:").nth(1) {
                    if let Some(num) = rest.split("###").next() {
                        if let Ok(port) = num.trim().parse::<u16>() {
                            discovered = Some(port);
                            break;
                        }
                    }
                }
            }
            Err(e) => return Err(format!("Failed reading conductor output: {e}")),
        }
    }

    match discovered {
        Some(port) => Ok((child, port)),
        None => {
            let _ = child.kill();
            Err("Conductor did not announce an admin port (###ADMIN_PORT:n###) -- \
                 it failed to start. Check the keystore path length and config validity."
                .to_string())
        }
    }
}

// Start Holochain conductor
#[tauri::command]
async fn start_holochain(app: tauri::AppHandle, state: State<'_, AppState>) -> Result<String, String> {
    {
        let guard = state
            .holochain_process
            .lock()
            .expect("holochain_process mutex poisoned");
        if guard.is_some() {
            return Ok("Holochain conductor already running".to_string());
        }
    }

    let data_dir = app
        .path()
        .app_data_dir()
        .map_err(|e| format!("Could not resolve app data directory: {e}"))?;

    let (child, port) = spawn_conductor(&data_dir)?;

    *state.admin_port.lock().expect("admin_port mutex poisoned") = port;
    *state
        .holochain_process
        .lock()
        .expect("holochain_process mutex poisoned") = Some(child);

    Ok(format!("Holochain conductor started on admin port {port}"))
}

#[tauri::command]
async fn stop_holochain(state: State<'_, AppState>) -> Result<String, String> {
    let mut process_guard = state.holochain_process.lock().expect("holochain_process mutex poisoned");

    match process_guard.take() {
        Some(mut child) => {
            match child.kill() {
                Ok(_) => Ok("Holochain conductor stopped successfully".to_string()),
                Err(e) => Err(format!("Failed to stop Holochain conductor: {}", e)),
            }
        }
        None => Ok("Holochain conductor is not running".to_string()),
    }
}

// Check if Holochain is running
#[tauri::command]
fn check_holochain_status(state: State<'_, AppState>) -> Result<String, String> {
    let mut process_guard = state.holochain_process.lock().expect("holochain_process mutex poisoned");

    match process_guard.as_mut() {
        Some(child) => {
            match child.try_wait() {
                Ok(Some(status)) => {
                    // Process has exited
                    *process_guard = None;
                    Ok(format!("Holochain conductor exited with status: {}", status))
                }
                Ok(None) => {
                    // Still running
                    Ok("Holochain conductor is running".to_string())
                }
                Err(e) => Err(format!("Error checking conductor status: {}", e)),
            }
        }
        None => Ok("Holochain conductor is not running".to_string()),
    }
}

// P2P network connection state
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NetworkInfo {
    pub connected_peers: Vec<String>,
    pub agent_pub_key: String,
    pub network_seed: Option<String>,
}

/// Connect to the conductor's admin interface.
///
/// Uses holochain_client's typed `AdminWebsocket`. The previous implementation
/// hand-rolled JSON-RPC 2.0 text frames (`{"jsonrpc":"2.0","method":...}`), a
/// protocol the Holochain admin API has never spoken -- it is MessagePack over
/// a websocket. That code could not ever have worked; this replaces it rather
/// than re-implementing the wire format by hand.
async fn admin_ws(port: u16) -> Result<AdminWebsocket, String> {
    AdminWebsocket::connect(format!("127.0.0.1:{port}"), None)
        .await
        .map_err(|e| format!("Failed to connect to admin interface on port {port}: {e:?}"))
}

// Connect to P2P network via Holochain conductor
#[tauri::command]
async fn connect_to_network(state: State<'_, AppState>) -> Result<String, String> {
    let port = *state.admin_port.lock().expect("admin_port mutex poisoned");
    let admin = admin_ws(port).await?;

    // Step 1: is the conductor actually reachable?
    let apps = admin
        .list_apps(None)
        .await
        .map_err(|e| format!("Holochain conductor not running or not reachable: {e:?}"))?;

    // Step 2: attach an app interface for the webview. Already-attached is not
    // fatal, so this warns rather than failing the whole connect.
    // Port 0 => the conductor picks a free one and returns it. attach_app_interface
    // returning u16 is the whole point; an already-attached interface is not fatal.
    match admin
        .attach_app_interface(0, None, AllowedOrigins::Any, None)
        .await
    {
        Ok(app_port) => log::info!("App interface attached on port {app_port}"),
        Err(e) => log::warn!("Could not attach app interface: {e:?}"),
    }

    // Step 3: agent infos across all DNAs (None = no DNA filter).
    let agent_infos = admin.agent_info(None).await.map_err(|e| {
        format!(
            "Holochain conductor running but P2P network not initialized. \
             Ensure a hApp is installed and enabled. Error: {e:?}"
        )
    })?;

    // Step 4: transport stats. Read generically via serde rather than binding to
    // kitsune2's struct shape, which is not part of this crate's contract.
    let peer_count = match admin.dump_network_stats().await {
        Ok(stats) => serde_json::to_value(&stats)
            .ok()
            .and_then(|v| {
                v.get("connections")
                    .and_then(|c| c.as_array())
                    .map(|a| a.len())
            })
            .unwrap_or(0),
        Err(e) => {
            log::warn!("dump_network_stats failed: {e:?}");
            0
        }
    };

    let network_info = NetworkInfo {
        connected_peers: agent_infos.clone(),
        agent_pub_key: agent_infos
            .first()
            .cloned()
            .unwrap_or_else(|| "unknown".to_string()),
        network_seed: None,
    };

    Ok(format!(
        "Connected to P2P network. Apps installed: {}, agent infos: {}, peers: {}",
        apps.len(),
        network_info.connected_peers.len(),
        peer_count
    ))
}

// Get list of installed apps
#[tauri::command]
async fn get_installed_apps(state: State<'_, AppState>) -> Result<String, String> {
    let port = *state.admin_port.lock().expect("admin_port mutex poisoned");

    let apps = admin_ws(port)
        .await?
        .list_apps(None)
        .await
        .map_err(|e| format!("list_apps failed: {e:?}"))?;

    serde_json::to_string_pretty(&apps).map_err(|e| format!("Failed to format response: {e}"))
}

// Get list of cells
#[tauri::command]
async fn get_cells(state: State<'_, AppState>) -> Result<String, String> {
    let port = *state.admin_port.lock().expect("admin_port mutex poisoned");

    let cell_ids = admin_ws(port)
        .await?
        .list_cell_ids()
        .await
        .map_err(|e| format!("list_cell_ids failed: {e:?}"))?;

    serde_json::to_string_pretty(&cell_ids).map_err(|e| format!("Failed to format response: {e}"))
}

// Enable an app
#[tauri::command]
async fn enable_app(state: State<'_, AppState>, app_id: String) -> Result<String, String> {
    let port = *state.admin_port.lock().expect("admin_port mutex poisoned");

    admin_ws(port)
        .await?
        .enable_app(app_id.clone())
        .await
        .map_err(|e| format!("enable_app failed: {e:?}"))?;

    Ok(format!("App '{app_id}' enabled successfully"))
}

// Disable an app
#[tauri::command]
async fn disable_app(state: State<'_, AppState>, app_id: String) -> Result<String, String> {
    let port = *state.admin_port.lock().expect("admin_port mutex poisoned");

    admin_ws(port)
        .await?
        .disable_app(app_id.clone())
        .await
        .map_err(|e| format!("disable_app failed: {e:?}"))?;

    Ok(format!("App '{app_id}' disabled successfully"))
}

// Get app info with details
#[tauri::command]
async fn get_app_info(state: State<'_, AppState>, app_id: String) -> Result<String, String> {
    let port = *state.admin_port.lock().expect("admin_port mutex poisoned");

    // The admin API has no get_app_info (that lives on the *app* interface), so
    // resolve it by filtering list_apps.
    let apps = admin_ws(port)
        .await?
        .list_apps(None)
        .await
        .map_err(|e| format!("list_apps failed: {e:?}"))?;

    let app = apps
        .into_iter()
        .find(|info| info.installed_app_id == app_id)
        .ok_or_else(|| format!("App '{app_id}' is not installed"))?;

    serde_json::to_string_pretty(&app).map_err(|e| format!("Failed to format response: {e}"))
}

// App API helper function for zome calls
async fn send_app_request(port: u16, method: &str, params: serde_json::Value) -> Result<serde_json::Value, String> {
    let url = format!("ws://localhost:{}", port);
    let (ws_stream, _) = connect_async(&url).await
        .map_err(|e| format!("Failed to connect to app interface: {}", e))?;

    let (mut write, mut read) = ws_stream.split();

    let request = serde_json::json!({
        "type": method,
        "data": params
    });

    write.send(Message::Text(request.to_string())).await
        .map_err(|e| format!("Failed to send request: {}", e))?;

    if let Some(Ok(Message::Text(response_text))) = read.next().await {
        let response: serde_json::Value = serde_json::from_str(&response_text)
            .map_err(|e| format!("Failed to parse response: {}", e))?;

        if let Some(error) = response.get("error") {
            return Err(format!("Request failed: {}", error));
        }

        Ok(response.get("data").unwrap_or(&serde_json::Value::Null).clone())
    } else {
        Err("No response received".to_string())
    }
}

// Generic zome call command
#[tauri::command]
async fn call_zome_function(
    state: State<'_, AppState>,
    zome_name: String,
    function_name: String,
    payload: Option<String>
) -> Result<String, String> {
    let config = state.config.lock().await;
    let app_port = 8889; // App interface port

    let payload_value: serde_json::Value = if let Some(p) = payload {
        serde_json::from_str(&p).unwrap_or_default()
    } else {
        serde_json::Value::Null
    };

    let params = serde_json::json!({
        "app_id": "mycelix-test",
        "cell_id": [
            config.dna_hash.clone(),
            config.agent_key.clone()
        ],
        "zome_name": zome_name,
        "fn_name": function_name,
        "payload": payload_value
    });

    let response = send_app_request(app_port, "call_zome", params).await?;
    Ok(serde_json::to_string_pretty(&response).unwrap_or_else(|_| response.to_string()))
}

// Convenience command: hello()
#[tauri::command]
async fn call_hello(state: State<'_, AppState>) -> Result<String, String> {
    call_zome_function(state, "hello".to_string(), "hello".to_string(), None).await
}

// Convenience command: whoami()
#[tauri::command]
async fn call_whoami(state: State<'_, AppState>) -> Result<String, String> {
    call_zome_function(state, "hello".to_string(), "whoami".to_string(), None).await
}

// Convenience command: echo(input)
#[tauri::command]
async fn call_echo(state: State<'_, AppState>, input: String) -> Result<String, String> {
    call_zome_function(state, "hello".to_string(), "echo".to_string(), Some(format!(r#"{{"input": "{}"}}"#, input))).await
}

// Convenience command: get_agent_info()
#[tauri::command]
async fn call_get_agent_info(state: State<'_, AppState>) -> Result<String, String> {
    call_zome_function(state, "hello".to_string(), "get_agent_info".to_string(), None).await
}

// Message broadcasting commands

// Send a message to the network
#[tauri::command]
async fn send_message(state: State<'_, AppState>, content: String) -> Result<String, String> {
    let payload = format!(r#"{{"content": "{}"}}"#, content);
    call_zome_function(state, "messages".to_string(), "broadcast_message".to_string(), Some(payload)).await
}

// Get all messages from the network
#[tauri::command]
async fn get_messages(state: State<'_, AppState>) -> Result<String, String> {
    call_zome_function(state, "messages".to_string(), "get_all_messages".to_string(), None).await
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
/// Headless end-to-end check of first-run conductor bring-up.
///
/// Exists because every path it exercises is otherwise reachable only through a
/// `#[tauri::command]`, i.e. only from a frontend, i.e. never in CI or on a
/// headless host -- WebKitGTK aborts with EGL_BAD_PARAMETER before any JS runs.
/// Without this, "the app launched" is the strongest claim available, and it is
/// worth nothing: the process can start, create its WebKit storage, and exit
/// cleanly having executed none of the logic below.
///
/// Uses a throwaway data directory so it never touches real user state. Exit 0
/// = the whole chain worked against a REAL bundled conductor.
fn run_self_test() -> i32 {
    let dir = std::env::temp_dir().join(format!("mycelix-selftest-{}", std::process::id()));
    println!("[self-test] data dir: {}", dir.display());

    let (mut child, port) = match spawn_conductor(&dir) {
        Ok(v) => v,
        Err(e) => {
            eprintln!("[self-test] FAIL spawn_conductor: {e}");
            let _ = fs::remove_dir_all(&dir);
            return 1;
        }
    };
    println!("[self-test] ok   conductor up, admin port {port}");

    let rt = match tokio::runtime::Runtime::new() {
        Ok(rt) => rt,
        Err(e) => {
            eprintln!("[self-test] FAIL tokio runtime: {e}");
            let _ = child.kill();
            return 1;
        }
    };

    let code = rt.block_on(async {
        let admin = match admin_ws(port).await {
            Ok(a) => a,
            Err(e) => {
                eprintln!("[self-test] FAIL admin connect: {e}");
                return 1;
            }
        };
        println!("[self-test] ok   admin websocket connected (MessagePack)");

        match admin.list_apps(None).await {
            Ok(apps) => println!("[self-test] ok   list_apps -> {} app(s)", apps.len()),
            Err(e) => {
                eprintln!("[self-test] FAIL list_apps: {e:?}");
                return 1;
            }
        }

        match admin.attach_app_interface(0, None, AllowedOrigins::Any, None).await {
            Ok(app_port) => println!("[self-test] ok   app interface attached on {app_port}"),
            Err(e) => {
                eprintln!("[self-test] FAIL attach_app_interface: {e:?}");
                return 1;
            }
        }

        // Optional stage: install a real hApp. Skipped unless a bundle is named,
        // so the default self-test stays fast and dependency-free -- but when a
        // bundle IS given this covers install_app -> enable_app -> auth token,
        // which are the prerequisites for any zome call. Without them the app
        // path (send_app_request) cannot be tested at all, only compiled.
        //   MYCELIX_SELFTEST_HAPP=/path/to/x.happ ./scripts/self-test.sh --no-build
        match std::env::var("MYCELIX_SELFTEST_HAPP") {
            Err(_) => println!("[self-test] skip app install (set MYCELIX_SELFTEST_HAPP to cover it)"),
            Ok(happ) => {
                let path = PathBuf::from(&happ);
                if !path.is_file() {
                    eprintln!("[self-test] FAIL MYCELIX_SELFTEST_HAPP is not a file: {happ}");
                    return 1;
                }
                let app_id = "mycelix-selftest".to_string();

                match admin
                    .install_app(holochain_types::app::InstallAppPayload {
                        source: holochain_types::app::AppBundleSource::Path(path),
                        agent_key: None,
                        installed_app_id: Some(app_id.clone()),
                        network_seed: None,
                        roles_settings: None,
                        ignore_genesis_failure: false,
                    })
                    .await
                {
                    Ok(info) => println!(
                        "[self-test] ok   install_app -> {} ({} cell role(s))",
                        info.installed_app_id,
                        info.cell_info.len()
                    ),
                    Err(e) => {
                        eprintln!("[self-test] FAIL install_app: {e:?}");
                        return 1;
                    }
                }

                if let Err(e) = admin.enable_app(app_id.clone()).await {
                    eprintln!("[self-test] FAIL enable_app: {e:?}");
                    return 1;
                }
                println!("[self-test] ok   enable_app");

                match admin
                    .issue_app_auth_token(IssueAppAuthenticationTokenPayload {
                        installed_app_id: app_id.clone(),
                        expiry_seconds: 60,
                        single_use: false,
                    })
                    .await
                {
                    Ok(t) => println!(
                        "[self-test] ok   issue_app_auth_token ({} bytes)",
                        t.token.len()
                    ),
                    Err(e) => {
                        eprintln!("[self-test] FAIL issue_app_auth_token: {e:?}");
                        return 1;
                    }
                }
            }
        }
        0
    });

    let _ = child.kill();
    let _ = child.wait();
    let _ = fs::remove_dir_all(&dir);
    println!(
        "[self-test] {}",
        if code == 0 { "PASS" } else { "FAIL" }
    );
    code
}

fn main() {
    if std::env::args().any(|a| a == "--self-test") {
        std::process::exit(run_self_test());
    }

    tauri::Builder::default()
        .manage(AppState {
            status: Mutex::new("Initializing...".to_string()),
            holochain_process: Mutex::new(None),
            admin_port: Mutex::new(8888), // Default admin port
            config: AsyncMutex::new(AppConfig {
                dna_hash: "uhC0k98l2UnvhZtuuze40Mlhxpon_fjBq6LVQ0Um9RREhC_maHDlI".to_string(),
                agent_key: "uhCAkHZV67oPQGx9-dVV2j73kJJgOBaP4N8Fqjq19o-v0sLhvb8Ik".to_string(),
            }),
        })
        .invoke_handler(tauri::generate_handler![
            greet,
            get_status,
            set_status,
            start_holochain,
            stop_holochain,
            check_holochain_status,
            connect_to_network,
            get_installed_apps,
            get_cells,
            enable_app,
            disable_app,
            get_app_info,
            call_zome_function,
            call_hello,
            call_whoami,
            call_echo,
            call_get_agent_info,
            send_message,
            get_messages,
        ])
        .run(tauri::generate_context!())
        .expect("error while running tauri application");
}