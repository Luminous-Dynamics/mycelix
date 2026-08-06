// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Supervised renderer ↔ Spore process lifecycle.
//!
//! The authenticated codec is only useful when both endpoints receive the same
//! fresh key and session identifier without leaking either value through argv,
//! environment variables, logs, or files. This module supplies that ceremony:
//!
//! 1. Prism creates a private per-session runtime directory.
//! 2. Prism generates a fresh 256-bit key and 128-bit session identifier.
//! 3. The bootstrap is written once to the child's inherited stdin.
//! 4. The Spore child binds the supplied Unix socket and accepts one peer.
//! 5. All subsequent messages use [`crate::BridgeCodec`].
//! 6. Dropping the supervisor terminates the child and removes session state.

use crate::{
    BridgeCodec, BridgeError, BridgeSessionKey, MAX_FRAME_SIZE, RendererToSpore, SporeToRenderer,
};
use serde::{Deserialize, Serialize};
use std::fmt;
use std::io::{Read, Write};
use std::path::{Path, PathBuf};
use std::process::{Child, Command, Stdio};
use std::time::Duration;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::{UnixListener, UnixStream};
use tokio::time::{Instant, sleep, timeout};

const BOOTSTRAP_VERSION: u16 = 1;
const MAX_BOOTSTRAP_SIZE: usize = 16 * 1024;
const DEFAULT_STARTUP_TIMEOUT: Duration = Duration::from_secs(10);

#[derive(Debug)]
pub enum BridgeRuntimeError {
    Bridge(BridgeError),
    Configuration(String),
    Bootstrap(String),
    ProcessExited(Option<i32>),
    StartupTimeout,
    HealthTimeout,
    UnexpectedHealthResponse,
    Io(std::io::Error),
}

impl fmt::Display for BridgeRuntimeError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Bridge(error) => write!(f, "bridge protocol error: {error}"),
            Self::Configuration(error) => write!(f, "bridge process configuration error: {error}"),
            Self::Bootstrap(error) => write!(f, "bridge bootstrap error: {error}"),
            Self::ProcessExited(code) => write!(f, "Spore process exited during startup: {code:?}"),
            Self::StartupTimeout => f.write_str("timed out waiting for the Spore bridge socket"),
            Self::HealthTimeout => f.write_str("timed out waiting for the Spore health response"),
            Self::UnexpectedHealthResponse => {
                f.write_str("Spore returned an unexpected response to the health check")
            }
            Self::Io(error) => write!(f, "bridge runtime I/O error: {error}"),
        }
    }
}

impl std::error::Error for BridgeRuntimeError {}

impl From<BridgeError> for BridgeRuntimeError {
    fn from(error: BridgeError) -> Self {
        Self::Bridge(error)
    }
}

impl From<std::io::Error> for BridgeRuntimeError {
    fn from(error: std::io::Error) -> Self {
        Self::Io(error)
    }
}

/// One-shot secret bootstrap delivered through inherited stdin.
///
/// The key is intentionally private and this type has a redacted `Debug`
/// implementation. A child converts it directly into a [`SporeBridgeEndpoint`]
/// rather than exposing the secret to application code.
#[derive(Clone, Serialize, Deserialize)]
struct BridgeBootstrap {
    version: u16,
    socket_path: PathBuf,
    session_id: [u8; 16],
    session_key: [u8; 32],
}

impl fmt::Debug for BridgeBootstrap {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("BridgeBootstrap")
            .field("version", &self.version)
            .field("socket_path", &self.socket_path)
            .field("session_id", &self.session_id)
            .field("session_key", &"[REDACTED]")
            .finish()
    }
}

impl BridgeBootstrap {
    fn new(socket_path: PathBuf) -> Result<Self, BridgeRuntimeError> {
        let mut session_id = [0u8; 16];
        let mut session_key = [0u8; 32];
        getrandom::fill(&mut session_id)
            .map_err(|error| BridgeRuntimeError::Bootstrap(error.to_string()))?;
        getrandom::fill(&mut session_key)
            .map_err(|error| BridgeRuntimeError::Bootstrap(error.to_string()))?;
        if session_id == [0; 16] {
            // A zero identifier is forbidden by BridgeCodec. The probability is
            // negligible, but failing closed is preferable to silently changing
            // generated security material.
            return Err(BridgeRuntimeError::Bootstrap(
                "random source returned an invalid zero session identifier".to_string(),
            ));
        }
        Ok(Self {
            version: BOOTSTRAP_VERSION,
            socket_path,
            session_id,
            session_key,
        })
    }

    fn write_to(&self, mut writer: impl Write) -> Result<(), BridgeRuntimeError> {
        let payload = rmp_serde::to_vec(self)
            .map_err(|error| BridgeRuntimeError::Bootstrap(error.to_string()))?;
        if payload.len() > MAX_BOOTSTRAP_SIZE {
            return Err(BridgeRuntimeError::Bootstrap(
                "serialized bootstrap exceeded its fixed size bound".to_string(),
            ));
        }
        writer.write_all(&(payload.len() as u32).to_be_bytes())?;
        writer.write_all(&payload)?;
        writer.flush()?;
        Ok(())
    }

    fn read_from(mut reader: impl Read) -> Result<Self, BridgeRuntimeError> {
        let mut length = [0u8; 4];
        reader.read_exact(&mut length)?;
        let length = u32::from_be_bytes(length) as usize;
        if length == 0 || length > MAX_BOOTSTRAP_SIZE {
            return Err(BridgeRuntimeError::Bootstrap(format!(
                "invalid bootstrap frame length {length}"
            )));
        }
        let mut payload = vec![0u8; length];
        reader.read_exact(&mut payload)?;
        let bootstrap: Self = rmp_serde::from_slice(&payload)
            .map_err(|error| BridgeRuntimeError::Bootstrap(error.to_string()))?;
        if bootstrap.version != BOOTSTRAP_VERSION {
            return Err(BridgeRuntimeError::Bootstrap(format!(
                "unsupported bootstrap version {}; expected {}",
                bootstrap.version, BOOTSTRAP_VERSION
            )));
        }
        if !bootstrap.socket_path.is_absolute() {
            return Err(BridgeRuntimeError::Bootstrap(
                "bootstrap socket path must be absolute".to_string(),
            ));
        }
        Ok(bootstrap)
    }
}

/// Child-side endpoint obtained by consuming the one-shot stdin bootstrap.
pub struct SporeBridgeEndpoint {
    socket_path: PathBuf,
    codec: BridgeCodec,
}

impl fmt::Debug for SporeBridgeEndpoint {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SporeBridgeEndpoint")
            .field("socket_path", &self.socket_path)
            .field("codec", &self.codec)
            .finish()
    }
}

impl SporeBridgeEndpoint {
    /// Read the supervisor bootstrap from stdin. Spore binaries should call
    /// this exactly once during startup and must not retain the raw bytes.
    pub fn from_stdin() -> Result<Self, BridgeRuntimeError> {
        Self::from_reader(std::io::stdin().lock())
    }

    pub fn from_reader(reader: impl Read) -> Result<Self, BridgeRuntimeError> {
        let bootstrap = BridgeBootstrap::read_from(reader)?;
        let codec = BridgeCodec::new(
            BridgeSessionKey::from_bytes(bootstrap.session_key),
            bootstrap.session_id,
        )?;
        Ok(Self {
            socket_path: bootstrap.socket_path,
            codec,
        })
    }

    pub fn socket_path(&self) -> &Path {
        &self.socket_path
    }

    /// Bind the private socket and accept the single supervised renderer peer.
    pub async fn accept(mut self) -> Result<SporeBridgeConnection, BridgeRuntimeError> {
        if self.socket_path.exists() {
            return Err(BridgeRuntimeError::Configuration(format!(
                "refusing to replace existing bridge socket {}",
                self.socket_path.display()
            )));
        }
        let listener = UnixListener::bind(&self.socket_path)?;
        set_private_socket_permissions(&self.socket_path)?;
        let (stream, _) = listener.accept().await?;
        Ok(SporeBridgeConnection {
            stream,
            codec: self.codec,
        })
    }
}

/// Child-side authenticated connection.
pub struct SporeBridgeConnection {
    stream: UnixStream,
    codec: BridgeCodec,
}

impl SporeBridgeConnection {
    pub async fn receive_renderer(&mut self) -> Result<RendererToSpore, BridgeRuntimeError> {
        let payload = read_frame(&mut self.stream).await?;
        Ok(self.codec.decode_renderer_message(&payload)?)
    }

    pub async fn send_spore(&mut self, message: &SporeToRenderer) -> Result<(), BridgeRuntimeError> {
        let frame = self.codec.encode_spore_message(message)?;
        self.stream.write_all(&frame).await?;
        self.stream.flush().await?;
        Ok(())
    }
}

/// Operator-owned process configuration. The executable must be an absolute,
/// canonical regular file; shell parsing is never used.
#[derive(Debug, Clone)]
pub struct SporeProcessConfig {
    executable: PathBuf,
    runtime_root: PathBuf,
    startup_timeout: Duration,
}

impl SporeProcessConfig {
    pub fn from_env() -> Result<Option<Self>, BridgeRuntimeError> {
        let Some(executable) = std::env::var_os("PRISM_SPORE_EXECUTABLE") else {
            return Ok(None);
        };
        let runtime_root = std::env::var_os("PRISM_BRIDGE_RUNTIME_DIR")
            .map(PathBuf::from)
            .or_else(default_runtime_root)
            .ok_or_else(|| {
                BridgeRuntimeError::Configuration(
                    "no private runtime directory is available".to_string(),
                )
            })?;
        Self::new(executable, runtime_root).map(Some)
    }

    pub fn new(
        executable: impl Into<PathBuf>,
        runtime_root: impl Into<PathBuf>,
    ) -> Result<Self, BridgeRuntimeError> {
        let executable = executable.into();
        if !executable.is_absolute() {
            return Err(BridgeRuntimeError::Configuration(
                "Spore executable path must be absolute".to_string(),
            ));
        }
        let executable = executable.canonicalize().map_err(|error| {
            BridgeRuntimeError::Configuration(format!(
                "Spore executable is unavailable: {error}"
            ))
        })?;
        if !executable.is_file() {
            return Err(BridgeRuntimeError::Configuration(
                "Spore executable is not a regular file".to_string(),
            ));
        }
        let runtime_root = runtime_root.into();
        if !runtime_root.is_absolute() {
            return Err(BridgeRuntimeError::Configuration(
                "bridge runtime directory must be absolute".to_string(),
            ));
        }
        Ok(Self {
            executable,
            runtime_root,
            startup_timeout: DEFAULT_STARTUP_TIMEOUT,
        })
    }

    pub fn with_startup_timeout(mut self, timeout: Duration) -> Self {
        self.startup_timeout = timeout;
        self
    }
}

/// Parent-side owner of the Spore child, authenticated stream, and private
/// session directory.
pub struct SupervisedSpore {
    child: Child,
    stream: UnixStream,
    codec: BridgeCodec,
    session_dir: PathBuf,
}

impl fmt::Debug for SupervisedSpore {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("SupervisedSpore")
            .field("process_id", &self.child.id())
            .field("session_dir", &self.session_dir)
            .field("codec", &self.codec)
            .finish()
    }
}

impl SupervisedSpore {
    pub async fn spawn(config: SporeProcessConfig) -> Result<Self, BridgeRuntimeError> {
        create_private_directory(&config.runtime_root)?;

        let session_dir = create_session_directory(&config.runtime_root)?;
        let socket_path = session_dir.join("spore.sock");
        let bootstrap = BridgeBootstrap::new(socket_path.clone())?;
        let codec = BridgeCodec::new(
            BridgeSessionKey::from_bytes(bootstrap.session_key),
            bootstrap.session_id,
        )?;

        let mut child = Command::new(&config.executable)
            .env("PRISM_BRIDGE_BOOTSTRAP", "stdin-v1")
            .stdin(Stdio::piped())
            .stdout(Stdio::inherit())
            .stderr(Stdio::inherit())
            .spawn()
            .map_err(|error| {
                BridgeRuntimeError::Configuration(format!(
                    "could not launch Spore executable: {error}"
                ))
            })?;

        let bootstrap_result = child
            .stdin
            .take()
            .ok_or_else(|| {
                BridgeRuntimeError::Bootstrap(
                    "child process did not expose the requested stdin pipe".to_string(),
                )
            })
            .and_then(|stdin| bootstrap.write_to(stdin));
        if let Err(error) = bootstrap_result {
            terminate_child(&mut child);
            let _ = std::fs::remove_dir_all(&session_dir);
            return Err(error);
        }

        let deadline = Instant::now() + config.startup_timeout;
        let stream = loop {
            if let Some(status) = child.try_wait()? {
                let _ = std::fs::remove_dir_all(&session_dir);
                return Err(BridgeRuntimeError::ProcessExited(status.code()));
            }
            match UnixStream::connect(&socket_path).await {
                Ok(stream) => break stream,
                Err(error)
                    if matches!(
                        error.kind(),
                        std::io::ErrorKind::NotFound | std::io::ErrorKind::ConnectionRefused
                    ) =>
                {
                    if Instant::now() >= deadline {
                        terminate_child(&mut child);
                        let _ = std::fs::remove_dir_all(&session_dir);
                        return Err(BridgeRuntimeError::StartupTimeout);
                    }
                    sleep(Duration::from_millis(25)).await;
                }
                Err(error) => {
                    terminate_child(&mut child);
                    let _ = std::fs::remove_dir_all(&session_dir);
                    return Err(error.into());
                }
            }
        };

        Ok(Self {
            child,
            stream,
            codec,
            session_dir,
        })
    }

    pub fn process_id(&self) -> u32 {
        self.child.id()
    }

    pub async fn send_renderer(
        &mut self,
        message: &RendererToSpore,
    ) -> Result<(), BridgeRuntimeError> {
        let frame = self.codec.encode_renderer_message(message)?;
        self.stream.write_all(&frame).await?;
        self.stream.flush().await?;
        Ok(())
    }

    pub async fn receive_spore(&mut self) -> Result<SporeToRenderer, BridgeRuntimeError> {
        let payload = read_frame(&mut self.stream).await?;
        Ok(self.codec.decode_spore_message(&payload)?)
    }

    /// Prove that the launched process has decoded the same authenticated
    /// session. Heartbeats may arrive before the matching pong and are ignored.
    pub async fn health_check(&mut self, duration: Duration) -> Result<(), BridgeRuntimeError> {
        let mut nonce_bytes = [0u8; 8];
        getrandom::fill(&mut nonce_bytes)
            .map_err(|error| BridgeRuntimeError::Bootstrap(error.to_string()))?;
        let nonce = u64::from_be_bytes(nonce_bytes);
        self.send_renderer(&RendererToSpore::Ping(nonce)).await?;

        timeout(duration, async {
            for _ in 0..8 {
                match self.receive_spore().await? {
                    SporeToRenderer::Pong(received) if received == nonce => return Ok(()),
                    SporeToRenderer::Heartbeat { .. } => continue,
                    _ => return Err(BridgeRuntimeError::UnexpectedHealthResponse),
                }
            }
            Err(BridgeRuntimeError::UnexpectedHealthResponse)
        })
        .await
        .map_err(|_| BridgeRuntimeError::HealthTimeout)?
    }

    pub fn try_wait(&mut self) -> Result<Option<std::process::ExitStatus>, BridgeRuntimeError> {
        Ok(self.child.try_wait()?)
    }

    pub fn shutdown(mut self) {
        terminate_child(&mut self.child);
        let _ = std::fs::remove_dir_all(&self.session_dir);
    }
}

impl Drop for SupervisedSpore {
    fn drop(&mut self) {
        terminate_child(&mut self.child);
        let _ = std::fs::remove_dir_all(&self.session_dir);
    }
}

async fn read_frame(stream: &mut UnixStream) -> Result<Vec<u8>, BridgeRuntimeError> {
    let mut length = [0u8; 4];
    stream.read_exact(&mut length).await?;
    let length = u32::from_be_bytes(length) as usize;
    if length == 0 || length > MAX_FRAME_SIZE {
        return Err(BridgeError::FrameTooLarge {
            size: length,
            max: MAX_FRAME_SIZE,
        }
        .into());
    }
    let mut payload = vec![0u8; length];
    stream.read_exact(&mut payload).await?;
    Ok(payload)
}

fn create_session_directory(root: &Path) -> Result<PathBuf, BridgeRuntimeError> {
    for _ in 0..8 {
        let mut random = [0u8; 12];
        getrandom::fill(&mut random)
            .map_err(|error| BridgeRuntimeError::Bootstrap(error.to_string()))?;
        let candidate = root.join(format!("session-{}", short_hex(&random)));
        match std::fs::create_dir(&candidate) {
            Ok(()) => {
                set_private_directory_permissions(&candidate)?;
                return Ok(candidate);
            }
            Err(error) if error.kind() == std::io::ErrorKind::AlreadyExists => continue,
            Err(error) => return Err(error.into()),
        }
    }
    Err(BridgeRuntimeError::Configuration(
        "could not allocate a unique bridge session directory".to_string(),
    ))
}

fn create_private_directory(path: &Path) -> Result<(), BridgeRuntimeError> {
    std::fs::create_dir_all(path)?;
    set_private_directory_permissions(path)
}

fn set_private_directory_permissions(path: &Path) -> Result<(), BridgeRuntimeError> {
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o700))?;
    }
    Ok(())
}

fn set_private_socket_permissions(path: &Path) -> Result<(), BridgeRuntimeError> {
    #[cfg(unix)]
    {
        use std::os::unix::fs::PermissionsExt;
        std::fs::set_permissions(path, std::fs::Permissions::from_mode(0o600))?;
    }
    Ok(())
}

fn terminate_child(child: &mut Child) {
    match child.try_wait() {
        Ok(Some(_)) => {}
        Ok(None) => {
            let _ = child.kill();
            let _ = child.wait();
        }
        Err(_) => {
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

fn default_runtime_root() -> Option<PathBuf> {
    std::env::var_os("XDG_RUNTIME_DIR")
        .map(PathBuf::from)
        .or_else(|| Some(std::env::temp_dir()))
        .map(|root| root.join("prism-spore"))
}

fn short_hex(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut output = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        output.push(HEX[(byte >> 4) as usize] as char);
        output.push(HEX[(byte & 0x0f) as usize] as char);
    }
    output
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    fn private_temp_root(name: &str) -> PathBuf {
        let mut random = [0u8; 8];
        getrandom::fill(&mut random).unwrap();
        std::env::temp_dir().join(format!("prism-bridge-test-{name}-{}", short_hex(&random)))
    }

    #[test]
    fn bootstrap_roundtrip_redacts_key_and_builds_endpoint() {
        let socket = private_temp_root("bootstrap").join("spore.sock");
        let bootstrap = BridgeBootstrap::new(socket.clone()).unwrap();
        assert!(format!("{bootstrap:?}").contains("[REDACTED]"));
        let mut bytes = Vec::new();
        bootstrap.write_to(&mut bytes).unwrap();
        let endpoint = SporeBridgeEndpoint::from_reader(Cursor::new(bytes)).unwrap();
        assert_eq!(endpoint.socket_path(), socket);
    }

    #[test]
    fn malformed_bootstrap_length_is_rejected() {
        let bytes = (MAX_BOOTSTRAP_SIZE as u32 + 1).to_be_bytes();
        assert!(matches!(
            SporeBridgeEndpoint::from_reader(Cursor::new(bytes)),
            Err(BridgeRuntimeError::Bootstrap(_))
        ));
    }

    #[test]
    fn process_configuration_rejects_relative_paths() {
        assert!(matches!(
            SporeProcessConfig::new("spore", private_temp_root("config")),
            Err(BridgeRuntimeError::Configuration(_))
        ));
    }

    #[tokio::test]
    async fn authenticated_socket_roundtrip_uses_bootstrap_session() {
        let root = private_temp_root("roundtrip");
        create_private_directory(&root).unwrap();
        let socket = root.join("spore.sock");
        let bootstrap = BridgeBootstrap::new(socket.clone()).unwrap();
        let parent_codec = BridgeCodec::new(
            BridgeSessionKey::from_bytes(bootstrap.session_key),
            bootstrap.session_id,
        )
        .unwrap();
        let mut bytes = Vec::new();
        bootstrap.write_to(&mut bytes).unwrap();
        let endpoint = SporeBridgeEndpoint::from_reader(Cursor::new(bytes)).unwrap();

        let child = tokio::spawn(async move {
            let mut connection = endpoint.accept().await.unwrap();
            match connection.receive_renderer().await.unwrap() {
                RendererToSpore::Ping(nonce) => {
                    connection
                        .send_spore(&SporeToRenderer::Pong(nonce))
                        .await
                        .unwrap();
                }
                _ => panic!("unexpected renderer message"),
            }
        });

        while !socket.exists() {
            sleep(Duration::from_millis(5)).await;
        }
        let stream = UnixStream::connect(&socket).await.unwrap();
        let mut parent = SupervisedTestConnection {
            stream,
            codec: parent_codec,
        };
        parent.send(RendererToSpore::Ping(41)).await.unwrap();
        assert!(matches!(parent.receive().await.unwrap(), SporeToRenderer::Pong(41)));
        child.await.unwrap();
        let _ = std::fs::remove_dir_all(root);
    }

    struct SupervisedTestConnection {
        stream: UnixStream,
        codec: BridgeCodec,
    }

    impl SupervisedTestConnection {
        async fn send(&mut self, message: RendererToSpore) -> Result<(), BridgeRuntimeError> {
            let frame = self.codec.encode_renderer_message(&message)?;
            self.stream.write_all(&frame).await?;
            Ok(())
        }

        async fn receive(&mut self) -> Result<SporeToRenderer, BridgeRuntimeError> {
            let payload = read_frame(&mut self.stream).await?;
            Ok(self.codec.decode_spore_message(&payload)?)
        }
    }
}
