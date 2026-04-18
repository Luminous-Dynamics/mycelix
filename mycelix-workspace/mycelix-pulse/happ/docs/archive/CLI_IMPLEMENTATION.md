# 🖥️ Mycelix Mail CLI Implementation

**Status**: In Development
**Target**: Production-ready CLI tool for Mycelix Mail
**Timeline**: 1-2 weeks

---

## 📋 Overview

Building a comprehensive Rust CLI tool that provides full access to Mycelix Mail functionality through the command line. This enables power users, automation, and serves as a foundation for other interfaces.

### Why CLI First?

1. **Fastest to Market**: Simpler than web UI, can be built in 1-2 weeks
2. **Power Users**: Enables automation and scripting
3. **Testing Platform**: Great for validating DNA functionality
4. **Developer Tool**: Essential for integration testing
5. **Foundation**: Can be library-ified for other interfaces

---

## 🎯 Features

### Core Commands

#### `mycelix-mail init`
Initialize user profile, generate keys, register DID

```bash
# Interactive setup
mycelix-mail init

# With email address
mycelix-mail init --email alice@mycelix.net

# Import existing keys
mycelix-mail init --import-keys ~/.mycelix/keys.json
```

#### `mycelix-mail send`
Send email messages

```bash
# Basic send
mycelix-mail send bob@mycelix.net \
  --subject "Hello!" \
  --body "How are you?"

# With attachments
mycelix-mail send bob@mycelix.net \
  --subject "Documents" \
  --body "See attached" \
  --attach report.pdf data.csv

# Reply to thread
mycelix-mail send bob@mycelix.net \
  --subject "Re: Hello" \
  --body "I'm great!" \
  --reply-to uhCkk...messageID

# Set epistemic tier
mycelix-mail send alice@mycelix.net \
  --subject "Verified data" \
  --body "Official results" \
  --tier 3  # Tier3CryptographicallyProven
```

#### `mycelix-mail inbox`
List received messages

```bash
# Show all messages
mycelix-mail inbox

# Filter by sender
mycelix-mail inbox --from alice@mycelix.net

# Filter by minimum trust score
mycelix-mail inbox --trust-min 0.7

# Show only unread
mycelix-mail inbox --unread

# Limit results
mycelix-mail inbox --limit 50

# JSON output
mycelix-mail inbox --format json
```

#### `mycelix-mail read`
Read specific message

```bash
# Read and mark as read
mycelix-mail read uhCkk...messageID

# Read without marking
mycelix-mail read uhCkk...messageID --mark-read=false
```

#### `mycelix-mail trust`
Manage trust scores

```bash
# Get trust score
mycelix-mail trust get did:mycelix:alice

# Set trust score
mycelix-mail trust set did:mycelix:alice 0.85

# List all scores
mycelix-mail trust list

# List high-trust only
mycelix-mail trust list --min 0.7 --sort

# Sync from MATL
mycelix-mail trust sync
mycelix-mail trust sync did:mycelix:alice
```

#### `mycelix-mail did`
Manage DIDs and contacts

```bash
# Show your DID
mycelix-mail did whoami

# Register new DID
mycelix-mail did register did:mycelix:alice --agent-key uhCAk...

# Resolve DID to agent key
mycelix-mail did resolve did:mycelix:bob

# List known DIDs
mycelix-mail did list
mycelix-mail did list --filter "alice"
```

#### `mycelix-mail search`
Search messages

```bash
# Search everywhere
mycelix-mail search "quarterly report"

# Search in subject only
mycelix-mail search "invoice" --in subject

# Search in body
mycelix-mail search "urgent" --in body --limit 100
```

#### `mycelix-mail export`
Export messages and data

```bash
# Export to JSON
mycelix-mail export --format json --output backup.json

# Export to mbox
mycelix-mail export --format mbox --output mail.mbox

# Export date range
mycelix-mail export --format json --output recent.json --since 2025-11-01
```

#### `mycelix-mail status`
Show system status

```bash
# Basic status
mycelix-mail status

# Detailed status
mycelix-mail status --detailed
```

#### `mycelix-mail sync`
Sync with DHT

```bash
# Normal sync
mycelix-mail sync

# Force full re-sync
mycelix-mail sync --force
```

---

## 🏗️ Architecture

### Project Structure

```
cli/
├── Cargo.toml                  # Dependencies and metadata
├── src/
│   ├── main.rs                 # CLI entry point with clap
│   ├── config.rs               # Configuration management
│   ├── client.rs               # Holochain client wrapper
│   ├── types.rs                # Shared types
│   └── commands/               # Command implementations
│       ├── mod.rs
│       ├── init.rs             # Initialize user
│       ├── send.rs             # Send messages
│       ├── inbox.rs            # List messages
│       ├── read.rs             # Read message
│       ├── trust.rs            # Trust management
│       ├── did.rs              # DID management
│       ├── search.rs           # Search functionality
│       ├── export.rs           # Export data
│       ├── status.rs           # Show status
│       └── sync.rs             # Sync operations
├── tests/
│   ├── integration_tests.rs   # End-to-end tests
│   └── fixtures/               # Test data
└── README.md                   # CLI documentation
```

### Key Dependencies

```toml
[dependencies]
clap = "4.5"                    # CLI framework
holochain_client = "0.5"        # Holochain integration
tokio = "1.40"                  # Async runtime
serde = "1.0"                   # Serialization
anyhow = "1.0"                  # Error handling
colored = "2.1"                 # Terminal colors
reqwest = "0.12"                # HTTP for DID/MATL
```

---

## 💻 Implementation Plan

### Phase 1: Core Infrastructure (Days 1-2)

**Tasks**:
- ✅ Create project structure
- ✅ Set up Cargo.toml with dependencies
- ✅ Implement main.rs with command structure
- ⏳ Implement config.rs (configuration management)
- ⏳ Implement client.rs (Holochain client wrapper)
- ⏳ Implement types.rs (message types, etc.)

**Deliverable**: CLI compiles and shows help

### Phase 2: Essential Commands (Days 3-5)

**Tasks**:
- ⏳ Implement `init` command
- ⏳ Implement `send` command
- ⏳ Implement `inbox` command
- ⏳ Implement `read` command
- ⏳ Basic error handling
- ⏳ Configuration file support

**Deliverable**: Can send and receive messages

### Phase 3: Trust & DID (Days 6-7)

**Tasks**:
- ⏳ Implement `trust` subcommands
- ⏳ Implement `did` subcommands
- ⏳ DID registry integration
- ⏳ MATL bridge integration
- ⏳ Trust score filtering

**Deliverable**: Full DID and trust functionality

### Phase 4: Advanced Features (Days 8-9)

**Tasks**:
- ⏳ Implement `search` command
- ⏳ Implement `export` command
- ⏳ Implement `status` command
- ⏳ Implement `sync` command
- ⏳ Attachment handling

**Deliverable**: Complete feature set

### Phase 5: Polish & Testing (Days 10-12)

**Tasks**:
- ⏳ Write integration tests
- ⏳ Add progress indicators
- ⏳ Improve error messages
- ⏳ Add colored output
- ⏳ Performance optimization
- ⏳ Documentation

**Deliverable**: Production-ready CLI

### Phase 6: Distribution (Days 13-14)

**Tasks**:
- ⏳ Create binary releases
- ⏳ Package for Homebrew
- ⏳ Package for Cargo
- ⏳ Create installation script
- ⏳ Write user guide

**Deliverable**: Easy installation for users

---

## 🧪 Testing Strategy

### Unit Tests

Test individual command logic:
```rust
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_email_address() {
        // Test email parsing
    }

    #[test]
    fn test_trust_score_validation() {
        // Test score bounds
    }
}
```

### Integration Tests

Test full command flow:
```rust
#[tokio::test]
async fn test_send_and_receive() {
    // Start test conductor
    // Send message
    // Check inbox
    // Verify received
}
```

### End-to-End Tests

Test against real conductor:
```bash
#!/bin/bash
# Start test conductor
hc sandbox create test-cli

# Run CLI commands
./target/release/mycelix-mail init --email test@test.com
./target/release/mycelix-mail send self@test.com --subject "Test"

# Verify results
./target/release/mycelix-mail inbox | grep "Test"
```

---

## 📦 Distribution

### Binary Releases

**Platforms**:
- Linux (x86_64, aarch64)
- macOS (x86_64, aarch64/M1)
- Windows (x86_64)

**Release Process**:
```bash
# Build for all platforms
cargo build --release --target x86_64-unknown-linux-gnu
cargo build --release --target aarch64-unknown-linux-gnu
cargo build --release --target x86_64-apple-darwin
cargo build --release --target aarch64-apple-darwin
cargo build --release --target x86_64-pc-windows-gnu

# Create releases
./scripts/create-release.sh v1.0.0
```

### Package Managers

**Cargo** (Rust):
```bash
cargo install mycelix-mail
```

**Homebrew** (macOS/Linux):
```bash
brew tap mycelix/tap
brew install mycelix-mail
```

**APT** (Debian/Ubuntu):
```bash
sudo add-apt-repository ppa:mycelix/ppa
sudo apt install mycelix-mail
```

**Nix**:
```bash
nix-env -iA nixpkgs.mycelix-mail
```

### Installation Script

```bash
curl -sSL https://install.mycelix.net | bash
```

---

## 📊 Success Metrics

### Functionality
- [ ] All core commands working
- [ ] DID/MATL integration functional
- [ ] Message send/receive working
- [ ] Trust filtering active

### Quality
- [ ] >80% test coverage
- [ ] <100ms startup time
- [ ] <500ms command execution
- [ ] Zero memory leaks

### Usability
- [ ] Clear help text
- [ ] Good error messages
- [ ] Progress indicators
- [ ] Auto-completion support

### Distribution
- [ ] Binary releases for 5+ platforms
- [ ] Cargo package published
- [ ] Homebrew formula working
- [ ] Installation guide complete

---

## 🔜 Next Steps (After CLI)

### Option C: Performance Testing
Once CLI is working, use it to:
- Send 10,000+ test messages
- Measure throughput
- Profile memory usage
- Stress test DID resolution
- Benchmark trust score queries

### Option A: Deploy Alpha
With working CLI, deploy alpha:
- Use CLI for administration
- Enable power users
- Gather performance data
- Validate in production

---

## 📝 Implementation Notes

### Configuration File

**Location**: `~/.mycelix-mail/config.toml`

```toml
[identity]
did = "did:mycelix:alice"
email = "alice@mycelix.net"
private_key_path = "~/.mycelix-mail/keys/private.key"

[conductor]
url = "ws://localhost:8888"
app_id = "mycelix_mail"

[services]
did_registry_url = "http://localhost:5000"
matl_bridge_url = "http://localhost:5001"

[preferences]
default_tier = 2
auto_sync = true
cache_ttl = 3600
```

### Key Management

Keys stored in: `~/.mycelix-mail/keys/`
- `private.key` - Ed25519 private key (encrypted)
- `public.key` - Ed25519 public key
- `agent.key` - Holochain agent key

### Error Handling

User-friendly errors:
```
❌ Failed to send message
│
├─ Recipient DID not found: did:mycelix:bob
│  Suggestion: Register the DID first with:
│    mycelix-mail did register did:mycelix:bob --agent-key <key>
│
└─ For more details, run with --verbose
```

---

## 🎯 Current Status

**Phase**: 1 (Core Infrastructure)
**Progress**: 30%
**Next**: Complete config.rs and client.rs modules

**Files Created**:
- ✅ cli/Cargo.toml
- ✅ cli/src/main.rs
- ⏳ cli/src/config.rs
- ⏳ cli/src/client.rs
- ⏳ cli/src/types.rs
- ⏳ cli/src/commands/*.rs

---

**Estimated Completion**: 10-14 days
**Current Blocker**: None
**Risk**: Holochain client API changes

🍄 **Building the command-line interface for decentralized email!** 🍄
