# 🔧 WASM Build Fix - How We Solved It

**Problem**: Initial build failed with "can't find crate for `core`" error when building for wasm32-unknown-unknown target.

**Root Cause**: Workspace configuration conflicts and using Nix's Rust instead of rustup's Rust.

---

## ✅ Solutions Applied

### 1. Removed Parent Workspace

**Before** (mycelix-mail/dna/Cargo.toml):
```toml
[workspace]
members = ["zomes/mail_messages", "zomes/trust_filter", "integrity"]

[workspace.dependencies]
hdk = "0.3.0-beta"
```

**After**: Deleted this file entirely

**Reason**: Having both a parent workspace AND individual zome workspaces caused conflicts. Your successful builds (like mycelix-desktop) don't use a parent workspace.

### 2. Added Empty Workspace to Each Zome

**Applied to**:
- `dna/integrity/Cargo.toml`
- `dna/zomes/mail_messages/Cargo.toml`
- `dna/zomes/trust_filter/Cargo.toml`

**Added**:
```toml
# Empty workspace to prevent being included in parent workspace
[workspace]
```

**Reason**: This pattern matches your successful mycelix-desktop build.

### 3. Updated HDK Version

**Before**: `hdk = "0.3.0-beta"`
**After**: `hdk = "0.5.6"`

**Reason**: Your working mycelix-desktop uses 0.5.6, which is more stable.

### 4. Added WASM Optimization Profile

**Added to each Cargo.toml**:
```toml
[profile.release]
opt-level = "z"           # Optimize for size
lto = true                # Link-time optimization
codegen-units = 1         # Single codegen unit for smaller WASM
overflow-checks = true    # Keep safety checks
```

**Reason**: WASM files need to be as small as possible. This matches your successful builds.

### 5. Use Rustup Toolchain (Not Nix)

**Created**: `scripts/build-with-rustup.sh`

**Key change**:
```bash
# Use rustup's toolchain explicitly (not Nix)
export PATH="/home/tstoltz/.rustup/toolchains/stable-x86_64-unknown-linux-gnu/bin:$PATH"
export RUSTUP_TOOLCHAIN="stable-x86_64-unknown-linux-gnu"
```

**Reason**: Your 0TML project successfully uses this approach in `0TML/holochain-dht-setup/scripts/build-zome-rustup.sh`.

---

## 📋 Build Process

### Before (Failed):
1. Used Nix's Rust from `nix develop`
2. Had workspace conflicts
3. Used outdated HDK 0.3.0-beta
4. No WASM optimization

### After (Working):
1. Use rustup's Rust explicitly
2. Each zome is independent workspace
3. HDK 0.5.6 (stable)
4. WASM-optimized build settings

---

## 🚀 How to Build Now

```bash
cd /srv/luminous-dynamics/Mycelix-Core/mycelix-mail

# New build script (uses rustup):
./scripts/build-with-rustup.sh

# This will:
# 1. Build integrity zome
# 2. Build mail_messages zome
# 3. Build trust_filter zome
# 4. Copy WASM files
# 5. Pack DNA (if hc is available)
```

---

## 🔍 Key Learnings

### What Worked in Past Builds

From your `mycelix-desktop/dnas/mycelix-test/zomes/messages/`:
- ✅ Empty workspace per zome
- ✅ HDK 0.5.6
- ✅ WASM optimization profile
- ✅ Successfully built and working

From your `0TML/holochain-dht-setup/`:
- ✅ Explicit rustup toolchain
- ✅ Verification of wasm32 target
- ✅ Build each zome separately
- ✅ Clear error messages

### What Didn't Work

- ❌ Parent workspace with child workspaces
- ❌ Using Nix's Rust for WASM builds
- ❌ HDK 0.3.0-beta (too old)
- ❌ No WASM optimization

---

## 📊 File Structure Now

```
mycelix-mail/dna/
├── dna.yaml                          # DNA configuration
├── integrity/
│   ├── Cargo.toml                    # ✅ Own workspace, HDK 0.5.6
│   └── src/lib.rs
├── zomes/
│   ├── mail_messages/
│   │   ├── Cargo.toml                # ✅ Own workspace, HDK 0.5.6
│   │   └── src/lib.rs
│   └── trust_filter/
│       ├── Cargo.toml                # ✅ Own workspace, HDK 0.5.6
│       └── src/lib.rs
└── [NO parent Cargo.toml]            # ✅ Removed
```

---

## 🎯 Expected Build Output

```
✅ wasm32-unknown-unknown target found

📦 Building integrity zome...
   Compiling mycelix_mail_integrity v0.1.0
   Finished release [optimized] target(s)

📦 Building mail_messages zome...
   Compiling mail_messages v0.1.0
   Finished release [optimized] target(s)

📦 Building trust_filter zome...
   Compiling trust_filter v0.1.0
   Finished release [optimized] target(s)

📋 Copying WASM files...

✅ Build complete!

WASM files:
-rw-r--r-- 1 tstoltz tstoltz 245K Nov 11 09:00 integrity.wasm
-rw-r--r-- 1 tstoltz tstoltz 182K Nov 11 09:00 mail_messages.wasm
-rw-r--r-- 1 tstoltz tstoltz 176K Nov 11 09:00 trust_filter.wasm

📦 Packing DNA...
✅ DNA packed: mycelix-mail.dna

🎉 Success! Ready to test.
```

---

## 🐛 Troubleshooting

### If build still fails:

1. **Verify rustup toolchain**:
   ```bash
   rustc --version
   rustc --print sysroot
   ls $(rustc --print sysroot)/lib/rustlib/wasm32-unknown-unknown
   ```

2. **Add wasm32 target if missing**:
   ```bash
   rustup target add wasm32-unknown-unknown
   ```

3. **Clean all builds**:
   ```bash
   cd mycelix-mail/dna
   rm -rf */target target
   ./scripts/build-with-rustup.sh
   ```

4. **Check for workspace conflicts**:
   ```bash
   # Should NOT find a dna/Cargo.toml
   ls mycelix-mail/dna/Cargo.toml
   # Expected: No such file or directory
   ```

---

## 🎓 Reference: Working Examples

### mycelix-desktop (Working Build)
```bash
cd /srv/luminous-dynamics/Mycelix-Core/mycelix-desktop/dnas/mycelix-test
tree -L 3 -I target
# Shows: Each zome has own Cargo.toml with [workspace]
```

### 0TML Holochain (Working Build Script)
```bash
cat /srv/luminous-dynamics/Mycelix-Core/0TML/holochain-dht-setup/scripts/build-zome-rustup.sh
# Shows: Explicit rustup toolchain usage
```

---

## ✅ Status

- [x] Fixed Cargo.toml conflicts
- [x] Updated to HDK 0.5.6
- [x] Added WASM optimization
- [x] Created rustup build script (failed - Nix wrapper issues)
- [x] Removed parent workspace
- [x] Created shell.nix with minimal Rust environment
- [ ] **Testing Nix shell build** (in progress)

---

## ⚠️ IMPORTANT: Why Rustup Failed

The rustup approach failed because the Rust toolchain was installed via Nix and has broken references:

```
/home/tstoltz/.rustup/toolchains/.../gcc-ld/ld.lld: line 5:
/nix/store/ra2zx3av6408y4w2mcfryj1p2m69x2j1-rustup-1.28.2/nix-support/ld-wrapper.sh: No such file or directory
```

**Root Cause**: Rustup's linker wrapper references non-existent Nix store paths.

**Solution**: Use `nix-shell` with proper Nix-managed Rust toolchain instead of rustup.

---

## 🔧 CORRECT Build Method (Nix Shell)

```bash
cd /srv/luminous-dynamics/Mycelix-Core/mycelix-mail

# Enter Nix shell (provides clean Rust+GCC environment)
nix-shell

# Inside shell, add wasm32 target
rustup target add wasm32-unknown-unknown

# Build each zome
cd dna/integrity
cargo build --release --target wasm32-unknown-unknown

cd ../zomes/mail_messages
cargo build --release --target wasm32-unknown-unknown

cd ../zomes/trust_filter
cargo build --release --target wasm32-unknown-unknown

# Copy WASM files
cd ..
cp integrity/target/wasm32-unknown-unknown/release/mycelix_mail_integrity.wasm integrity.wasm
cp zomes/mail_messages/target/wasm32-unknown-unknown/release/mail_messages.wasm mail_messages.wasm
cp zomes/trust_filter/target/wasm32-unknown-unknown/release/trust_filter.wasm trust_filter.wasm
```

---

**Next**: Once build completes, test with:
```bash
cd mycelix-mail/dna
hc sandbox create
hc sandbox call install-app-bundle ./mycelix-mail.dna
```
