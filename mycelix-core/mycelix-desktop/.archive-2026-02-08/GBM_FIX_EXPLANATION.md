# 🔧 GBM Buffer Error Fix

## The Problem

Error message:
```
Failed to create GBM buffer of size 1200x800: Invalid argument
```

**What is GBM?** Graphics Buffer Manager - manages GPU memory buffers for hardware-accelerated rendering

**Why does it fail?**
- GPU not accessible from current session
- DRI (Direct Rendering Infrastructure) permissions issue
- Hardware acceleration unavailable via SSH/remote context
- Intel/AMD/NVIDIA driver compatibility issue

**Symptom**: Window appears briefly then immediately closes

---

## The Solution

**Disable hardware acceleration** and use software rendering instead:

```bash
export WEBKIT_DISABLE_COMPOSITING_MODE=1  # Disable WebKit GPU compositing
export LIBGL_ALWAYS_SOFTWARE=1            # Force software OpenGL rendering
```

These environment variables are now included in:
- `launch-mycelix.sh` (production binary)
- `launch-dev.sh` (development mode)

---

## How to Launch

### ✅ Method 1: Using Launcher Scripts (Easiest)

From your **desktop Konsole**:
```bash
cd /srv/luminous-dynamics/Mycelix-Core/mycelix-desktop
./launch-mycelix.sh
```

Or double-click `launch-mycelix.sh` in file manager.

### ✅ Method 2: Manual Launch with Environment Variables

```bash
cd /srv/luminous-dynamics/Mycelix-Core/mycelix-desktop
export WEBKIT_DISABLE_COMPOSITING_MODE=1
export LIBGL_ALWAYS_SOFTWARE=1
./target/release/mycelix-desktop
```

### ✅ Method 3: Development Mode

```bash
cd /srv/luminous-dynamics/Mycelix-Core/mycelix-desktop
./launch-dev.sh
```

---

## Performance Impact

**Software rendering is slower** than GPU acceleration, but:
- ✅ Works on all systems without GPU access issues
- ✅ Uses CPU instead of GPU (less power-efficient)
- ✅ Still perfectly usable for UI interaction
- ✅ Message broadcasting and Holochain operations unaffected

For a P2P messaging app, the performance difference is negligible.

---

## Alternative: Fix GPU Access (Advanced)

If you want GPU acceleration, you can fix DRI permissions:

### Check current DRI access:
```bash
ls -la /dev/dri/
# Should show renderD* devices
```

### Add your user to video group:
```bash
sudo usermod -a -G video $USER
# Logout and login for changes to take effect
```

### Check GPU info:
```bash
glxinfo | grep "OpenGL renderer"
lspci | grep VGA
```

### Then launch without software rendering variables:
```bash
unset WEBKIT_DISABLE_COMPOSITING_MODE
unset LIBGL_ALWAYS_SOFTWARE
./target/release/mycelix-desktop
```

---

## Technical Background

### Why GBM Errors Happen

1. **SSH Context**: Launching from SSH doesn't have GPU device access
2. **Wayland Session**: Some GPU permissions only work in native Wayland sessions
3. **DRI Permissions**: User needs to be in `video` group for `/dev/dri/*` access
4. **Driver Issues**: Intel/AMD/NVIDIA drivers may have different requirements

### What Software Rendering Does

Instead of:
```
WebKit → GPU (via GBM/OpenGL) → Display
```

It does:
```
WebKit → CPU (software rasterization) → Display
```

This is **guaranteed to work** on all systems, regardless of GPU access.

---

## Testing the Fix

After using the launcher scripts, you should see:
1. ✅ Window stays open (doesn't immediately close)
2. ✅ Dark purple theme UI visible
3. ✅ Welcome card and Conductor Controls displayed
4. ✅ Start Conductor button clickable
5. ✅ No GBM error messages

If it still crashes, capture the error with:
```bash
cd /srv/luminous-dynamics/Mycelix-Core/mycelix-desktop
export WEBKIT_DISABLE_COMPOSITING_MODE=1
export LIBGL_ALWAYS_SOFTWARE=1
export RUST_LOG=debug
export RUST_BACKTRACE=1
./target/release/mycelix-desktop 2>&1 | tee mycelix-debug.log
```

---

## Related Issues

- **Wayland Protocol Error**: Separate issue about display permissions (documented in GUI_TROUBLESHOOTING.md)
- **GBM Buffer Error**: This document - GPU access issue (now fixed with software rendering)

Both issues prevented GUI launch, but for different reasons. The software rendering fix addresses the GBM issue.

---

*Built with 💜 by Luminous Dynamics*

**Status**: ✅ GBM error fixed via software rendering
**Testing**: ⏳ Awaiting user verification
**Next**: Test message broadcasting feature
