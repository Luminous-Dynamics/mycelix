# 🔧 GUI Launch Troubleshooting

**Problem**: App opens for a second then closes
**Root Cause**: GPU Graphics Buffer Manager (GBM) error - can't create graphics buffers
**Solution**: Use updated launcher scripts with software rendering enabled

**Status**: ✅ **FIXED** - Launcher scripts updated with GBM fix (see `GBM_FIX_EXPLANATION.md`)

---

## ✅ The Working Solution (FIXED)

### Method 1: Use Fixed Launcher Script (BEST) ⭐

1. **On your physical desktop**, press `Ctrl+Alt+T` or open **Konsole**
2. Run the fixed launcher:
   ```bash
   cd /srv/luminous-dynamics/Mycelix-Core/mycelix-desktop
   ./launch-mycelix.sh
   ```
3. App should now stay open! ✅

**What changed?** The launcher now disables GPU acceleration:
```bash
export WEBKIT_DISABLE_COMPOSITING_MODE=1
export LIBGL_ALWAYS_SOFTWARE=1
```

This fixes the GBM buffer error by using CPU-based software rendering.

---

### Method 2: File Manager (Also Works Now)

1. Open **Dolphin** file manager
2. Navigate to: `/srv/luminous-dynamics/Mycelix-Core/mycelix-desktop/`
3. **Double-click** `launch-mycelix.sh` ✅
4. App should launch with software rendering enabled

**Why this now works**: The launcher script includes the GBM fix.

---

## 🔍 Root Cause Identified

**The Real Error** (found via debugging):
```
Failed to create GBM buffer of size 1200x800: Invalid argument
```

**What is GBM?** Graphics Buffer Manager - manages GPU memory for rendering

**Why does it fail?**
- GPU not accessible from SSH/remote session
- DRI (Direct Rendering Infrastructure) permissions issue
- WebKit trying to use hardware acceleration but can't access GPU

**The Fix**: Disable GPU acceleration, use software rendering instead ✅

---

## ⚠️ Secondary Issue: Wayland from SSH

If launching from SSH, you may also see:
```
Gdk-Message: Error 71 (Protocol error) dispatching to Wayland display.
```

**This happens because:**
- SSH sessions don't have Wayland display access
- GUI apps need desktop context

**Solution**: Always launch from desktop terminal (Konsole)

---

## 🎯 Test if Desktop Launch Works

### Quick Test in Konsole

Open Konsole **on your desktop** and try:

```bash
# Test 1: Simple X11 app
xclock &
# Should show a clock window

# Test 2: Our app
cd /srv/luminous-dynamics/Mycelix-Core/mycelix-desktop
./target/release/mycelix-desktop
# Should show Mycelix Desktop

# If both work, environment is correct!
```

---

## 📖 Full Technical Details

See **[GBM_FIX_EXPLANATION.md](./GBM_FIX_EXPLANATION.md)** for:
- Complete technical explanation of GBM errors
- Why software rendering is the solution
- Performance impact (negligible for this app)
- Advanced GPU access fix (optional)
- Alternative troubleshooting steps

---

## 🐛 If It Still Closes Immediately

### Check 1: Verify You're Using the Fixed Launcher

Make sure you're running `./launch-mycelix.sh` (which includes the GBM fix), not the raw binary.

### Check 2: Missing Frontend Assets

```bash
cd /srv/luminous-dynamics/Mycelix-Core/mycelix-desktop
ls -la dist/

# Should see:
# dist/index.html
# dist/assets/
```

If `dist/` is empty:
```bash
npm run build
./target/release/mycelix-desktop
```

---

### Check 2: Missing Dependencies

From desktop Konsole:
```bash
cd /srv/luminous-dynamics/Mycelix-Core/mycelix-desktop
ldd ./target/release/mycelix-desktop | grep "not found"
```

If you see "not found" errors, dependencies are missing. Run:
```bash
nix develop
./target/release/mycelix-desktop
```

---

### Check 3: Actual Error Message

Run with full output:
```bash
cd /srv/luminous-dynamics/Mycelix-Core/mycelix-desktop
./target/release/mycelix-desktop 2>&1 | tee mycelix-errors.log
```

This will show the real error instead of just closing.

---

## 🔍 Debug Mode

For maximum detail:
```bash
cd /srv/luminous-dynamics/Mycelix-Core/mycelix-desktop

# Set debug environment
export RUST_LOG=debug
export RUST_BACKTRACE=1

# Run app
./target/release/mycelix-desktop
```

---

## 📱 Alternative: Development Mode

If production binary has issues, try development mode:

```bash
cd /srv/luminous-dynamics/Mycelix-Core/mycelix-desktop

# Enter Nix shell (provides all dependencies)
nix develop

# Run in dev mode
npm run tauri dev
```

This:
- Starts Vite dev server
- Launches app with hot-reload
- Shows detailed console output
- Usually more forgiving about environment

---

## ✅ Success Checklist

When app launches successfully, you should see:

1. **Window appears** and stays open
2. **Dark theme UI** with purple accents
3. **Welcome card** at top
4. **Conductor Controls** section
5. **"Start Conductor" button** is clickable

If you see all of these → **SUCCESS!** You can now test the message broadcasting feature.

---

## 🎯 Next Steps After Successful Launch

1. **Follow**: [MESSAGE_BROADCASTING_TEST_GUIDE.md](./MESSAGE_BROADCASTING_TEST_GUIDE.md)
2. **Test**: Message sending and receiving
3. **Document**: Any bugs or issues you find
4. **Report**: Results in Session Handoff

---

## 💡 Pro Tips

### Keep Terminal Open

When launching, keep the terminal window open to see:
- Conductor connection status
- WebSocket messages
- Any error messages
- Debug output

### Multiple Monitors

If you have multiple monitors:
- Terminal on one monitor
- App window on another
- Easy to see both console output and UI

### Konsole Session

Save your Konsole session:
1. Window → Save Session
2. Name it "Mycelix Dev"
3. Quick launch next time!

---

## 🆘 Still Having Issues?

### Collect Debug Info

```bash
# System info
uname -a
echo "Desktop: $XDG_SESSION_TYPE"
echo "Display: $DISPLAY"

# App info
cd /srv/luminous-dynamics/Mycelix-Core/mycelix-desktop
ls -lh target/release/mycelix-desktop
ls -la dist/

# Try launch with full debug
RUST_LOG=trace ./target/release/mycelix-desktop 2>&1 | tee /tmp/mycelix-full-debug.log
```

Send `/tmp/mycelix-full-debug.log` for analysis.

---

## 📚 Related Documentation

- [HOW_TO_LAUNCH_GUI.md](./HOW_TO_LAUNCH_GUI.md) - Basic launch instructions
- [MESSAGE_BROADCASTING_TEST_GUIDE.md](./MESSAGE_BROADCASTING_TEST_GUIDE.md) - Testing procedures
- [SESSION_HANDOFF.md](./SESSION_HANDOFF.md) - Development status

---

*Built with 💜 by Luminous Dynamics*

**Remember**: GUI apps need desktop context. Always launch from Konsole or file manager on your actual desktop! 🖥️
