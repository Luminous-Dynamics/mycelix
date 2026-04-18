# 🚀 How to Launch Mycelix Desktop GUI

**Status**: App is compiled and ready to run!
**Location**: `/srv/luminous-dynamics/Mycelix-Core/mycelix-desktop`

---

## ✅ Quick Start (Easiest Method)

### Option 1: Desktop Launcher Script ⭐ RECOMMENDED

1. **Open file manager** (Dolphin in KDE)
2. **Navigate to**: `/srv/luminous-dynamics/Mycelix-Core/mycelix-desktop`
3. **Double-click**: `launch-mycelix.sh`
4. Application window should appear!

---

### Option 2: Terminal on Desktop

1. **Open Konsole** (KDE terminal) from your desktop
2. **Run**:
   ```bash
   cd /srv/luminous-dynamics/Mycelix-Core/mycelix-desktop
   ./target/release/mycelix-desktop
   ```

That's it!

---

## 🖥️ What You Should See

When the app launches, you'll see:

1. **Beautiful dark theme UI** with purple accents
2. **Welcome card** with app description
3. **Conductor Controls**:
   - Start Conductor button
   - Connection status
   - Peer count

4. **After Starting Conductor**:
   - Conductor status shows "Connected"
   - Admin API section becomes active
   - Zome Functions section appears
   - **NEW: Message Broadcasting section** 💬

---

## 💬 Testing Message Broadcasting

### Single-Node Test

1. **Start the conductor** (click "Start Conductor")
2. **Wait for "Connected" status**
3. **Scroll down to "Message Broadcasting" card**
4. **Type a message** in the input field
5. **Click "Send"** or press Enter
6. **Look for**:
   - ✅ "Message sent!" notification
   - ✅ Input field clears
   - ✅ Message count updates

### Multi-Node P2P Test (Advanced)

For full P2P testing, see: [MESSAGE_BROADCASTING_TEST_GUIDE.md](./MESSAGE_BROADCASTING_TEST_GUIDE.md)

---

## 🐛 Troubleshooting

### App doesn't start?

**Check 1**: Verify binary exists
```bash
ls -lh /srv/luminous-dynamics/Mycelix-Core/mycelix-desktop/target/release/mycelix-desktop
# Should show: 10MB file
```

**Check 2**: Run from terminal to see errors
```bash
cd /srv/luminous-dynamics/Mycelix-Core/mycelix-desktop
./target/release/mycelix-desktop
# Watch for error messages
```

**Check 3**: Verify Holochain is available
```bash
which holochain
# Should show: /nix/store/.../bin/holochain
```

If `holochain` not found:
```bash
cd /srv/luminous-dynamics/Mycelix-Core/mycelix-desktop
nix develop
# Now holochain should be available
./target/release/mycelix-desktop
```

---

### Conductor won't start?

**Check config file exists**:
```bash
ls -lh conductor-config.yaml
# Should exist in project root
```

**Try manual start**:
```bash
cd /srv/luminous-dynamics/Mycelix-Core/mycelix-desktop
echo '' | holochain --piped -c conductor-config.yaml
# Watch for errors
```

**Check logs**:
```bash
tail -f .holochain/conductor/holochain.log
```

---

### Message sending doesn't work?

1. **Verify conductor is "Connected"** - check status badge
2. **Check admin port** - should be 8888
3. **Check app port** - should be 8889
4. **Look at console** - F12 in app to see errors

---

## 📋 Full Testing Checklist

Once the GUI is running, follow: **[MESSAGE_BROADCASTING_TEST_GUIDE.md](./MESSAGE_BROADCASTING_TEST_GUIDE.md)**

That guide includes:
- ✅ 10 single-node tests (30 min)
- ✅ 10 multi-node P2P tests (45 min)
- ✅ 6 edge case tests (30 min)
- ✅ Result templates
- ✅ Expected outcomes

---

## 🎯 Key Features to Test

### Must Test
1. ✅ App launches and displays
2. ✅ Conductor starts successfully
3. ✅ Message input accepts text
4. ✅ Send button works
5. ✅ Toast notifications appear

### Should Test
6. ✅ Auto-refresh updates messages
7. ✅ Manual refresh button works
8. ✅ Empty state shows correctly
9. ✅ Error handling works
10. ✅ Responsive design (resize window)

### Nice to Test
11. ✅ Enter key sends messages
12. ✅ Long messages word-wrap
13. ✅ Special characters work
14. ✅ Multiple rapid sends
15. ✅ Multi-node P2P sync

---

## 💡 Development Mode (Optional)

For live reloading during development:

```bash
cd /srv/luminous-dynamics/Mycelix-Core/mycelix-desktop
nix develop
npm run tauri dev
```

This will:
- Start Vite dev server
- Launch app with hot-reload
- Show console output
- Rebuild on file changes

---

## 🎊 What Makes This Special

This isn't just a desktop app - it's a **real P2P application** built on Holochain:

- **No central server** - truly peer-to-peer
- **DHT synchronization** - data shared across nodes
- **Cryptographic identity** - agent keys for all users
- **Beautiful native UI** - Tauri + SolidJS
- **Production ready** - 10 MB binary, 3.2 MB packages

---

## 📚 Related Documentation

- **[MESSAGE_BROADCASTING_TEST_GUIDE.md](./MESSAGE_BROADCASTING_TEST_GUIDE.md)** - Complete test procedures
- **[SESSION_HANDOFF.md](./SESSION_HANDOFF.md)** - Current development status
- **[WEEK_2_DAY_1_CHANGES.md](./WEEK_2_DAY_1_CHANGES.md)** - What was implemented
- **[QUICKSTART.md](./QUICKSTART.md)** - General project guide

---

## 🙋 Need Help?

If you encounter issues:

1. **Check logs**: `/tmp/mycelix-*.log`
2. **Run from terminal**: See error messages directly
3. **Verify environment**: `nix develop` provides all deps
4. **Read troubleshooting**: This guide's troubleshooting section
5. **Check documentation**: Comprehensive guides available

---

*Built with 💜 by Luminous Dynamics*

**App Status**: ✅ Ready to Launch
**Testing Status**: ⏳ Awaiting GUI Testing
**Documentation**: ✅ Complete

**Your desktop is ready! Just run the launcher or open in terminal.** 🚀
