# 🧪 Mycelix Desktop - UI Testing Guide

**Date**: September 30, 2025
**Purpose**: Comprehensive guide for testing the Holochain conductor integration via Tauri UI

---

## 🎯 Testing Overview

This guide covers manual and automated testing of the Holochain conductor integration through the Tauri desktop application interface.

---

## 🚀 Prerequisites

### System Requirements
- NixOS or Linux with Nix installed
- X11 display server (for GUI)
- Holochain 0.5.6 (provided by nix develop)
- Node.js 20+ (provided by nix develop)
- Rust 1.90+ (provided by nix develop)

### Start the Application

```bash
# Method 1: Development mode with auto-reload
cd /srv/luminous-dynamics/Mycelix-Core/mycelix-desktop
nix develop
GDK_BACKEND=x11 npm run tauri dev

# Method 2: Built binary
nix develop
GDK_BACKEND=x11 ./target/debug/mycelix-desktop
```

---

## 🧩 Test Scenarios

### Scenario 1: Basic UI Verification

**Objective**: Verify the application launches and UI is rendered correctly

**Steps**:
1. Launch the application using one of the methods above
2. **Expected**: Window appears with Mycelix branding
3. **Verify**:
   - Title: "Mycelix Network - P2P Consciousness Platform"
   - Mode indicator shows "Desktop Mode (Tauri)" with green checkmark
   - Status section displays "Status: Initializing..."
   - Six action buttons are visible and properly styled

**Pass Criteria**: All UI elements render correctly, no visual glitches

---

### Scenario 2: Status Management

**Objective**: Test the status update functionality

**Steps**:
1. Click "Set Status" button
2. **Expected**: Status changes to "Status: Ready"
3. **Verify**: Status text updates in the Status Display section

**Pass Criteria**: Status updates correctly without errors

---

### Scenario 3: Start Holochain Conductor

**Objective**: Test the conductor startup via UI

**Steps**:
1. Click "Start Holochain" button
2. **Expected**:
   - Loading state activates
   - Success message: "Holochain conductor started successfully with config: conductor-config.yaml"
   - Status updates to show conductor is running
3. **Verify**:
   - No error messages
   - Process starts in background
   - Admin interface becomes accessible

**Pass Criteria**:
- Success message displayed
- No errors in browser console
- Conductor process running (verify with `ps aux | grep holochain`)

**Known Issues**:
- First start may take a few seconds to initialize database
- Lair keystore creates connection socket

---

### Scenario 4: Check Holochain Status

**Objective**: Verify status checking works

**Steps**:
1. After starting conductor (Scenario 3), click "Check Status" button
2. **Expected**: Message displays "Holochain conductor is running"
3. Click "Check Status" again when conductor not running
4. **Expected**: Message displays "Holochain conductor is not running"

**Pass Criteria**: Status accurately reflects conductor state

---

### Scenario 5: Stop Holochain Conductor

**Objective**: Test graceful conductor shutdown

**Steps**:
1. Start conductor (if not running)
2. Click "Stop Holochain" button
3. **Expected**: Message displays "Holochain conductor stopped successfully"
4. **Verify**:
   - Process terminated (`ps aux | grep holochain` shows nothing)
   - No zombie processes
   - Can restart successfully

**Pass Criteria**:
- Success message displayed
- Process cleanly terminated
- No errors during shutdown

---

### Scenario 6: Duplicate Start Prevention

**Objective**: Verify conductor can't be started twice

**Steps**:
1. Start conductor (if not running)
2. Click "Start Holochain" button again
3. **Expected**: Message displays "Holochain conductor already running"
4. **Verify**: Only one conductor process exists

**Pass Criteria**: Duplicate start prevented with clear message

---

### Scenario 7: Network Connection (Placeholder)

**Objective**: Test network connection placeholder

**Steps**:
1. Click "Connect to Network" button
2. **Expected**: Simulated connection success message
3. **Note**: This is a placeholder for future P2P functionality

**Pass Criteria**: Placeholder works without errors

---

### Scenario 8: Lifecycle Testing

**Objective**: Test complete conductor lifecycle

**Steps**:
1. Start conductor → Verify running
2. Check status → Verify reports "running"
3. Stop conductor → Verify stopped
4. Check status → Verify reports "not running"
5. Restart conductor → Verify works correctly

**Pass Criteria**: Complete lifecycle works without errors

---

## 🔍 WebSocket Admin Interface Testing

### Verify Admin Interface Accessibility

**After starting conductor**:

```bash
# Test WebSocket connection
curl -i -N -H "Connection: Upgrade" \
     -H "Upgrade: websocket" \
     -H "Sec-WebSocket-Version: 13" \
     -H "Sec-WebSocket-Key: test" \
     ws://localhost:8888
```

**Expected**: Connection upgrade response or WebSocket handshake

**Alternative test with wscat** (if available):
```bash
# Install wscat (if needed)
npm install -g wscat

# Connect to admin interface
wscat -c ws://localhost:8888
```

**Expected**: Connection established, can send admin commands

---

## 🐛 Error Scenarios to Test

### Error 1: Missing Config File

**Setup**: Temporarily rename `conductor-config.yaml`

**Steps**:
1. Click "Start Holochain"
2. **Expected**: Error message "Conductor config file not found..."

**Cleanup**: Rename file back

---

### Error 2: Holochain Not in PATH

**Setup**: Exit nix develop environment

**Steps**:
1. Try starting conductor
2. **Expected**: Error message "Failed to start... Make sure Holochain is installed (run 'nix develop' first)"

**Cleanup**: Re-enter nix develop

---

### Error 3: Port Already in Use

**Setup**: Start another process on port 8888

```bash
# In separate terminal
nc -l 8888
```

**Steps**:
1. Try starting conductor
2. **Expected**: Error about port in use (may take a moment to detect)

**Cleanup**: Kill the nc process

---

## 📊 Testing Checklist

### Manual Testing

- [ ] Application launches successfully
- [ ] UI renders correctly in both light/dark themes
- [ ] Mode detection shows "Desktop Mode (Tauri)"
- [ ] Status updates work correctly
- [ ] Conductor starts successfully
- [ ] Conductor stops successfully
- [ ] Status check works correctly
- [ ] Duplicate start prevention works
- [ ] Error messages are clear and helpful
- [ ] No console errors during normal operation

### Integration Testing

- [ ] Conductor process spawns correctly
- [ ] Database directory created (`.holochain/`)
- [ ] Keystore initializes successfully
- [ ] Admin interface accessible on port 8888
- [ ] Process lifecycle management works
- [ ] Process cleanup on app exit

### Performance Testing

- [ ] Conductor starts within 3 seconds
- [ ] UI remains responsive during operations
- [ ] Status updates are instant
- [ ] No memory leaks after multiple start/stop cycles

---

## 🔧 Troubleshooting Testing Issues

### Application Won't Start

```bash
# Check if ports are available
lsof -i :1420  # Vite dev server
lsof -i :8888  # Holochain admin

# Check Rust compilation
cd src-tauri
cargo check

# Rebuild if needed
cargo build
```

### Conductor Fails to Start

```bash
# Test conductor manually
echo '' | holochain --piped -c conductor-config.yaml

# Check config syntax
cat conductor-config.yaml

# Verify Holochain is available
which holochain
holochain --version
```

### UI Not Updating

1. Check browser console for errors
2. Verify Tauri IPC is working (no "invoke" errors)
3. Restart application
4. Clear browser cache if using browser mode

---

## 📈 Success Metrics

### Functionality
- ✅ 100% of basic operations work without errors
- ✅ All error scenarios handled gracefully
- ✅ Process lifecycle management robust

### User Experience
- ✅ Clear, helpful error messages
- ✅ Responsive UI (no freezing)
- ✅ Visual feedback for all operations
- ✅ Intuitive button labels and organization

### Reliability
- ✅ No crashes during normal operation
- ✅ Process cleanup works correctly
- ✅ Can recover from error states
- ✅ Multiple start/stop cycles work reliably

---

## 🚀 Next Steps After UI Testing

Once UI testing is complete:

1. **Create Test DNA**:
   - Set up minimal DNA for testing
   - Install to conductor via admin interface
   - Verify installation successful

2. **Test P2P Networking**:
   - Start two conductor instances
   - Verify peer discovery
   - Test data synchronization

3. **Build Production Features**:
   - Implement actual P2P messaging
   - Add identity management
   - Create Mycelix-specific DNAs

---

## 📝 Test Results Template

```markdown
## Test Session: [Date]

### Environment
- OS: [Linux/NixOS version]
- Node: [version]
- Rust: [version]
- Holochain: [version]

### Results
- Scenario 1: ✅ Pass / ❌ Fail - [Notes]
- Scenario 2: ✅ Pass / ❌ Fail - [Notes]
- Scenario 3: ✅ Pass / ❌ Fail - [Notes]
- ...

### Issues Found
1. [Description]
   - Severity: [Critical/High/Medium/Low]
   - Steps to reproduce
   - Expected vs actual behavior

### Performance Notes
- Conductor start time: [X seconds]
- UI responsiveness: [Good/Acceptable/Poor]
- Memory usage: [X MB]

### Recommendations
- [List any recommended changes or improvements]
```

---

*Built with 💜 by Luminous Dynamics*
*Holochain v0.5.6 | Tauri v2.8.5 | Rust 1.90.0*

**Status**: Ready for Manual UI Testing 🧪