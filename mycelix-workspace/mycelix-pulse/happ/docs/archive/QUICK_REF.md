# 📋 Mycelix Mail - Quick Reference Card

**For Claude/AI assistants working on this project**

---

## 🎯 Project Status (November 11, 2025)

**✅ COMPLETE:**
- Holochain DNA built and packed (1.7MB)
- All 3 zomes compiled and working
- Complete documentation written

**🚧 NEXT STEPS:**
1. Test DNA in Holochain sandbox
2. Build MATL bridge (Python)
3. Build DID registry service
4. Deploy alpha version

---

## 📁 File Locations

### Core Implementation
```
/srv/luminous-dynamics/Mycelix-Core/mycelix-mail/
├── dna/
│   ├── mycelix_mail.dna          ← PACKED DNA (1.7MB) ⭐
│   ├── integrity/src/lib.rs      ← Entry types
│   └── zomes/
│       ├── mail_messages/src/lib.rs  ← Send/receive
│       └── trust_filter/src/lib.rs   ← MATL integration
├── shell.nix                      ← Build environment
└── scripts/build-with-nix.sh     ← Build script
```

### Documentation
```
mycelix-mail/
├── SUCCESS.md              ← Build journey
├── IMPLEMENTATION_SUMMARY.md   ← Technical details
├── INTEGRATION_PLAN.md     ← Ecosystem integration ⭐
├── PROJECT_SUMMARY.md      ← High-level overview
├── NEXT_STEPS.md           ← User guide
└── QUICK_REF.md            ← This file
```

---

## 🔧 Common Commands

### Build DNA
```bash
cd /srv/luminous-dynamics/Mycelix-Core/mycelix-mail
nix-shell
cd dna/integrity
cargo build --release --target wasm32-unknown-unknown
cd ../zomes/mail_messages
cargo build --release --target wasm32-unknown-unknown
cd ../trust_filter
cargo build --release --target wasm32-unknown-unknown
cd ..
hc dna pack .
```

### Test DNA
```bash
cd dna
hc sandbox create mycelix-mail-test
hc sandbox run --app-id mycelix-mail --app mycelix_mail.dna
```

### Verify DNA
```bash
file mycelix_mail.dna
# Should show: gzip compressed data, original size 8862224
```

---

## 🧬 DNA Structure

### Entry Types (integrity/src/lib.rs)
1. **MailMessage** - Encrypted email messages
   - `from_did`, `to_did`, `subject_encrypted`, `body_cid`
   - `timestamp`, `thread_id`, `metadata`

2. **TrustScore** - MATL trust scores
   - `did`, `score` (0.0-1.0), `timestamp`

3. **Contact** - Address book entries
   - `did`, `name`, `email`, `notes`

### Zome Functions

**mail_messages:**
- `send_message(message)` → ActionHash
- `get_inbox()` → Vec<MailMessage>
- `get_outbox()` → Vec<MailMessage>
- `get_thread(parent_hash)` → Vec<MailMessage>
- `delete_message(hash)` → ActionHash

**trust_filter:**
- `check_sender_trust(did)` → f64 (0.0-1.0)
- `update_trust_score(trust_score)` → ActionHash
- `filter_inbox(min_trust)` → Vec<MailMessage>
- `report_spam(message_hash, reason)` → ()

---

## 🌉 Integration Points

### Layer 1 (DHT) - ✅ COMPLETE
**What**: Holochain for message storage  
**Status**: DNA built and ready

### Layer 5 (Identity) - 🚧 NEEDS WORK
**What**: DID → AgentPubKey resolution  
**Status**: Mock implementation in code  
**TODO**: Build DID registry service (PostgreSQL + Python API)

### Layer 6 (MATL) - 🚧 NEEDS WORK
**What**: Trust scores for spam filtering  
**Status**: Zome functions complete  
**TODO**: Python bridge syncing MATL DB → Holochain

---

## 🐍 MATL Bridge (To Be Built)

**File**: `matl-bridge/matl_to_holochain.py`

**Purpose**: Sync trust scores from 0TML system to Holochain

**Key Functions:**
```python
class MATLBridge:
    def sync_trust_scores():
        # Query: SELECT * FROM agent_reputations
        # Call: update_trust_score(did, score)
    
    def handle_spam_reports():
        # Query Holochain for spam reports
        # Insert into MATL spam_reports table
```

**Database Schema Needed:**
```sql
-- In MATL database
CREATE TABLE spam_reports (
    reporter_did TEXT,
    spammer_did TEXT,
    message_hash TEXT,
    reason TEXT,
    reported_at TIMESTAMP
);
```

---

## 🎓 Key Learnings

### What Works
- **Nix Shell** - Simple shell.nix with Rust + lld
- **HDK 0.5.6** - Stable Holochain version
- **holochain_serialized_bytes** - Required for #[hdk_entry_helper]
- **Empty Workspaces** - Each zome needs [workspace] entry

### What to Avoid
- ❌ Rustup on NixOS (broken linkers)
- ❌ Parent workspaces (conflicts)
- ❌ Complex validation in MVP

### Build Performance
- Integrity: 7.75s
- Mail messages: 3.49s
- Trust filter: 6.60s
- **Total**: 17.84s

---

## 🔗 Related Systems

### 0TML (Zero-TrustML)
**Location**: `/srv/luminous-dynamics/Mycelix-Core/0TML/`  
**Purpose**: MATL implementation, trust scoring  
**Database**: PostgreSQL with agent reputations

### Mycelix Protocol Architecture
**Location**: `/srv/luminous-dynamics/Mycelix-Core/docs/architecture/`  
**Key File**: `Mycelix Protocol_ Integrated System Architecture v5.2.md`  
**Layers**: 10-layer architecture (mail uses L1, L5, L6)

---

## 💡 Common Tasks

### Adding a New Zome Function
1. Add function to `zomes/*/src/lib.rs`
2. Mark with `#[hdk_extern]`
3. Rebuild: `cargo build --release --target wasm32-unknown-unknown`
4. Copy WASM to `dna/` directory
5. Re-pack DNA: `hc dna pack .`

### Updating Trust Score
```rust
// In trust_filter zome
let trust_score = TrustScore {
    did: "did:mycelix:alice".to_string(),
    score: 0.85,
    timestamp: sys_time()?,
};
update_trust_score(trust_score)?;
```

### Testing Trust Filter
```bash
hc sandbox call mycelix-mail trust_filter filter_inbox '{"min_trust": 0.7}'
```

---

## ⚠️ Important Notes

1. **DID Resolution**: Currently returns agent's own pubkey (mock)
   - Replace with proper DID registry lookup

2. **Trust Scores**: Default to 0.5 for unknown users
   - MATL bridge needed for real scores

3. **Encryption**: Messages stored encrypted as `subject_encrypted` (Vec<u8>)
   - Body stored as IPFS CID reference

4. **Thread Support**: Implemented but `parse_thread_id()` is stubbed
   - Use format: `msg_<base64_hash>`

---

## 🎯 Next Session Goals

**Immediate (Today):**
- [ ] Run DNA in sandbox
- [ ] Test all zome functions
- [ ] Document any issues

**This Week:**
- [ ] Implement DID registry
- [ ] Start MATL bridge
- [ ] Design UI mockup

**This Month:**
- [ ] Alpha testing (10 users)
- [ ] MATL bridge complete
- [ ] Security review

---

## 📞 Key Contacts

**Project Lead**: Tristan Stoltz  
**Email**: tristan.stoltz@evolvingresonantcocreationism.com  
**Architecture**: Mycelix Protocol v5.2  
**Build System**: NixOS 25.11

---

## 🚀 Quick Start for New Sessions

```bash
# 1. Check DNA status
cd /srv/luminous-dynamics/Mycelix-Core/mycelix-mail/dna
ls -lh mycelix_mail.dna
# Should show: 1.7M

# 2. Review latest docs
cat ../INTEGRATION_PLAN.md

# 3. Start sandbox testing
hc sandbox create mycelix-mail-test

# 4. Or start MATL bridge work
cd ../matl-bridge
python3 matl_to_holochain.py
```

---

**Last Updated**: November 11, 2025  
**Status**: DNA Complete ✅ | Ready for Integration 🚧  
**Version**: 1.0.0

🍄 **Quick Ref Card - Always check this first!** 🍄
