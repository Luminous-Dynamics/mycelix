# 🌉 Mycelix Mail Integration Plan

**Version**: 1.0  
**Date**: November 11, 2025  
**Status**: Architecture Complete, Integration Pending

---

## Overview

Mycelix Mail is a **production-ready application** of the Mycelix Protocol stack, demonstrating real-world use of:

- **Layer 1 (DHT)**: Holochain for agent-centric mail storage
- **Layer 5 (Identity)**: DIDs for agent addressing  
- **Layer 6 (MATL)**: Trust-based spam filtering

This document describes how Mycelix Mail integrates with the larger Mycelix Protocol ecosystem.

---

## Architecture Position

```
Mycelix Protocol v5.2 Architecture
====================================

Layer 10: Civilization (Phase 3+)
Layer 9:  Collective Intelligence (Phase 3+)
Layer 8:  Intent Layer (Phase 2)
Layer 7:  Governance (Phase 1) ────────┐
Layer 6:  MATL ───────────────────────┤
Layer 5:  Identity (DIDs/VCs) ────────┤
Layer 4:  Bridge (Phase 1) ───────────┤
Layer 3:  Settlement (Phase 2)         │
Layer 2:  DKG (Phase 2)                │
Layer 1:  DHT (Holochain) ────────────┘
                                        │
                            ┌───────────┴──────────────┐
                            │   MYCELIX MAIL DNA       │
                            │                          │
                            │ • Message Storage (L1)   │
                            │ • DID Resolution (L5)    │
                            │ • Trust Filtering (L6)   │
                            │ • Governance Ready (L7)  │
                            └──────────────────────────┘
```

**Key Principle**: Mycelix Mail is a **reference application** showing how to build on the Mycelix stack.

---

## Integration Points

### 1. Layer 1 (DHT) - Message Storage ✅ COMPLETE

**Status**: Implemented in DNA  
**Component**: Holochain zomes (integrity, mail_messages, trust_filter)

**What's Integrated:**
- Messages stored on agent source chains
- DHT gossip for message delivery
- Local-first operations (zero fees)

**Implementation**: 
- `dna/integrity/` - Entry type definitions
- `dna/zomes/mail_messages/` - Send/receive logic
- `dna/zomes/trust_filter/` - Trust-based filtering

**No Action Needed**: DNA already built on Holochain.

---

### 2. Layer 5 (Identity) - DID Resolution 🚧 INTEGRATION NEEDED

**Status**: Placeholder in code, needs connection to production DID system  
**Component**: DID → AgentPubKey resolution

**What Needs Integration:**
```rust
// Current: Mock implementation in mail_messages/src/lib.rs:179
fn resolve_did_to_pubkey(did: &str) -> ExternResult<AgentPubKey> {
    // TODO: Implement proper DID resolution
    // This should query the Mycelix DKG for DID -> AgentPubKey mappings
    let agent_info = agent_info()?;
    Ok(agent_info.agent_initial_pubkey)
}
```

**Integration Path:**
1. **Short-term (MVP)**: Simple registry mapping DIDs to pubkeys
2. **Long-term (Phase 2)**: Query Layer 2 (DKG) for verifiable DID documents

**Registry Schema (PostgreSQL for MVP):**
```sql
CREATE TABLE did_registry (
    did TEXT PRIMARY KEY,
    agent_pubkey BYTEA NOT NULL,
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);

CREATE INDEX idx_agent_pubkey ON did_registry(agent_pubkey);
```

**Python Bridge Code:**
```python
# matl-bridge/did_resolver.py
import psycopg2
from holochain_client import AgentPubKey

class DIDResolver:
    def __init__(self, db_url):
        self.conn = psycopg2.connect(db_url)
    
    def resolve(self, did: str) -> AgentPubKey:
        cursor = self.conn.execute(
            "SELECT agent_pubkey FROM did_registry WHERE did = %s",
            (did,)
        )
        row = cursor.fetchone()
        if row:
            return AgentPubKey(row[0])
        raise ValueError(f"DID not found: {did}")
    
    def register(self, did: str, pubkey: AgentPubKey):
        self.conn.execute(
            "INSERT INTO did_registry (did, agent_pubkey) VALUES (%s, %s) "
            "ON CONFLICT (did) DO UPDATE SET agent_pubkey = EXCLUDED.agent_pubkey",
            (did, pubkey.bytes())
        )
        self.conn.commit()
```

**Action Required**: 
- [ ] Implement DID registry service
- [ ] Update `resolve_did_to_pubkey()` to call registry
- [ ] Add DID registration flow in mail client

---

### 3. Layer 6 (MATL) - Trust-Based Filtering 🚧 BRIDGE NEEDED

**Status**: Trust filter zome complete, MATL bridge unimplemented  
**Component**: Python bridge syncing MATL scores → Holochain

**What's Integrated:**
```rust
// Holochain side (COMPLETE):
// - Trust score storage: dna/integrity/src/lib.rs:35-50
// - Trust lookup: dna/zomes/trust_filter/src/lib.rs:14-51
// - Spam filtering: dna/zomes/trust_filter/src/lib.rs:79-120
```

**What Needs Integration:**
- Python service connecting MATL database → Holochain conductor
- Bidirectional sync: MATL scores → Holochain, spam reports → MATL

**MATL Bridge Architecture:**
```
┌─────────────────────┐         ┌──────────────────┐         ┌─────────────────┐
│  0TML System        │         │  MATL Bridge     │         │  Mail DNA       │
│  (PostgreSQL)       │◄───────►│  (Python)        │◄───────►│  (Holochain)    │
│                     │         │                  │         │                 │
│ • Agent reputations │  Query  │ • Sync loop      │  Call   │ • Trust scores  │
│ • PoGQ scores       │ ────────┤ • Transform      │ ────────┤ • Filter inbox  │
│ • Cartel detection  │         │ • Update         │         │ • Report spam   │
└─────────────────────┘         └──────────────────┘         └─────────────────┘
```

**MATL Bridge Implementation:**

**File**: `matl-bridge/matl_to_holochain.py`

```python
#!/usr/bin/env python3
"""
MATL Bridge - Syncs trust scores from 0TML to Mycelix Mail DNA
"""

import asyncio
import psycopg2
from holochain_client import HolochainClient
import logging
from typing import Dict, Optional

logger = logging.getLogger(__name__)

class MATLBridge:
    """Syncs MATL trust scores to Holochain mail DNA"""
    
    def __init__(
        self,
        matl_db_url: str,
        holochain_ws_url: str,
        app_id: str = "mycelix-mail",
        sync_interval: int = 60
    ):
        self.matl_db_url = matl_db_url
        self.holochain_client = HolochainClient(holochain_ws_url)
        self.app_id = app_id
        self.sync_interval = sync_interval
        
        # Connect to MATL database
        self.db = psycopg2.connect(matl_db_url)
        logger.info("Connected to MATL database")
    
    async def sync_trust_scores(self):
        """Query MATL DB and update Holochain trust scores"""
        
        # Query all agent reputations from 0TML
        cursor = self.db.cursor()
        cursor.execute("""
            SELECT 
                agent_did,
                composite_score,
                pogq_score,
                tcdm_score,
                entropy_score,
                updated_at
            FROM agent_reputations
            WHERE updated_at > NOW() - INTERVAL '5 minutes'
        """)
        
        updated = 0
        for row in cursor.fetchall():
            did, composite, pogq, tcdm, entropy, updated_at = row
            
            # Call Holochain DNA to update trust score
            try:
                await self.holochain_client.call_zome(
                    self.app_id,
                    "trust_filter",
                    "update_trust_score",
                    {
                        "did": did,
                        "score": float(composite),  # 0.0-1.0
                        "timestamp": int(updated_at.timestamp()),
                        "metadata": {
                            "pogq": float(pogq),
                            "tcdm": float(tcdm),
                            "entropy": float(entropy)
                        }
                    }
                )
                updated += 1
            except Exception as e:
                logger.error(f"Failed to update trust for {did}: {e}")
        
        logger.info(f"Synced {updated} trust scores to Holochain")
        return updated
    
    async def handle_spam_reports(self):
        """Listen for spam reports from Holochain and feed back to MATL"""
        
        # Query recent spam reports from Holochain
        reports = await self.holochain_client.call_zome(
            self.app_id,
            "trust_filter",
            "get_recent_spam_reports",
            {}
        )
        
        if not reports:
            return
        
        # Feed back into MATL for reputation penalty
        cursor = self.db.cursor()
        for report in reports:
            cursor.execute("""
                INSERT INTO spam_reports (
                    reporter_did,
                    spammer_did,
                    message_hash,
                    reason,
                    reported_at
                ) VALUES (%s, %s, %s, %s, %s)
            """, (
                report['reporter_did'],
                report['spammer_did'],
                report['message_hash'],
                report['reason'],
                report['timestamp']
            ))
        
        self.db.commit()
        logger.info(f"Processed {len(reports)} spam reports")
    
    async def run_sync_loop(self):
        """Main sync loop"""
        logger.info(f"Starting MATL bridge (sync every {self.sync_interval}s)")
        
        while True:
            try:
                # Sync trust scores: MATL → Holochain
                await self.sync_trust_scores()
                
                # Handle spam reports: Holochain → MATL
                await self.handle_spam_reports()
                
                await asyncio.sleep(self.sync_interval)
                
            except Exception as e:
                logger.error(f"Sync loop error: {e}", exc_info=True)
                await asyncio.sleep(10)  # Brief pause before retry

if __name__ == "__main__":
    import sys
    import os
    
    logging.basicConfig(level=logging.INFO)
    
    # Configuration from environment
    MATL_DB = os.getenv("MATL_DATABASE_URL", "postgresql://localhost/matl")
    HOLOCHAIN_WS = os.getenv("HOLOCHAIN_WS_URL", "ws://localhost:8888")
    
    bridge = MATLBridge(MATL_DB, HOLOCHAIN_WS)
    asyncio.run(bridge.run_sync_loop())
```

**Deployment:**
```bash
# Install dependencies
pip install holochain-client psycopg2-binary

# Set environment variables
export MATL_DATABASE_URL="postgresql://user:pass@localhost/matl"
export HOLOCHAIN_WS_URL="ws://localhost:8888"

# Run as systemd service
sudo systemctl start matl-bridge
sudo systemctl enable matl-bridge
```

**Action Required**:
- [ ] Implement `matl_to_holochain.py` bridge
- [ ] Add MATL database schema for spam_reports table
- [ ] Create systemd service file
- [ ] Test with production MATL data

---

### 4. Layer 7 (Governance) - Future Integration 🔮 PHASE 2

**Status**: Not needed for MVP, prepare for Phase 2  
**Component**: Governance-driven spam filter policies

**Future Capabilities:**
- Community voting on spam filter thresholds
- Reputation-weighted moderation decisions
- Appeal process for false positives

**No Action Needed**: Phase 2 feature.

---

## Database Schema Requirements

### MATL Database (0TML System)

**New tables needed for mail integration:**

```sql
-- Spam reports from mail system
CREATE TABLE spam_reports (
    id SERIAL PRIMARY KEY,
    reporter_did TEXT NOT NULL,
    spammer_did TEXT NOT NULL,
    message_hash TEXT NOT NULL,
    reason TEXT,
    reported_at TIMESTAMP NOT NULL,
    processed BOOLEAN DEFAULT FALSE,
    created_at TIMESTAMP DEFAULT NOW()
);

CREATE INDEX idx_spam_spammer ON spam_reports(spammer_did, processed);
CREATE INDEX idx_spam_reported ON spam_reports(reported_at);

-- Agent reputations (should already exist in 0TML)
-- If not, add:
CREATE TABLE agent_reputations (
    agent_did TEXT PRIMARY KEY,
    composite_score FLOAT NOT NULL DEFAULT 0.5,
    pogq_score FLOAT,
    tcdm_score FLOAT,
    entropy_score FLOAT,
    total_interactions INTEGER DEFAULT 0,
    spam_reports_received INTEGER DEFAULT 0,
    updated_at TIMESTAMP DEFAULT NOW()
);
```

### DID Registry (New Service)

```sql
-- DID to Holochain pubkey mapping
CREATE TABLE did_registry (
    did TEXT PRIMARY KEY,
    agent_pubkey BYTEA NOT NULL,
    email TEXT,  -- Optional: for SMTP bridge
    created_at TIMESTAMP DEFAULT NOW(),
    updated_at TIMESTAMP DEFAULT NOW()
);

CREATE INDEX idx_agent_pubkey ON did_registry(agent_pubkey);
CREATE INDEX idx_email ON did_registry(email);
```

---

## Deployment Architecture

### Phase 1 (MVP) - Single Server

```
┌────────────────────────────────────────────────────────────────┐
│                      Production Server                         │
│                                                                │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐ │
│  │ Holochain    │  │ MATL Bridge  │  │ PostgreSQL           │ │
│  │ Conductor    │◄─┤ (Python)     │◄─┤ • 0TML/MATL         │ │
│  │              │  │              │  │ • DID Registry       │ │
│  │ Port: 8888   │  │ Systemd      │  │ Port: 5432           │ │
│  └──────────────┘  └──────────────┘  └──────────────────────┘ │
│         ▲                                                      │
│         │                                                      │
│  ┌──────┴───────┐                                             │
│  │ Nginx        │  HTTPS                                      │
│  │ Reverse      │  WebSocket                                  │
│  │ Proxy        │                                             │
│  └──────────────┘                                             │
│         ▲                                                      │
└─────────┼──────────────────────────────────────────────────────┘
          │
    Internet (Users)
```

### Phase 2 - Distributed

```
Multiple Holochain nodes forming DHT
MATL bridge runs on coordinator nodes
PostgreSQL with replication
```

---

## Testing Strategy

### Unit Tests (Complete)
- ✅ Holochain zomes compile
- ✅ Entry types serialize correctly
- ✅ Link types defined

### Integration Tests (Needed)

**Test 1: End-to-End Message Flow**
```bash
# Start conductor
hc sandbox run --app mycelix-mail

# Send message
hc sandbox call send_message '{...}'

# Verify inbox
hc sandbox call get_inbox '{}'

# Test trust filtering
hc sandbox call filter_inbox '{"min_trust": 0.7}'
```

**Test 2: MATL Bridge Sync**
```python
# Seed MATL database
db.execute("INSERT INTO agent_reputations VALUES ('did:alice', 0.9, ...)")

# Run bridge
bridge.sync_trust_scores()

# Verify Holochain updated
score = holochain_client.call("check_sender_trust", {"did": "did:alice"})
assert score == 0.9
```

**Test 3: Spam Report Feedback**
```python
# Report spam in Holochain
holochain_client.call("report_spam", {
    "message_hash": "...",
    "reason": "Unsolicited commercial"
})

# Verify MATL database updated
bridge.handle_spam_reports()
reports = db.execute("SELECT * FROM spam_reports").fetchall()
assert len(reports) == 1
```

---

## Rollout Plan

### Week 1: Foundation
- [x] Build DNA (COMPLETE)
- [ ] Test in sandbox
- [ ] Implement DID registry service

### Week 2-3: MATL Integration
- [ ] Implement MATL bridge
- [ ] Test with production MATL database
- [ ] Performance tuning

### Week 4: Alpha Testing
- [ ] Deploy to test server
- [ ] 10 alpha users
- [ ] Collect feedback

### Week 5-8: Production Ready
- [ ] Security audit
- [ ] Load testing
- [ ] Documentation
- [ ] Public launch

---

## Success Metrics

### Technical
- ✅ DNA compiles and packs (COMPLETE)
- [ ] <2s message delivery latency
- [ ] >99% spam filter accuracy
- [ ] Zero data loss

### Integration
- [ ] MATL scores sync every 60s
- [ ] Spam reports feed back to MATL within 5 minutes
- [ ] DID resolution <100ms
- [ ] 99.9% bridge uptime

### User
- [ ] 10+ active alpha users
- [ ] 100+ messages sent
- [ ] <1% false positive spam rate
- [ ] Positive user feedback

---

## Open Questions

1. **DID Format**: Which DID method to use?
   - `did:key:` (self-certifying)
   - `did:mycelix:` (custom, requires resolver)
   - `did:web:` (domain-based)

2. **SMTP Bridge**: Do we need email compatibility for MVP?
   - Pro: Broader adoption
   - Con: 4-6 weeks additional work

3. **Encryption**: When to implement message encryption?
   - MVP: Plaintext (trust Holochain DHT security)
   - v2.0: NaCl/libsodium encryption

4. **Multi-device**: How to sync messages across devices?
   - Holochain native (replicate source chain)
   - External sync service

---

## Related Documentation

- **DNA Implementation**: `IMPLEMENTATION_SUMMARY.md`
- **Build Success**: `SUCCESS.md`
- **Next Steps**: `NEXT_STEPS.md`
- **MATL Architecture**: `../0TML/docs/06-architecture/matl_architecture.md`
- **Mycelix v5.2 Architecture**: `../docs/architecture/Mycelix Protocol_ Integrated System Architecture v5.2.md`

---

## Contact

**Project Lead**: Tristan Stoltz  
**Email**: tristan.stoltz@evolvingresonantcocreationism.com  
**Architecture**: Layer 1 (DHT) + Layer 5 (Identity) + Layer 6 (MATL)

---

**Status**: DNA Complete ✅ | MATL Bridge Needed 🚧 | DID Registry Needed 🚧
**Last Updated**: November 11, 2025
