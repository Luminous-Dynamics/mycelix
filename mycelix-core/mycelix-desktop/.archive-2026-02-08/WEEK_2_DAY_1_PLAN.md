# 🚀 Week 2 Day 1: Message Broadcasting System

**Date**: September 30, 2025
**Status**: Ready to Begin
**Goal**: Implement real P2P messaging between Holochain nodes

---

## 🎯 Day 1 Objectives

Build a complete message broadcasting system that allows users to:
1. Send messages to the network
2. Receive messages from other peers
3. View a chronological message feed
4. See who sent each message

---

## 📋 Implementation Checklist

### Phase 1: Zome Functions (2-3 hours)

#### 1.1 Message Data Structure
```rust
// File: dnas/mycelix-test/zomes/messages/src/lib.rs

#[derive(Serialize, Deserialize, Debug, Clone)]
pub struct Message {
    pub content: String,
    pub timestamp: Timestamp,
    pub author: AgentPubKey,
}

#[hdk_entry_helper]
pub struct MessageEntry {
    pub content: String,
}
```

#### 1.2 Core Zome Functions
```rust
// Broadcast a message to the network
#[hdk_extern]
pub fn broadcast_message(content: String) -> ExternResult<ActionHash> {
    let message = MessageEntry { content };
    create_entry(&EntryTypes::Message(message.clone()))?;
    let hash = hash_entry(&message)?;
    Ok(hash)
}

// Get all messages from the DHT
#[hdk_extern]
pub fn get_all_messages() -> ExternResult<Vec<Message>> {
    // Query DHT for all message entries
    // Return with author and timestamp
}

// Get messages from a specific author
#[hdk_extern]
pub fn get_messages_by_author(agent: AgentPubKey) -> ExternResult<Vec<Message>> {
    // Filter messages by author
}
```

#### 1.3 Entry Definitions
```rust
#[hdk_entry_defs]
#[unit_enum(UnitEntryTypes)]
pub enum EntryTypes {
    Message(MessageEntry),
}
```

**Tasks**:
- [ ] Create new zome directory: `dnas/mycelix-test/zomes/messages/`
- [ ] Set up Cargo.toml with HDK dependencies
- [ ] Implement Message struct
- [ ] Implement broadcast_message()
- [ ] Implement get_all_messages()
- [ ] Implement get_messages_by_author()
- [ ] Add entry definitions
- [ ] Compile to WASM

---

### Phase 2: DNA Integration (30 minutes)

#### 2.1 Update DNA Manifest
```yaml
# File: dnas/mycelix-test/dna.yaml
manifest_version: "1"
name: mycelix-test
uid: 00000000-0000-0000-0000-000000000000
properties: ~
origin_time: 2024-01-01T00:00:00.000000Z

integrity:
  - name: messages_integrity
    bundled: zomes/messages.wasm

coordinator:
  - name: messages
    bundled: zomes/messages.wasm
    dependencies:
      - name: messages_integrity
```

**Tasks**:
- [ ] Update dna.yaml with messages zome
- [ ] Repack DNA: `hc dna pack dnas/mycelix-test`
- [ ] Verify DNA hash remains consistent or update configs

---

### Phase 3: Rust Backend (1-2 hours)

#### 3.1 Add Tauri Commands
```rust
// File: src-tauri/src/main.rs

#[tauri::command]
async fn send_message(
    state: State<'_, AppState>,
    content: String
) -> Result<String, String> {
    let config = state.config.lock().await;
    let params = serde_json::json!({
        "app_id": "mycelix-test",
        "cell_id": [config.dna_hash.clone(), config.agent_key.clone()],
        "zome_name": "messages",
        "fn_name": "broadcast_message",
        "payload": { "content": content }
    });

    let response = send_app_request(8889, "call_zome", params).await?;
    Ok(serde_json::to_string_pretty(&response).unwrap())
}

#[tauri::command]
async fn get_messages(state: State<'_, AppState>) -> Result<String, String> {
    let config = state.config.lock().await;
    let params = serde_json::json!({
        "app_id": "mycelix-test",
        "cell_id": [config.dna_hash.clone(), config.agent_key.clone()],
        "zome_name": "messages",
        "fn_name": "get_all_messages",
        "payload": null
    });

    let response = send_app_request(8889, "call_zome", params).await?;
    Ok(serde_json::to_string_pretty(&response).unwrap())
}
```

**Tasks**:
- [ ] Add send_message() command
- [ ] Add get_messages() command
- [ ] Add get_messages_by_author() command
- [ ] Register commands in invoke_handler
- [ ] Test compilation

---

### Phase 4: Frontend UI (2-3 hours)

#### 4.1 State Management
```typescript
// File: src/App.tsx

const [messages, setMessages] = createSignal<any[]>([]);
const [messageInput, setMessageInput] = createSignal("");
const [isSending, setIsSending] = createSignal(false);
```

#### 4.2 Message Functions
```typescript
async function sendMessage() {
    if (!messageInput().trim() || !isTauriMode()) return;

    setIsSending(true);
    try {
        const result = await invoke<string>("send_message", {
            content: messageInput()
        });

        addNotification("Message sent!", "success");
        setMessageInput("");

        // Refresh messages after sending
        setTimeout(() => fetchMessages(), 1000);
    } catch (error) {
        addNotification(`Failed to send message: ${error}`, "error");
    } finally {
        setIsSending(false);
    }
}

async function fetchMessages() {
    if (!isTauriMode()) return;

    try {
        const result = await invoke<string>("get_messages");
        const data = JSON.parse(result);
        setMessages(data || []);
    } catch (error) {
        console.error("Failed to fetch messages:", error);
    }
}

// Auto-refresh messages every 5 seconds
setInterval(() => {
    if (holochainState() === "connected") {
        fetchMessages();
    }
}, 5000);
```

#### 4.3 Message Feed UI
```tsx
{/* Message Broadcasting Card */}
<Show when={holochainState() === "connected"}>
  <div class="card card-interactive">
    <div class="card-header">
      <div class="card-icon">💬</div>
      <h2>Message Broadcasting</h2>
    </div>

    <div class="card-content">
      {/* Message Input */}
      <div class="input-group">
        <input
          type="text"
          placeholder="Type your message..."
          value={messageInput()}
          onInput={(e) => setMessageInput(e.currentTarget.value)}
          onKeyPress={(e) => e.key === 'Enter' && sendMessage()}
        />
        <button
          onClick={sendMessage}
          disabled={!messageInput().trim() || isSending()}
        >
          {isSending() ? "Sending..." : "Send"}
        </button>
      </div>

      {/* Message Feed */}
      <div class="message-feed">
        <h3>Messages ({messages().length})</h3>
        <div class="messages-container">
          <For each={messages()}>
            {(message) => (
              <div class="message-item">
                <div class="message-author">
                  {message.author.substring(0, 8)}...
                </div>
                <div class="message-content">
                  {message.content}
                </div>
                <div class="message-timestamp">
                  {new Date(message.timestamp / 1000).toLocaleString()}
                </div>
              </div>
            )}
          </For>
        </div>
      </div>

      {/* Refresh Button */}
      <button onClick={fetchMessages} class="secondary">
        🔄 Refresh Messages
      </button>
    </div>
  </div>
</Show>
```

#### 4.4 Styling
```css
/* File: src/styles.css */

.message-feed {
    margin-top: 20px;
}

.messages-container {
    max-height: 400px;
    overflow-y: auto;
    border: 1px solid rgba(255, 255, 255, 0.1);
    border-radius: 8px;
    padding: 10px;
    margin-top: 10px;
}

.message-item {
    background: rgba(255, 255, 255, 0.05);
    border-radius: 6px;
    padding: 12px;
    margin-bottom: 10px;
    border-left: 3px solid var(--accent);
}

.message-author {
    font-size: 0.85rem;
    color: var(--accent);
    font-weight: 600;
    margin-bottom: 5px;
}

.message-content {
    font-size: 1rem;
    margin-bottom: 5px;
    word-wrap: break-word;
}

.message-timestamp {
    font-size: 0.75rem;
    color: rgba(255, 255, 255, 0.5);
    text-align: right;
}
```

**Tasks**:
- [ ] Add message state signals
- [ ] Implement sendMessage() function
- [ ] Implement fetchMessages() function
- [ ] Add auto-refresh interval
- [ ] Create message feed UI component
- [ ] Add message input form
- [ ] Add styling for message feed
- [ ] Test in browser mode (mock data)

---

### Phase 5: Testing & Validation (1-2 hours)

#### 5.1 Single Node Testing
```bash
# 1. Build frontend
npm run build

# 2. Build Tauri app
npm run tauri dev

# 3. Test message sending
# - Type a message
# - Click Send
# - Verify message appears in feed
# - Check conductor logs for DHT operations
```

#### 5.2 Multi-Node Testing
```bash
# Terminal 1: First conductor
cd /srv/luminous-dynamics/Mycelix-Core/mycelix-desktop
echo '' | holochain --piped -c conductor-config.yaml

# Terminal 2: Second conductor
cd /srv/luminous-dynamics/Mycelix-Core/mycelix-desktop
echo '' | holochain --piped -c conductor-config-2.yaml

# Terminal 3: Tauri app
npm run tauri dev

# Test P2P messaging:
# 1. Send message from Node 1
# 2. Wait 10-15 seconds for DHT sync
# 3. Refresh messages on Node 2
# 4. Verify message appears
```

#### 5.3 Validation Checklist
- [ ] Messages persist to DHT
- [ ] Messages sync between peers
- [ ] Author information is correct
- [ ] Timestamps are accurate
- [ ] UI updates properly
- [ ] Error handling works
- [ ] Auto-refresh functions

---

## 🔧 Technical Details

### DHT Synchronization
- **Initial sync**: 10-15 seconds
- **Subsequent updates**: 5-10 seconds
- **Ops per message**: ~3-5 DHT operations

### Data Flow
```
User Input → Tauri Command → WebSocket → Conductor → Zome Function → DHT
DHT → Conductor → WebSocket → Tauri Command → Frontend → UI Update
```

### Error Handling
- Network disconnection
- Invalid message content
- DHT timeout
- Conductor not running
- Zome call failures

---

## 📊 Success Metrics

### Functional Requirements ✅
- [ ] User can send messages
- [ ] User can view all messages
- [ ] Messages display author info
- [ ] Messages display timestamps
- [ ] Messages sync between 2+ peers

### Performance Requirements ✅
- [ ] Message send: <500ms
- [ ] Message retrieval: <1000ms
- [ ] P2P sync: <15 seconds
- [ ] UI responsiveness: <100ms

### Quality Requirements ✅
- [ ] No console errors
- [ ] Proper error messages
- [ ] Clean, readable code
- [ ] Comprehensive comments
- [ ] Updated documentation

---

## 📝 Documentation Updates

After implementation:
- [ ] Update QUICK_REFERENCE.md with new commands
- [ ] Create MESSAGE_BROADCASTING_GUIDE.md
- [ ] Update UI_TESTING_GUIDE.md with new test scenarios
- [ ] Add screenshots to documentation
- [ ] Update WEEK_2_ROADMAP.md with Day 1 completion status

---

## 🐛 Known Issues to Address

### From Week 1
1. AppImage build failure (low priority)
2. GUI testing not yet performed
3. No automated tests yet

### Potential Day 1 Issues
1. DHT sync latency
2. Message ordering
3. Large message handling
4. Concurrent send conflicts

---

## 🚀 Next Steps (Day 2)

After completing Day 1:
1. Implement message filtering
2. Add message search
3. Implement direct messaging
4. Add message reactions
5. Improve UI/UX based on testing

---

## 🎓 Learning Objectives

### Technical Skills
- HDK entry definitions
- DHT data persistence
- Holochain query patterns
- P2P data synchronization
- Real-time UI updates

### System Understanding
- How DHT stores data
- How agents discover content
- How data propagates
- How conflicts are resolved

---

## 💡 Tips & Best Practices

1. **Start Simple**: Basic broadcast before advanced features
2. **Test Often**: Test after each phase completion
3. **Log Everything**: Use console.log and conductor logs
4. **Document Issues**: Note any problems for future reference
5. **Commit Frequently**: Git commit after each working phase

---

## 🔗 Related Documentation

- [WEEK_2_ROADMAP.md](./WEEK_2_ROADMAP.md) - Full Week 2 plan
- [HOLOCHAIN_INTEGRATION.md](./HOLOCHAIN_INTEGRATION.md) - Integration guide
- [OPTION_B_COMPLETE.md](./OPTION_B_COMPLETE.md) - Zome call reference
- [HDK Documentation](https://docs.rs/hdk/latest/hdk/) - Holochain HDK docs

---

*Built with 💜 by Luminous Dynamics*

**Status**: 📋 Ready to Begin
**Estimated Time**: 6-8 hours
**Difficulty**: Medium
**Prerequisites**: Week 1 Complete ✅

**Last Updated**: September 30, 2025
