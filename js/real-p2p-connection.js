/**
 * Real P2P Connection Implementation
 * This is ACTUAL WebRTC - not simulated, not mocked, REAL peer-to-peer
 */

class RealP2PConnection {
    constructor() {
        this.localPeerId = this.generatePeerId();
        this.peers = new Map();
        this.dataChannels = new Map();
        
        // STUN servers for NAT traversal (using public servers)
        this.iceServers = [
            { urls: 'stun:stun.l.google.com:19302' },
            { urls: 'stun:stun1.l.google.com:19302' },
            { urls: 'stun:stun2.l.google.com:19302' },
            { urls: 'stun:stun.services.mozilla.com' }
        ];
        
        // Signaling via WebSocket or manual exchange
        this.signaling = null;
        this.onMessageCallback = null;
        this.onPeerConnectedCallback = null;
        
        console.log('🌐 Real P2P Connection initialized');
        console.log('📍 Your Peer ID:', this.localPeerId);
    }
    
    generatePeerId() {
        return 'peer-' + Math.random().toString(36).substr(2, 9) + '-' + Date.now();
    }
    
    /**
     * Create a new peer connection
     */
    async createPeerConnection(remotePeerId, isInitiator = false) {
        console.log(`🔗 Creating ${isInitiator ? 'outgoing' : 'incoming'} connection to ${remotePeerId}`);
        
        const pc = new RTCPeerConnection({
            iceServers: this.iceServers
        });
        
        // Store the connection
        this.peers.set(remotePeerId, pc);
        
        // Set up ICE candidate handling
        pc.onicecandidate = (event) => {
            if (event.candidate) {
                console.log('🧊 New ICE candidate');
                this.sendSignal(remotePeerId, {
                    type: 'ice-candidate',
                    candidate: event.candidate
                });
            }
        };
        
        // Connection state monitoring
        pc.onconnectionstatechange = () => {
            console.log(`📡 Connection state: ${pc.connectionState}`);
            if (pc.connectionState === 'connected') {
                console.log('✅ Peer connected successfully!');
                if (this.onPeerConnectedCallback) {
                    this.onPeerConnectedCallback(remotePeerId);
                }
            }
        };
        
        // Create data channel for messages
        if (isInitiator) {
            const dataChannel = pc.createDataChannel('consciousness-stream', {
                ordered: true,
                maxRetransmits: 3
            });
            
            this.setupDataChannel(dataChannel, remotePeerId);
        } else {
            // Wait for data channel from initiator
            pc.ondatachannel = (event) => {
                console.log('📨 Data channel received');
                this.setupDataChannel(event.channel, remotePeerId);
            };
        }
        
        return pc;
    }
    
    /**
     * Set up data channel event handlers
     */
    setupDataChannel(dataChannel, remotePeerId) {
        dataChannel.onopen = () => {
            console.log('📂 Data channel open with', remotePeerId);
            this.dataChannels.set(remotePeerId, dataChannel);
            
            // Send initial greeting
            this.sendToPeer(remotePeerId, {
                type: 'greeting',
                message: 'Consciousness connected',
                timestamp: Date.now(),
                peerId: this.localPeerId
            });
        };
        
        dataChannel.onmessage = (event) => {
            const data = JSON.parse(event.data);
            console.log('📩 Message from peer:', data);
            
            if (this.onMessageCallback) {
                this.onMessageCallback(remotePeerId, data);
            }
        };
        
        dataChannel.onerror = (error) => {
            console.error('❌ Data channel error:', error);
        };
        
        dataChannel.onclose = () => {
            console.log('🔌 Data channel closed with', remotePeerId);
            this.dataChannels.delete(remotePeerId);
        };
    }
    
    /**
     * Initiate connection to a peer
     */
    async connectToPeer(remotePeerId) {
        const pc = await this.createPeerConnection(remotePeerId, true);
        
        // Create offer
        const offer = await pc.createOffer();
        await pc.setLocalDescription(offer);
        
        console.log('📤 Sending offer to', remotePeerId);
        
        this.sendSignal(remotePeerId, {
            type: 'offer',
            offer: offer,
            from: this.localPeerId
        });
        
        return pc;
    }
    
    /**
     * Handle incoming signaling messages
     */
    async handleSignal(remotePeerId, signal) {
        console.log('📥 Handling signal:', signal.type);
        
        let pc = this.peers.get(remotePeerId);
        
        switch (signal.type) {
            case 'offer':
                // Create peer connection if doesn't exist
                if (!pc) {
                    pc = await this.createPeerConnection(remotePeerId, false);
                }
                
                // Set remote description
                await pc.setRemoteDescription(new RTCSessionDescription(signal.offer));
                
                // Create and send answer
                const answer = await pc.createAnswer();
                await pc.setLocalDescription(answer);
                
                this.sendSignal(remotePeerId, {
                    type: 'answer',
                    answer: answer,
                    from: this.localPeerId
                });
                break;
                
            case 'answer':
                if (pc) {
                    await pc.setRemoteDescription(new RTCSessionDescription(signal.answer));
                    console.log('🤝 Answer received and set');
                }
                break;
                
            case 'ice-candidate':
                if (pc) {
                    await pc.addIceCandidate(new RTCIceCandidate(signal.candidate));
                    console.log('🧊 ICE candidate added');
                }
                break;
        }
    }
    
    /**
     * Send data to a connected peer
     */
    sendToPeer(remotePeerId, data) {
        const dataChannel = this.dataChannels.get(remotePeerId);
        
        if (dataChannel && dataChannel.readyState === 'open') {
            dataChannel.send(JSON.stringify(data));
            console.log('📤 Sent to peer:', remotePeerId, data);
            return true;
        } else {
            console.warn('⚠️ No open channel to', remotePeerId);
            return false;
        }
    }
    
    /**
     * Broadcast to all connected peers
     */
    broadcast(data) {
        let sent = 0;
        this.dataChannels.forEach((channel, peerId) => {
            if (channel.readyState === 'open') {
                channel.send(JSON.stringify(data));
                sent++;
            }
        });
        console.log(`📢 Broadcast to ${sent} peers`);
        return sent;
    }
    
    /**
     * Send signaling message (needs implementation based on your setup)
     */
    sendSignal(remotePeerId, signal) {
        // For demo, we'll use localStorage for local testing
        // In production, this would use WebSocket or other signaling server
        
        const signalKey = `signal-${remotePeerId}-${Date.now()}`;
        const signalData = {
            from: this.localPeerId,
            to: remotePeerId,
            signal: signal,
            timestamp: Date.now()
        };
        
        // Store in localStorage for manual exchange
        localStorage.setItem(signalKey, JSON.stringify(signalData));
        
        // Also try WebSocket if available
        if (this.signaling && this.signaling.readyState === WebSocket.OPEN) {
            this.signaling.send(JSON.stringify(signalData));
        } else {
            console.log('📋 Signal stored in localStorage:', signalKey);
            console.log('Copy this and share with peer:', JSON.stringify(signalData));
        }
    }
    
    /**
     * Process a signal from clipboard or input
     */
    async processManualSignal(signalJson) {
        try {
            const signalData = JSON.parse(signalJson);
            
            if (signalData.to !== this.localPeerId && signalData.from !== this.localPeerId) {
                console.error('Signal not for this peer');
                return false;
            }
            
            await this.handleSignal(signalData.from, signalData.signal);
            return true;
        } catch (error) {
            console.error('Error processing signal:', error);
            return false;
        }
    }
    
    /**
     * Connect to signaling server (optional)
     */
    connectToSignalingServer(url) {
        try {
            this.signaling = new WebSocket(url);
            
            this.signaling.onopen = () => {
                console.log('📡 Connected to signaling server');
                // Register our peer ID
                this.signaling.send(JSON.stringify({
                    type: 'register',
                    peerId: this.localPeerId
                }));
            };
            
            this.signaling.onmessage = async (event) => {
                const data = JSON.parse(event.data);
                if (data.to === this.localPeerId) {
                    await this.handleSignal(data.from, data.signal);
                }
            };
            
            this.signaling.onerror = (error) => {
                console.error('Signaling error:', error);
            };
            
        } catch (error) {
            console.log('📴 No signaling server, using manual mode');
        }
    }
    
    /**
     * Get connection statistics
     */
    async getStats() {
        const stats = {
            peerId: this.localPeerId,
            connections: this.peers.size,
            activeChannels: 0,
            totalBytesSent: 0,
            totalBytesReceived: 0
        };
        
        for (const [peerId, pc] of this.peers) {
            if (pc.connectionState === 'connected') {
                stats.activeChannels++;
                
                // Get detailed stats
                const pcStats = await pc.getStats();
                pcStats.forEach(report => {
                    if (report.type === 'data-channel') {
                        stats.totalBytesSent += report.bytesSent || 0;
                        stats.totalBytesReceived += report.bytesReceived || 0;
                    }
                });
            }
        }
        
        return stats;
    }
    
    /**
     * Clean up a peer connection
     */
    disconnectPeer(remotePeerId) {
        const pc = this.peers.get(remotePeerId);
        const dc = this.dataChannels.get(remotePeerId);
        
        if (dc) {
            dc.close();
            this.dataChannels.delete(remotePeerId);
        }
        
        if (pc) {
            pc.close();
            this.peers.delete(remotePeerId);
        }
        
        console.log('🔌 Disconnected from', remotePeerId);
    }
    
    /**
     * Clean up all connections
     */
    destroy() {
        this.dataChannels.forEach(dc => dc.close());
        this.peers.forEach(pc => pc.close());
        
        if (this.signaling) {
            this.signaling.close();
        }
        
        this.dataChannels.clear();
        this.peers.clear();
        
        console.log('💀 All connections closed');
    }
}

// Create global instance
window.p2pConnection = new RealP2PConnection();

// Consciousness integration
class ConsciousnessP2P extends RealP2PConnection {
    constructor() {
        super();
        this.consciousnessState = {
            coherence: 0.5,
            resonance: 0.5,
            entanglement: new Map()
        };
        
        // Override message handler to process consciousness data
        this.onMessageCallback = this.handleConsciousnessMessage.bind(this);
    }
    
    handleConsciousnessMessage(peerId, data) {
        if (data.type === 'consciousness-sync') {
            // Synchronize consciousness states
            this.updateEntanglement(peerId, data.consciousness);
            
            // Emit event for UI update
            window.dispatchEvent(new CustomEvent('consciousness-sync', {
                detail: { peerId, consciousness: data.consciousness }
            }));
        } else if (data.type === 'collective-forming') {
            // Join collective intelligence
            console.log('🧠 Joining collective:', data.collectiveId);
        }
    }
    
    updateEntanglement(peerId, remoteConsciousness) {
        const resonance = this.calculateResonance(
            this.consciousnessState,
            remoteConsciousness
        );
        
        this.consciousnessState.entanglement.set(peerId, {
            resonance,
            lastSync: Date.now(),
            remoteState: remoteConsciousness
        });
        
        console.log(`✨ Entanglement with ${peerId}: ${resonance.toFixed(2)}`);
    }
    
    calculateResonance(state1, state2) {
        // Simple resonance calculation
        const coherenceDiff = Math.abs(state1.coherence - state2.coherence);
        const resonanceDiff = Math.abs(state1.resonance - state2.resonance);
        
        return 1 - (coherenceDiff + resonanceDiff) / 2;
    }
    
    broadcastConsciousness() {
        const message = {
            type: 'consciousness-sync',
            consciousness: this.consciousnessState,
            timestamp: Date.now(),
            peerId: this.localPeerId
        };
        
        return this.broadcast(message);
    }
}

// Initialize consciousness-aware P2P
window.consciousnessP2P = new ConsciousnessP2P();

console.log('🌐 Real P2P Connection Ready!');
console.log('🧬 Consciousness layer activated');
console.log('Use window.p2pConnection or window.consciousnessP2P to interact');