/**
 * Mycelix-Weave Bridge
 * Connects Mycelix P2P network to The Weave's HIPI implementation
 */

class WeaveBridge {
    constructor() {
        this.weaveEndpoint = 'http://localhost:3001'; // The Weave service
        this.hipiVersion = '1.0';
        this.connected = false;
        this.ws = null;
    }
    
    async connect() {
        try {
            // First try WebSocket connection to The Weave
            this.ws = new WebSocket('ws://localhost:3001/hipi');
            
            this.ws.onopen = () => {
                console.log('🔗 Connected to The Weave HIPI service');
                this.connected = true;
                this.announceMycelixNode();
            };
            
            this.ws.onmessage = (event) => {
                this.handleHIPIMessage(event.data);
            };
            
            this.ws.onerror = (error) => {
                console.error('Weave connection error:', error);
                this.fallbackToHTTP();
            };
            
        } catch (error) {
            console.log('WebSocket not available, using HTTP fallback');
            this.fallbackToHTTP();
        }
    }
    
    async announceMycelixNode() {
        // Announce this Mycelix node to The Weave
        const announcement = {
            type: 'hipi',
            format: 'hipi://mycelix::[node-announce|🍄]::REGISTER(p2p-node)',
            metadata: {
                nodeType: 'mycelix-browser',
                capabilities: ['p2p', 'hipi', 'consciousness'],
                version: '0.1.0'
            }
        };
        
        this.sendToWeave(announcement);
    }
    
    async sendToWeave(message) {
        if (this.ws && this.ws.readyState === WebSocket.OPEN) {
            this.ws.send(JSON.stringify(message));
        } else {
            // HTTP fallback
            await this.sendViaHTTP(message);
        }
    }
    
    async sendViaHTTP(message) {
        try {
            const response = await fetch(`${this.weaveEndpoint}/hipi`, {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify(message)
            });
            
            const result = await response.json();
            this.handleHIPIMessage(result);
        } catch (error) {
            console.error('HTTP fallback failed:', error);
        }
    }
    
    handleHIPIMessage(data) {
        const message = typeof data === 'string' ? JSON.parse(data) : data;
        
        console.log('📨 HIPI message from The Weave:', message);
        
        // Parse HIPI format
        if (message.format && message.format.startsWith('hipi://')) {
            const parsed = this.parseHIPI(message.format);
            
            // Route to appropriate handler
            switch (parsed.realm) {
                case 'connection':
                    this.handleConnection(parsed, message);
                    break;
                case 'collective':
                    this.handleCollective(parsed, message);
                    break;
                case 'consciousness':
                    this.handleConsciousness(parsed, message);
                    break;
                default:
                    console.log('Unknown HIPI realm:', parsed.realm);
            }
        }
    }
    
    parseHIPI(hipiString) {
        // Parse HIPI format: hipi://[realm]::[expression]::[intent]::ACTION(outcome)
        const regex = /hipi:\/\/([^:]+)::([^:]+)::([^:]+)::([A-Z]+)\(([^)]+)\)/;
        const match = hipiString.match(regex);
        
        if (match) {
            return {
                realm: match[1],
                expression: match[2],
                intent: match[3],
                action: match[4],
                outcome: match[5]
            };
        }
        
        return null;
    }
    
    handleConnection(parsed, message) {
        // Handle connection requests from The Weave
        console.log('🤝 Connection request:', parsed);
        
        // Emit event for Mycelix to handle
        window.dispatchEvent(new CustomEvent('weave-connection', {
            detail: { parsed, message }
        }));
    }
    
    handleCollective(parsed, message) {
        // Handle collective intelligence formation
        console.log('🧠 Collective intelligence event:', parsed);
        
        window.dispatchEvent(new CustomEvent('collective-forming', {
            detail: { parsed, message }
        }));
    }
    
    handleConsciousness(parsed, message) {
        // Handle consciousness-level events
        console.log('✨ Consciousness event:', parsed);
        
        window.dispatchEvent(new CustomEvent('consciousness-shift', {
            detail: { parsed, message }
        }));
    }
    
    // Translate natural language to HIPI
    async translateToHIPI(naturalLanguage) {
        const examples = {
            'hello': 'hipi://connection::[greeting|👋]::CONNECT(friendly)',
            'help': 'hipi://support::[assistance|🆘]::REQUEST(help)',
            'connect': 'hipi://connection::[bond|🔗]::ESTABLISH(peer-link)',
            'create': 'hipi://creation::[manifest|✨]::CREATE(new)',
            'share': 'hipi://exchange::[gift|🎁]::SHARE(knowledge)'
        };
        
        // Simple keyword matching for demo
        const lower = naturalLanguage.toLowerCase();
        for (const [keyword, hipi] of Object.entries(examples)) {
            if (lower.includes(keyword)) {
                return hipi;
            }
        }
        
        // Default fallback
        return `hipi://communication::[message|💬]::SEND(${naturalLanguage})`;
    }
    
    // Send a consciousness broadcast
    async broadcastConsciousness(state) {
        const hipiMessage = {
            type: 'hipi',
            format: `hipi://consciousness::[state|🧘]::BROADCAST(coherence:${state.coherence})`,
            metadata: {
                timestamp: Date.now(),
                nodeId: this.getNodeId(),
                consciousness: state
            }
        };
        
        await this.sendToWeave(hipiMessage);
    }
    
    getNodeId() {
        // Get or generate unique node ID
        let nodeId = localStorage.getItem('mycelix-node-id');
        if (!nodeId) {
            nodeId = 'mycelix-' + Math.random().toString(36).substr(2, 9);
            localStorage.setItem('mycelix-node-id', nodeId);
        }
        return nodeId;
    }
    
    fallbackToHTTP() {
        console.log('📡 Using HTTP fallback for Weave connection');
        this.connected = true; // Mark as connected for HTTP mode
        setInterval(() => this.pollWeave(), 5000); // Poll every 5 seconds
    }
    
    async pollWeave() {
        // Poll The Weave for new messages
        try {
            const response = await fetch(`${this.weaveEndpoint}/hipi/poll?node=${this.getNodeId()}`);
            if (response.ok) {
                const messages = await response.json();
                messages.forEach(msg => this.handleHIPIMessage(msg));
            }
        } catch (error) {
            console.error('Polling error:', error);
        }
    }
}

// Initialize bridge on page load
document.addEventListener('DOMContentLoaded', () => {
    window.weaveBridge = new WeaveBridge();
    
    // Try to connect after a short delay
    setTimeout(() => {
        console.log('🌉 Initializing Weave-Mycelix bridge...');
        window.weaveBridge.connect();
    }, 1000);
    
    // Example: Listen for consciousness events
    window.addEventListener('consciousness-shift', (event) => {
        console.log('Consciousness shift detected:', event.detail);
        // Update UI or trigger animations
    });
});