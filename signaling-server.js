#!/usr/bin/env node

/**
 * Minimal Signaling Server for Mycelix P2P Network
 * 
 * This is ONLY for initial connection establishment.
 * Once peers are connected, they communicate directly without this server.
 * 
 * The server can be completely optional - peers can exchange signals manually.
 */

const WebSocket = require('ws');
const http = require('http');

const PORT = process.env.PORT || 8765;

// Create HTTP server
const server = http.createServer((req, res) => {
    // Basic CORS headers
    res.setHeader('Access-Control-Allow-Origin', '*');
    res.setHeader('Access-Control-Allow-Methods', 'GET, POST, OPTIONS');
    res.setHeader('Access-Control-Allow-Headers', 'Content-Type');
    
    if (req.method === 'OPTIONS') {
        res.writeHead(200);
        res.end();
        return;
    }
    
    if (req.url === '/') {
        res.writeHead(200, { 'Content-Type': 'text/html' });
        res.end(`
            <!DOCTYPE html>
            <html>
            <head>
                <title>Mycelix Signaling Server</title>
                <style>
                    body {
                        background: #0a0a0f;
                        color: #00a8cc;
                        font-family: monospace;
                        padding: 2rem;
                        text-align: center;
                    }
                    .stats {
                        background: rgba(0, 168, 204, 0.1);
                        border: 1px solid #00a8cc;
                        border-radius: 10px;
                        padding: 2rem;
                        margin: 2rem auto;
                        max-width: 600px;
                    }
                    h1 { color: #7c3aed; }
                    .count { font-size: 3rem; font-weight: bold; }
                </style>
            </head>
            <body>
                <h1>🍄 Mycelix Signaling Server</h1>
                <div class="stats">
                    <p>Active Peers</p>
                    <div class="count" id="peer-count">0</div>
                    <p>This server only helps peers find each other.</p>
                    <p>All communication happens directly between browsers.</p>
                </div>
                <p style="opacity: 0.7;">
                    Connect at: ws://localhost:${PORT}
                </p>
                <script>
                    // Auto-refresh stats
                    setInterval(() => {
                        fetch('/stats')
                            .then(r => r.json())
                            .then(data => {
                                document.getElementById('peer-count').textContent = data.peers;
                            });
                    }, 1000);
                </script>
            </body>
            </html>
        `);
    } else if (req.url === '/stats') {
        res.writeHead(200, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({
            peers: peers.size,
            timestamp: Date.now()
        }));
    } else {
        res.writeHead(404);
        res.end('Not found');
    }
});

// Create WebSocket server
const wss = new WebSocket.Server({ server });

// Store connected peers
const peers = new Map();

// Handle WebSocket connections
wss.on('connection', (ws, req) => {
    let peerId = null;
    
    console.log('🔗 New connection from:', req.socket.remoteAddress);
    
    ws.on('message', (message) => {
        try {
            const data = JSON.parse(message);
            
            switch (data.type) {
                case 'register':
                    // Register this peer
                    peerId = data.peerId;
                    peers.set(peerId, ws);
                    
                    console.log(`✅ Registered peer: ${peerId}`);
                    console.log(`📊 Total peers: ${peers.size}`);
                    
                    // Send confirmation
                    ws.send(JSON.stringify({
                        type: 'registered',
                        peerId: peerId,
                        peers: Array.from(peers.keys()).filter(id => id !== peerId)
                    }));
                    
                    // Notify other peers
                    broadcast({
                        type: 'peer-joined',
                        peerId: peerId
                    }, peerId);
                    break;
                    
                case 'signal':
                    // Forward signal to target peer
                    const targetWs = peers.get(data.to);
                    if (targetWs && targetWs.readyState === WebSocket.OPEN) {
                        targetWs.send(JSON.stringify({
                            type: 'signal',
                            from: data.from || peerId,
                            signal: data.signal
                        }));
                        console.log(`📨 Forwarded signal: ${data.from || peerId} → ${data.to}`);
                    } else {
                        console.log(`❌ Target peer not found: ${data.to}`);
                        ws.send(JSON.stringify({
                            type: 'error',
                            message: 'Target peer not connected'
                        }));
                    }
                    break;
                    
                case 'broadcast':
                    // Broadcast to all peers except sender
                    broadcast({
                        type: 'broadcast',
                        from: peerId,
                        data: data.data
                    }, peerId);
                    break;
                    
                case 'ping':
                    // Simple ping-pong for connection testing
                    ws.send(JSON.stringify({
                        type: 'pong',
                        timestamp: Date.now()
                    }));
                    break;
                    
                default:
                    // Forward any unknown message types to target
                    if (data.to) {
                        const targetWs = peers.get(data.to);
                        if (targetWs && targetWs.readyState === WebSocket.OPEN) {
                            targetWs.send(JSON.stringify(data));
                        }
                    }
            }
        } catch (error) {
            console.error('Error processing message:', error);
        }
    });
    
    ws.on('close', () => {
        if (peerId) {
            peers.delete(peerId);
            console.log(`👋 Peer disconnected: ${peerId}`);
            console.log(`📊 Total peers: ${peers.size}`);
            
            // Notify other peers
            broadcast({
                type: 'peer-left',
                peerId: peerId
            }, peerId);
        }
    });
    
    ws.on('error', (error) => {
        console.error('WebSocket error:', error);
    });
    
    // Send initial welcome
    ws.send(JSON.stringify({
        type: 'welcome',
        message: 'Connected to Mycelix signaling server',
        timestamp: Date.now()
    }));
});

// Broadcast to all peers except sender
function broadcast(message, excludePeerId) {
    const messageStr = JSON.stringify(message);
    
    peers.forEach((ws, peerId) => {
        if (peerId !== excludePeerId && ws.readyState === WebSocket.OPEN) {
            ws.send(messageStr);
        }
    });
}

// Start server
server.listen(PORT, () => {
    console.log('🍄 Mycelix Signaling Server');
    console.log('================================');
    console.log(`📡 WebSocket: ws://localhost:${PORT}`);
    console.log(`🌐 Status: http://localhost:${PORT}`);
    console.log('================================');
    console.log('This server only helps peers find each other.');
    console.log('All actual communication is P2P (browser-to-browser).');
    console.log('');
    console.log('Peers can also connect without this server by');
    console.log('manually exchanging connection signals.');
    console.log('');
    console.log('🌟 The network is alive...');
});

// Graceful shutdown
process.on('SIGINT', () => {
    console.log('\n🛑 Shutting down signaling server...');
    
    // Notify all peers
    broadcast({
        type: 'server-shutdown',
        message: 'Signaling server shutting down'
    });
    
    // Close all connections
    peers.forEach(ws => ws.close());
    
    server.close(() => {
        console.log('👋 Server closed');
        process.exit(0);
    });
});