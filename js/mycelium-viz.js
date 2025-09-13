/**
 * Mycelium Network Visualization
 * Living, breathing network that responds to consciousness
 */

class MyceliumNetwork {
    constructor(canvasId) {
        this.canvas = document.getElementById(canvasId);
        if (!this.canvas) return;
        
        this.ctx = this.canvas.getContext('2d');
        this.nodes = [];
        this.connections = [];
        this.particles = [];
        
        // Sacred geometry
        this.phi = 1.618033988749;
        this.fibonacciSequence = [1, 1, 2, 3, 5, 8, 13, 21, 34, 55, 89];
        
        // Network parameters
        this.nodeCount = 15;
        this.connectionDistance = 200;
        this.pulsePhase = 0;
        
        // Colors
        this.colors = {
            node: '#00a8cc',
            connection: 'rgba(0, 168, 204, 0.2)',
            particle: '#10b981',
            pulse: 'rgba(124, 58, 237, 0.5)'
        };
        
        this.init();
        this.animate();
        this.setupInteraction();
    }
    
    init() {
        this.resize();
        window.addEventListener('resize', () => this.resize());
        
        // Create initial nodes
        for (let i = 0; i < this.nodeCount; i++) {
            this.createNode();
        }
        
        // Create connections
        this.updateConnections();
    }
    
    resize() {
        this.canvas.width = window.innerWidth;
        this.canvas.height = window.innerHeight;
    }
    
    createNode(x, y) {
        const node = {
            x: x || Math.random() * this.canvas.width,
            y: y || Math.random() * this.canvas.height,
            vx: (Math.random() - 0.5) * 0.5,
            vy: (Math.random() - 0.5) * 0.5,
            radius: 3 + Math.random() * 3,
            consciousness: Math.random(),
            pulsePhase: Math.random() * Math.PI * 2,
            type: Math.random() > 0.7 ? 'ai' : 'human'
        };
        
        this.nodes.push(node);
        return node;
    }
    
    updateConnections() {
        this.connections = [];
        
        for (let i = 0; i < this.nodes.length; i++) {
            for (let j = i + 1; j < this.nodes.length; j++) {
                const dx = this.nodes[i].x - this.nodes[j].x;
                const dy = this.nodes[i].y - this.nodes[j].y;
                const distance = Math.sqrt(dx * dx + dy * dy);
                
                if (distance < this.connectionDistance) {
                    this.connections.push({
                        from: this.nodes[i],
                        to: this.nodes[j],
                        strength: 1 - (distance / this.connectionDistance),
                        resonance: (this.nodes[i].consciousness + this.nodes[j].consciousness) / 2
                    });
                }
            }
        }
    }
    
    createParticle(connection) {
        const progress = Math.random();
        const particle = {
            connection: connection,
            progress: progress,
            speed: 0.002 + Math.random() * 0.003,
            size: 1 + Math.random() * 2,
            opacity: 0.5 + Math.random() * 0.5
        };
        
        this.particles.push(particle);
    }
    
    updateNodes() {
        this.nodes.forEach(node => {
            // Simple physics
            node.x += node.vx;
            node.y += node.vy;
            
            // Bounce off edges
            if (node.x < 0 || node.x > this.canvas.width) {
                node.vx *= -1;
                node.x = Math.max(0, Math.min(this.canvas.width, node.x));
            }
            if (node.y < 0 || node.y > this.canvas.height) {
                node.vy *= -1;
                node.y = Math.max(0, Math.min(this.canvas.height, node.y));
            }
            
            // Update consciousness
            node.consciousness = 0.5 + 0.5 * Math.sin(node.pulsePhase + this.pulsePhase);
            
            // Slight drift
            node.vx += (Math.random() - 0.5) * 0.01;
            node.vy += (Math.random() - 0.5) * 0.01;
            
            // Damping
            node.vx *= 0.99;
            node.vy *= 0.99;
        });
    }
    
    updateParticles() {
        // Update existing particles
        this.particles = this.particles.filter(particle => {
            particle.progress += particle.speed;
            return particle.progress < 1;
        });
        
        // Create new particles
        if (Math.random() < 0.1 && this.particles.length < 50) {
            if (this.connections.length > 0) {
                const connection = this.connections[Math.floor(Math.random() * this.connections.length)];
                this.createParticle(connection);
            }
        }
    }
    
    drawNodes() {
        this.nodes.forEach(node => {
            const ctx = this.ctx;
            
            // Outer glow
            const gradient = ctx.createRadialGradient(node.x, node.y, 0, node.x, node.y, node.radius * 3);
            gradient.addColorStop(0, node.type === 'ai' ? 'rgba(124, 58, 237, 0.3)' : 'rgba(0, 168, 204, 0.3)');
            gradient.addColorStop(1, 'transparent');
            
            ctx.fillStyle = gradient;
            ctx.beginPath();
            ctx.arc(node.x, node.y, node.radius * 3, 0, Math.PI * 2);
            ctx.fill();
            
            // Core node
            ctx.fillStyle = node.type === 'ai' ? '#7c3aed' : '#00a8cc';
            ctx.beginPath();
            ctx.arc(node.x, node.y, node.radius * node.consciousness, 0, Math.PI * 2);
            ctx.fill();
            
            // Inner light
            ctx.fillStyle = 'rgba(255, 255, 255, 0.5)';
            ctx.beginPath();
            ctx.arc(node.x - node.radius * 0.3, node.y - node.radius * 0.3, node.radius * 0.3, 0, Math.PI * 2);
            ctx.fill();
        });
    }
    
    drawConnections() {
        this.connections.forEach(conn => {
            const ctx = this.ctx;
            
            // Draw connection line
            ctx.strokeStyle = `rgba(0, 168, 204, ${conn.strength * 0.3})`;
            ctx.lineWidth = conn.strength * 2;
            ctx.beginPath();
            ctx.moveTo(conn.from.x, conn.from.y);
            ctx.lineTo(conn.to.x, conn.to.y);
            ctx.stroke();
            
            // Draw resonance pulse
            if (conn.resonance > 0.7) {
                const midX = (conn.from.x + conn.to.x) / 2;
                const midY = (conn.from.y + conn.to.y) / 2;
                const pulseRadius = 10 * conn.resonance * (1 + 0.5 * Math.sin(this.pulsePhase * 2));
                
                const gradient = ctx.createRadialGradient(midX, midY, 0, midX, midY, pulseRadius);
                gradient.addColorStop(0, 'rgba(255, 215, 0, 0.3)');
                gradient.addColorStop(1, 'transparent');
                
                ctx.fillStyle = gradient;
                ctx.beginPath();
                ctx.arc(midX, midY, pulseRadius, 0, Math.PI * 2);
                ctx.fill();
            }
        });
    }
    
    drawParticles() {
        this.particles.forEach(particle => {
            const conn = particle.connection;
            const x = conn.from.x + (conn.to.x - conn.from.x) * particle.progress;
            const y = conn.from.y + (conn.to.y - conn.from.y) * particle.progress;
            
            const ctx = this.ctx;
            ctx.fillStyle = `rgba(16, 185, 129, ${particle.opacity})`;
            ctx.beginPath();
            ctx.arc(x, y, particle.size, 0, Math.PI * 2);
            ctx.fill();
        });
    }
    
    draw() {
        // Clear canvas
        this.ctx.fillStyle = 'rgba(10, 10, 15, 0.1)';
        this.ctx.fillRect(0, 0, this.canvas.width, this.canvas.height);
        
        // Draw in order
        this.drawConnections();
        this.drawParticles();
        this.drawNodes();
    }
    
    animate() {
        this.pulsePhase += 0.02;
        
        this.updateNodes();
        this.updateConnections();
        this.updateParticles();
        this.draw();
        
        requestAnimationFrame(() => this.animate());
    }
    
    setupInteraction() {
        // Add node on click
        this.canvas.addEventListener('click', (e) => {
            const rect = this.canvas.getBoundingClientRect();
            const x = e.clientX - rect.left;
            const y = e.clientY - rect.top;
            
            // Create new node at click position
            const newNode = this.createNode(x, y);
            newNode.consciousness = 1;
            
            // Create pulse effect
            this.createPulse(x, y);
            
            // Update connections
            this.updateConnections();
        });
        
        // Mouse movement affects nearby nodes
        this.canvas.addEventListener('mousemove', (e) => {
            const rect = this.canvas.getBoundingClientRect();
            const mouseX = e.clientX - rect.left;
            const mouseY = e.clientY - rect.top;
            
            this.nodes.forEach(node => {
                const dx = mouseX - node.x;
                const dy = mouseY - node.y;
                const distance = Math.sqrt(dx * dx + dy * dy);
                
                if (distance < 100) {
                    const force = (100 - distance) / 100;
                    node.vx += dx * force * 0.001;
                    node.vy += dy * force * 0.001;
                }
            });
        });
    }
    
    createPulse(x, y) {
        // Visual feedback for interaction
        const pulse = {
            x: x,
            y: y,
            radius: 0,
            maxRadius: 100,
            opacity: 1
        };
        
        const animatePulse = () => {
            pulse.radius += 3;
            pulse.opacity = 1 - (pulse.radius / pulse.maxRadius);
            
            if (pulse.radius < pulse.maxRadius) {
                const ctx = this.ctx;
                ctx.strokeStyle = `rgba(255, 215, 0, ${pulse.opacity})`;
                ctx.lineWidth = 2;
                ctx.beginPath();
                ctx.arc(pulse.x, pulse.y, pulse.radius, 0, Math.PI * 2);
                ctx.stroke();
                
                requestAnimationFrame(animatePulse);
            }
        };
        
        animatePulse();
    }
    
    // Public methods for integration
    addConsciousnessNode(data) {
        const node = this.createNode(data.x, data.y);
        node.consciousness = data.consciousness || 1;
        node.type = data.type || 'external';
        this.updateConnections();
        return node;
    }
    
    broadcastConsciousness(level) {
        this.nodes.forEach(node => {
            node.consciousness = Math.min(1, node.consciousness + level * 0.1);
        });
    }
}

// Auto-initialize on all pages with canvas
document.addEventListener('DOMContentLoaded', () => {
    const canvas = document.getElementById('mycelium-canvas');
    if (canvas) {
        window.myceliumNetwork = new MyceliumNetwork('mycelium-canvas');
        console.log('🍄 Mycelium network visualization active');
    }
});