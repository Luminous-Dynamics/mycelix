// Mycelix Network Connection Handler
// Simulating P2P connection initiation

function connectToNetwork() {
    const button = event.target.closest('button');
    const originalText = button.innerHTML;
    
    // Stage 1: Initializing
    button.innerHTML = '<span class="spinner">⚡</span> Initializing node...';
    button.disabled = true;
    
    setTimeout(() => {
        // Stage 2: Finding peers
        button.innerHTML = '<span class="spinner">🔍</span> Finding peers...';
        
        setTimeout(() => {
            // Stage 3: Establishing connections
            button.innerHTML = '<span class="spinner">🔗</span> Establishing connections...';
            
            setTimeout(() => {
                // Stage 4: Connected
                button.innerHTML = '<span>✅</span> Connected to Mycelix';
                button.style.background = 'linear-gradient(135deg, #10b981 0%, #00a8cc 100%)';
                
                // Show connection notification
                showNotification('Welcome to the Mycelix Network', 'You are now part of the living web.');
                
                // Update node count
                const nodeCount = document.getElementById('node-count');
                if (nodeCount) {
                    const current = parseInt(nodeCount.textContent.replace(',', ''));
                    nodeCount.textContent = (current + 1).toLocaleString();
                }
                
                // Restore button after delay
                setTimeout(() => {
                    button.innerHTML = originalText;
                    button.disabled = false;
                    button.style.background = '';
                }, 3000);
            }, 1500);
        }, 1500);
    }, 1000);
}

function showNotification(title, message) {
    // Create notification element
    const notification = document.createElement('div');
    notification.className = 'network-notification';
    notification.innerHTML = `
        <div class="notification-content">
            <h4>${title}</h4>
            <p>${message}</p>
        </div>
    `;
    
    // Add styles
    notification.style.cssText = `
        position: fixed;
        top: 100px;
        right: 20px;
        background: linear-gradient(135deg, rgba(124, 58, 237, 0.9) 0%, rgba(0, 168, 204, 0.9) 100%);
        color: white;
        padding: 1.5rem;
        border-radius: 15px;
        box-shadow: 0 20px 40px rgba(0, 0, 0, 0.3);
        z-index: 10000;
        animation: slideIn 0.5s ease;
        max-width: 300px;
    `;
    
    document.body.appendChild(notification);
    
    // Remove after delay
    setTimeout(() => {
        notification.style.animation = 'slideOut 0.5s ease';
        setTimeout(() => notification.remove(), 500);
    }, 4000);
}

// Add CSS animations
const style = document.createElement('style');
style.textContent = `
    @keyframes slideIn {
        from {
            transform: translateX(400px);
            opacity: 0;
        }
        to {
            transform: translateX(0);
            opacity: 1;
        }
    }
    
    @keyframes slideOut {
        from {
            transform: translateX(0);
            opacity: 1;
        }
        to {
            transform: translateX(400px);
            opacity: 0;
        }
    }
    
    @keyframes spin {
        from { transform: rotate(0deg); }
        to { transform: rotate(360deg); }
    }
    
    .spinner {
        display: inline-block;
        animation: spin 1s linear infinite;
    }
`;
document.head.appendChild(style);

// Handle intention setting from homepage
function setIntention(intention) {
    // Store intention
    localStorage.setItem('mycelix-intention', intention);
    
    // Smooth scroll to relevant section
    let targetSection;
    switch(intention) {
        case 'explore':
            targetSection = '#vision';
            break;
        case 'build':
            targetSection = '#build';
            break;
        case 'invest':
            targetSection = '#join';
            break;
        case 'join':
            targetSection = '#join';
            break;
        default:
            targetSection = '#vision';
    }
    
    const element = document.querySelector(targetSection);
    if (element) {
        element.scrollIntoView({ behavior: 'smooth' });
    }
}

// Initialize on load
document.addEventListener('DOMContentLoaded', () => {
    // Check for stored intention
    const intention = localStorage.getItem('mycelix-intention');
    if (intention) {
        console.log(`Welcome back. Your intention: ${intention}`);
        // Could customize experience based on intention
    }
    
    // Add smooth scrolling to all anchor links
    document.querySelectorAll('a[href^="#"]').forEach(anchor => {
        anchor.addEventListener('click', function (e) {
            e.preventDefault();
            const target = document.querySelector(this.getAttribute('href'));
            if (target) {
                target.scrollIntoView({ behavior: 'smooth' });
            }
        });
    });
    
    // Parallax effect on scroll
    let ticking = false;
    function updateParallax() {
        const scrolled = window.pageYOffset;
        const parallax = document.querySelector('.mycelium-network');
        if (parallax) {
            parallax.style.transform = `translateY(${scrolled * 0.5}px)`;
        }
        ticking = false;
    }
    
    window.addEventListener('scroll', () => {
        if (!ticking) {
            requestAnimationFrame(updateParallax);
            ticking = true;
        }
    });
});