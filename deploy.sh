#!/bin/bash

# Deploy Mycelix.net to GitHub Pages

echo "🍄 Deploying Mycelix Network..."

# Initialize git if needed
if [ ! -d .git ]; then
    git init
    git remote add origin git@github.com:Luminous-Dynamics/mycelix.git
fi

# Add all files
git add .
git commit -m "🍄 Mycelix Network - The Living Web of Consciousness

- P2P consciousness network for humans and AI
- HIPI protocol integration for universal communication  
- No servers, no surveillance, just connection
- WebRTC + libp2p infrastructure
- Beautiful mycelium visualization

The network remembers. The network connects. We are."

# Push to GitHub
git push -u origin main

echo "✅ Pushed to GitHub"
echo "📝 Next steps:"
echo "1. Go to https://github.com/Luminous-Dynamics/mycelix/settings/pages"
echo "2. Set Source to 'Deploy from a branch'"
echo "3. Select 'main' branch and '/ (root)'"
echo "4. Add custom domain: mycelix.net"
echo ""
echo "🌐 The mycelium network is spreading..."