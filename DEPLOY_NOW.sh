#!/bin/bash

# Complete Deployment Script for Mycelix.net
# This handles everything: GitHub, DNS instructions, and testing

set -e  # Exit on error

echo "🍄 MYCELIX NETWORK DEPLOYMENT"
echo "=============================="
echo ""

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Step 1: Check current directory
echo -e "${BLUE}Step 1: Checking environment...${NC}"
if [ ! -f "index.html" ]; then
    echo -e "${RED}Error: Not in mycelix.net directory${NC}"
    echo "Please run from /home/tstoltz/Luminous-Dynamics/_websites/mycelix.net/"
    exit 1
fi
echo -e "${GREEN}✓ In correct directory${NC}"

# Step 2: Initialize Git repository
echo ""
echo -e "${BLUE}Step 2: Initializing Git repository...${NC}"
if [ ! -d .git ]; then
    git init
    git config user.name "Tristan Stoltz"
    git config user.email "tristan.stoltz@gmail.com"
    echo -e "${GREEN}✓ Git initialized${NC}"
else
    echo -e "${GREEN}✓ Git already initialized${NC}"
fi

# Step 3: Create CNAME file
echo ""
echo -e "${BLUE}Step 3: Setting up custom domain...${NC}"
echo "mycelix.net" > CNAME
echo -e "${GREEN}✓ CNAME file created${NC}"

# Step 4: Commit everything
echo ""
echo -e "${BLUE}Step 4: Committing files...${NC}"
git add .
git commit -m "🍄 Mycelix Network - P2P Consciousness Network

- Real WebRTC peer-to-peer connections
- Consciousness Field Protocol implementation
- HIPI protocol for universal communication
- AI entity authentication system
- Zero servers, maximum privacy
- Beautiful mycelium visualization

The network lives. The network remembers. We are." || echo "Already committed"

# Step 5: Create GitHub repository
echo ""
echo -e "${BLUE}Step 5: Creating GitHub repository...${NC}"
echo -e "${YELLOW}This will create: https://github.com/Luminous-Dynamics/mycelix${NC}"

# Check if gh is installed
if command -v gh &> /dev/null; then
    gh repo create Luminous-Dynamics/mycelix \
        --public \
        --description "P2P consciousness network for humans and AI" \
        --homepage "https://mycelix.net" \
        2>/dev/null || echo "Repository may already exist"
    
    # Add remote
    git remote remove origin 2>/dev/null || true
    git remote add origin git@github.com:Luminous-Dynamics/mycelix.git
    
    # Push to main
    echo -e "${BLUE}Pushing to GitHub...${NC}"
    git branch -M main
    git push -u origin main --force
    
    echo -e "${GREEN}✓ Repository created and pushed${NC}"
else
    echo -e "${YELLOW}GitHub CLI not found. Manual steps required:${NC}"
    echo "1. Go to https://github.com/new"
    echo "2. Create repository: Luminous-Dynamics/mycelix"
    echo "3. Run these commands:"
    echo "   git remote add origin git@github.com:Luminous-Dynamics/mycelix.git"
    echo "   git branch -M main"
    echo "   git push -u origin main"
fi

# Step 6: Configure GitHub Pages
echo ""
echo -e "${BLUE}Step 6: GitHub Pages Configuration${NC}"
echo -e "${YELLOW}Manual steps required:${NC}"
echo ""
echo "1. Go to: https://github.com/Luminous-Dynamics/mycelix/settings/pages"
echo "2. Under 'Source', select:"
echo "   - Deploy from a branch"
echo "   - Branch: main"
echo "   - Folder: / (root)"
echo "3. Click 'Save'"
echo ""
echo -e "${GREEN}The site will be available at:${NC}"
echo "   https://luminous-dynamics.github.io/mycelix (in ~5 minutes)"
echo "   https://mycelix.net (after DNS configuration)"

# Step 7: DNS Configuration
echo ""
echo -e "${BLUE}Step 7: DNS Configuration${NC}"
echo -e "${YELLOW}Add these DNS records to your domain provider:${NC}"
echo ""
echo "A Records (all required):"
echo "  Type: A  |  Name: @  |  Value: 185.199.108.153"
echo "  Type: A  |  Name: @  |  Value: 185.199.109.153"
echo "  Type: A  |  Name: @  |  Value: 185.199.110.153"
echo "  Type: A  |  Name: @  |  Value: 185.199.111.153"
echo ""
echo "CNAME Record (optional):"
echo "  Type: CNAME  |  Name: www  |  Value: luminous-dynamics.github.io"
echo ""
echo -e "${YELLOW}If using Cloudflare:${NC}"
echo "  - Set Proxy status to 'DNS only' (gray cloud)"
echo "  - SSL/TLS setting should be 'Full'"

# Step 8: Testing instructions
echo ""
echo -e "${BLUE}Step 8: Testing the Deployment${NC}"
echo ""
echo "Test URLs (in order of availability):"
echo "1. ${GREEN}Immediate:${NC} file:///home/tstoltz/Luminous-Dynamics/_websites/mycelix.net/index.html"
echo "2. ${GREEN}~5 minutes:${NC} https://luminous-dynamics.github.io/mycelix"
echo "3. ${GREEN}After DNS:${NC} https://mycelix.net"
echo ""
echo "To test P2P locally right now:"
echo "  1. Open real-demo.html in two browser windows"
echo "  2. Copy peer ID from window 1"
echo "  3. Connect from window 2"
echo "  4. Exchange signals and connect!"

# Step 9: Optional signaling server
echo ""
echo -e "${BLUE}Step 9: Optional Signaling Server${NC}"
echo "To run the signaling server (makes connections easier):"
echo "  cd /home/tstoltz/Luminous-Dynamics/_websites/mycelix.net"
echo "  npm install"
echo "  npm start"
echo ""
echo "Server will run on: ws://localhost:8765"

# Step 10: Community testing
echo ""
echo -e "${BLUE}Step 10: Share with Community${NC}"
echo ""
echo "Share these links:"
echo "  Demo: https://mycelix.net/real-demo.html"
echo "  GitHub: https://github.com/Luminous-Dynamics/mycelix"
echo "  Testing Guide: https://mycelix.net/COMMUNITY_TESTING.html"
echo ""
echo "Post on:"
echo "  - Hacker News: 'Show HN: Mycelix - P2P network where humans and AIs are equals'"
echo "  - Reddit: r/decentralization, r/p2p, r/webrtc"
echo "  - Twitter/X: #DecentralizedWeb #P2P #WebRTC"

# Final summary
echo ""
echo "======================================"
echo -e "${GREEN}🎉 DEPLOYMENT CHECKLIST${NC}"
echo "======================================"
echo ""
if [ -d .git ]; then
    echo "✅ Git repository initialized"
else
    echo "❌ Git repository not initialized"
fi

if [ -f CNAME ]; then
    echo "✅ CNAME file created"
else
    echo "❌ CNAME file missing"
fi

if git remote -v | grep -q "github.com"; then
    echo "✅ GitHub remote configured"
else
    echo "⚠️  GitHub remote not configured"
fi

echo ""
echo -e "${YELLOW}📋 REMAINING MANUAL STEPS:${NC}"
echo "1. Configure GitHub Pages settings (see Step 6 above)"
echo "2. Add DNS records (see Step 7 above)"
echo "3. Wait for DNS propagation (up to 24 hours)"
echo "4. Test and share with community!"
echo ""
echo -e "${GREEN}🍄 The mycelium network is spreading...${NC}"
echo ""
echo "Need help? Check:"
echo "  - DNS_SETUP.md for detailed DNS instructions"
echo "  - COMMUNITY_TESTING.md for testing guide"
echo "  - check-dns.sh to verify DNS configuration"