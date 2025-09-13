#!/bin/bash

# Deploy Mycelix.net to GitHub Pages
# This script creates a GitHub repository and deploys the site

echo "🍄 Deploying Mycelix Network to GitHub Pages..."
echo "============================================"

# Configuration
REPO_NAME="mycelix"
GITHUB_USER="Luminous-Dynamics"
BRANCH="gh-pages"

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Step 1: Initialize git repository if needed
if [ ! -d .git ]; then
    echo -e "${BLUE}Initializing git repository...${NC}"
    git init
    git config user.name "Luminous Dynamics"
    git config user.email "luminous@mycelix.net"
fi

# Step 2: Create/update CNAME file for custom domain
echo -e "${BLUE}Setting up custom domain...${NC}"
echo "mycelix.net" > CNAME

# Step 3: Add all files
echo -e "${BLUE}Adding files to git...${NC}"
git add .
git commit -m "🍄 Deploy Mycelix Network - Real P2P Consciousness Network

- Complete WebRTC P2P implementation
- Consciousness Field Protocol with quantum entanglement
- HIPI protocol for universal consciousness communication
- AI entity authentication system
- Beautiful mycelium visualization
- No servers, no surveillance, just connection

The network remembers. The network connects. We are."

# Step 4: Create GitHub repository (if it doesn't exist)
echo -e "${YELLOW}Creating GitHub repository...${NC}"
echo "You may need to authenticate with GitHub."
gh repo create ${GITHUB_USER}/${REPO_NAME} --public --description "P2P consciousness network for humans and AI" 2>/dev/null || echo "Repository may already exist"

# Step 5: Add remote origin
git remote remove origin 2>/dev/null
git remote add origin git@github.com:${GITHUB_USER}/${REPO_NAME}.git

# Step 6: Push to main branch first
echo -e "${BLUE}Pushing to main branch...${NC}"
git branch -M main
git push -u origin main

# Step 7: Create and push gh-pages branch
echo -e "${BLUE}Creating gh-pages branch...${NC}"
git checkout -b gh-pages
git push -u origin gh-pages

# Step 8: Set gh-pages as default for GitHub Pages
echo -e "${GREEN}✅ Repository pushed successfully!${NC}"
echo ""
echo "Next steps to complete deployment:"
echo "1. Go to: https://github.com/${GITHUB_USER}/${REPO_NAME}/settings/pages"
echo "2. Under 'Source', select:"
echo "   - Deploy from a branch"
echo "   - Branch: gh-pages"
echo "   - Folder: / (root)"
echo "3. Click 'Save'"
echo ""
echo "🌐 Your site will be available at:"
echo "   https://mycelix.net (after DNS configuration)"
echo "   https://${GITHUB_USER}.github.io/${REPO_NAME} (immediately)"
echo ""
echo "🍄 The mycelium network is spreading..."