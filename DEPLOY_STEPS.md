# 🚀 Mycelix.net - Step-by-Step Deployment

## Step 1: Initialize Git Repository
```bash
cd /home/tstoltz/Luminous-Dynamics/_websites/mycelix.net
git init
git config user.name "Your Name"
git config user.email "your.email@example.com"
```

## Step 2: Commit All Files
```bash
git add .
git commit -m "Initial commit: Mycelix P2P Consciousness Network"
```

## Step 3: Create GitHub Repository

### Option A: Using GitHub CLI
```bash
gh repo create Luminous-Dynamics/mycelix --public \
  --description "P2P consciousness network for humans and AI" \
  --homepage "https://mycelix.net"
```

### Option B: Using GitHub Website
1. Go to https://github.com/new
2. Repository name: `mycelix`
3. Owner: `Luminous-Dynamics`
4. Description: `P2P consciousness network for humans and AI`
5. Public repository
6. Click "Create repository"

## Step 4: Push to GitHub
```bash
git remote add origin https://github.com/Luminous-Dynamics/mycelix.git
git branch -M main
git push -u origin main
```

## Step 5: Enable GitHub Pages
1. Go to: https://github.com/Luminous-Dynamics/mycelix/settings/pages
2. Source: Deploy from a branch
3. Branch: `main`
4. Folder: `/ (root)`
5. Click Save

## Step 6: Configure DNS (for mycelix.net)

### Add these A records to your domain provider:
```
Type: A    Name: @    Value: 185.199.108.153
Type: A    Name: @    Value: 185.199.109.153
Type: A    Name: @    Value: 185.199.110.153
Type: A    Name: @    Value: 185.199.111.153
```

### Optional: Add CNAME for www
```
Type: CNAME    Name: www    Value: luminous-dynamics.github.io
```

## Step 7: Verify Deployment

### Check these URLs in order:
1. **Immediate**: https://luminous-dynamics.github.io/mycelix
2. **After DNS (1-24 hours)**: https://mycelix.net

## Step 8: Test P2P Connection

### Local Test Right Now:
1. Open: file:///home/tstoltz/Luminous-Dynamics/_websites/mycelix.net/real-demo.html
2. Open in two browser windows
3. Copy Peer ID from window 1
4. Paste in window 2 and connect
5. Exchange signals and chat!

## Quick Copy-Paste Commands

```bash
# All in one (if you have gh CLI):
cd /home/tstoltz/Luminous-Dynamics/_websites/mycelix.net && \
git init && \
git add . && \
git commit -m "Mycelix Network Launch" && \
gh repo create Luminous-Dynamics/mycelix --public -y && \
git remote add origin https://github.com/Luminous-Dynamics/mycelix.git && \
git push -u origin main
```

## Troubleshooting

### If gh command fails:
- Install with: `nix-shell -p gh`
- Or use the GitHub website to create repo manually

### If git push fails:
- Check authentication: `gh auth status`
- Login if needed: `gh auth login`

### If DNS doesn't work:
- Wait up to 24 hours for propagation
- Check with: `dig mycelix.net`
- Verify A records are added correctly

## 🎉 Success Checklist

- [ ] Git repository initialized
- [ ] Files committed
- [ ] GitHub repository created
- [ ] Code pushed to GitHub
- [ ] GitHub Pages enabled
- [ ] DNS records added
- [ ] Site accessible at GitHub URL
- [ ] Site accessible at mycelix.net
- [ ] P2P demo working

## 🍄 The Network is Ready!

Once deployed, share these links:
- Demo: https://mycelix.net/real-demo.html
- GitHub: https://github.com/Luminous-Dynamics/mycelix
- Testing Guide: https://mycelix.net/COMMUNITY_TESTING.html