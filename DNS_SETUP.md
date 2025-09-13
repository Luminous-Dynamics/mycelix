# 🌐 DNS Configuration for mycelix.net

## Quick Setup Guide

### Step 1: GitHub Pages IP Addresses
Add these A records to your DNS provider:

```
Type: A
Name: @
Value: 185.199.108.153

Type: A
Name: @
Value: 185.199.109.153

Type: A
Name: @
Value: 185.199.110.153

Type: A
Name: @
Value: 185.199.111.153
```

### Step 2: WWW Subdomain (Optional)
Add a CNAME record for www:

```
Type: CNAME
Name: www
Value: luminous-dynamics.github.io
```

### Step 3: Verify CNAME File
Ensure the CNAME file in your repository contains:
```
mycelix.net
```

## DNS Provider-Specific Instructions

### Cloudflare
1. Log into Cloudflare Dashboard
2. Select mycelix.net domain
3. Go to DNS settings
4. Add the A records above
5. **IMPORTANT**: Set Proxy status to "DNS only" (gray cloud, not orange)
6. SSL/TLS setting should be "Full"

### Namecheap
1. Sign in to Namecheap
2. Go to Domain List → Manage
3. Advanced DNS tab
4. Add A Records with @ as host
5. Add all 4 GitHub IPs

### GoDaddy
1. Sign in to GoDaddy
2. Go to My Products → DNS
3. Add → A record
4. Points to: GitHub IPs
5. TTL: 600 seconds

## Verification Steps

### 1. Check DNS Propagation
```bash
# Check A records
dig mycelix.net

# Check from different server
nslookup mycelix.net 8.8.8.8

# Check CNAME
dig www.mycelix.net
```

### 2. Test GitHub Pages
Visit: https://luminous-dynamics.github.io/mycelix
Should work immediately after deployment

### 3. Test Custom Domain
Visit: https://mycelix.net
May take up to 24 hours for DNS propagation

## SSL Certificate

GitHub Pages automatically provisions an SSL certificate via Let's Encrypt once:
1. DNS is properly configured
2. Domain is verified
3. May take up to 24 hours

Check status at:
https://github.com/Luminous-Dynamics/mycelix/settings/pages

## Troubleshooting

### "404 Not Found" Error
- Ensure CNAME file exists in repository
- Check that gh-pages branch is selected in settings
- Wait 10 minutes for GitHub to update

### "DNS Check Failed" in GitHub
- Verify all 4 A records are added
- Ensure no conflicting AAAA (IPv6) records
- If using Cloudflare, disable proxy (gray cloud)

### SSL Certificate Not Working
- Wait up to 24 hours for provisioning
- Check "Enforce HTTPS" in GitHub Pages settings
- Ensure DNS is correctly pointing to GitHub

### Site Shows Old Content
- Clear browser cache
- Try incognito/private browsing
- Check GitHub Actions for deployment status

## Command Line Verification

```bash
# Full DNS check
./check-dns.sh

# Test connection
curl -I https://mycelix.net

# Check SSL certificate
openssl s_client -connect mycelix.net:443 -servername mycelix.net
```

## Timeline

- **Immediate**: Site available at luminous-dynamics.github.io/mycelix
- **10-30 minutes**: GitHub recognizes custom domain
- **1-6 hours**: DNS propagation (usually)
- **Up to 24 hours**: Full global DNS propagation
- **Up to 24 hours**: SSL certificate provisioning

## Support

- GitHub Pages: https://docs.github.com/en/pages
- DNS Issues: Check with your domain registrar
- Community: Open issue on GitHub repository

---

🍄 Once configured, the mycelium network will be globally accessible at https://mycelix.net