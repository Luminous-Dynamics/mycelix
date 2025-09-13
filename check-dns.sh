#!/bin/bash

# DNS Configuration Checker for mycelix.net
# This script verifies that DNS is properly configured for GitHub Pages

echo "🔍 Checking DNS Configuration for mycelix.net"
echo "============================================="
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m'

# GitHub Pages IPs
GITHUB_IPS=(
    "185.199.108.153"
    "185.199.109.153"
    "185.199.110.153"
    "185.199.111.153"
)

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check for required tools
echo "Checking required tools..."
if command_exists dig; then
    echo -e "${GREEN}✓${NC} dig is installed"
else
    echo -e "${RED}✗${NC} dig is not installed. Installing..."
    sudo apt-get update && sudo apt-get install -y dnsutils
fi

if command_exists curl; then
    echo -e "${GREEN}✓${NC} curl is installed"
else
    echo -e "${RED}✗${NC} curl is not installed"
fi

echo ""
echo "Checking A Records..."
echo "---------------------"

# Check A records
A_RECORDS=$(dig +short mycelix.net A)

if [ -z "$A_RECORDS" ]; then
    echo -e "${RED}✗${NC} No A records found for mycelix.net"
    echo "  Please add A records pointing to GitHub Pages IPs"
else
    echo "Found A records:"
    echo "$A_RECORDS" | while read -r ip; do
        if [[ " ${GITHUB_IPS[@]} " =~ " ${ip} " ]]; then
            echo -e "  ${GREEN}✓${NC} $ip (GitHub Pages)"
        else
            echo -e "  ${YELLOW}⚠${NC} $ip (Not a GitHub Pages IP)"
        fi
    done
    
    # Check if all GitHub IPs are present
    missing_ips=()
    for github_ip in "${GITHUB_IPS[@]}"; do
        if ! echo "$A_RECORDS" | grep -q "$github_ip"; then
            missing_ips+=("$github_ip")
        fi
    done
    
    if [ ${#missing_ips[@]} -gt 0 ]; then
        echo -e "${YELLOW}⚠${NC} Missing GitHub Pages IPs:"
        for ip in "${missing_ips[@]}"; do
            echo "  - $ip"
        done
    else
        echo -e "${GREEN}✓${NC} All GitHub Pages IPs are configured"
    fi
fi

echo ""
echo "Checking CNAME Record (www)..."
echo "-------------------------------"

# Check CNAME for www
CNAME_RECORD=$(dig +short www.mycelix.net CNAME)
if [ -z "$CNAME_RECORD" ]; then
    echo -e "${YELLOW}⚠${NC} No CNAME record for www.mycelix.net (optional)"
else
    echo -e "${GREEN}✓${NC} CNAME record: $CNAME_RECORD"
fi

echo ""
echo "Checking HTTP/HTTPS Response..."
echo "--------------------------------"

# Check HTTP response from GitHub Pages URL
echo "Testing GitHub Pages URL..."
GITHUB_URL="https://luminous-dynamics.github.io/mycelix"
if curl -s -o /dev/null -w "%{http_code}" "$GITHUB_URL" | grep -q "200\|301\|302"; then
    echo -e "${GREEN}✓${NC} $GITHUB_URL is responding"
else
    echo -e "${RED}✗${NC} $GITHUB_URL is not responding (may not be deployed yet)"
fi

# Check custom domain
echo ""
echo "Testing custom domain..."
CUSTOM_URL="https://mycelix.net"
HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" -L "$CUSTOM_URL")
if [ "$HTTP_CODE" = "200" ]; then
    echo -e "${GREEN}✓${NC} $CUSTOM_URL is working!"
elif [ "$HTTP_CODE" = "000" ]; then
    echo -e "${YELLOW}⚠${NC} $CUSTOM_URL is not reachable (DNS may still be propagating)"
else
    echo -e "${YELLOW}⚠${NC} $CUSTOM_URL returned HTTP $HTTP_CODE"
fi

echo ""
echo "Checking SSL Certificate..."
echo "---------------------------"

# Check SSL certificate
if command_exists openssl; then
    SSL_CHECK=$(echo | openssl s_client -connect mycelix.net:443 -servername mycelix.net 2>/dev/null | openssl x509 -noout -dates 2>/dev/null)
    if [ -n "$SSL_CHECK" ]; then
        echo -e "${GREEN}✓${NC} SSL certificate is installed"
        echo "$SSL_CHECK" | grep "notAfter"
    else
        echo -e "${YELLOW}⚠${NC} SSL certificate not found (may still be provisioning)"
    fi
else
    echo -e "${YELLOW}⚠${NC} openssl not installed, cannot check SSL"
fi

echo ""
echo "DNS Propagation Status..."
echo "-------------------------"

# Check DNS from multiple resolvers
RESOLVERS=(
    "8.8.8.8:Google"
    "1.1.1.1:Cloudflare"
    "208.67.222.222:OpenDNS"
)

for resolver_info in "${RESOLVERS[@]}"; do
    IFS=':' read -r resolver name <<< "$resolver_info"
    result=$(dig @$resolver +short mycelix.net A 2>/dev/null | head -n1)
    if [ -n "$result" ]; then
        echo -e "${GREEN}✓${NC} $name DNS: $result"
    else
        echo -e "${RED}✗${NC} $name DNS: No response"
    fi
done

echo ""
echo "========================================="
echo "Summary"
echo "========================================="

# Final summary
all_good=true

# Check A records
if echo "$A_RECORDS" | grep -q "185.199"; then
    echo -e "${GREEN}✓${NC} A records configured"
else
    echo -e "${RED}✗${NC} A records need configuration"
    all_good=false
fi

# Check if site is accessible
if [ "$HTTP_CODE" = "200" ]; then
    echo -e "${GREEN}✓${NC} Site is accessible"
else
    echo -e "${YELLOW}⚠${NC} Site not yet accessible (may need time)"
    all_good=false
fi

echo ""
if [ "$all_good" = true ]; then
    echo -e "${GREEN}🎉 DNS configuration looks good!${NC}"
    echo "Your site should be available at https://mycelix.net"
else
    echo -e "${YELLOW}📝 DNS configuration needs attention${NC}"
    echo "Please check DNS_SETUP.md for configuration instructions"
fi

echo ""
echo "🍄 The mycelium network awaits..."