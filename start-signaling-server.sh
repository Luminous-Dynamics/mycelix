#!/bin/bash

# Start the Mycelix Signaling Server
# This is optional - the P2P network works without it

echo "🍄 Starting Mycelix Signaling Server"
echo "===================================="
echo ""
echo "This server only helps peers find each other."
echo "All actual communication is P2P (browser-to-browser)."
echo ""

# Check if Node.js is installed
if ! command -v node &> /dev/null; then
    echo "❌ Node.js is not installed"
    echo "Please install Node.js first:"
    echo "  nix-shell -p nodejs"
    echo "  or"
    echo "  sudo apt install nodejs"
    exit 1
fi

# Check if npm is installed
if ! command -v npm &> /dev/null; then
    echo "❌ npm is not installed"
    echo "Please install npm first"
    exit 1
fi

# Install dependencies if needed
if [ ! -d "node_modules" ]; then
    echo "📦 Installing dependencies..."
    npm install
fi

# Configuration
PORT=${PORT:-8765}
HOST=${HOST:-0.0.0.0}

echo "Starting server on $HOST:$PORT"
echo ""
echo "📡 WebSocket endpoint: ws://localhost:$PORT"
echo "🌐 Status page: http://localhost:$PORT"
echo ""
echo "Peers can connect to this server for easier discovery,"
echo "but they can also connect directly using manual signal exchange."
echo ""
echo "Press Ctrl+C to stop the server"
echo ""

# Start the server
PORT=$PORT node signaling-server.js