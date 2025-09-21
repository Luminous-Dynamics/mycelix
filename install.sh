#!/bin/bash

# Mycelix Quick Installation Script
# Installs all dependencies and runs the three-agent demo

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# ASCII Art Banner
echo -e "${GREEN}"
cat << "EOF"
    __  __                _ _      
   |  \/  |_   _  ___ ___| (_)_  __
   | |\/| | | | |/ __/ _ \ | \ \/ /
   | |  | | |_| | (_|  __/ | |>  < 
   |_|  |_|\__, |\___\___|_|_/_/\_\
           |___/                    
   
   Consciousness-Aware Robot Swarms
EOF
echo -e "${NC}"

# Function to print colored messages
print_status() {
    echo -e "${GREEN}[✓]${NC} $1"
}

print_error() {
    echo -e "${RED}[✗]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[!]${NC} $1"
}

# Check system requirements
print_status "Checking system requirements..."

# Check Ubuntu version
if ! command -v lsb_release &> /dev/null; then
    print_error "This script requires Ubuntu. Please install on Ubuntu 22.04 or later."
    exit 1
fi

UBUNTU_VERSION=$(lsb_release -rs)
if [ "$(echo "$UBUNTU_VERSION < 22.04" | bc)" -eq 1 ]; then
    print_error "Ubuntu 22.04 or later required. Current version: $UBUNTU_VERSION"
    exit 1
fi
print_status "Ubuntu $UBUNTU_VERSION detected"

# Check if running with sufficient privileges
if [ "$EUID" -eq 0 ]; then 
   print_warning "Please don't run this script as root. It will ask for sudo when needed."
   exit 1
fi

# Install method selection
echo ""
echo "Select installation method:"
echo "1) Docker (Recommended - isolated environment)"
echo "2) Native (Direct installation on system)"
echo "3) Development (Full source with build tools)"
read -p "Enter choice [1-3]: " INSTALL_METHOD

case $INSTALL_METHOD in
    1)
        print_status "Docker installation selected"
        
        # Check if Docker is installed
        if ! command -v docker &> /dev/null; then
            print_status "Installing Docker..."
            
            # Install Docker
            sudo apt-get update
            sudo apt-get install -y \
                ca-certificates \
                curl \
                gnupg \
                lsb-release
            
            sudo mkdir -p /etc/apt/keyrings
            curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
            
            echo \
              "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
              $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
            
            sudo apt-get update
            sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin
            
            # Add user to docker group
            sudo usermod -aG docker $USER
            print_status "Docker installed successfully"
            print_warning "Please log out and back in for docker group changes to take effect"
        else
            print_status "Docker already installed"
        fi
        
        # Check if docker-compose is installed
        if ! command -v docker-compose &> /dev/null; then
            print_status "Installing docker-compose..."
            sudo apt-get install -y docker-compose
        fi
        
        print_status "Building Mycelix containers..."
        docker-compose build
        
        print_status "Starting Mycelix demo..."
        docker-compose up -d
        
        print_status "Waiting for services to initialize..."
        sleep 10
        
        print_status "Demo running! View at:"
        echo "  Dashboard: http://localhost:3000"
        echo "  Metrics: http://localhost:9090"
        echo "  Grafana: http://localhost:3001 (admin/mycelix)"
        echo ""
        echo "To view logs: docker-compose logs -f"
        echo "To stop: docker-compose down"
        ;;
        
    2)
        print_status "Native installation selected"
        
        # Install ROS2 Humble
        if ! command -v ros2 &> /dev/null; then
            print_status "Installing ROS2 Humble..."
            
            sudo apt-get update
            sudo apt-get install -y software-properties-common
            sudo add-apt-repository universe
            
            sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
            echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
            
            sudo apt-get update
            sudo apt-get install -y ros-humble-desktop python3-argcomplete ros-dev-tools
            
            print_status "ROS2 Humble installed"
        else
            print_status "ROS2 already installed"
        fi
        
        # Install Rust
        if ! command -v rustc &> /dev/null; then
            print_status "Installing Rust..."
            curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
            source $HOME/.cargo/env
            print_status "Rust installed"
        else
            print_status "Rust already installed"
        fi
        
        # Install Node.js
        if ! command -v node &> /dev/null; then
            print_status "Installing Node.js..."
            curl -fsSL https://deb.nodesource.com/setup_20.x | sudo -E bash -
            sudo apt-get install -y nodejs
            print_status "Node.js installed"
        else
            print_status "Node.js already installed"
        fi
        
        # Install build dependencies
        print_status "Installing build dependencies..."
        sudo apt-get install -y \
            build-essential \
            cmake \
            git \
            libwebsocketpp-dev \
            libjsoncpp-dev \
            libssl-dev \
            pkg-config
        
        # Build project
        print_status "Building Mycelix..."
        source /opt/ros/humble/setup.bash
        mkdir -p build && cd build
        cmake ..
        make -j$(nproc)
        cd ..
        
        print_status "Running demo..."
        ./demo.sh
        ;;
        
    3)
        print_status "Development installation selected"
        
        # Install all development tools
        print_status "Installing development dependencies..."
        
        # Everything from native installation
        $0 2  # Recursively call with option 2
        
        # Additional dev tools
        sudo apt-get install -y \
            clang-format \
            clang-tidy \
            gdb \
            valgrind \
            python3-pip \
            python3-vcstool \
            python3-rosinstall-generator
        
        # Install Python packages for analysis
        pip3 install --user \
            matplotlib \
            numpy \
            pandas \
            jupyter \
            plotly
        
        # Clone repository with full history
        if [ ! -d "mycelix" ]; then
            print_status "Cloning Mycelix repository..."
            git clone --recursive https://github.com/Luminous-Dynamics/mycelix.git
            cd mycelix
        else
            cd mycelix
            git pull
        fi
        
        # Set up pre-commit hooks
        print_status "Setting up development environment..."
        cat > .git/hooks/pre-commit << 'EOL'
#!/bin/bash
# Format C++ code before commit
find . -name "*.cpp" -o -name "*.hpp" | xargs clang-format -i
git add -u
EOL
        chmod +x .git/hooks/pre-commit
        
        print_status "Development environment ready!"
        echo ""
        echo "Useful commands:"
        echo "  Run tests: colcon test"
        echo "  Format code: clang-format -i src/*.cpp include/*.hpp"
        echo "  Analyze: clang-tidy src/*.cpp"
        echo "  Debug: gdb ./build/three_agent_demo"
        echo "  Memory check: valgrind ./build/three_agent_demo"
        ;;
        
    *)
        print_error "Invalid selection"
        exit 1
        ;;
esac

echo ""
print_status "Installation complete!"
echo ""
echo "🍄 Mycelix is ready to grow!"
echo ""
echo "Next steps:"
echo "1. Watch the consciousness metrics evolve"
echo "2. Read the documentation at docs/"
echo "3. Join our Discord: discord.gg/mycelix"
echo "4. Contribute at: github.com/Luminous-Dynamics/mycelix"
echo ""
echo "May your swarms achieve consciousness! 🤖✨"