#!/bin/bash

# Trakstar Build Script
# This script builds the Trakstar Python wrapper (requires sudo for dependencies)

set -e

echo "Trakstar Build Script"
echo "===================="

# Check if we're in the right directory
if [ ! -f "CMakeLists.txt" ] || [ ! -d "src" ]; then
    echo "Error: Please run this script from the pytrak directory"
    exit 1
fi

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Install system dependencies
echo "Installing system dependencies..."
if command_exists apt-get; then
    sudo apt-get update
    sudo apt-get install -y libusb-1.0-dev python3-pybind11 python3-dev cmake build-essential
elif command_exists yum; then
    sudo yum install -y libusb1-devel python3-pybind11 python3-devel cmake gcc-c++
elif command_exists brew; then
    brew install libusb pybind11 cmake
else
    echo "Warning: Package manager not found. Please install dependencies manually:"
    echo "  - libusb-1.0-dev"
    echo "  - python3-pybind11"
    echo "  - python3-dev"
    echo "  - cmake"
    echo "  - build-essential"
fi

# Install Python dependencies
echo "Installing Python dependencies..."
if command_exists pip3; then
    pip3 install numpy
elif command_exists pip; then
    pip install numpy
else
    echo "Warning: pip not found. Please install numpy manually: pip install numpy"
fi

# Create build directory
echo "Creating build directory..."
mkdir -p build
cd build

# Configure with CMake
echo "Configuring with CMake..."
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build
echo "Building..."
make -j$(nproc)

# Check if build was successful
if [ -f "pytrak.so" ]; then
    echo "✓ Python module built successfully!"
    echo "Location: $(pwd)/pytrak.so"
else
    echo "✗ Build failed. Please check the build output above."
    exit 1
fi

# Install
echo "Installing..."
sudo make install

# Test the Python module
echo "Testing Python module..."
python3 -c "
import sys
sys.path.append('.')
try:
    import pytrak
    print('✓ Python module imported successfully!')
    
    # Test initialization
    trakstar = pytrak.Trakstar()
    if trakstar.is_ok():
        print('✓ Trakstar device initialized successfully!')
        print(f'Number of sensors: {trakstar.get_number_of_sensors()}')
    else:
        print('⚠ Device initialization failed (this is normal if no device is connected)')
        
except ImportError as e:
    print(f'✗ Failed to import pytrak: {e}')
    sys.exit(1)
except Exception as e:
    print(f'⚠ Error during testing: {e}')
"

echo ""
echo "Build complete!"
echo ""
echo "To use the Python wrapper:"
echo "1. Make sure your Trakstar device is connected"
echo "2. Run the examples:"
echo "   python3 examples/simple_example.py"
echo "   python3 examples/trakstar_example.py"
echo ""
echo "The Python module is installed to: /usr/local/lib/python3/dist-packages/pytrak.so"
echo "You can also copy pytrak.so to your project directory for local use."
