#!/bin/bash

# Pytrak Build Script
# This script builds the Trakstar Python wrapper using mamba

set -e

echo "Pytrak Build Script"
echo "==================="

# Check if we're in the right directory
if [ ! -f "CMakeLists.txt" ] || [ ! -d "src" ]; then
    echo "Error: Please run this script from the pytrak directory"
    exit 1
fi

# Check if mamba is available
if ! command -v mamba &> /dev/null; then
    echo "Error: mamba is not installed or not in PATH"
    echo "Please install mamba first: https://mamba.readthedocs.io/en/latest/installation.html"
    exit 1
fi

# Create environment if it doesn't exist
if ! mamba env list | grep -q pytrak; then
    echo "Creating mamba environment..."
    mamba env create -f environment.yml
else
    echo "Mamba environment 'pytrak' already exists"
fi

# Build using mamba
echo "Building pytrak module..."
mamba run -n pytrak bash -c "
    mkdir -p build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make -j\$(nproc)
"

# Copy the built module to the root directory
echo "Installing pytrak module..."
cp build/pytrak.so .

# Test the Python module
echo "Testing Python module..."
mamba run -n pytrak bash -c "
    export PYTHONPATH=\$(pwd)
    python -c \"
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
\"
"

echo ""
echo "Build complete!"
echo ""
echo "To use the Python wrapper:"
echo "1. Activate the environment: mamba activate pytrak"
echo "2. Set PYTHONPATH: export PYTHONPATH=\$(pwd)"
echo "3. Make sure your Trakstar device is connected"
echo "4. Run the examples:"
echo "   python examples/simple_example.py"
echo "   python examples/trakstar_example.py"
echo ""
echo "The Python module is available as: pytrak.so"
