#!/bin/bash

# Expected directory components
EXPECTED_DIR="APF-HGG/apf"

# Function to find the correct directory
find_project_root() {
  local dir="$PWD"
  while [[ "$dir" != "/" ]]; do
    if [[ -d "$dir/$EXPECTED_DIR" ]]; then
      echo "$dir/$EXPECTED_DIR"
      return
    fi
    dir=$(dirname "$dir")
  done
  echo ""
}

# Get the correct project directory
PROJECT_DIR=$(find_project_root)

if [[ -z "$PROJECT_DIR" ]]; then
  echo "Error: Could not find the project directory within the directory tree. Please ensure you are in the directory apf/."
  exit 1
fi

BUILD_DIR="$PROJECT_DIR/build"

# Remove the build directory if it exists
if [[ -d "$BUILD_DIR" ]]; then
  rm -rf "$BUILD_DIR"
  echo "Removed existing build directory."
fi

# Create a new build directory
mkdir -p "$BUILD_DIR"

# Navigate to the build directory
cd "$BUILD_DIR" || { echo "Failed to change directory to $BUILD_DIR"; exit 1; }

# Run CMake and build the project
cmake ..
cmake --build .