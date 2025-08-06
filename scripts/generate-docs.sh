#!/bin/bash
set -e

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
WORK_DIR="$REPO_ROOT/work"
DOCS_OUTPUT_DIR="$REPO_ROOT"

# ROS 2 distributions to build
DISTRIBUTIONS=("humble" "jazzy" "rolling")

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log() {
    echo -e "${GREEN}[$(date +'%Y-%m-%d %H:%M:%S')] $1${NC}"
}

warn() {
    echo -e "${YELLOW}[$(date +'%Y-%m-%d %H:%M:%S')] WARNING: $1${NC}"
}

error() {
    echo -e "${RED}[$(date +'%Y-%m-%d %H:%M:%S')] ERROR: $1${NC}"
}

# Function to get the latest tag for a distribution
get_latest_tag() {
    local distribution=$1
    local repo_dir="$2"
    
    cd "$repo_dir"
    
    if [ "$distribution" = "rolling" ]; then
        # For rolling, use the latest commit
        echo "rolling-$(git rev-parse --short HEAD)"
    else
        # For stable distributions, get the latest tag matching the distribution
        local latest_tag=$(git tag -l "*$distribution*" --sort=-version:refname | head -n1)
        if [ -z "$latest_tag" ]; then
            # Fallback to distribution name if no specific tag found
            echo "$distribution"
        else
            echo "$latest_tag"
        fi
    fi
}

# Function to clone or update rclcpp repository
setup_source() {
    local distribution=$1
    local source_dir="$WORK_DIR/rclcpp-$distribution"
    
    log "Setting up source for $distribution..."
    
    if [ -d "$source_dir" ]; then
        log "Updating existing repository for $distribution"
        cd "$source_dir"
        git fetch --all --tags
        git clean -fd
    else
        log "Cloning rclcpp repository for $distribution"
        mkdir -p "$WORK_DIR"
        git clone https://github.com/ros2/rclcpp.git "$source_dir"
        cd "$source_dir"
    fi
    
    # Switch to the appropriate branch/tag
    if [ "$distribution" = "rolling" ]; then
        git checkout rolling
        git pull origin rolling
    else
        # Try to checkout the distribution branch
        if git rev-parse --verify "origin/$distribution" >/dev/null 2>&1; then
            git checkout "$distribution"
            git pull "origin/$distribution"
        else
            warn "No $distribution branch found, using main/master"
            git checkout main || git checkout master
            git pull
        fi
    fi
    
    echo "$source_dir"
}

# Function to generate doxygen documentation
generate_doxygen() {
    local distribution=$1
    local source_dir="$2"
    local output_dir="$DOCS_OUTPUT_DIR/$distribution"
    
    log "Generating documentation for $distribution..."
    
    # Create output directory
    mkdir -p "$output_dir"
    
    # Get version information
    local version=$(get_latest_tag "$distribution" "$source_dir")
    local commit_hash=$(cd "$source_dir" && git rev-parse HEAD)
    local build_date=$(date -u +"%Y-%m-%d")
    
    # Create Doxyfile from template
    local doxyfile="$WORK_DIR/Doxyfile.$distribution"
    sed "s|{{DISTRIBUTION}}|$distribution|g; \
         s|{{VERSION}}|$version|g; \
         s|{{OUTPUT_DIR}}|$output_dir|g; \
         s|{{SOURCE_DIR}}|$source_dir|g" \
         "$REPO_ROOT/doxygen/Doxyfile.template" > "$doxyfile"
    
    # Run doxygen
    cd "$WORK_DIR"
    doxygen "$doxyfile"
    
    # Update distribution metadata
    local dist_file="$REPO_ROOT/_distributions/$distribution.md"
    if [ -f "$dist_file" ]; then
        # Update the last_updated field in the front matter
        sed -i "s/^last_updated:.*/last_updated: $(date -u +"%Y-%m-%d")/" "$dist_file"
        
        # Add build metadata if not present
        if ! grep -q "build_date:" "$dist_file"; then
            sed -i "/^last_updated:/a build_date: $build_date\ncommit_hash: $commit_hash" "$dist_file"
        else
            sed -i "s/^build_date:.*/build_date: $build_date/" "$dist_file"
            sed -i "s/^commit_hash:.*/commit_hash: $commit_hash/" "$dist_file"
        fi
    fi
    
    log "Documentation generated for $distribution (version: $version)"
}

# Main execution
main() {
    log "Starting ROS 2 rclcpp documentation generation"
    
    # Check if doxygen is installed
    if ! command -v doxygen &> /dev/null; then
        error "Doxygen is not installed. Please install it first."
        exit 1
    fi
    
    # Create work directory
    mkdir -p "$WORK_DIR"
    
    # Process each distribution
    for distribution in "${DISTRIBUTIONS[@]}"; do
        log "Processing $distribution distribution..."
        
        # Setup source code
        source_dir=$(setup_source "$distribution")
        
        # Generate documentation
        generate_doxygen "$distribution" "$source_dir"
        
        log "Completed $distribution distribution"
        echo
    done
    
    # Cleanup work directory
    log "Cleaning up work directory..."
    rm -rf "$WORK_DIR"
    
    log "Documentation generation completed for all distributions!"
}

# Run main function
main "$@"