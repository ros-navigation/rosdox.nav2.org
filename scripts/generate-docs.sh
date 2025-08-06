#!/bin/bash
set -e

# Configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"
WORK_DIR="$REPO_ROOT/work"
DOCS_OUTPUT_DIR="$REPO_ROOT"

# ROS 2 distributions to build
DISTRIBUTIONS=("humble" "jazzy" "rolling")

# Colors for output (only in CI environment)
if [ -n "$CI" ]; then
    RED=''
    GREEN=''
    YELLOW=''
    NC=''
else
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    YELLOW='\033[1;33m'
    NC='\033[0m'
fi

log() {
    echo -e "${GREEN}[$(date +'%Y-%m-%d %H:%M:%S')] $1${NC}" >&2
}

warn() {
    echo -e "${YELLOW}[$(date +'%Y-%m-%d %H:%M:%S')] WARNING: $1${NC}" >&2
}

error() {
    echo -e "${RED}[$(date +'%Y-%m-%d %H:%M:%S')] ERROR: $1${NC}" >&2
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
        git fetch --all --tags 2>/dev/null || true
        git clean -fd 2>/dev/null || true
    else
        log "Cloning rclcpp repository for $distribution"
        mkdir -p "$WORK_DIR"
        git clone https://github.com/ros2/rclcpp.git "$source_dir" 2>/dev/null
        cd "$source_dir"
    fi
    
    # Switch to the appropriate branch/tag
    if [ "$distribution" = "rolling" ]; then
        if git show-ref --verify --quiet refs/remotes/origin/rolling; then
            git checkout rolling 2>/dev/null || git checkout -b rolling origin/rolling 2>/dev/null
            git pull origin rolling 2>/dev/null || true
        else
            log "No rolling branch found, using main/master"
            git checkout main 2>/dev/null || git checkout master 2>/dev/null
            git pull 2>/dev/null || true
        fi
    else
        # Try to checkout the distribution branch
        if git show-ref --verify --quiet refs/remotes/origin/$distribution; then
            git checkout $distribution 2>/dev/null || git checkout -b $distribution origin/$distribution 2>/dev/null
            git pull origin $distribution 2>/dev/null || true
        else
            warn "No $distribution branch found, checking for release tags"
            # Look for release tags
            local release_tag=$(git tag -l "*$distribution*" --sort=-version:refname | head -n1)
            if [ -n "$release_tag" ]; then
                log "Using release tag: $release_tag"
                git checkout "$release_tag" 2>/dev/null
            else
                warn "No $distribution branch or tags found, using main/master"
                git checkout main 2>/dev/null || git checkout master 2>/dev/null
                git pull 2>/dev/null || true
            fi
        fi
    fi
    
    # Output the source directory path (this is the return value)
    echo "$source_dir"
}

# Function to create fallback documentation when doxygen fails
create_fallback_docs() {
    local distribution=$1
    local output_dir="$2"
    
    mkdir -p "$output_dir/html"
    cat > "$output_dir/html/index.html" <<EOF
<!DOCTYPE html>
<html><head><title>$distribution Documentation</title></head>
<body><h1>Documentation generation failed for $distribution</h1>
<p>The documentation build encountered an error. Please check the GitHub Actions logs.</p>
<p>This may be due to:</p>
<ul>
<li>Missing dependencies</li>
<li>Build configuration issues</li>
<li>Source code changes that broke the build</li>
</ul>
</body></html>
EOF
}

# Function to generate doxygen documentation
generate_doxygen() {
    local distribution=$1
    local source_dir="$2"
    local output_dir="$DOCS_OUTPUT_DIR/$distribution"
    
    log "Generating documentation for $distribution..."
    
    # Verify source directory exists and contains rclcpp
    if [ ! -d "$source_dir/rclcpp" ]; then
        error "rclcpp directory not found in $source_dir"
        return 1
    fi
    
    # Create output directory
    mkdir -p "$output_dir"
    
    # Get version information
    local version=$(get_latest_tag "$distribution" "$source_dir")
    local commit_hash=$(cd "$source_dir" && git rev-parse HEAD)
    local build_date=$(date -u +"%Y-%m-%d")
    
    # Use rclcpp's built-in Doxyfile and modify it for our needs
    local original_doxyfile="$source_dir/rclcpp/Doxyfile"
    local doxyfile="$WORK_DIR/Doxyfile.$distribution"
    
    if [ -f "$original_doxyfile" ]; then
        log "Using rclcpp's built-in Doxyfile for $distribution"
        # Copy and modify the original Doxyfile
        cp "$original_doxyfile" "$doxyfile"
        
        # Update PROJECT_NUMBER to include distribution
        sed -i "s/PROJECT_NUMBER.*/PROJECT_NUMBER = $distribution ($version)/" "$doxyfile"
        
        # Update OUTPUT_DIRECTORY to our target
        sed -i "s|OUTPUT_DIRECTORY.*|OUTPUT_DIRECTORY = $output_dir|" "$doxyfile"
        
        # Ensure INPUT points to the include directory
        sed -i "s|INPUT.*|INPUT = ./include|" "$doxyfile"
        
        # Change to rclcpp directory and run doxygen
        cd "$source_dir/rclcpp"
        if ! doxygen "$doxyfile"; then
            warn "Doxygen failed for $distribution using built-in Doxyfile, trying fallback..."
            
            # Fallback: try with our template
            sed "s|{{DISTRIBUTION}}|$distribution|g; \
                 s|{{VERSION}}|$version|g; \
                 s|{{OUTPUT_DIR}}|$output_dir|g; \
                 s|{{SOURCE_DIR}}|$source_dir|g" \
                 "$REPO_ROOT/doxygen/Doxyfile.template" > "$doxyfile"
            
            cd "$WORK_DIR"
            if ! doxygen "$doxyfile" 2>/dev/null; then
                warn "Both Doxyfiles failed for $distribution, creating placeholder..."
                create_fallback_docs "$distribution" "$output_dir"
            fi
        fi
    else
        warn "No built-in Doxyfile found, using template for $distribution"
        # Fallback to our template
        sed "s|{{DISTRIBUTION}}|$distribution|g; \
             s|{{VERSION}}|$version|g; \
             s|{{OUTPUT_DIR}}|$output_dir|g; \
             s|{{SOURCE_DIR}}|$source_dir|g" \
             "$REPO_ROOT/doxygen/Doxyfile.template" > "$doxyfile"
        
        cd "$WORK_DIR"
        if ! doxygen "$doxyfile" 2>/dev/null; then
            warn "Template Doxyfile failed for $distribution, creating placeholder..."
            create_fallback_docs "$distribution" "$output_dir"
        fi
    fi
    
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
        
        # Setup source code and capture the directory path
        source_dir=$(setup_source "$distribution")
        
        if [ -z "$source_dir" ] || [ ! -d "$source_dir" ]; then
            error "Failed to setup source for $distribution"
            continue
        fi
        
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