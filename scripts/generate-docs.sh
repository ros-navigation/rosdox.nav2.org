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

# Function to clone or update repositories (rclcpp and rcl)
setup_source() {
    local distribution=$1
    local rclcpp_source_dir="$WORK_DIR/rclcpp-$distribution"
    local rcl_source_dir="$WORK_DIR/rcl-$distribution"
    
    log "Setting up source repositories for $distribution..."
    
    # Setup rclcpp repository
    if [ -d "$rclcpp_source_dir" ]; then
        log "Updating existing rclcpp repository for $distribution"
        cd "$rclcpp_source_dir" || { error "Failed to cd to $rclcpp_source_dir"; return 1; }
        git fetch --all --tags || { warn "Failed to fetch rclcpp updates"; }
        git clean -fd || { warn "Failed to clean rclcpp directory"; }
    else
        log "Cloning rclcpp repository for $distribution"
        mkdir -p "$WORK_DIR" || { error "Failed to create work directory"; return 1; }
        
        if ! git clone https://github.com/ros2/rclcpp.git "$rclcpp_source_dir"; then
            error "Failed to clone rclcpp repository"
            return 1
        fi
    fi
    
    # Setup rcl repository
    if [ -d "$rcl_source_dir" ]; then
        log "Updating existing rcl repository for $distribution"
        cd "$rcl_source_dir" || { error "Failed to cd to $rcl_source_dir"; return 1; }
        git fetch --all --tags || { warn "Failed to fetch rcl updates"; }
        git clean -fd || { warn "Failed to clean rcl directory"; }
    else
        log "Cloning rcl repository for $distribution"
        
        if ! git clone https://github.com/ros2/rcl.git "$rcl_source_dir"; then
            error "Failed to clone rcl repository"
            return 1
        fi
        
        cd "$rcl_source_dir" || { error "Failed to cd to cloned rcl directory"; return 1; }
    fi
    
    # Setup branches for both repositories
    setup_repository_branch "$distribution" "$rclcpp_source_dir" "rclcpp"
    setup_repository_branch "$distribution" "$rcl_source_dir" "rcl"
    
    # Output both source directories (this is the return value)
    echo "$rclcpp_source_dir:$rcl_source_dir"
}

# Helper function to setup repository branch
setup_repository_branch() {
    local distribution=$1
    local repo_dir="$2"
    local repo_name="$3"
    
    cd "$repo_dir" || { error "Failed to cd to $repo_dir"; return 1; }
    
    # Verify we have a git repository
    if ! git rev-parse --git-dir >/dev/null 2>&1; then
        error "Not a git repository: $repo_dir"
        return 1
    fi
    
    # Switch to the appropriate branch/tag (redirect git output to avoid contaminating function output)
    if [ "$distribution" = "rolling" ]; then
        log "Setting up rolling distribution for $repo_name"
        # For rolling, just use the default branch (main/master)
        if git show-ref --verify --quiet refs/remotes/origin/rolling; then
            log "Checking out rolling branch for $repo_name"
            git checkout rolling >/dev/null 2>&1 || git checkout -b rolling origin/rolling >/dev/null 2>&1 || {
                warn "Failed to checkout rolling branch for $repo_name, using default"
                git checkout main >/dev/null 2>&1 || git checkout master >/dev/null 2>&1 || {
                    error "Failed to checkout any branch for $repo_name"
                    return 1
                }
            }
        else
            log "No rolling branch found for $repo_name, using default branch"
            git checkout main >/dev/null 2>&1 || git checkout master >/dev/null 2>&1 || {
                error "Failed to checkout default branch for $repo_name"
                return 1
            }
        fi
    else
        log "Setting up $distribution distribution for $repo_name"
        # For stable distributions, try various approaches
        if git show-ref --verify --quiet "refs/remotes/origin/$distribution"; then
            log "Found $distribution branch for $repo_name, checking out"
            git checkout "$distribution" >/dev/null 2>&1 || git checkout -b "$distribution" "origin/$distribution" >/dev/null 2>&1 || {
                warn "Failed to checkout $distribution branch for $repo_name"
                setup_fallback_branch "$distribution" "$repo_name"
            }
        else
            log "No $distribution branch found for $repo_name, trying alternative approaches"
            setup_fallback_branch "$distribution" "$repo_name"
        fi
    fi
    
    # Verify we successfully checked out something
    if ! git rev-parse HEAD >/dev/null 2>&1; then
        error "Failed to checkout any valid branch/tag for $distribution in $repo_name"
        return 1
    fi
    
    local current_branch=$(git branch --show-current 2>/dev/null || echo "detached")
    log "Successfully set up $distribution for $repo_name using: $current_branch"
}

# Helper function to set up fallback branch when main distribution branch doesn't exist
setup_fallback_branch() {
    local distribution=$1
    local repo_name=$2
    
    # Try to find release tags matching the distribution
    local release_tag=$(git tag -l "*$distribution*" --sort=-version:refname | head -n1)
    if [ -n "$release_tag" ]; then
        log "Using release tag: $release_tag"
        if git checkout "$release_tag" >/dev/null 2>&1; then
            return 0
        fi
    fi
    
    # Try common tag patterns
    for pattern in "${distribution}" "${distribution}-*" "*${distribution}*" "release/${distribution}*"; do
        release_tag=$(git tag -l "$pattern" --sort=-version:refname | head -n1)
        if [ -n "$release_tag" ]; then
            log "Trying tag pattern '$pattern': found $release_tag"
            if git checkout "$release_tag" >/dev/null 2>&1; then
                return 0
            fi
        fi
    done
    
    # Last resort: use main/master branch
    warn "No $distribution-specific branch or tags found, using main/master"
    git checkout main >/dev/null 2>&1 || git checkout master >/dev/null 2>&1 || {
        error "Failed to checkout fallback branch"
        return 1
    }
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
    local source_dirs="$2"  # Now contains both directories separated by ":"
    local output_dir="$DOCS_OUTPUT_DIR/$distribution"
    
    # Parse the source directories
    local rclcpp_source_dir=$(echo "$source_dirs" | cut -d':' -f1)
    local rcl_source_dir=$(echo "$source_dirs" | cut -d':' -f2)
    
    log "Generating documentation for $distribution..."
    log "rclcpp source: $rclcpp_source_dir"
    log "rcl source: $rcl_source_dir"
    
    # Verify source directories exist and contain expected content
    if [ ! -d "$rclcpp_source_dir/rclcpp" ]; then
        error "rclcpp directory not found in $rclcpp_source_dir"
        return 1
    fi
    
    if [ ! -d "$rcl_source_dir/rcl" ]; then
        error "rcl directory not found in $rcl_source_dir"
        return 1
    fi
    
    # Create output directory
    mkdir -p "$output_dir"
    
    # Get version information
    local version=$(get_latest_tag "$distribution" "$rclcpp_source_dir")
    local commit_hash=$(cd "$rclcpp_source_dir" && git rev-parse HEAD)
    local build_date=$(date -u +"%Y-%m-%d")
    
    # Use our template that includes both rclcpp and rcl
    local doxyfile="$WORK_DIR/Doxyfile.$distribution"
    
    log "Using template Doxyfile for $distribution with rclcpp and rcl support"
    
    # Generate Doxyfile from template with both source directories
    sed "s|{{DISTRIBUTION}}|$distribution|g; \
         s|{{VERSION}}|$version|g; \
         s|{{OUTPUT_DIR}}|$output_dir|g; \
         s|{{RCLCPP_SOURCE_DIR}}|$rclcpp_source_dir|g; \
         s|{{RCL_SOURCE_DIR}}|$rcl_source_dir|g" \
         "$REPO_ROOT/doxygen/Doxyfile.template" > "$doxyfile"
    
    cd "$WORK_DIR"
    if ! doxygen "$doxyfile" 2>/dev/null; then
        warn "Doxygen failed for $distribution, creating placeholder..."
        create_fallback_docs "$distribution" "$output_dir"
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
        
        # Setup source code and capture the directory paths
        if source_dirs=$(setup_source "$distribution"); then
            # Parse the source directories
            local rclcpp_source_dir=$(echo "$source_dirs" | cut -d':' -f1)
            local rcl_source_dir=$(echo "$source_dirs" | cut -d':' -f2)
            
            if [ -n "$rclcpp_source_dir" ] && [ -d "$rclcpp_source_dir" ] && [ -n "$rcl_source_dir" ] && [ -d "$rcl_source_dir" ]; then
                log "Source setup successful for $distribution"
                log "  rclcpp: $rclcpp_source_dir"
                log "  rcl: $rcl_source_dir"
                
                # Generate documentation
                generate_doxygen "$distribution" "$source_dirs"
                
                log "Completed $distribution distribution"
            else
                error "Setup returned invalid source directories for $distribution"
                error "  rclcpp: '$rclcpp_source_dir'"
                error "  rcl: '$rcl_source_dir'"
                continue
            fi
        else
            error "Failed to setup source for $distribution"
            continue
        fi
        
        echo
    done
    
    # Cleanup work directory
    log "Cleaning up work directory..."
    rm -rf "$WORK_DIR"
    
    log "Documentation generation completed for all distributions!"
}

# Run main function
main "$@"