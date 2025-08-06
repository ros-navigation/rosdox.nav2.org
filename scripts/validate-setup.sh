#!/bin/bash

# Validation script for rosdox.nav2.org setup

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

log() {
    echo -e "${GREEN}✓ $1${NC}"
}

warn() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

error() {
    echo -e "${RED}✗ $1${NC}"
}

echo "=== Validating rosdox.nav2.org Setup ==="
echo

# Check directory structure
echo "Checking directory structure..."

required_files=(
    "_config.yml"
    "Gemfile"
    "index.md"
    "CNAME"
    "_layouts/distribution.html"
    "_distributions/humble.md"
    "_distributions/jazzy.md"
    "_distributions/rolling.md"
    "doxygen/Doxyfile.template"
    "scripts/generate-docs.sh"
    ".github/workflows/build-docs.yml"
    ".github/workflows/test-build.yml"
    "assets/css/main.scss"
)

for file in "${required_files[@]}"; do
    if [ -f "$REPO_ROOT/$file" ]; then
        log "Found: $file"
    else
        error "Missing: $file"
        exit 1
    fi
done

echo

# Check Jekyll configuration
echo "Validating Jekyll configuration..."

if grep -q "title:" "$REPO_ROOT/_config.yml"; then
    log "Jekyll config has title"
else
    error "Jekyll config missing title"
fi

if grep -q "rosdox.nav2.org" "$REPO_ROOT/_config.yml"; then
    log "Jekyll config has correct URL"
else
    warn "Jekyll config might need URL update"
fi

echo

# Check distribution files
echo "Validating distribution files..."

distributions=("humble" "jazzy" "rolling")

for dist in "${distributions[@]}"; do
    file="$REPO_ROOT/_distributions/$dist.md"
    if [ -f "$file" ]; then
        if grep -q "title:" "$file" && grep -q "description:" "$file"; then
            log "Distribution $dist has proper front matter"
        else
            error "Distribution $dist missing front matter"
        fi
    else
        error "Distribution $dist file not found"
    fi
done

echo

# Check GitHub Actions workflow
echo "Validating GitHub Actions workflow..."

if grep -q "cron:" "$REPO_ROOT/.github/workflows/build-docs.yml"; then
    log "Build workflow has scheduled trigger"
else
    error "Build workflow missing cron schedule"
fi

if grep -q "doxygen" "$REPO_ROOT/.github/workflows/build-docs.yml"; then
    log "Build workflow installs doxygen"
else
    error "Build workflow missing doxygen installation"
fi

echo

# Check script permissions
echo "Checking script permissions..."

if [ -x "$REPO_ROOT/scripts/generate-docs.sh" ]; then
    log "Documentation generation script is executable"
else
    error "Documentation generation script is not executable"
    echo "Run: chmod +x scripts/generate-docs.sh"
fi

echo

# Check CNAME configuration
echo "Validating CNAME configuration..."

if [ -f "$REPO_ROOT/CNAME" ]; then
    cname_content=$(cat "$REPO_ROOT/CNAME")
    if [ "$cname_content" = "rosdox.nav2.org" ]; then
        log "CNAME correctly configured for rosdox.nav2.org"
    else
        error "CNAME contains: $cname_content (should be rosdox.nav2.org)"
    fi
else
    error "CNAME file not found"
fi

echo

# Summary
echo "=== Validation Complete ==="
echo
log "Basic setup validation passed!"
echo
echo "Next steps:"
echo "1. Push to GitHub repository"
echo "2. Enable GitHub Pages in repository settings"
echo "3. Configure DNS for rosdox.nav2.org"
echo "4. Run the GitHub Action workflow"
echo
warn "Note: This validation doesn't test actual documentation generation"
warn "Run the GitHub Action workflow to test the complete pipeline"