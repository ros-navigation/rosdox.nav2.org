# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a Jekyll-based static site that generates developer-friendly ROS 2 rclcpp API documentation using Doxygen. It automatically clones the rclcpp repository, generates documentation for multiple ROS 2 distributions (Humble, Jazzy, Rolling), and hosts them on GitHub Pages.

## Development Commands

### Local Development
```bash
# Install Jekyll dependencies
bundle install

# Serve the site locally (without documentation generation)
bundle exec jekyll serve

# Generate documentation for all distributions (requires doxygen)
./scripts/generate-docs.sh

# Build Jekyll site
bundle exec jekyll build
```

### Prerequisites
```bash
# Install system dependencies (Ubuntu/Debian)
sudo apt-get install doxygen graphviz

# Install Ruby gems
gem install bundler
bundle install
```

### Validation
```bash
# Validate setup and configuration
./scripts/validate-setup.sh
```

## Architecture

### Core Components
- **Jekyll Site**: Static site generator with custom layouts and collections
- **Documentation Generation**: Automated Doxygen processing via `scripts/generate-docs.sh`
- **GitHub Actions**: Weekly automated builds and deployment to GitHub Pages
- **Distribution System**: Jekyll collections to organize docs by ROS 2 distribution

### File Structure
```
├── _config.yml              # Jekyll configuration
├── _layouts/                 # Jekyll templates (default.html, home.html)
├── _distributions/           # ROS 2 distribution metadata (humble.md, jazzy.md, rolling.md)
├── scripts/
│   ├── generate-docs.sh     # Main documentation generation script
│   └── validate-setup.sh    # Setup validation script
├── doxygen/
│   └── Doxyfile.template    # Doxygen configuration template
├── .github/workflows/       # GitHub Actions workflows
└── assets/                  # Static assets (CSS, images)
```

### Documentation Generation Process
1. `generate-docs.sh` clones rclcpp repository for each distribution
2. Creates distribution-specific Doxygen configuration from template
3. Generates HTML documentation in distribution directories (`/humble/`, `/jazzy/`, `/rolling/`)
4. Updates distribution metadata with build information
5. Jekyll builds the site including the generated documentation

### Distribution Management
Each ROS 2 distribution has:
- Metadata file in `_distributions/` with status, description, and build info
- Generated documentation directory at root level
- Automatic weekly updates via GitHub Actions

## GitHub Actions Workflow

The site uses automated builds that:
- Run weekly on Sundays at 2:00 AM UTC
- Can be triggered manually via workflow_dispatch
- Install dependencies (Ruby, Doxygen, Graphviz)
- Generate documentation for all distributions
- Build and deploy Jekyll site to GitHub Pages

## Important Notes

- Documentation generation requires significant time (15-30 minutes for first run)
- The site is designed to handle build failures gracefully with fallback pages
- Distribution branches/tags are resolved automatically with fallback logic
- All generated documentation is excluded from git via `.gitignore`