# Setup Instructions for ROS 2 rclcpp Documentation

This document outlines the steps needed to set up and deploy the ROS 2 rclcpp documentation website on GitHub Pages.

## GitHub Pages Configuration

### 1. Repository Setup

1. Create a new repository on GitHub named `rosdox.nav2.org` (or any name you prefer)
2. Push this code to the repository
3. The site will be available at: `https://[your-username].github.io/rosdox.nav2.org/`

### 2. Enable GitHub Pages

1. Go to your repository settings on GitHub
2. Navigate to "Pages" in the left sidebar
3. Under "Source", select "GitHub Actions"
4. The site will be automatically deployed when the workflow runs

### 3. No DNS Configuration Needed

The site will automatically be hosted at:
```
https://[your-username].github.io/rosdox.nav2.org/
```

No custom domain setup or DNS configuration required!

## GitHub Actions Setup

### 1. Repository Settings

Ensure the following permissions are set in your repository:

1. Go to "Settings" → "Actions" → "General"
2. Under "Workflow permissions", select "Read and write permissions"
3. Check "Allow GitHub Actions to create and approve pull requests"

### 2. Pages Deployment

The workflow is configured to:
- Run weekly on Sundays at 2:00 AM UTC
- Allow manual triggering via "workflow_dispatch"
- Trigger on pushes to main branch (for configuration changes)

### 3. Environment Variables

No additional environment variables are needed. The workflow uses:
- `GITHUB_TOKEN`: Automatically provided by GitHub
- Repository permissions for Pages deployment

## Manual Testing

To test the setup locally:

### 1. Install Dependencies

```bash
# Install Ruby and Bundler
gem install bundler

# Install Jekyll dependencies
bundle install

# Install system dependencies (Ubuntu/Debian)
sudo apt-get install doxygen graphviz
```

### 2. Run Documentation Generation

```bash
# Generate documentation for all distributions
./scripts/generate-docs.sh

# Or for testing, modify the script to build just one distribution
```

### 3. Test Jekyll Build

```bash
# Build and serve the site locally
bundle exec jekyll serve

# The site will be available at http://localhost:4000
```

## Troubleshooting

### Common Issues

1. **Doxygen not found**: Install doxygen on the system
2. **Ruby version issues**: Use Ruby 3.1+ as specified in the workflow
3. **Permission errors**: Check GitHub Actions permissions
4. **DNS propagation**: DNS changes can take up to 24-48 hours to propagate

### Debug Steps

1. Check GitHub Actions logs in the "Actions" tab
2. Verify the workflow runs successfully
3. Check that documentation files are generated in the correct directories
4. Ensure Jekyll builds without errors

## Monitoring and Maintenance

### Weekly Updates

The documentation automatically updates every Sunday. Monitor:
- GitHub Actions workflow success
- Generated documentation quality
- Website accessibility

### Manual Updates

To trigger documentation updates manually:
1. Go to "Actions" tab in GitHub
2. Select "Build and Deploy ROS 2 rclcpp Documentation"
3. Click "Run workflow"

## Integration with nav2.org

### Linking from nav2.org

You can link to the documentation from nav2.org using the GitHub Pages URL:

```html
<a href="https://[your-username].github.io/rosdox.nav2.org/">ROS 2 rclcpp API Documentation</a>
```

### Navigation Integration

Consider adding navigation links in the Jekyll layout to connect back to nav2.org:

```yaml
# In _config.yml
navigation:
  - title: "Navigation2"
    url: "https://navigation.ros.org/"
  - title: "API Docs"
    url: "/"
```

## Security Considerations

- The workflow uses minimal permissions
- No secrets are stored or transmitted
- Documentation is generated from public ROS 2 repositories
- All dependencies are pinned to specific versions