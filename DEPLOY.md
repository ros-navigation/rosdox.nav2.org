# Quick Deployment Guide

## Step-by-Step Deployment to GitHub Pages

### 1. Create GitHub Repository
1. Go to GitHub.com and create a new repository
2. Name it `rosdox.nav2.org` (or any name you prefer)
3. Make it public
4. Don't initialize with README (we already have files)

### 2. Push Code to GitHub
```bash
cd /home/steve/Documents/rosdox_ws/rosdox.nav2.org
git add .
git commit -m "Initial commit: ROS 2 rclcpp documentation site"
git branch -M main
git remote add origin https://github.com/[your-username]/rosdox.nav2.org.git
git push -u origin main
```

### 3. Enable GitHub Pages
1. Go to repository Settings
2. Click "Pages" in the left sidebar
3. Under "Source" select "GitHub Actions"
4. Save the settings

### 4. Enable GitHub Actions
1. Go to repository Settings → Actions → General
2. Under "Workflow permissions" select "Read and write permissions"
3. Check "Allow GitHub Actions to create and approve pull requests"
4. Save

### 5. Run the Workflow
1. Go to "Actions" tab
2. Click "Build and Deploy ROS 2 rclcpp Documentation"
3. Click "Run workflow" → "Run workflow"
4. Wait for completion (15-30 minutes for first run)

### 6. Access Your Site
Your site will be available at:
```
https://[your-username].github.io/rosdox.nav2.org/
```

## Expected Results

After the workflow completes successfully:
- ✅ Home page with distribution cards
- ✅ Humble documentation (LTS)
- ✅ Jazzy documentation (Active)
- ✅ Rolling documentation (Development)
- ✅ Automatic weekly updates every Sunday

## Troubleshooting

### Common Issues:
1. **Permissions Error**: Enable "Read and write permissions" in Actions settings
2. **Workflow Fails**: Check Actions tab for error logs
3. **Site Not Loading**: Wait 5-10 minutes after deployment completes
4. **Documentation Empty**: First run takes 15-30 minutes to generate docs

### Getting Help:
- Check GitHub Actions logs for detailed error messages
- Ensure all required permissions are enabled
- Verify repository is public (required for free GitHub Pages)