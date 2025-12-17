## Problem, Hypothesis, Resolution (PHR)

**Problem:** The user requested to host their Docusaurus book on GitHub Pages, specifically at `https://Areeba-binteali.github.io/Physical-AI-Textbook/`. The existing Docusaurus project was not configured for GitHub Pages deployment, and direct `npm run deploy` attempts failed due to incorrect Docusaurus configuration parameters and Git remote issues in a subdirectory context.

**Hypothesis:** By correctly configuring `docusaurus.config.js` with the GitHub Pages specific `url`, `baseUrl`, `organizationName`, `projectName`, and `deploymentBranch`, and by implementing a robust GitHub Actions workflow for automated build and deployment from the `website` subdirectory, the Docusaurus site will be successfully deployed to GitHub Pages upon pushing changes to the designated branch.

**Resolution:**

1.  **Docusaurus Configuration (`docusaurus.config.js`):**
    *   Updated `url` to `https://Areeba-binteali.github.io`.
    *   Updated `baseUrl` to `/Physical-AI-Textbook/`.
    *   Updated `organizationName` to `Areeba-binteali`.
    *   Confirmed `projectName` as `Physical-AI-Textbook`.
    *   Added `deploymentBranch: 'gh-pages'` to specify the deployment branch for GitHub Pages.
    *   Removed previously (incorrectly) added `repo` and `deploymentRepository` properties.

2.  **GitHub Actions Workflow (`.github/workflows/deploy.yml`):**
    *   Created a new GitHub Actions workflow file to automate the build and deployment process.
    *   Configured the workflow to trigger on `push` events to the `004-module-4-vla` branch.
    *   Set the `working-directory` to `./website` for all Docusaurus-related commands (install, build).
    *   Configured `actions/upload-pages-artifact` with `path: ./build` (relative to the `working-directory`).

3.  **Git Operations:**
    *   Added the remote `origin` to the root Git repository, pointing to `https://github.com/Areeba-binteali/Physical-AI-Textbook.git`.
    *   Staged and committed all relevant changes (updated `docusaurus.config.js`, new landing page files, new GitHub Actions workflow file, and previously fixed markdown files).
    *   Pushed the `004-module-4-vla` branch to the remote repository, which triggered the newly configured GitHub Actions workflow.

Upon successful completion of the GitHub Actions workflow, the Docusaurus book will be accessible at `https://Areeba-binteali.github.io/Physical-AI-Textbook/`.
