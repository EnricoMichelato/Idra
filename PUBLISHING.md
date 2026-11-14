# Before the first push

1. **Name.** If the project gets a new name, also change:
   - the README title;
   - `REPO` in `tools/build_site.py`;
   - the `.../idra/` links in the README.
2. **Owner.** Replace `OWNER` with your GitHub user in two places:
   - in the README (all site links);
   - in `tools/build_site.py`.

   Then run `python3 tools/build_site.py` again.
3. **Licence.** Without a `LICENSE` file the project stays "all rights reserved". Typical choices for an open hardware project:
   - **CERN-OHL-S** or **CERN-OHL-P** for mechanics and electronics;
   - **MIT** for software;
   - **CC BY-SA 4.0** for documents.
4. **Push.**
   ```bash
   git remote add origin https://github.com/OWNER/idra.git
   git push -u origin main
   ```
5. **GitHub Pages.** In Settings → Pages choose "Deploy from a branch", then `main` and the `/docs` folder. After a minute the site is live at `https://OWNER.github.io/idra/`.

## Renaming the repository later

Settings → General → Repository name. GitHub redirects the old repository address, including `git` remotes, but **not** the Pages site. After renaming:
- update the links in the README and `REPO` in `tools/build_site.py`;
- rebuild `docs/`;
- share the new site address.

## Updating the site

Rebuild the pages (see README), then run `python3 tools/build_site.py` and commit `docs/`.

The 3D viewer is about 8 MB and every version stays in the repository history, so update it only when the model really changes.
