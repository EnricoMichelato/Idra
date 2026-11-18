# Publishing

Repository: `https://github.com/enricomichelato/Idra` · site: `https://enricomichelato.github.io/Idra/`

## First push

1. Create an empty repository named **Idra** on GitHub (no README, no licence, no .gitignore).
2. Commit and push:
   ```bash
   git commit -m "IDRA: design model, simulator and build files"
   git remote add origin https://github.com/enricomichelato/Idra.git
   git push -u origin main
   ```
3. **GitHub Pages.** In Settings → Pages choose "Deploy from a branch", then `main` and the `/docs` folder. After a minute the site is live.

## Licence

`LICENSE` is "all rights reserved": the most restrictive choice. Anyone can read a public repository and fork it on GitHub (the platform terms allow that), but nobody may copy, build or reuse the design.

To open the project later, typical choices are CERN-OHL-S or CERN-OHL-P for mechanics and electronics, MIT for software, CC BY-SA 4.0 for documents. Opening later is easy; closing again something already published under an open licence is not.

## What the public pages say

The README and the site state that the wrist roll is endless, without explaining how. The explanation is still in the repository: `electronics/`, `mechanical/cad/BOM.md`, `mechanical/cad/arm_v0/base_notes.md`, the hand notes and the CAD scripts. Remove or keep those out of the public repository if the method must stay private.

## Renaming the repository later

Settings → General → Repository name. GitHub redirects the old repository address, including `git` remotes, but **not** the Pages site. After renaming:
- update the links in the README and `OWNER` / `REPO` in `tools/build_site.py`;
- rebuild `docs/`;
- share the new site address.

## Updating the site

Rebuild the pages (see README), then run `python3 tools/build_site.py` and commit `docs/`.

The 3D viewer is about 9 MB and every version stays in the repository history, so update it only when the model really changes.
