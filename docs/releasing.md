# Source-only releases

Publish one source release per version for Harmonic, Jetty and Classic 11.
Users install the dependencies and compile the selected backend on their own
computer, following the [README](../README.md).

GitHub automatically creates **Source code (zip)** and **Source code (tar.gz)**
from each tag. Leave the upload field empty: no plugin binaries, separate distro
archives, custom source ZIPs or checksum files are needed.
See [GitHub's release documentation](https://docs.github.com/en/repositories/releasing-projects-on-github/about-releases).

## Clean up existing releases

Open the [releases page](https://github.com/Xovium-Tech/gz-dynamic-terrain-plugin/releases).
Edit each release and remove manually uploaded attachments and references to
prebuilt packages. Keep the automatic source archives and existing version tags.

At the September 16, 2026 audit, the only published release was `v0.1.0`.
Its sole uploaded attachment, `gz-dynamic-terrain-plugin-v0.1.0.zip`, is a duplicate
source archive, not a compiled binary. Remove that attachment using the release
editor and save the release. GitHub's automatic source downloads remain available.
The `v0.2.0` tag exists but has no published release.

If you want to remove an obsolete release entry entirely, use **Delete this
release** on that entry. Keep its Git tag so existing source references still
resolve. These are separate operations; no tag deletion or history rewrite is
needed for source-only distribution.

## Push the source-only update

The next version prepared in this checkout is `v0.2.1`; `v0.2.0` already exists.
Use the repository's configured SSH remote and account:

```bash
git remote -v
git config --local --get user.name
git config --local --get user.email
git fetch origin --tags
git tag --list v0.2.1
```

The final command must print nothing. Then commit and push:

```bash
git status --short
git diff --check
git add -A
git diff --cached --stat
git diff --cached --check
git commit -m "Simplify source-only releases and local builds"
git pull --rebase origin main
git push origin main
```

Resolve any rebase conflicts before pushing. If branch protection requires a
pull request, push a feature branch and merge through the normal review process.

## Wait for CI and tag

Wait for **Build and smoke test** to pass on the
[Actions page](https://github.com/Xovium-Tech/gz-dynamic-terrain-plugin/actions).
It checks the core, Harmonic GUI OFF/ON, Jetty GUI OFF/ON and Classic 11.

```bash
git switch main
git pull --ff-only origin main
git log -1 --format='%H %s'
```

Confirm that commit passed CI, then create the new tag:

```bash
git tag -a v0.2.1 -m "v0.2.1: source-only releases and local builds"
git push origin v0.2.1
```

## Publish

1. Open [Create a new release](https://github.com/Xovium-Tech/gz-dynamic-terrain-plugin/releases/new).
2. Select the existing `v0.2.1` tag.
3. Set the title to **v0.2.1 — Source-only release**.
4. Paste [release-notes.md](release-notes.md) into the description.
5. Leave the binary/file upload field empty.
6. Publish and mark this as the latest release.

The two automatic source downloads contain the build files, source, examples
and documentation. CI validates the source; it does not upload binaries.

For subsequent releases, update the project version, HTTP User-Agent and both
startup/configuration log versions together, then use a new tag.
