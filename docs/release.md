# Releasing zapp

Pushing a `v*` tag runs `.github/workflows/release.yml`. It builds and signs the
binaries, creates the GitHub release, and publishes the same archives to
CrabNebula Cloud, where `zapp upgrade` looks for updates.

## Before tagging

- Set `version` in `zapp/Cargo.toml` to the tag without its `v`. The `preflight`
  job fails otherwise: `zapp upgrade` compares the server's version with the
  version built into the binary.
- Add `release-notes/<tag>.md` (for example `release-notes/v1.1.11.md`). The
  `preflight` job fails if it is missing or empty. The file is the body of the
  GitHub release and the notes `zapp upgrade` shows.

## Secrets

Set these under Settings → Secrets and variables → Actions, next to the existing
Apple and Azure signing secrets.

The three downloads secrets (`DOWNLOADS_CLOUDFLARE_API_TOKEN`,
`CLOUDFLARE_ACCOUNT_ID`, `DOWNLOADS_DISPATCH_TOKEN`) live in the `release`
environment (Settings → Environments → `release`), not at repo level; the
`downloads` job reads them from there.

| Secret | What it is |
| --- | --- |
| `CN_API_KEY` | CrabNebula Cloud API key with read/write, from the org's API keys tab |
| `ZAPP_UPDATER_PRIVATE_KEY` | Contents of `~/.tauri/zapp.key`, the updater signing key |
| `DOWNLOADS_CLOUDFLARE_API_TOKEN` | Cloudflare API token with R2 write access to the `zsa-downloads` bucket (Object Read & Write, scoped to that bucket) |
| `CLOUDFLARE_ACCOUNT_ID` | The Cloudflare account ID that owns `zsa-downloads` |
| `DOWNLOADS_DISPATCH_TOKEN` | Fine-grained GitHub token limited to `zsa/downloads`, with Contents read and write |

```sh
gh secret set ZAPP_UPDATER_PRIVATE_KEY < ~/.tauri/zapp.key
```

The key has no password. Its public half (`~/.tauri/zapp.key.pub`) is
`PUBLIC_KEY` in `zapp/src/upgrade.rs`. Back the private key up: installed
copies only accept archives signed with it.

## CrabNebula Cloud

`CN_APPLICATION` in the workflow is the org/app slug:
`zsa-technology-labs-inc/io-zsa-zapp`.

The release job signs each archive with the Tauri signer (`<archive>.sig`),
drafts a Cloud release with the notes file, uploads the archives, creates the
GitHub release, then publishes the Cloud release.

| Archive | Update platform | Public platform |
| --- | --- | --- |
| `zapp-linux-x86_64.tar.gz` | `linux-x86_64` | `cli-linux-x86_64` |
| `zapp-linux-aarch64.tar.gz` | `linux-aarch64` | `cli-linux-aarch64` |
| `zapp-windows-x86_64.zip` | `windows-x86_64` | `cli-windows-x86_64` |
| `zapp-windows-aarch64.zip` | `windows-aarch64` | `cli-windows-aarch64` |
| `zapp-macos-universal.tar.gz` | `darwin-x86_64` | `cli-darwin-x86_64` |
| `zapp-macos-universal.tar.gz` | `darwin-aarch64` | `cli-darwin-aarch64` |

The per-arch macOS archives stay on GitHub only.

The update endpoint is
`https://cdn.crabnebula.app/update/zsa-technology-labs-inc/io-zsa-zapp/<platform>/<version>`.
It answers 204 when there is nothing to offer and caches replies for up to an
hour. It also answers 200 when the installed version is newer than the
published one, so `zapp upgrade` only installs a strictly newer version.

## downloads.zsa.io

After the Cloud release is published, the `downloads` job copies the public
files to https://downloads.zsa.io. It:

1. takes the universal macOS archive, both Windows zips and both Linux archives
   (not the per-arch macOS archives or the `.sig` files),
2. uploads each to the `zsa-downloads` R2 bucket as
   `files/zapp/<version>/<file name>`,
3. tells the `zsa/downloads` repo about the release (version, date, the notes
   file, and each file's size and SHA-256), which updates the site.

It is a separate job on purpose. If it fails, the run shows red but the Cloud
release, the GitHub release and the updater are already live and unaffected. Fix
the cause (usually an expired or missing secret), then open the run and choose
**Re-run failed jobs**: only `downloads` runs again. Uploads overwrite, so a
re-run is safe.

## How `zapp upgrade` installs

It checks the signature against the archive before unpacking anything, then
replaces its own executable. It refuses before downloading when it is installed
by Nix, Homebrew, mise or a Linux package (under `/usr`, except `/usr/local`),
or when its folder is not writable.

## Testing an upgrade end to end

1. Publish a release, say `v1.1.11`.
2. Build a zapp that reports an older version: set `version` in
   `zapp/Cargo.toml` below the published one (do not commit it), run
   `cargo build --release -p zapp`, and copy `target/release/zapp` to a
   writable folder outside the repo. Releases before `v1.1.11` have no
   `zapp upgrade`.
3. Run `./zapp upgrade --check` from that copy. It should list the new version
   and its notes.
4. Run `./zapp upgrade`, answer `y`, then `./zapp --version`.
5. Optionally check the endpoint by hand:
   `curl -i https://cdn.crabnebula.app/update/zsa-technology-labs-inc/io-zsa-zapp/linux-x86_64/1.1.10`.
