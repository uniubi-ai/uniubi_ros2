#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
mkdir -p "$root/.deps"
while read -r name url commit extra || [ -n "${name:-}" ]; do
  [ -z "${name:-}" ] && continue
  case "$name" in \#*) continue ;; uniubi_robot_sdk|uniubi_robot_msgs) ;; *) echo "Unknown dependency: $name" >&2; exit 1 ;; esac
  [ "$url" = "https://github.com/uniubi-ai/$name.git" ] || { echo "Unexpected URL for $name" >&2; exit 1; }
  case "$commit" in *[!0123456789abcdef]*|"") echo "Invalid commit for $name" >&2; exit 1 ;; esac
  [ "${#commit}" -eq 40 ] && [ -z "${extra:-}" ] || { echo "Invalid lock row for $name" >&2; exit 1; }
  dest="$root/.deps/$name"
  if [ ! -e "$dest" ]; then git clone --quiet "$url" "$dest"; fi
  [ -d "$dest/.git" ] || { echo "Not a Git checkout: $dest" >&2; exit 1; }
  actual=$(git -C "$dest" remote get-url origin)
  [ "$actual" = "$url" ] || { echo "Wrong origin for $dest: $actual" >&2; exit 1; }
  [ -z "$(git -C "$dest" status --porcelain)" ] || { echo "Dirty dependency: $dest" >&2; exit 1; }
  if ! git -C "$dest" cat-file -e "$commit^{commit}" 2>/dev/null; then git -C "$dest" fetch --quiet origin "$commit"; fi
  git -C "$dest" checkout --quiet --detach "$commit"
  [ "$(git -C "$dest" rev-parse HEAD)" = "$commit" ] || exit 1
  echo "$name $commit"
done < "$root/dependencies.lock"
