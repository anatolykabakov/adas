#!/usr/bin/env bash
# Fetch the assets the APK packages, from the places entitled to distribute them.
#
# Why this exists rather than committing the files: a model in the repository is a copy handed to everyone
# who clones, and to everyone who downloads a release. Fetching from upstream instead means upstream
# distributes and we do not — which is the difference between needing a licence notice for a
# redistribution and not redistributing at all. THIRD_PARTY.md has the details per asset.
#
# It also keeps ~155 MB of binaries out of the history.
#
#   ./scripts/fetch_models.sh              # fetch what is missing, verify what is present
#   ./scripts/fetch_models.sh --check      # verify only, download nothing
#   ./scripts/fetch_models.sh --record     # print manifest lines with the hashes of what is on disk
#   ./scripts/fetch_models.sh --force      # re-download even if the hash matches
#
# Exit codes: 0 everything present and verified, 1 something is missing or does not match.

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
MANIFEST="$SCRIPT_DIR/models.manifest"

CHECK_ONLY=false
RECORD=false
FORCE=false

for arg in "$@"; do
    case "$arg" in
        --check)  CHECK_ONLY=true ;;
        --record) RECORD=true ;;
        --force)  FORCE=true ;;
        -h|--help)
            sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *)
            echo "unknown option: $arg (see --help)" >&2
            exit 2
            ;;
    esac
done

if [ ! -f "$MANIFEST" ]; then
    echo "manifest not found: $MANIFEST" >&2
    exit 2
fi

sha_of() { sha256sum "$1" 2>/dev/null | cut -d' ' -f1; }

problems=0
present=0
fetched=0

while read -r dest want kind source; do
    # skip comments and blank lines
    case "${dest:-}" in ''|'#'*) continue ;; esac
    [ -n "${source:-}" ] || { echo "malformed manifest line for $dest" >&2; problems=$((problems + 1)); continue; }

    path="$REPO_ROOT/$dest"

    if $RECORD; then
        if [ -f "$path" ]; then
            printf '%s  %s  %s  %s\n' "$dest" "$(sha_of "$path")" "$kind" "$source"
        else
            echo "# $dest — not on disk, nothing to record"
        fi
        continue
    fi

    have=""
    [ -f "$path" ] && have="$(sha_of "$path")"

    # Already good?
    if [ -n "$have" ] && ! $FORCE; then
        if [ "$want" = "-" ]; then
            echo "present  $dest  (no hash recorded; --record to pin it)"
            present=$((present + 1))
            continue
        fi
        if [ "$have" = "$want" ]; then
            echo "ok       $dest"
            present=$((present + 1))
            continue
        fi
        echo "MISMATCH $dest" >&2
        echo "         expected $want" >&2
        echo "         on disk  $have" >&2
        echo "         a wrong model does not crash, it reports plausible numbers about the wrong road." >&2
        echo "         Delete the file and re-run, or fix the manifest if the new file is the intended one." >&2
        problems=$((problems + 1))
        continue
    fi

    if $CHECK_ONLY; then
        echo "MISSING  $dest" >&2
        problems=$((problems + 1))
        continue
    fi

    case "$kind" in
        url)
            mkdir -p "$(dirname "$path")"
            echo "fetch    $dest"
            echo "         from $source"
            # -C - resumes a partial file, -f fails on HTTP errors instead of saving the error page.
            if ! curl -fL -C - -o "$path" "$source"; then
                echo "         download failed" >&2
                problems=$((problems + 1))
                continue
            fi
            got="$(sha_of "$path")"
            if [ "$want" != "-" ] && [ "$got" != "$want" ]; then
                echo "         hash mismatch after download: got $got, expected $want" >&2
                problems=$((problems + 1))
                continue
            fi
            [ "$want" = "-" ] && echo "         sha256 $got  (record it in the manifest)"
            fetched=$((fetched + 1))
            ;;
        derive)
            echo "MISSING  $dest — produced locally, not downloaded" >&2
            echo "         run: $source" >&2
            problems=$((problems + 1))
            ;;
        unknown)
            echo "MISSING  $dest — provenance not established" >&2
            echo "         The file we have been driving on hashes to:" >&2
            echo "           $want" >&2
            echo "         but no URL, release or commit is recorded for it anywhere in the repository," >&2
            echo "         so it cannot be fetched and its licence notice cannot be written honestly." >&2
            echo "         Fill in the source in scripts/models.manifest, or copy the file in by hand" >&2
            echo "         and note where it came from. See THIRD_PARTY.md." >&2
            problems=$((problems + 1))
            ;;
        *)
            echo "unknown kind '$kind' for $dest" >&2
            problems=$((problems + 1))
            ;;
    esac
done < "$MANIFEST"

$RECORD && exit 0

echo
echo "verified $present, fetched $fetched, unresolved $problems"
if [ "$problems" -gt 0 ]; then
    echo "gradle will keep whatever is already in app/src/main/assets/ for anything missing here —" >&2
    echo "which is how a stale or pointer file ends up in an APK without a word. Resolve the list above." >&2
    exit 1
fi
