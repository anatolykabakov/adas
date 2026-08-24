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
#   ./scripts/fetch_models.sh --with-optional  # also the assets marked -opt in the manifest
#   ./scripts/fetch_models.sh --best-effort    # a failed derive is SKIPPED, not fatal — for CI runners
#                                              # without onnx/tinygrad/GPU; downloads stay verified
#
# Exit codes: 0 everything present and verified, 1 something is missing or does not match.

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
MANIFEST="$SCRIPT_DIR/models.manifest"

CHECK_ONLY=false
RECORD=false
FORCE=false
WITH_OPTIONAL=false
BEST_EFFORT=false

for arg in "$@"; do
    case "$arg" in
        --check)  CHECK_ONLY=true ;;
        --record) RECORD=true ;;
        --force)  FORCE=true ;;
        --with-optional) WITH_OPTIONAL=true ;;
        --best-effort) BEST_EFFORT=true ;;
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
derived=0
skipped=0

while read -r dest want kind source; do
    # skip comments and blank lines
    case "${dest:-}" in ''|'#'*) continue ;; esac
    [ -n "${source:-}" ] || { echo "malformed manifest line for $dest" >&2; problems=$((problems + 1)); continue; }

    path="$REPO_ROOT/$dest"

    # An asset marked -opt is not part of a default build: the traffic detector is derived from
    # AGPL-licensed weights and putting it in the APK is distribution, so a plain checkout must not
    # end up with it in models/ where gradle would package it (THIRD_PARTY.md).
    case "$kind" in
        *-opt)
            if [ "$WITH_OPTIONAL" = false ]; then
                echo "skip     $dest  (optional; --with-optional to build it)"
                skipped=$((skipped + 1))
                continue
            fi
            ;;
    esac

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
        url|url-opt)
            mkdir -p "$(dirname "$path")"
            echo "fetch    $dest"
            echo "         from $source"
            # -C - resumes a partial file, -f fails on HTTP errors instead of saving the error page.
            if ! curl -fL -C - -o "$path" "$source"; then
                rm -f "$path"
                if [ "$kind" = "url-opt" ]; then
                    echo "SKIPPED  $dest — download failed and this asset is optional" >&2
                    skipped=$((skipped + 1))
                else
                    echo "         download failed" >&2
                    problems=$((problems + 1))
                fi
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
        derive|derive-opt|derive-url|url-derive)
            # derive-url and url-derive carry two sources split by ' ||| ': the command, then a URL
            # holding the same artifact. The derive is bit-for-bit reproducible, so the download
            # verifies against the same pinned sha256 — same provenance either way. url-derive tries
            # the download FIRST (any machine, no GPU or extra tooling), and derives only when the
            # shelf is unreachable; derive-url is the same pair in the opposite order.
            fallback_url=""
            case "$kind" in derive-url|url-derive)
                fallback_url="${source#* ||| }"
                source="${source%% ||| *}"
            ;; esac
            if [ "$kind" = "url-derive" ] && [ -n "$fallback_url" ]; then
                mkdir -p "$(dirname "$path")"
                echo "fetch    $dest"
                echo "         from $fallback_url  (derive is the fallback)"
                if curl -fL -C - -o "$path" "$fallback_url"; then
                    got="$(sha_of "$path")"
                    if [ "$want" != "-" ] && [ "$got" = "$want" ]; then
                        fetched=$((fetched + 1))
                        continue
                    fi
                    echo "         hash mismatch ($got) — falling back to deriving" >&2
                    rm -f "$path"
                else
                    echo "         download failed — falling back to deriving" >&2
                    rm -f "$path"
                fi
                fallback_url=""   # the URL had its chance; below is a plain derive
            fi
            echo "derive   $dest"
            echo "         $source"
            # The exit status is not enough: a tool that writes its output under a name of its own
            # choosing exits 0 and leaves the declared destination missing. Without this the line
            # would report success and gradle would package whatever was in assets/ from before.
            if ( cd "$REPO_ROOT" && bash -c "$source" ) && [ -f "$path" ]; then
                got="$(sha_of "$path")"
                if [ "$want" != "-" ] && [ "$got" != "$want" ]; then
                    echo "MISMATCH $dest derived to $got, manifest says $want" >&2
                    problems=$((problems + 1))
                else
                    derived=$((derived + 1))
                fi
            elif [ -n "$fallback_url" ]; then
                echo "         derive failed — fetching the same artifact instead" >&2
                echo "         from $fallback_url"
                if curl -fL -C - -o "$path" "$fallback_url"; then
                    got="$(sha_of "$path")"
                    if [ "$want" != "-" ] && [ "$got" != "$want" ]; then
                        echo "MISMATCH $dest downloaded as $got, manifest says $want" >&2
                        rm -f "$path"
                        problems=$((problems + 1))
                    else
                        fetched=$((fetched + 1))
                    fi
                elif [ "$BEST_EFFORT" = true ]; then
                    rm -f "$path"
                    echo "SKIPPED  $dest — derive and download both failed" >&2
                    skipped=$((skipped + 1))
                else
                    rm -f "$path"
                    echo "FAILED   $dest — derive and download both failed" >&2
                    problems=$((problems + 1))
                fi
            elif [ "$kind" = "derive-opt" ] || [ "$BEST_EFFORT" = true ]; then
                # derive-opt is optional by design; --best-effort extends the same mercy to every
                # derive, for runners without the tooling (CI has no tinygrad and no OpenCL GPU).
                # gradle's sync tasks tolerate a missing model, so the APK just ships without it.
                echo "SKIPPED  $dest — derive failed and this run is allowed to go on without it" >&2
                skipped=$((skipped + 1))
            else
                echo "FAILED   $dest — the command above did not produce it" >&2
                problems=$((problems + 1))
            fi
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
echo "verified $present, fetched $fetched, derived $derived, skipped $skipped, unresolved $problems"
if [ "$problems" -gt 0 ]; then
    echo "gradle will keep whatever is already in app/src/main/assets/ for anything missing here —" >&2
    echo "which is how a stale or pointer file ends up in an APK without a word. Resolve the list above." >&2
    exit 1
fi
