#!/bin/bash
# Build both language versions into one publishable tree.
#
#   docs/_site/            English, the site root
#   docs/_site/ru/         Russian
#
# The ENG/RU button in the navbar maps between them by inserting or removing that `ru/` segment, computed
# from each page's own `data-content_root` — so this layout is the contract, not a detail. Change it and the
# button breaks. See docs/book/_static/lang-switch.js.
#
# The button only appears where it can work. `jupyter-book build` leaves its own output in
# `docs/book/_build/html/`; that tree is openable, carries the switch script, and has no `ru/` in it — so the
# button used to render there and every click gave the browser's "Your file couldn't be accessed". This
# script therefore stamps `data-adas-bilingual` onto every page of `_site` *after* both languages are copied
# in, and the script draws nothing without that marker. The intermediate trees are now honestly monolingual.
#
#   ./docs/build_book.sh              # both languages, then serve instructions
#   ./docs/build_book.sh --en         # English only, faster while writing
#   ./docs/build_book.sh --check      # translation sync + runnable snippets, no build

set -euo pipefail

DOCS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SITE="$DOCS/_site"

only_en=false
check_only=false
for arg in "$@"; do
  case "$arg" in
    --en) only_en=true ;;
    --check) check_only=true ;;
    *) echo "unknown option: $arg" >&2; exit 1 ;;
  esac
done

echo "=== проверки"
# The button's whole safety property is that it refuses to render without the marker this script stamps.
# Drop the gate from the script and the intermediate `_build/html` trees silently get a button that leads
# nowhere again — which is what happened once and is not obvious from either file alone.
if ! grep -q "data-adas-bilingual" "$DOCS/book/_static/lang-switch.js"; then
  echo "lang-switch.js больше не проверяет метку data-adas-bilingual:" >&2
  echo "  без неё кнопка появится и в docs/book/_build/html, где второго языка нет," >&2
  echo "  и каждое нажатие даст «Your file couldn't be accessed». См. комментарий в самом скрипте." >&2
  exit 1
fi
# Snippets first: a chapter whose examples do not run is worse than a chapter without examples.
python3 "$DOCS/book/check_snippets.py"
# Then structure: the Russian tree must have the same pages with the same code in them.
python3 "$DOCS/book/sync_translation.py"
# Then references: every repository file the book names must still exist — the project deletes
# aggressively, and a mention of a deleted doc teaches the reader to distrust the whole book.
python3 "$DOCS/book/check_repo_links.py"

if $check_only; then
  exit 0
fi

# The shared assets live in the English tree and are copied, not duplicated by hand — the switch script
# and the figures have no language.
echo
echo "=== общие файлы в русское дерево"
for f in _static/lang-switch.js _static/lang-switch.css logo.svg _toc.yml; do
  mkdir -p "$(dirname "$DOCS/book_ru/$f")"
  cp -f "$DOCS/book/$f" "$DOCS/book_ru/$f"
done
while IFS= read -r fig; do
  rel="${fig#"$DOCS/book/"}"
  mkdir -p "$DOCS/book_ru/$(dirname "$rel")"
  cp -rf "$fig" "$DOCS/book_ru/$(dirname "$rel")/"
done < <(find "$DOCS/book" -type d -name figures -not -path '*/_build/*')

echo
echo "=== сборка EN"
jupyter-book build "$DOCS/book" >/dev/null
rm -rf "$SITE"
mkdir -p "$SITE"
cp -r "$DOCS/book/_build/html/." "$SITE/"

if ! $only_en; then
  echo "=== сборка RU"
  jupyter-book build "$DOCS/book_ru" >/dev/null
  mkdir -p "$SITE/ru"
  cp -r "$DOCS/book_ru/_build/html/." "$SITE/ru/"
fi

if ! $only_en; then
  echo
  echo "=== метка двуязычного сайта"
  # Only now, with both languages in place, may the button exist. Stamped here and not in the templates
  # precisely so that a single-language build tree cannot carry it.
  python3 - "$SITE" <<'PYSTAMP'
import re
import sys
from pathlib import Path

site = Path(sys.argv[1])
stamped = skipped = 0
for page in sorted(site.rglob("*.html")):
    html = page.read_text()
    if "data-adas-bilingual" in html:
        stamped += 1
        continue
    # Redirect stubs and the theme's macro fragments have no <html> element and need no marker.
    new, n = re.subn(r"<html(?=[\s>])", "<html data-adas-bilingual", html, count=1)
    if n == 0:
        skipped += 1
        continue
    page.write_text(new)
    stamped += 1
print(f"  помечено страниц: {stamped}, без <html> (заглушки-редиректы): {skipped}")

# The marker is what makes the button appear, so a page with a navbar and no marker is a silent
# regression — exactly the failure this whole mechanism exists to prevent.
missing = [
    p.relative_to(site)
    for p in site.rglob("*.html")
    if "data-content_root" in p.read_text() and "data-adas-bilingual" not in p.read_text()
]
if missing:
    print("  страницы с навбаром, но без метки:")
    for m in missing[:10]:
        print(f"    {m}")
    sys.exit(1)
PYSTAMP

  echo
  echo "=== проверка: кнопка ENG/RU никуда не ведёт в 404"
  # Mirrors the mapping in lang-switch.js against the actually built pages, using each page's own
  # data-content_root — the same hint the script reads. A browser is not needed to catch the only failure
  # that matters: a page present in one language and missing in the other.
  python3 - "$SITE" <<'PYCHECK'
import re
import sys
from pathlib import Path

site = Path(sys.argv[1]).resolve()
ru = (site / "ru").resolve()
bad = checked = 0
for page in sorted(site.rglob("*.html")):
    html = page.read_text()
    m = re.search(r'data-content_root="([^"]*)"', html)
    if not m:
        continue                      # redirect stubs carry no navbar and no button
    root = (page.parent / (m.group(1) or "./")).resolve()
    page_path = page.resolve().relative_to(root)
    target_root = site if root == ru else ru
    checked += 1
    if not (target_root / page_path).exists():
        print(f"  {page.relative_to(site)} -> {(target_root / page_path).relative_to(site)} отсутствует")
        bad += 1
print(f"  проверено переходов: {checked}, битых: {bad}")
sys.exit(1 if bad else 0)
PYCHECK
fi

echo
echo "готово: $SITE"
echo "  EN  file://$SITE/index.html"
$only_en || echo "  RU  file://$SITE/ru/index.html"
echo
echo "локально удобнее через сервер, иначе относительные ссылки кнопки языка ведут себя иначе:"
echo "  python3 -m http.server -d $SITE 8000"
echo
echo "открывать нужно $SITE, а не docs/book/_build/html — там одна книга без второго языка,"
echo "и кнопка ENG/RU там сознательно не рисуется (раньше она рисовалась и вела в никуда)."
