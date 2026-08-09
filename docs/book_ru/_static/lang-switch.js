/* ENG/RU switch in the top navbar.
 *
 * Why JavaScript and not a theme template: sphinx-book-theme sits on pydata-sphinx-theme, and the name of
 * the navbar container has changed between their versions more than once. A script that tries a few
 * selectors and falls back to a fixed-position button keeps working across an upgrade; a template that
 * targets `navbar_end` stops rendering silently.
 *
 * URL mapping. English is the site root, Russian lives under `ru/`:
 *
 *     <base>/Control/Overview.html   <->   <base>/ru/Control/Overview.html
 *
 * When the button exists at all. This is the part that was learned the hard way: `jupyter-book build` leaves
 * its own output in `docs/book/_build/html/`, that tree is openable, it carries this script — and it has no
 * `ru/` inside it, because the two languages are only brought together when `build_book.sh` assembles
 * `docs/_site/`. So in the intermediate tree the button rendered happily and every click produced the
 * browser's "Your file couldn't be accessed". A button whose destination cannot exist must not be drawn, so
 * this script now requires an explicit marker that the surrounding tree is the assembled bilingual site:
 * `data-adas-bilingual` on the <html> element, stamped by `build_book.sh` only after both languages are in
 * place. No marker, no button — which is the honest rendering of a monolingual tree.
 *
 * The base comes from Sphinx's own `data-content_root` attribute — a relative path from the current page to
 * the book root — with `DOCUMENTATION_OPTIONS.pagename` as a fallback. Not `URL_ROOT`: that was removed in
 * recent Sphinx and reading it silently yields undefined, which would make every nested page think it is the
 * root. Either of the two supported hints means the button works from a local `_build/html` and from a
 * GitHub Pages subdirectory alike, with no hard-coded prefix.
 */
(function () {
  "use strict";

  /** Relative path from this page to the book root, e.g. "../" or "./". */
  function contentRootRel() {
    var el = document.querySelector("[data-content_root]");
    if (el) {
      var v = el.getAttribute("data-content_root");
      return v ? v : "./";
    }
    // Fallback: pagename is the source path without extension, so its depth gives the same answer.
    var opts = window.DOCUMENTATION_OPTIONS;
    if (opts && typeof opts.pagename === "string") {
      var depth = opts.pagename.split("/").length - 1;
      return depth > 0 ? new Array(depth + 1).join("../") : "./";
    }
    return "./";
  }

  function bookRoot() {
    return new URL(contentRootRel(), window.location.href);
  }

  /** Where the same page lives in the other language, or null if we cannot tell. */
  function counterpart() {
    var root = bookRoot();
    var here = window.location.href;
    var pagePath = here.slice(root.href.length); // e.g. "Control/Overview.html#section"
    var isRu = /(^|\/)ru\/$/.test(root.pathname);

    if (isRu) {
      var enRoot = new URL("../", root);
      return { href: new URL(pagePath, enRoot).href, label: "ENG", other: "EN" };
    }
    var ruRoot = new URL("ru/", root);
    return { href: new URL(pagePath, ruRoot).href, label: "RU", other: "RU" };
  }

  function build(target) {
    var a = document.createElement("a");
    a.className = "lang-switch";
    a.href = target.href;
    a.textContent = target.label;
    a.title = target.other === "RU" ? "Читать по-русски" : "Read in English";
    a.setAttribute("aria-label", a.title);
    // A page missing in the other language would 404. Rather than pretend, note it on the link so a
    // reader who lands on a stub knows why.
    a.rel = "alternate";
    return a;
  }

  function place(el) {
    var containers = [
      ".navbar-header-items__end",
      ".navbar-header-items .navbar-item:last-child",
      ".bd-header .navbar-nav",
      ".bd-header",
    ];
    for (var i = 0; i < containers.length; i++) {
      var host = document.querySelector(containers[i]);
      if (host) {
        var wrap = document.createElement("div");
        wrap.className = "navbar-item lang-switch-item";
        wrap.appendChild(el);
        host.appendChild(wrap);
        return true;
      }
    }
    el.classList.add("lang-switch--floating");
    document.body.appendChild(el);
    return false;
  }

  /** Is this the assembled two-language site, or a single-language build tree? */
  function bilingualSite() {
    var root = document.documentElement;
    return !!(root && root.hasAttribute("data-adas-bilingual"));
  }

  document.addEventListener("DOMContentLoaded", function () {
    if (!bilingualSite()) {
      return;   // a monolingual tree has nowhere to switch to; see the header comment
    }
    var target = counterpart();
    if (target) {
      place(build(target));
    }
  });
})();
