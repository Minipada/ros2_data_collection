// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Distro version switcher: the sidebar selector on every book page (doc/theme/index.hbs
// ships the empty container). This list is the site's single source of distro order —
// no humble: the legacy line has its own docs. tools/ci/deploy_doc_site.sh repeats the
// ids for its gh-pages cleanup keep-list, and "/" redirects straight into rolling.
const DC_DISTROS = [
    { id: "rolling", name: "Rolling" },
    { id: "lyrical", name: "Lyrical" },
    { id: "jazzy", name: "Jazzy" },
];

// The distro section the current page lives in, or null outside one (a local build).
function dcCurrentDistro() {
    return window.location.pathname.split("/").find((segment) =>
        DC_DISTROS.some((distro) => distro.id === segment)) || null;
}

// Same page in another distro's section — every section mirrors one book layout, so the
// path carries over. Outside a published section there is no path to carry; fall back to
// that distro's root on the published site.
function dcDistroUrl(target) {
    const segments = window.location.pathname.split("/");
    const current = segments.indexOf(dcCurrentDistro());
    if (current === -1) {
        return "https://minipada.github.io/ros2_data_collection/" + target + "/";
    }
    segments[current] = target;
    return segments.join("/") + window.location.search + window.location.hash;
}

// Sidebar selector, one link per distro; the current one is highlighted.
const selector = document.getElementById("distro-selector");
if (selector) {
    const current = dcCurrentDistro();
    for (const distro of DC_DISTROS) {
        const link = document.createElement("a");
        link.textContent = distro.name;
        link.href = dcDistroUrl(distro.id);
        if (distro.id === current) {
            link.className = "current";
            link.setAttribute("aria-current", "page");
        } else {
            link.title = "Read this page in the " + distro.name + " docs";
        }
        selector.appendChild(link);
    }
}
