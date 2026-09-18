// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// Distro version switcher, shared by every distro section and the landing page: the
// sidebar selector on book pages (doc/theme/index.hbs ships the empty container) and the
// landing page's distro cards (doc/landing/index.html ships that one). This list is the
// site's single source of distro order — no humble: the legacy line has its own docs.
// tools/ci/deploy_doc_site.sh repeats the ids for its gh-pages cleanup keep-list.
const DC_DISTROS = [
    { id: "rolling", name: "Rolling", blurb: "Development tip, tracks ROS 2 rolling head" },
    { id: "lyrical", name: "Lyrical", blurb: "ROS 2 Lyrical (Ubuntu resolute)" },
    { id: "jazzy", name: "Jazzy", blurb: "ROS 2 Jazzy (Ubuntu noble)" },
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

// Landing page cards. The landing is served from the site root, so its links stay
// section-relative.
const cards = document.querySelector("[data-distro-cards]");
if (cards) {
    for (const distro of DC_DISTROS) {
        const card = document.createElement("a");
        card.className = "distro-card";
        card.href = distro.id + "/";
        const name = document.createElement("h2");
        name.textContent = distro.name;
        const blurb = document.createElement("p");
        blurb.textContent = distro.blurb;
        card.appendChild(name);
        card.appendChild(blurb);
        cards.appendChild(card);
    }
}
