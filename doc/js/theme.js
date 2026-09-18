// SPDX-FileCopyrightText: 2022-2026 David Bensoussan
// SPDX-License-Identifier: MPL-2.0

// To reload mermaid graphs
document.getElementById('light').onclick = function () {
    location.reload();
};

document.getElementById('rust').onclick = function () {
    location.reload();
};

document.getElementById('coal').onclick = function () {
    location.reload();
};

document.getElementById('navy').onclick = function () {
    location.reload();
};

document.getElementById('ayu').onclick = function () {
    location.reload();
};

// The logo image lives at the section root (images/), so use the per-page relative
// prefix mdbook already emits (path_to_root, set in index.hbs) rather than counting URL
// segments — that arithmetic was off by one on section-root pages and, since #534, the
// site root no longer carries an images/ copy to fall back into.
document.getElementById("logo-dc").src = path_to_root + "images/dc.png"

if (window.location.href.indexOf('http://localhost/') == 0) {
    document.getElementById("a-logo-dc").href = "/"
} else if (window.location.href.indexOf('https://minipada.github.io/ros2_data_collection/') == 0) {
    document.getElementById("a-logo-dc").href = "https://minipada.github.io/ros2_data_collection"
}
