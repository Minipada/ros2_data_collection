function get_theme_mermaid() {
    let mermaid_theme = "light";

    let mdbook_theme = localStorage.getItem('mdbook-theme')

    if (mdbook_theme == "ayu" || mdbook_theme == "navy" || mdbook_theme == "coal") {
        mermaid_theme = "dark";
    }
    return mermaid_theme
}

mermaid.initialize({ startOnLoad: true, theme: get_theme_mermaid() });

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
