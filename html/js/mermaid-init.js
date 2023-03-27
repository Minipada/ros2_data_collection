function get_theme_mermaid() {
    let mermaid_theme = "light";

    let mdbook_theme = localStorage.getItem('mdbook-theme')

    if (mdbook_theme == "ayu" || mdbook_theme == "navy" || mdbook_theme == "coal" || mdbook_theme == undefined) {
        // If undefined it is still dark, since default is navy mode, defined in book.toml
        mermaid_theme = "dark";
    }
    return mermaid_theme
}

mermaid.initialize({ startOnLoad: true, theme: get_theme_mermaid() });
