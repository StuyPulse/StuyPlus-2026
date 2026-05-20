// hella tuff javascript code to make javadocs look way nicer

const placeFavicon = () => {
    const link = document.createElement("link");
    link.setAttribute("rel", "icon");
    link.setAttribute("href", "/StuyPlus-2026/favicon.ico?t=" + Date.now()); // This forces the browser 
                                                                            // to reload the favicon everytime instead of getting the cached version
    link.setAttribute("type", "image/x-icon");
    document.head.appendChild(link);
}

const syntaxHighlight = () => {
    const isSourcePage = document.querySelector("body.source-page") !== null;
    if (!isSourcePage) return;

    // import highlight.js + css
    const link = document.createElement("link");
    link.setAttribute("rel", "stylesheet");
    link.setAttribute("href", "https://cdnjs.cloudflare.com/ajax/libs/highlight.js/11.11.1/styles/github-dark.min.css");
    document.head.appendChild(link);
    
    // get all lines of code and highlight them
    const script = document.createElement("script");
    script.src = "https://cdnjs.cloudflare.com/ajax/libs/highlight.js/11.11.1/highlight.min.js";
    script.onload = () => {
        const pre = document.querySelector('.source-container > pre');
        const lines = pre.querySelectorAll(':scope > [id*="line-"]');
        const source = Array.from(lines).map(line => line.textContent).join("\n");
        const lineNumbers = Array.from(lines).map((_, i) => `<span>${i + 1}</span>`).join("\n");

        pre.outerHTML = 
        `
        <div class="code-wrapper">
            <div class="line-numbers">
                ${lineNumbers}
            </div>

            <pre>
                <code class="language-java" hlfs>${source}</code>
            </pre>
        <div>
        `
        // pre.innerHTML = `<code class="language-java" hljs>${source}</code>`;
        hljs.highlightAll();
    };

    script.onerror = () => console.error("highlight.js failed to load 🤤");
    document.head.appendChild(script);
}

placeFavicon();
syntaxHighlight();
