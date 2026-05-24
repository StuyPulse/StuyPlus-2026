// hella tuff javascript code to make javadocs look way nicer

let hljsCSSLink = null;
let hljsScript = null;

const placeFavicon = () => {
    const link = document.createElement("link");
    link.setAttribute("rel", "icon");
    link.setAttribute("href", "/StuyPlus-2026/favicon.ico?t=" + Date.now()); // This forces the browser 
                                                                            // to reload the favicon everytime instead of getting the cached version
    link.setAttribute("type", "image/x-icon");
    document.head.appendChild(link);
}

const goToLineNumberByHash = () => {
    const isSourcePage = document.querySelector("body.source-page") !== null;
    if (!isSourcePage) return;

    const lineHashRegex = /^#line-\d+$/;
    if (!hljsScript) {
        console.warn("highlight.js not loaded yet .line-numbers container not created yet 🤤.");
        return;
    }

    const hash = window.location.hash;
    if (!lineHashRegex.test(hash)) {
        return;
    }

    // The hash exactly matches the id already so we can just scroll to it cuz querySelector is so tuff
    const targetElement =  document.querySelector(hash);
    if (targetElement) {
        const prefersReducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
        const behavior = prefersReducedMotion ? "auto" : "smooth";
        targetElement.scrollIntoView({ behavior, block: "start" });
    }
}

const syntaxHighlight = () => {
    const isSourcePage = document.querySelector("body.source-page") !== null;
    if (!isSourcePage) return;

    const isLightMode = window.matchMedia('(prefers-color-scheme: light)').matches;
    const theme = isLightMode ? "github" : "github-dark";

    // import highlight.js + css
    if (!hljsCSSLink) {
        const link = document.createElement("link");
        link.setAttribute("rel", "stylesheet");
        link.setAttribute("href", `https://cdnjs.cloudflare.com/ajax/libs/highlight.js/11.11.1/styles/${theme}.min.css`);
        document.head.appendChild(link);
        hljsCSSLink = link;
    } else {
        hljsCSSLink.href = `https://cdnjs.cloudflare.com/ajax/libs/highlight.js/11.11.1/styles/${theme}.min.css`;
    }
    
    // get all lines of code and highlight them
    if (!hljsScript) {
        const script = document.createElement("script");
        script.src = "https://cdnjs.cloudflare.com/ajax/libs/highlight.js/11.11.1/highlight.min.js";
        script.onload = () => {
            const pre = document.querySelector('.source-container > pre');
            const lines = pre.querySelectorAll(':scope > [id*="line-"]');
            const source = Array.from(lines).map(line => line.textContent).join("\n");
            const lineNumbers = Array.from(lines).map((_, i) => `<span id="line-${i + 1}">${i + 1}</span>`).join("\n");

            pre.outerHTML = 
            `
            <button class="back-button">BACK</button>
            <div class="code-wrapper">
                <div class="line-numbers">
                    ${lineNumbers}
                </div>

                <pre>
                    <code class="language-java" hlfs></code>
                </pre>
            <div>
            `
            const codeBlock = document.querySelector(".code-wrapper > pre > code");
            codeBlock.textContent = source;
            hljs.highlightElement(codeBlock);

            const backButton = document.querySelector(".back-button");
            backButton.addEventListener("click", () => {
                const currentURL = new URL(window.location.href);
                currentURL.pathname = currentURL.pathname.replace("/src-html", "");
                currentURL.hash = "";
                window.location.href = currentURL.toString();
            });

            goToLineNumberByHash();
        };

        script.onerror = () => console.error("highlight.js failed to load 🤤");
        document.head.appendChild(script);
        hljsScript = script;
    }
}

const createListeners = () => {
    window.matchMedia('(prefers-color-scheme: light)').addEventListener('change', syntaxHighlight);
    window.addEventListener("hashchange", goToLineNumberByHash);
}

placeFavicon();
syntaxHighlight();
createListeners();
