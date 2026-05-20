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
    // import highlight.js + css
    const link = document.createElement("link");
    link.setAttribute("rel", "stylesheet");
    link.setAttribute("href", "https://cdnjs.cloudflare.com/ajax/libs/highlight.js/11.11.1/styles/default.min.css");
    document.head.appendChild(link);
    
    // get all lines of code and highlight them
    const script = document.createElement("script");
    script.src = "https://cdnjs.cloudflare.com/ajax/libs/highlight.js/11.11.1/highlight.min.js";
    // script.onload = () => {
    //         const codeBlocks = document.querySelectorAll('.source-container > pre > [id*="line-"]');
    //         codeBlocks.forEach((line) => {
    //         const rawLine = line.textContent;
    //         const highlightedLine = hljs.highlight(rawLine, { language: 'java' }).value;

    //         line.innerHTML = highlightedLine;
    //     });
    // };
    script.onerror = () => console.error("highlight.js failed to load 🤤");
    document.head.appendChild(script);
}

placeFavicon();
syntaxHighlight();
