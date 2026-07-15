// full update
// hella tuff javascript code to make javadocs look way nicer

let hljsCSSLink = null;
let hljsScript = null;

const getSectionFlashLengthMilliseconds = () => {
    const element = document.querySelector("section.detail");
    if (element) {
        const rawDuration = getComputedStyle(element).getPropertyValue("--flash-duration").trim();
        const seconds = parseFloat(rawDuration);
        const milliseconds = seconds * 1000;

        if (!isNaN(milliseconds)) {
            return milliseconds;
        } else {
            console.warn(`--flash-duration variable not found. Fallbacking to 2300ms.`);
            return 2300; // Default to 2.3 seconds if the value is invalid
        }
    }

    return 2300;
}

const getLineFlashLengthMilliseconds = () => {
    const element = document.querySelector(".line-highlight");
    if (element) {
        const rawDuration = getComputedStyle(element).getPropertyValue("--flash-duration").trim();
        const seconds = parseFloat(rawDuration);
        const milliseconds = seconds * 1000;

        if (!isNaN(milliseconds)) {
            return milliseconds;
        } else {
            console.warn(`--flash-duration variable not found. Fallbacking to 2300ms.`);
            return 2300; // Default to 2.3 seconds if the value is invalid
        }
    }

    return 2300;
};

let targetElementInView = null;
let timeoutCancelId = null;

const sectionInViewObserver = new IntersectionObserver((entries) => {
    for (const entry of entries) {
        console.log(
        `entry.target: ${entry.target.id}, isIntersecting: ${entry.isIntersecting}, intersectionRatio: ${entry.intersectionRatio}`
        )

        console.log(
            "OBSERVED:",
            entry.target.id,
            "CURRENT TARGET:",
            targetElementInView?.id
        );
        if (entry.target.id !== targetElementInView?.id) continue;

        if (!entry.isIntersecting || entry.intersectionRatio < 0.99) continue;

        console.log(`Target element ${entry.target.id} is in view. Flashing it now.`);
        entry.target.classList.add("flash");
        sectionInViewObserver.unobserve(entry.target);

        timeoutCancelId = setTimeout(() => {
            entry.target.classList.remove("flash");
            targetElementInView = null;
        }, getSectionFlashLengthMilliseconds());
    }
}, { threshold: 0.99 });

const placeFavicon = () => {
    const link = document.createElement("link");
    link.setAttribute("rel", "icon");
    link.setAttribute("href", "/StuyPlus-2026/favicon.ico?t=" + Date.now());
    link.setAttribute("type", "image/x-icon");
    document.head.appendChild(link);
}

const goToLineNumberByHash = () => {
    const isSourcePage = document.querySelector("body.source-page") !== null;
    if (!isSourcePage) return;

    const lineHashRegex = /^#line-\d+$/;
    if (!hljsScript) {
        console.warn("highlight.js not loaded yet & .line-numbers container not created yet 🤤.");
        return;
    }

    const hash = window.location.hash;
    if (!lineHashRegex.test(hash)) {
        return;
    }

    // The hash exactly matches the id already so we can just scroll to it cuz querySelector is so tuff
    const targetElement = document.querySelector(hash);
    if (targetElement) {
        const prefersReducedMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
        const behavior = prefersReducedMotion ? "auto" : "smooth";

        const lineInViewObserver = new IntersectionObserver((entries) => {
            for (const entry of entries) {
                if (!(entry.target === targetElement)) continue;

                if (entry.isIntersecting && entry.intersectionRatio >= 1.0) {
                    const highlightDiv = document.querySelector(".line-highlight");
                    const lineHeight = (() => {
                        const computed = window.getComputedStyle(highlightDiv);
                        const lineHeight = computed.getPropertyValue('--line-height').trim();
                        return parseFloat(lineHeight);
                    })();
                    const lineNumber = parseInt(entry.target.id.replace("line-", ""));
                    const translateY = (lineNumber - 1) * lineHeight;
                    highlightDiv.style.setProperty("transform", `translateY(${translateY}em)`);
                    highlightDiv.classList.add("flash");

                    setTimeout(() => {
                        entry.target.classList.remove("flash");
                    }, getLineFlashLengthMilliseconds());

                    lineInViewObserver.disconnect();
                }
            }
        }, { threshold: 1.0 })

        lineInViewObserver.observe(targetElement);
        targetElement.scrollIntoView({ behavior, block: "start", block: "center" });
    }
}

const detectTargetElementInViewOnHashChange = () => {
    const isClassDeclarationPage = document.querySelector("body.class-declaration-page") !== null;
    if (!isClassDeclarationPage) return;

    const targetElement = (() => {
        try {
            const rawHash = window.location.hash.slice(1);
            const decodedHash = decodeURIComponent(rawHash);
            return document.querySelector(`#${CSS.escape(decodedHash)}`);
        } catch (error) {
            return null;
        }
    })();
    if (!targetElement || !targetElement.matches("section.detail")) {
        console.log("NOT");
        return;
    }

    if (timeoutCancelId) {
        clearTimeout(timeoutCancelId);
        timeoutCancelId = null;
    }

    if (targetElementInView) {
        targetElementInView.classList.remove("flash");
        sectionInViewObserver.unobserve(targetElementInView);
        targetElementInView = null;
    }

    if (targetElementInView && targetElementInView !== targetElement) {
        sectionInViewObserver.unobserve(targetElementInView);
        targetElementInView.classList.remove("flash");
    }

    targetElementInView = targetElement;
    sectionInViewObserver.observe(targetElement);

    targetElement.scrollIntoView({ behavior: "smooth", block: "center" });
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

            const highlightDiv = document.createElement("div");
            highlightDiv.setAttribute("class", "line-highlight");
            codeBlock.appendChild(highlightDiv);

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
    window.addEventListener("hashchange", () => {
        goToLineNumberByHash();
        detectTargetElementInViewOnHashChange();
    });
}

placeFavicon();
syntaxHighlight();

goToLineNumberByHash();
detectTargetElementInViewOnHashChange();
createListeners();