/************************* PROJECT RON *************************/
/* Copyright (c) 2026 StuyPulse Robotics. All rights reserved. */
/* Use of this source code is governed by an MIT-style license */
/* that can be found in the repository LICENSE file.           */
/***************************************************************/

/************************* javadocs.js ************************/
/* The following code is used to customize the javadoc pages  */
/* to add custom features to improve UX and readability. It   */
/* adds the following feature:                                */
/* 1. Syntax highlighting for source code pages (via          */
/* highlight.js)                                              */
/* 2. Back button on source code pages to return to class     */
/* declaration page                                           */
/* 3. Theming for light and dark mode                         */
/* 4. Smooth scrolling to line numbers on source code pages   */
/* and sections on class declaration pages on URL hash change */
/* 5. Flashing of line numbers and sections when scrolled in  */
/* view                                                       */
/**************************************************************/
import { scrollToElementWithRespectToMotionPreference, isSourcePage, isClassDeclarationPage } from "./utils.js";
import { getEffectiveTheme } from "./theme.js";

let hljsCSSLink = null;
let hljsScript = null;

let animationEndEventListener = null;
let targetElementInView = null;
const sectionInViewObserver = new IntersectionObserver((entries) => {
    for (const entry of entries) {
        if (entry.target.id !== targetElementInView?.id) continue;

        if (!entry.isIntersecting || entry.intersectionRatio < 0.99) continue;

        entry.target.classList.add("flash");
        sectionInViewObserver.unobserve(entry.target);

        const handleAnimationEnd = (animationEvent) => {
            if (animationEvent.animationName !== "flashBorder") return;

            entry.target.classList.remove("flash");
            targetElementInView = null;
        }
        entry.target.addEventListener("animationend", handleAnimationEnd, { once: true });
    }
}, { threshold: 0.99 });

const goToLineNumberByHash = () => {
    if (!isSourcePage()) return;

    const lineHashRegex = /^#line-\d+$/;
    if (!hljsScript) {
        console.warn("highlight.js not loaded yet & .line-numbers container not created yet 🤤.");
        return;
    }

    const hash = window.location.hash;
    if (!lineHashRegex.test(hash)) {
        return;
    }

    const targetElement = document.querySelector(hash);
    if (targetElement) {
        const lineInViewObserver = new IntersectionObserver((entries) => {
            for (const entry of entries) {
                if (!(entry.target === targetElement)) continue;

                if (entry.isIntersecting && entry.intersectionRatio >= 1.0) {
                    const highlightDiv = document.querySelector(".line-highlight");
                    const lineHeight = (() => {
                        const computed = window.getComputedStyle(highlightDiv);
                        const lineHeight = computed.getPropertyValue("--line-height").trim();
                        return parseFloat(lineHeight);
                    })();
                    const lineNumber = parseInt(entry.target.id.replace("line-", ""));
                    const translateY = (lineNumber - 1) * lineHeight;
                    highlightDiv.style.setProperty("transform", `translateY(${translateY}em)`);
                    highlightDiv.classList.add("flash");

                    const handleAnimationEnd = (animationEvent) => {
                        if (animationEvent.animationName !== "flashLine") return;

                        highlightDiv.classList.remove("flash");
                    }
                    highlightDiv.addEventListener("animationend", handleAnimationEnd, { once: true });
                    animationEndEventListener = handleAnimationEnd;

                    lineInViewObserver.disconnect();
                }
            }
        }, { threshold: 1.0 })

        lineInViewObserver.observe(targetElement);
        scrollToElementWithRespectToMotionPreference(targetElement, "center");
    }
}

const detectTargetElementInViewOnHashChange = () => {
    if (!isClassDeclarationPage()) return;

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
        return;
    }

    if (animationEndEventListener) {
        const highlightDiv = document.querySelector(".line-highlight");
        highlightDiv.removeEventListener("animationend", animationEndEventListener);
        animationEndEventListener = null;
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

    scrollToElementWithRespectToMotionPreference(targetElement, "center");
}

const syntaxHighlight = () => {
    if (!isSourcePage()) return;

    const isLightMode = getEffectiveTheme() === "light";
    const theme = isLightMode ? "github" : "github-dark";

    if (!hljsCSSLink) {
        const link = document.createElement("link");
        link.setAttribute("rel", "stylesheet");
        link.setAttribute("href", `https://cdnjs.cloudflare.com/ajax/libs/highlight.js/11.11.1/styles/${theme}.min.css`);
        document.head.appendChild(link);
        hljsCSSLink = link;
    } else {
        hljsCSSLink.href = `https://cdnjs.cloudflare.com/ajax/libs/highlight.js/11.11.1/styles/${theme}.min.css`;
    }
    
    if (!hljsScript) {
        const script = document.createElement("script");
        script.src = "https://cdnjs.cloudflare.com/ajax/libs/highlight.js/11.11.1/highlight.min.js";
        script.onload = () => {
            const pre = document.querySelector(".source-container > pre");
            const lines = pre.querySelectorAll(":scope > [id*=\"line-\"]");
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

const handleThemeChangeEvent = (event) => {
    const isLightMode = event.matches;
    document.documentElement.setAttribute("data-theme", isLightMode ? "light" : "dark");
};
const handleTheme = () => {
    const isSourcePage = document.querySelector("body.source-page") !== null;
    if (isSourcePage) return;

    let themeState = localStorage.getItem("theme");
    if (!themeState) {
        themeState = "system";
        localStorage.setItem("theme", themeState);
    }
    if (themeState === "light") {
        document.documentElement.setAttribute("data-theme", "light");
    } else if (themeState === "dark") {
        document.documentElement.setAttribute("data-theme", "dark");
    } else if (themeState === "system") {
        const isLightMode = window.matchMedia("(prefers-color-scheme: light)").matches;
        document.documentElement.setAttribute("data-theme", isLightMode ? "light" : "dark");

        window.matchMedia("(prefers-color-scheme: light)").addEventListener("change", handleThemeChangeEvent);
    }

    const topNavbar = document.querySelector("#navbar-top");
    
    const themeToggleWrapper = document.createElement("div");
    themeToggleWrapper.setAttribute("class", "theme-toggle-wrapper");
    topNavbar.appendChild(themeToggleWrapper);

    const themeToggleButton = document.createElement("button");
    themeToggleButton.setAttribute("class", "theme-toggle-button");
    themeToggleButton.setAttribute("aria-label", "Toggle theme");
    themeToggleWrapper.appendChild(themeToggleButton);
}

const createListeners = () => {
    window.matchMedia("(prefers-color-scheme: light)").addEventListener("change", syntaxHighlight);
    window.addEventListener("hashchange", () => {
        goToLineNumberByHash();
        detectTargetElementInViewOnHashChange();
    });
}

syntaxHighlight();

goToLineNumberByHash();
detectTargetElementInViewOnHashChange();
createListeners();
