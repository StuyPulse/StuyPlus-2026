const scrollToElementWithRespectToMotionPreference = (element, block) => {
    const prefersReducedMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches;
    const behavior = prefersReducedMotion ? "auto" : "smooth";

    element.scrollIntoView({ behavior, block });
}

const isSourcePage = () => {
    return document.querySelector("body.source-page") !== null;
}

const isClassDeclarationPage = () => {
    return document.querySelector("body.class-declaration-page") !== null;
}

export { scrollToElementWithRespectToMotionPreference, isSourcePage, isClassDeclarationPage };