import process from "process";
import fs from "fs";
import path from "path";
import { fileURLToPath } from "url";

const __filename = fileURLToPath(import.meta.url);
const __dirname = (() => {
    process.chdir(path.join(path.dirname(__filename), "../"));
    return process.cwd();
})();

const javadocDirectory = path.join(__dirname, "build/docs/javadoc");
const assetsDirectory = path.join(__dirname, "assets");

const getRelativePath = (filePath) => {
    return path.relative(__dirname, filePath);
}

const getRelativePathBetweenFiles = (fromFile, toFile) => {
    const relativePath = path.relative(
        path.dirname(fromFile),
        toFile
    );

    return relativePath.replaceAll("\\", "/");
}

const log = (message, color = "yellow") => {
    const RESET_ANSI = "\x1b[0m";

    const colorCode = (() => {
        if (color === "green") {
            return "\x1b[32m";
        }
        if (color === "yellow") {
            return "\x1b[33m";
        }
        if (color === "red") {
            return "\x1b[31m";
        }
    })();

    console.log(`${colorCode}${message}${RESET_ANSI}`);
}

const fileWalker = (directory) => {
    const files = fs.readdirSync(directory, { withFileTypes: true });

    for (const file of files) {
        const filePath = path.join(directory, file.name);

        if (file.isDirectory()) {
            fileWalker(filePath);
        } else if (file.isFile()) {
            if (filePath.endsWith(".html")) {
                htmlPatcher(filePath);
            }
        }
    }
}

// Patch the script.js file because of the scroll position bug that comes with older versions of Javadoc.
const scriptJsPatcher = (filePath) => {
    const text = fs.readFileSync(filePath, "utf-8");

    const newText = (() => {
        let updatedText = text.replace(
            /^(\s*)var contentDiv = document\.querySelector\("div\.flex-content"\);/m,
            `$1var timeoutID;\n$1var contentDiv = document.querySelector("div.flex-content");`
        )

        updatedText = updatedText.replace(
            /(contentDiv\.addEventListener\("scroll", function\(e\) {\s*)var timeoutID;\s*/m,
            "$1"
        )

        return updatedText;
    })();

    if (newText === text) {
        log(`Failed to patch ${getRelativePath(filePath)}`, "red");
        return;
    }

    fs.writeFileSync(filePath, newText, "utf-8");
    log(`Patched ${getRelativePath(filePath)}`);
};

const htmlPatcher = (filePath) => {
    const text = fs.readFileSync(filePath, "utf-8");

    const javadocsFile = path.join(javadocDirectory, "javadocs.js");
    const faviconFile = path.join(javadocDirectory, "favicon.ico");
    const replacement =
    `<script defer src="${getRelativePathBetweenFiles(filePath, javadocsFile)}"></script>` +
    `<link rel="icon" href="${getRelativePathBetweenFiles(filePath, faviconFile)}?t=${Date.now()}" type="image/x-icon"></head>`;

    const newText = text.replace(
        "</head>",
        replacement
    );

    if (newText === text) {
        log(`Failed to patch ${getRelativePath(filePath)}`, "red");
        return;
    }

    fs.writeFileSync(filePath, newText, "utf-8");
    log(`Patched ${getRelativePath(filePath)}`);
}

const javadocAssets = [
    "javadoc/favicon.ico",
    "javadoc/javadocs.js",
    "logos/StuyPlusLogo.png"
]
const addJavadocAssets = () => {
    for (const asset of javadocAssets) {
        const sourcePath = path.join(assetsDirectory, asset);
        const assetName = path.basename(asset);
        const destPath = path.join(javadocDirectory, assetName);
        fs.copyFileSync(sourcePath, destPath);
        log(`Copied ${getRelativePath(sourcePath)} to javadoc directory`);
    }
}

const checkpointFile = path.join(javadocDirectory, ".patched");

if (fs.existsSync(checkpointFile)) {
    log("Javadoc already patched, skipping patcher", "green"); 
} else {
    scriptJsPatcher(path.join(javadocDirectory, "script.js"));
    fileWalker(javadocDirectory);
    addJavadocAssets();

    log("Patcher complete", "green");
    fs.writeFileSync(checkpointFile, "done", "utf-8");
}
