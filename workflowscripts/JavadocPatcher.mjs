import fs from "fs";
import path from "path";
import { fileURLToPath } from "url";

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

const javadocDirectory = path.join(__dirname, "../build/docs/javadoc");

const fileWalker = (directory) => {
    const files = fs.readdirSync(directory);

    for (const file of files) {
        const filePath = path.join(directory, file);
        const stat = fs.statSync(filePath);

        if (stat.isDirectory()) {
            fileWalker(filePath);
        } else if (stat.isFile()) {
            if (filePath.endsWith(".html")) {
                htmlPatcher(filePath);
            }

            // Patch the script.js file because of the scroll position bug that comes with older versions of Javadoc.
            if (filePath.endsWith("script.js")) {
                const text = fs.readFileSync(filePath, "utf-8");
                const firstReplace = text.replace("var timeoutID;", "");
                const secondReplace = firstReplace.replace(
                    "var contentDiv = document.querySelector(\"div.flex-content\");",
                    "var timeoutID;\n    var contentDiv = document.querySelector(\"div.flex-content\");"
                )
                fs.writeFileSync(filePath, secondReplace, "utf-8");
            }
        }
    }
}

const htmlPatcher = (filePath) => {
    const text = fs.readFileSync(filePath, "utf-8");
    const newText = text.replace("</head>", '<script defer src="/StuyPlus-2026/javadocs.js"></script></head>');
    fs.writeFileSync(filePath, newText, "utf-8");
}

fileWalker(javadocDirectory);
console.log("Patcher complete");
