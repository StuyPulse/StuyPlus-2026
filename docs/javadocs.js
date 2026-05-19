// hella tuff javascript code to make javadocs look way nicer

const placeFavicon = () => {
    const link = document.createElement("link");
    link.setAttribute("rel", "icon");
    link.setAttribute("href", "/StuyPlus-2026/favicon.ico?t=" + Date.now()); // This forces the browser 
                                                                            // to reload the favicon everytime instead of getting the cached version
    link.setAttribute("type", "image/x-icon");
    document.head.appendChild(link);
}

placeFavicon();