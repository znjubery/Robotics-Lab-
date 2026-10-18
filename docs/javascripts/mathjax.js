window.MathJax = {
  tex: {
    inlineMath: [["\\(", "\\)"]],
    displayMath: [["\\[", "\\]"]],
    processEscapes: true,
    processEnvironments: true,
    macros: {
      SO: "\\mathrm{SO}",
      SE: "\\mathrm{SE}",
      so: "\\mathfrak{so}",
      se: "\\mathfrak{se}",
      Ad: "\\mathrm{Ad}",
      R: "\\mathbb{R}",
      bracket: ["\\left[#1\\right]", 1]
    }
  },
  options: {
    ignoreHtmlClass: ".*|",
    processHtmlClass: "arithmatex"
  }
};

document$.subscribe(() => {
  MathJax.startup.output.clearCache();
  MathJax.typesetClear();
  MathJax.texReset();
  MathJax.typesetPromise();
});
