window.addEventListener('DOMContentLoaded', () => {
  const replaceText = (selector: string, text: string) => {
    const element = document.getElementById(selector);
    if (element) element.innerText = text;
  };

  const dependencies: (keyof NodeJS.ProcessVersions)[] = ['chrome', 'node', 'electron'];
  
  for (const dependency of dependencies) {
    replaceText(`${dependency}-version`, process.versions[dependency] ?? 'unknown');
  }
});