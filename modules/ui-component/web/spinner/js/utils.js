async function addTemplate(path, container, registerListener) {
    const response = await fetch(path)
    const html = await response.text()

    const temp = document.createElement('div')
    temp.innerHTML = html

    const templateElement = temp.querySelector('.template')
    const clone = document.importNode(templateElement.content, true)

    container.appendChild(clone)

    const styleElement = temp.querySelector('style')
    const newStyle = document.createElement('style')
    newStyle.textContent = styleElement ? styleElement.textContent : ''
    document.head.appendChild(newStyle)

    const scriptElement = temp.querySelector('script')
    const newScript = document.createElement('script')
    newScript.textContent = scriptElement ? scriptElement.textContent : ''
    document.body.appendChild(newScript)

    registerListener(container)
}