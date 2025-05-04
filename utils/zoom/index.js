const container = document.getElementById('container')

let scale = 1

function zoom(event, delta) {
    const rect = container.getBoundingClientRect()
    
    const offsetX = event.clientX - rect.left
    const offsetY = event.clientY - rect.top

    const originX = (offsetX / rect.width) * 100
    const originY = (offsetY / rect.height) * 100
    container.style.transformOrigin = `${originX}% ${originY}%`

    scale = Math.min(3, Math.max(0.5, scale + delta))

    container.style.transform = `translate(-50%, -50%) scale(${scale})`
}


document.addEventListener('wheel', (event) => {
    if (event.ctrlKey) {
        event.preventDefault()

        const delta = event.deltaY < 0 ? 0.1 : -0.1
        zoom(event, delta)
    }
}, { passive: false })