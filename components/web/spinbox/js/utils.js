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

/**
 * 슬라이더와 스핀박스 연동.
 * 
 * @param  {...HTMLElement} containers - 'input[type="number"]'와 'input[type="range"]'를 자식으로 갖는 DOM 요소들.
 */
function connectSliderAndSpinbox(...containers) {
    containers.forEach(container => {
        const spinbox = container.querySelector('input[type="number"]')
        const slider = container.querySelector('input[type="range"]')
        spinbox.addEventListener('input', function () {
            slider.value = this.value
        })
        slider.addEventListener('input', function () {
            spinbox.value = this.value
        })
    })
}

/**
 * 입력된 숫자 값을 검증하고, 해당 값을 슬라이더에 동기화.
 * 
 * @param {Event} event - 이벤트 객체.
 * @param {HTMLInputElement} slider - 슬라이더 요소로, 숫자 입력 필드와 동기화할 슬라이더.
 * 
 * 초기 설정으로 숫자 입력 필드와 슬라이더가 연동되어 있지만( connectSliderAndSpinbox ),
 * 특정 경우, 예를 들어 숫자 입력 필드에 0+를 입력하면,
 * 슬라이더는 50, 숫자 입력 필드는 0이 할당.
 * 이를 방지하기 위해 슬라이더를 여기서 직접 동기화.
 */
function onNumberInputChange(event, slider) {
    let value = Number(event.target.value)
    const min = Number(event.target.min)
    const max = Number(event.target.max)
    const step = Number(event.target.step)

    if (value < min) value = min
    else if (value > max) value = max

    value = Math.round(value / step) * step                             // 스텝 반올림.
    const decimalPlaces = step.toString().split('.')[1]?.length || 0    // 소수점 이하 자릿수.
    value = parseFloat(value.toFixed(decimalPlaces))    

    event.target.value = value
    slider.value = value
}