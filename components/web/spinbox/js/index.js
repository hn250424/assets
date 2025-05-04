const speed = document.getElementById('speed')
const power = document.getElementById('power')

document.addEventListener('DOMContentLoaded', async () => {
    await addTemplate('./templates/spinbox.html', speed, () => new Spinbox(speed, {min: 0.4, max: 100, value: 77, step: 0.46, slider: speed.querySelector('input[type="range"]')}))
    await addTemplate('./templates/spinbox.html', power, () => new Spinbox(power, {min: 0, max: 100, value: 77, step: 1, slider: power.querySelector('input[type="range"]')}))
    
    connectSliderAndSpinbox(speed, power)

    // 넘버 타입 입력 후 포커스아웃 시 유효성 검사.
    // note: js는 포커스 된 상태에서 곧장 다른 버튼을 눌러도 포커스아웃 이벤트가 발생.
    speed.querySelector('input[type="number"]').addEventListener('focusout', (event) => onNumberInputChange(event, speed.querySelector('input[type="range"]')))
    power.querySelector('input[type="number"]').addEventListener('focusout', (event) => onNumberInputChange(event, power.querySelector('input[type="range"]')))
})
