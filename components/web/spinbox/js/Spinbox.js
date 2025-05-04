/**
 * @param {HTMLElement} container - 스핀박스의 HTML 요소를 포함하는 컨테이너 즉 부모 요소.
 * @param {Object} options - 스핀박스 설정을 포함하는 객체.
 *     - {number} options.min - 입력 값의 최소값.
 *     - {number} options.max - 입력 값의 최대값.
 *     - {number} options.value - 초기 입력 값.
 *     - {number} options.step - 입력 값의 증감 단위.
 *     - {HTMLElement} options.slider - 연동할 슬라이더 요소.
 */
class Spinbox {
    // regex = /^[\-\+]?[0-9]+(\.[0-9]+)?$/

    constructor(container, options) {
        this.container = container
        this.input = container.querySelector('.input')
        this.minus = container.querySelector('.minus')
        this.plus = container.querySelector('.plus')

        // this.preInputKey = null

        this.init(options)
    }

    init(options) {
        // 버튼은 정사각형, 사이즈는 input 창의 높이와 같게 설정.
        const inputHeight = this.input.offsetHeight
        this.minus.style.height = `${inputHeight}px`
        this.plus.style.height = `${inputHeight}px`
        this.minus.style.width = `${inputHeight}px`
        this.plus.style.width = `${inputHeight}px`

        this.input.min = options.min
        this.input.max = options.max
        this.input.value = options.value
        this.input.step = options.step
        this.slider = options.slider

        this.decimalPlaces = this.getDecimalPlaces(options.step)
        this.minus.addEventListener('click', () => this.decrement())
        this.plus.addEventListener('click', () => this.increment())

        // 입력 범위 제어.
        // this.input.addEventListener('keydown', (event) => this.preInputKey = event.key)
        // this.input.addEventListener('input', () => this.handleInputRange())
    }

    getDecimalPlaces(step) {
        return step.toString().includes('.') ? step.toString().split('.')[1].length : 0
    }

    decrement() {
        let newValue = Number(this.input.value) - Number(this.input.step)
        newValue = Math.max(Number(this.input.min), parseFloat(newValue.toFixed(this.decimalPlaces)))
        this.updateValue(newValue)
    }

    increment() {
        let newValue = Number(this.input.value) + Number(this.input.step)
        newValue = Math.min(Number(this.input.max), parseFloat(newValue.toFixed(this.decimalPlaces)))
        this.updateValue(newValue)
    }

    updateValue(value) {
        const formattedValue = parseFloat(value.toFixed(this.decimalPlaces)) // 포맷 적용
        this.input.value = formattedValue
    }

    // handleInputRange() {

    // }
}