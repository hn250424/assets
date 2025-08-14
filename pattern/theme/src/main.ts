import './style.scss'

const themeDefault = document.getElementById('default') as HTMLElement
const themeLight = document.getElementById('light') as HTMLElement
const themeDark = document.getElementById('dark') as HTMLElement

themeDefault.addEventListener('click', () => {
    document.documentElement.classList.remove('light', 'dark')
})

themeLight.addEventListener('click', () => {
    document.documentElement.classList.remove('dark')
    document.documentElement.classList.add('light')
})

themeDark.addEventListener('click', () => {
    document.documentElement.classList.remove('light')
    document.documentElement.classList.add('dark')
})