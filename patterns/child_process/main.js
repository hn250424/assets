
const { registerCameraListeners, executeCameraScript } = require('./cameraHandler')

registerCameraListeners()

setTimeout(async () => {
    const jsonResponse = await executeCameraScript('connect', '70000720')
    const objResponse = JSON.parse(jsonResponse)

    if (objResponse.result === 1) console.log(`시리얼 번호 ${objResponse.data} 연결 성공.`)
    else console.log(`시리얼 번호 ${objResponse.data} 연결 실패.`)
}, 0)

setTimeout(async () => {
    const jsonResponse = await executeCameraScript('capture', 'filename', 'savePath')
    const objResponse = JSON.parse(jsonResponse)

    if (objResponse.result === 1) console.log(`${objResponse.data.savePath}/${objResponse.data.filename}.png 저장.`)
    else console.log('이미지 캡처 실패.')    
}, 1000)

setTimeout(async () => {
    const jsonResponse = await executeCameraScript('capture', 'filename-2', 'savePath')
    const objResponse = JSON.parse(jsonResponse)

    if (objResponse.result === 1) console.log(`${objResponse.data.savePath}/${objResponse.data.filename}.png 저장.`)
    else console.log('이미지 캡처 실패.')    
}, 2000)

setTimeout(async () => {
    const jsonResponse = await executeCameraScript('exit')
}, 3000)
