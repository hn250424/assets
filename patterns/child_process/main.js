
const { registerCameraListeners, executeCameraScript } = require('./cameraHandler')

registerCameraListeners()

let requestId = 0

setTimeout(async () => {
    const objResponse = await executeCameraScript('connect', requestId++, '70000720')

    if (objResponse.result === 1) console.log(`시리얼 번호 ${objResponse.data} 연결 성공.`)
    else console.log(`시리얼 번호 ${objResponse.data} 연결 실패.`)
}, 0)

setTimeout(async () => {
    const objResponse = await executeCameraScript('capture', requestId++, 'filename', 'savePath')

    if (objResponse.result === 1) console.log(`${objResponse.data.savePath}/${objResponse.data.filename}.png 저장.`)
    else console.log('이미지 캡처 실패.')    
}, 1000)

setTimeout(async () => {
    const objResponse = await executeCameraScript('capture', requestId++, 'filename-2', 'savePath')

    if (objResponse.result === 1) console.log(`${objResponse.data.savePath}/${objResponse.data.filename}.png 저장.`)
    else console.log('이미지 캡처 실패.')    
}, 2000)

setTimeout(async () => {
    const objResponse = await executeCameraScript('exit', requestId++)

    if (objResponse.result === 1) console.log(`exit...`)
}, 3000)
