const { spawn } = require('child_process')

const cameraHandler = spawn('python', ['cameraHandler.py'])

function registerCameraListeners() {
    cameraHandler.stdout.on('data', (data) => {
        const response = data.toString().trim()
        const objResponse = JSON.parse(response)
        const requestId = objResponse.requestId
        // console.log(`Python stdout: ${objResponse}`)

        cameraHandler.emit(requestId, objResponse)
    })

    cameraHandler.stderr.on('data', (data) => { console.error(`Python stderr: ${data.toString()}`) })
    cameraHandler.on('close', (code) => { console.log(`Python process exited with code ${code}`) })
}

function executeCameraScript(command, requestId, ...args) {
    return new Promise((resolve) => {
        // Python 명령 결과 리스닝
        const onResponse = (objResponse) => {
            const requestId = objResponse.requestId

            cameraHandler.off(requestId, onResponse) // 명령 보내기 전 등록한 리스너, 응답 오면 리스너 삭제.
            resolve(objResponse)
        }

        cameraHandler.on(requestId, onResponse) // 명령 보내기 전 리스너 등록.

        switch (command) {
            case 'connect':
                if (args.length === 0) cameraHandler.stdin.write(`${command} ${requestId}\n`)
                else if (args.length === 1) cameraHandler.stdin.write(`${command} ${requestId} ${args[0]}\n`)
                else console.error('Invalid parameter')
                break
            case 'capture':
                if (args.length === 2) cameraHandler.stdin.write(`${command} ${requestId} ${args[0]} ${args[1]}\n`)
                else console.error('Invalid parameter')
                break
            case 'exit':
                if (args.length === 0) cameraHandler.stdin.write(`${command} ${requestId}\n`)
                else console.error('Invalid parameter')
                break
        }
    })
}

module.exports = { registerCameraListeners, executeCameraScript }
