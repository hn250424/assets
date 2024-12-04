const { spawn } = require('child_process')

const cameraHandler = spawn('python', ['cameraHandler.py'])

function registerCameraListeners() {
    cameraHandler.stdout.on('data', (data) => {
        const message = data.toString().trim()
        // console.log(`Python stdout: ${message}`)

        cameraHandler.emit('response', message)
    })

    cameraHandler.stderr.on('data', (data) => { console.error(`Python stderr: ${data.toString()}`) })
    cameraHandler.on('close', (code) => { console.log(`Python process exited with code ${code}`) })
}

function executeCameraScript(command, ...args) {
    return new Promise((resolve) => {
        // Python 명령 결과 리스닝
        const onResponse = (message) => {
            cameraHandler.off('response', onResponse) // 명령 보내기 전 등록한 리스너, 응답 오면 리스너 삭제.
            resolve(message)
        }

        cameraHandler.on('response', onResponse) // 명령 보내기 전 리스너 등록.

        switch (command) {
            case 'connect':
                if (args.length === 0) cameraHandler.stdin.write(`${command}\n`)
                else if (args.length === 1) cameraHandler.stdin.write(`${command} ${args[0]}\n`)
                else console.error('Invalid parameter')
                break
            case 'capture':
                if (args.length === 2) cameraHandler.stdin.write(`${command} ${args[0]} ${args[1]}\n`)
                else console.error('Invalid parameter')
                break
            case 'exit':
                if (args.length === 0) cameraHandler.stdin.write(`${command}\n`)
                else console.error('Invalid parameter')
                break
        }
    })
}

module.exports = { registerCameraListeners, executeCameraScript }
