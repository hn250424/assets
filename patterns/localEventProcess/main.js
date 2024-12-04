const MySocket = require('./MySocket')
const mySocket = new MySocket()

async function specificListener(call) {
    return new Promise((resolve, reject) => {
        const dataListener = (data) => { mySocket.emit(call, data) }
        const eventHandler = (data) => {
            resolve(data)
            mySocket.off(call, eventHandler)
            mySocket.off('data', dataListener)
        }

        mySocket.on('data', dataListener)
        mySocket.on(call, eventHandler)

        try {
            mySocket.receiveData(`파일 또는 특정 메서드 ${call}에서 socket.write() 썼다고 가정. 그에 대한 응답: blah blah...`)
        } catch (error) {
            reject(error)
        }
    })
}

async function main() {
    try {
        console.log(await specificListener('can.js'))
        console.log(await specificListener('model.js'))
    } catch (error) {
        console.error('Error:', error)
    }
}

main()
