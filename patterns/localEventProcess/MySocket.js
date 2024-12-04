class MySocket {
    constructor() { this.events = {} }

    on(eventName, listener) { 
        (this.events[eventName] ||= []).push(listener) 
        return listener
    }

    emit(eventName, ...args) {
        if (this.events[eventName]) {
            this.events[eventName].forEach(listener => {
                listener(...args)
            })
        }
    }

    off(eventName, listener) {
        if (this.events[eventName]) {
            this.events[eventName] = this.events[eventName].filter(
                l => l !== listener
            );
        }
    }

    listenerCount(eventName) { return (this.events[eventName] ||= []).length }

    // 데이터 수신을 시뮬레이션하는 메서드. 이벤트 이름 'data'로 들어온다고 가정.
    receiveData(data) { this.emit('data', data) }
}

module.exports = MySocket
