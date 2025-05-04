

async function sleep(ms) {
    return new Promise(resolve => { setTimeout(resolve, ms) })
}

async function print1() {
    console.log(111)
    await sleep(1000)
    console.log(333)
}

async function print2() {
    console.log(222)
}

print1()
print2()