const fs = require('fs')

function isBinaryFile(filePath, sampleSize = 8000) {
    const buffer = fs.readFileSync(filePath)
    const len = Math.min(buffer.length, sampleSize)

    let suspicious = 0

    for (let i = 0; i < len; i++) {
        const byte = buffer[i]

        // if null exists, 
        if (byte === 0) return true

        // if it contains something that is not used often.
        if ((byte > 0 && byte < 0x09) || (byte > 0x0D && byte < 0x20)) {
            suspicious++
        }
    }

    return suspicious / len > 0.3
}