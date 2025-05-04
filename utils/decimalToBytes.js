function decimalToBytes(decimalValue, byteSize) {
    // if (decimalValue < 0 || decimalValue >= (1 << (byteSize * 8)) ) throw new Error(`Decimal value ${decimalValue} is out of range for ${byteSize} bytes`) // js에서 비트 연산은 32비트 정수로 제한.
    if (decimalValue < 0 || decimalValue >= Math.pow(2, byteSize * 8)) throw new Error(`Decimal value ${decimalValue} is out of range for ${byteSize} bytes`)

    const bytes = new Uint8Array(byteSize)
    for (let i = 0; i < byteSize; i++) {
        bytes[byteSize - 1 - i] = (decimalValue >> (8 * i)) & 0xFF
    }

    return bytes
}