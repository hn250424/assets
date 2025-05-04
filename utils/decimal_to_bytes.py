def decimal_to_bytes(decimal_value: int, byte_size: int) -> bytes:
    '''
    Convert a decimal integer to a byte array with specified byte size.
    If the integer requires fewer bytes than the specified byte size, the bytes are filled with zeros on the left.
    '''
    
    # decimal_value의 범위는 0보다 크거나 같고 변환할 바이트 어레이 사이즈보다 작아야 한다.
    if decimal_value < 0 or decimal_value >= (1 << (byte_size * 8)):
        raise ValueError(f"Decimal value {decimal_value} is out of range for {byte_size} bytes")

    '''
    struct.pack에 사용될 포맷 문자열 정의.

    >: 빅 엔디안 - 중요한, 큰 값을 메모리 낮은 주소에.
    B: 0~255까지의 값을 가진 1바이트 정수를 표현.
    '''
    format_string = f'>{byte_size}B'

    '''
    지정한 바이트 사이즈를 순회하는데 상위 비트가 먼저 들어가야(struct.pack) 하므로 역으로 높은 수부터으로 순회.
    차례가 된 바이트 구역 아래 하위 비트들은(i * 8) 시프트 연산자 >>를 통해 오른쪽으로 이동시켜 1바이트만을 남기고 잘라낸다.
    차례가 된 바이트 구역 위 상위 비트들은 AND 연산자 & 0xFF( 0b11111111 )를 통해 하위 8비트만 남기고 그 상위는 잘라낸다.
    언패킹 연산자 *로 리스트나 튜플의 요소들을 개별 인자로 펼쳐서 전달.
    '''
    bytes_value = struct.pack(
        format_string, 
        *[decimal_value >> (8 * i) & 0xFF for i in reversed(range(byte_size))]
    )
    
    return bytes_value