def calculate_crc8(text: str) -> int:
    """
    Calculates 8-bit CRC for the given string using polynomial 0x07.
    Consistent with the STM32 implementation.
    """
    crc = 0x00
    if not text:
        return 0
        
    for char in text:
        byte = ord(char)
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x07
            else:
                crc <<= 1
            crc &= 0xFF
            
    return crc
