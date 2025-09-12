def calc_checksum(data: bytes) -> int:
    return sum(data) % 256