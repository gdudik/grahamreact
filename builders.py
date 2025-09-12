import checksum
from typing import Literal


def build_ping_packet(block_id: int) -> bytes:
    header = bytes([STX, block_id, CMD_PING, 0])  # 0-length payload
    return header + bytes([checksum.calc_checksum(header)])


def build_gender_packet(block_id: int, gender: Literal['M', 'F']) -> bytes:
    payload = gender.encode('utf-8')
    header = bytes([STX, block_id, CMD_SET_GENDER, 1])
    packet = header + payload
    return packet + bytes([checksum.calc_checksum(packet)])


def build_arm_packet() -> bytes:
    header = bytes([STX, BROADCAST_ID, CMD_ARM, 0])
    return header + bytes([checksum.calc_checksum(header)])


def build_set_packet() -> bytes:
    header = bytes([STX, BROADCAST_ID, CMD_SET, 0])
    return header + bytes([checksum.calc_checksum(header)])


def build_sensor_type_packet(block_id: int, sensor_type: Literal['NC', 'NO']) -> bytes:
    payload = sensor_type.encode('utf-8')
    header = bytes([STX, block_id, CMD_SET_SENSOR, 2])
    packet = header + payload
    return packet + bytes([checksum.calc_checksum(packet)])


def build_send_report_packet(block_id: int) -> bytes:
    header = bytes([STX, block_id, CMD_SEND_RT_REPORT, 0])
    return header + bytes([checksum.calc_checksum(header)])


def build_dump_packet(block_id: int) -> bytes:
    header = bytes([STX, block_id, CMD_DUMP, 0])
    return header + bytes([checksum.calc_checksum(header)])