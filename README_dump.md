# Block Dump Functionality

This document describes the new dump functionality added to the main.py file.

## Overview

The dump functionality allows you to retrieve binary files from all connected blocks (IDs 1-10) and save them as separate files on the host machine.

## Implementation Details

### New Functions Added

1. **`build_dump_packet(block_id: int)`** - Creates a CMD_DUMP command packet for a specific block
2. **`read_dump_chunks(ser, expected_block_id, timeout_seconds=5.0)`** - Reads chunked binary data from a block
3. **`dump_all_blocks()`** - Main function that handles the entire dump process for all blocks

### New API Endpoint

**GET `/dump`** - Triggers the dump operation for all blocks

Returns a JSON response with:
- `results`: Array of per-block results with status, filename, and bytes received
- `summary`: Overall statistics including successful/failed dumps and total bytes

## Usage

### Via FastAPI Server

1. Start the FastAPI server:
   ```bash
   poetry run poe api
   ```

2. Make a GET request to the dump endpoint:
   ```bash
   curl http://blockcontroller.local:8000/dump
   ```

### Direct Function Call

You can also call the dump function directly:

```python
from main import dump_all_blocks

results = dump_all_blocks()
```


## Output Files

The dump operation creates binary files named:
- `block_1_dump.bin`
- `block_2_dump.bin`
- ...
- `block_10_dump.bin`

Each file contains the complete binary data received from the corresponding block.

## Error Handling

The implementation includes comprehensive error handling for:
- Blocks that don't respond to the initial dump command
- Timeout during chunk reception
- Checksum validation failures
- File write errors
- Serial communication issues

## Protocol Details

The dump process follows this sequence:

1. Send CMD_DUMP command to each block
2. Immediately start listening for binary data chunks (no ACK wait)
3. Receive binary data in chunks (up to 256 bytes per chunk)
4. Validate checksums for each chunk
5. Receive final ACK packet (zero-length) indicating end of transmission
6. Reassemble chunks into complete file
7. Save to disk with appropriate filename

The block-side code sends data in chunks with the format:
```
[STX, BLOCK_ID, CMD_DUMP, chunk_length] + chunk_data + checksum
```

After all data chunks are sent, the block sends an ACK packet:
```
[STX, BLOCK_ID, CMD_DUMP, 0] + checksum
```

This ACK packet (with length 0) indicates the end of transmission.

## Configuration

Key configuration parameters:
- `BLOCK_IDS`: Range of block IDs to query (default: 1-10)
- `SERIAL_PORT`: Serial port for communication (default: '/dev/ttyUSB0')
- `BAUD`: Baud rate (default: 1000000)
- Chunk timeout: 5 seconds per chunk
- Inter-block delay: 0.1 seconds

## Troubleshooting

Common issues and solutions:

1. **"no_ack_response"** - Block didn't respond to dump command
   - Check if block is powered and connected
   - Verify block ID is correct

2. **"no_data_received"** - Block acknowledged but sent no data
   - Block may not have any data to dump
   - Check block-side file existence

3. **"file_write_error"** - Error saving file to disk
   - Check disk space and write permissions
   - Verify directory is writable

4. **Partial data received** - Transmission interrupted
   - Check serial connection stability
   - May need to retry the operation
