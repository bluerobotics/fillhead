# Fix: USB Chunk Reassembly Bug (Telemetry Corrupting Status Messages)

## Problem Statement

Status messages received over USB were showing telemetry data appended to them:

```
FILLHEAD_INFO: Vacuum: Leak Test: Target reached.:Not Homed,h_st_str:Off,vac_st_str:Pulldown,lc:0
FILLHEAD_INFO: Vacuum: Leak Test: Starting measureNot Homed,h_st_str:Off,vac_st_str:Settling,lc:0
FILLHEAD_DONE: Vacuum: vacuum_leak_test complete.Not Homed,h_st_str:Off,vac_st_str:Leak Test,lc:0
FILLHEAD_ERROR: Injector: Invalid JOG_MOVE format.st:Not Homed,h_st_str:Off,vac_st_str:Off,lc:0
```

In every case the status message is ~50 characters (matching `CHUNK_SIZE`) and is followed
by the tail of a telemetry string (`vac_v_st:Not Homed,h_st_str:...,vac_st_str:...,lc:0`).

## Root Cause Analysis

### Firmware side (`comms_controller.cpp` — `processTxQueue`)

Messages longer than `CHUNK_SIZE` (50 bytes) were split into numbered chunks with headers
(`CHUNK_1/N:...`, `CHUNK_2/N:...`). Each chunk was sent as a separate `\n`-terminated line
over USB. Per-chunk timeouts (3 ms) and a global loop timeout (100 ms) meant individual
chunks could be silently dropped when the USB write buffer was under pressure.

### App side (`br-equipment-control-app/src/comms/serial.py`)

The chunk reassembly protocol accumulated chunks in a flat `chunk_buffer` list and triggered
reassembly when `len(chunk_buffer) == total_chunks` (taken from the **last received chunk**).
Critically, the buffer had **no message ID** — chunks from different messages could be mixed.

### How the false reassembly happens

1. A telemetry message (~600 bytes, 12 chunks) has some chunks dropped. One stray chunk
   (e.g. the last one, `CHUNK_12/12`) lingers in `chunk_buffer`.
2. A status message (>50 bytes) arrives as 2 chunks (`CHUNK_1/2`, `CHUNK_2/2`).
3. `CHUNK_1/2` is added to the buffer. `len(chunk_buffer) == 2`, `total_chunks == 2`
   → **false reassembly triggered**.
4. After sorting by `chunk_num`, chunk 1 (status first 50 chars) comes before chunk 12
   (telemetry tail). They are concatenated and dispatched as a single message.

Result: `"<first 50 chars of status><last ~50 chars of telemetry>"` — exactly what the logs show.

## Change Made

**File:** `src/comms_controller.cpp` — `CommsController::processTxQueue()`

Replaced the `CHUNK_N/T:` protocol with raw byte streaming:

```cpp
if (m_usbHostConnected) {
    int msgLen = (int)strlen(msg.buffer);
    const int USB_STREAM_BLOCK_SIZE = 64;
    const uint32_t USB_SEND_TIMEOUT_MS = 100;
    uint32_t sendStart = Milliseconds();
    int offset = 0;

    while (offset < msgLen) {
        if (Milliseconds() - sendStart > USB_SEND_TIMEOUT_MS) break;
        int avail = ConnectorUsb.AvailableForWrite();
        if (avail <= 0) continue;
        int remaining = msgLen - offset;
        int toSend = remaining;
        if (toSend > avail) toSend = avail;
        if (toSend > USB_STREAM_BLOCK_SIZE) toSend = USB_STREAM_BLOCK_SIZE;
        char buf[USB_STREAM_BLOCK_SIZE + 1];
        memcpy(buf, msg.buffer + offset, toSend);
        buf[toSend] = '\0';
        ConnectorUsb.Send(buf);
        offset += toSend;
    }

    // Always terminate the line so partial messages don't
    // concatenate with subsequent ones.
    uint32_t nlStart = Milliseconds();
    while (ConnectorUsb.AvailableForWrite() < 1) {
        if (Milliseconds() - nlStart > 10) break;
    }
    if (ConnectorUsb.AvailableForWrite() >= 1) {
        ConnectorUsb.Send("\n");
    }
}
```

### What the new code does

- Writes message bytes in 64-byte blocks (matching USB Full Speed max packet size) directly
  to the USB serial port, **without any chunk headers**.
- Waits for USB buffer space between blocks, with a 100 ms total timeout (well under the
  256 ms watchdog).
- **Always sends a `\n` terminator** — even after a timeout — so partial messages never
  concatenate with subsequent ones.
- The app's `serial.py` `readline()` naturally collects all bytes until `\n`, so no
  reassembly protocol is needed. Non-chunked lines go through the existing `else` branch.

### What was removed

- The `CHUNK_SIZE = 50` constant and the entire chunked-send for-loop.
- The `CHUNK_N/T:` header format is no longer emitted by the firmware.

## Audit Checklist for Reviewing Agent

### 1. Correctness of the streaming approach
- [ ] Verify that `ConnectorUsb.Send(const char*)` accepts a null-terminated string and
      writes it to the USB transmit FIFO. Confirm it does **not** block indefinitely.
- [ ] Confirm the 64-byte block size is appropriate for the ClearCore SAMD51 USB CDC
      endpoint buffer size (USB Full Speed bulk endpoint max = 64 bytes).
- [ ] Verify that `ConnectorUsb.AvailableForWrite()` returns the number of bytes of free
      space in the transmit buffer (not something else like bytes pending).
- [ ] Check that `memcpy` is available via the existing `<string.h>` include in
      `comms_controller.h`.

### 2. Watchdog safety
- [ ] The streaming loop has a 100 ms total timeout (`USB_SEND_TIMEOUT_MS`). The watchdog
      timeout is 256 ms. Confirm that 100 ms + the rest of the main loop iteration is safely
      within 256 ms. Consider whether the watchdog is fed before/after `processTxQueue`.
- [ ] The `\n` terminator wait has a separate 10 ms timeout. Confirm this is acceptable.
- [ ] Check if there's a busy-wait concern: the `while (avail <= 0) continue;` spin inside
      the streaming loop could delay the main loop. Is there a risk of starving other tasks?

### 3. Partial message handling
- [ ] If the 100 ms timeout fires before the full message is sent, the message is truncated
      but still `\n`-terminated. The app receives a shorter-than-expected message. Confirm the
      app handles truncated status/telemetry messages gracefully (doesn't crash on parse).
- [ ] If the `\n` send itself fails (10 ms timeout, no buffer space), the message has no
      terminator. The next message's bytes will be appended to it on the wire. Assess the
      likelihood of this happening and whether a fallback is needed.

### 4. App-side chunk code (`serial.py`)
- [ ] The chunk reassembly code at lines 146–232 of `serial.py` is now dead code (firmware
      no longer sends `CHUNK_` prefixed lines). Confirm it doesn't interfere — a stray line
      starting with the literal text `CHUNK_` (unlikely but possible in error messages) would
      still enter the chunk path. Consider whether to remove or keep as dead code.
- [ ] Verify `ser.readline()` has an appropriate timeout configured so it doesn't block
      forever waiting for `\n` if a partial message is stuck.

### 5. Telemetry string length
- [ ] The telemetry message (`publishTelemetry` in `fillhead.cpp`) uses a 1024-byte stack
      buffer. Verify the actual formatted telemetry string fits within this. The injector's
      `m_telemetryBuffer` is only 256 bytes — check if `getTelemetryString()` truncates, and
      if so, whether the truncated output is still valid comma-separated key-value pairs.
- [ ] A ~600 byte telemetry message streamed in 64-byte blocks at USB Full Speed should take
      well under 100 ms. Confirm this assumption for the ClearCore hardware.

### 6. Regression risk
- [ ] The change only affects USB serial output in `processTxQueue`. UDP transmission is
      unchanged. Verify no other code paths depend on the `CHUNK_` format.
- [ ] Search for any references to `CHUNK_SIZE`, `CHUNK_`, or chunk-related logic elsewhere
      in the fillhead firmware to confirm nothing else depends on the old protocol.
- [ ] If the Pressboi firmware (`pressboi` repo) has the same chunk protocol, note that it
      has the same latent bug and should be updated separately.
