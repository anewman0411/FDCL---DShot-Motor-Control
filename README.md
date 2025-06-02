# FDCL---DShot-Motor-Control

Below is a suggested README section (in Markdown) that explains exactly how to compute the 4-bit CRC used by DShot (as implemented in BlueJay). You can drop this into your project’s README under an appropriate heading.

---

## CRC Calculation for DShot Packets

DShot packets embed an 11-bit command (or data) field, a 1-bit telemetry flag, and a 4-bit checksum. In BlueJay (and most DShot implementations), the checksum is computed as a simple “inverted XOR” over those 12 bits.

### 1. Form the 12-bit payload

* **Bits \[11:1]**: 11-bit command or data value (range 0 … 2047)
* **Bit 0**: telemetry request (0 = no telemetry, 1 = request telemetry)

In C pseudocode, if you already have an `uint16_t command11` (0..2047) and a `uint8_t telemetry` flag (0 or 1), you form:

```c
uint16_t payload12 = ((command11 & 0x07FF) << 1) | (telemetry & 0x01);
```

This yields a 12-bit value in the range 0x000 … 0xFFF.

### 2. Compute the 4-bit “inverted XOR” checksum

1. **XOR the high byte into the low byte**
    Let `csum = payload12 ^ (payload12 >> 8)`.
   Because `payload12` is 12 bits, `(payload12 >> 8)` effectively isolates bits \[11:8] (the “upper nibble”).

2. **XOR the upper nibble into the lower nibble**
    Update `csum = csum ^ (csum >> 4)`.
   Now the low nibble of `csum` has influenced all higher bits, and the top nibble of the original 12-bit word has been folded down.

3. **Invert and mask to 4 bits**
    Finally, do `crc4 = (~csum) & 0x0F`.
   The result `crc4` is a number from 0 … 15 (0x0 … 0xF).

By construction, when you decode a received packet you can recompute those same steps and confirm that the inverted‐XOR result matches the lower 4 bits you actually received. If it doesn’t match, the packet is corrupt.

### 3. Assemble the full 16-bit DShot packet

Once you have `payload12` and its 4-bit checksum (`crc4`), place the checksum in the lowest nibble and the payload in the upper 12 bits:

```c
uint16_t packet16 = (payload12 << 4) | (crc4 & 0x0F);
/* Bits [15:4] = payload12
   Bits  [3:0] = crc4 */
```

When you send `packet16` out (via PWM/DMA or bitbang), the ESC will interpret the first 12 bits as the command+telemetry flag and check the last 4 bits against its own CRC routine.

---

## Example Code Snippet (Python)

If you want a quick reference or unit‐test harness, here is the same logic expressed in Python. Drop this into any test script or leverage it for offline verification:

```python
def dshot_crc4(payload12: int) -> int:
    """
    Compute the 4-bit inverted-XOR checksum for a 12-bit value.

    Arguments:
        payload12: int (0..0xFFF) — 12-bit command/data + telemetry

    Returns:
        int (0..0xF) — 4-bit inverted-XOR CRC
    """
    # Ensure only low 12 bits are used
    value = payload12 & 0x0FFF

    # Step 1: XOR upper byte into lower byte
    csum = value ^ (value >> 8)

    # Step 2: XOR upper nibble into lower nibble
    csum = csum ^ (csum >> 4)

    # Step 3: Invert and keep only 4 bits
    crc4 = (~csum) & 0x0F
    return crc4


def make_dshot_packet(command11: int, telemetry: int) -> int:
    """
    Form a 16-bit DShot packet: [11 bits data][1 bit telemetry][4 bits CRC].

    Arguments:
        command11: int (0..2047) — 11-bit command or data value
        telemetry:  int (0 or 1)      — telemetry request flag

    Returns:
        int (0..0xFFFF) — full 16-bit DShot packet
    """
    # Build 12-bit payload
    payload12 = ((command11 & 0x07FF) << 1) | (telemetry & 0x01)

    # Compute checksum
    crc4 = dshot_crc4(payload12)

    # Place CRC in the low nibble
    packet16 = (payload12 << 4) | (crc4 & 0x0F)
    return packet16


# Example usage:
if __name__ == "__main__":
    # 1) Send throttle=1000, no telemetry
    pkt = make_dshot_packet(command11=1000, telemetry=0)
    print(f"DShot packet (hex): 0x{pkt:04X}")

    # 2) Request telemetry only (command=0, telemetry=1)
    pkt_tlm = make_dshot_packet(command11=0, telemetry=1)
    print(f"Telemetry packet (hex): 0x{pkt_tlm:04X}")
```

---

## Worked Example

* **Input bits = `0x000`** (all zeros in 12 bits: `0000 0000 0000`)

  1. `csum = 0x000 ^ (0x000 >> 8) = 0x000 ^ 0x000 = 0x000`
  2. `csum = 0x000 ^ (0x000 >> 4) = 0x000 ^ 0x000 = 0x000`
  3. `crc4 = (~0x000) & 0x0F = 0xFFF...F & 0x0F = 0x0F`

  So the 4-bit CRC for `0x000` is `0xF`.
  The full 16-bit packet becomes:

  ```
  payload12 = 0x000
     crc4  = 0xF
  packet16 = (0x000 << 4) | 0x0F = 0x000F
  ```

* **Input bits = `0x123`** (example 12-bit payload)

  1. `csum = 0x123 ^ (0x123 >> 8) = 0x123 ^ 0x001 = 0x122`
  2. `csum = 0x122 ^ (0x122 >> 4) = 0x122 ^ 0x012 = 0x130`
  3. `crc4 = (~0x130) & 0x0F = 0x...EDF & 0x0F = 0x0D`

  Full packet: `packet16 = (0x123 << 4) | 0x0D = 0x1230 | 0x000D = 0x123D`.

---

### Summary

1. **Build your 12-bit payload** by combining 11 bits of command/data and 1 bit of telemetry.
2. **Compute the inverted-XOR checksum**:

   * `csum = payload12 ^ (payload12 >> 8)`
   * `csum = csum ^ (csum >> 4)`
   * `crc4 = (~csum) & 0x0F`
3. **Append CRC to form the 16-bit word**: `(payload12 << 4) | crc4`.

Refer to the Python snippet above (or your own C version) whenever you need to confirm that your bit-banging, DMA-driven PWM, or any other DShot output routine is generating the correct CRC. Once the ESC sees a matching CRC, it will accept the packet and act on the command (or send back telemetry if requested).
