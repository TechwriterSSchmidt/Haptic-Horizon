import sys
import struct
import os

def convert_hex_to_uf2(hex_content):
    # UF2 Constants
    UF2_MAGIC_START0 = 0x0A324655
    UF2_MAGIC_START1 = 0x9E5D5157
    UF2_MAGIC_END    = 0x0AB16F30
    UF2_FLAG_FAMILY_ID_PRESENT = 0x00002000
    NRF52840_FAMILY_ID = 0xADA52840

    # Parse HEX
    data = {}
    upper_addr = 0

    for line in hex_content.splitlines():
        line = line.strip()
        if not line.startswith(':'): continue
        
        byte_count = int(line[1:3], 16)
        addr = int(line[3:7], 16)
        record_type = int(line[7:9], 16)
        checksum = int(line[-2:], 16)
        
        if record_type == 0: # Data
            current_addr = (upper_addr << 16) + addr
            payload = bytes.fromhex(line[9:-2])
            for i, b in enumerate(payload):
                data[current_addr + i] = b
        elif record_type == 1: # EOF
            break
        elif record_type == 2: # Extended Segment Address
            upper_addr = int(line[9:-2], 16)
        elif record_type == 4: # Extended Linear Address
            upper_addr = int(line[9:-2], 16)

    # Convert to UF2 Blocks
    uf2_blocks = []
    if not data:
        return b""

    min_addr = min(data.keys())
    max_addr = max(data.keys())
    
    # Align to 256 bytes
    num_blocks = (max_addr - min_addr + 256) // 256
    
    # We need to group data into 256 byte chunks
    # But data might be sparse.
    # Standard UF2 converters fill gaps with 0 or skip?
    # Usually we iterate through 256-byte aligned addresses.
    
    start_addr = min_addr & ~0xFF
    end_addr = (max_addr + 0xFF) & ~0xFF
    
    blocks = []
    
    curr = start_addr
    while curr < end_addr:
        payload = bytearray(476)
        has_data = False
        for i in range(256):
            if (curr + i) in data:
                payload[i] = data[curr + i]
                has_data = True
        
        if has_data:
            blocks.append((curr, payload))
        
        curr += 256

    total_blocks = len(blocks)
    out_data = bytearray()

    for i, (addr, payload) in enumerate(blocks):
        # Struct format:
        # I (Magic0)
        # I (Magic1)
        # I (Flags)
        # I (Target Addr)
        # I (Payload Size)
        # I (Block No)
        # I (Num Blocks)
        # I (Family ID)
        # 476s (Data)
        # I (Magic End)
        
        head = struct.pack(
            "<IIIIIIII",
            UF2_MAGIC_START0,
            UF2_MAGIC_START1,
            UF2_FLAG_FAMILY_ID_PRESENT,
            addr,
            256, # Payload size
            i,
            total_blocks,
            NRF52840_FAMILY_ID
        )
        
        block = head + payload + struct.pack("<I", UF2_MAGIC_END)
        out_data.extend(block)

    return out_data

try:
    Import("env")
except ImportError:
    pass

def after_build(source, target, env):
    hex_path = str(source[0])
    uf2_path = hex_path.replace(".hex", ".uf2")
    
    # Fix output path naming
    uf2_path = hex_path.replace(".hex", ".uf2")
    
    print(f"Generating UF2 from {hex_path} to {uf2_path}")
    
    # Use 'utf-8' or 'latin-1' to avoid charmap errors, though hex should be ASCII.
    # If the file contains binary garbage, 'latin-1' is safer.
    # Also, the source[0] passed by SCons might be the ELF file, not HEX, depending on the rule.
    # Let's verify the extension.
    
    src_path = str(source[0])
    if not src_path.endswith(".hex"):
        # Try to find the hex file in the same dir
        hex_candidate = os.path.splitext(src_path)[0] + ".hex"
        if os.path.exists(hex_candidate):
            hex_path = hex_candidate
        else:
            print(f"Skipping UF2 generation: Source {src_path} is not a .hex file")
            return

    with open(hex_path, "r", encoding="latin-1") as f:
        hex_content = f.read()
        
    uf2_content = convert_hex_to_uf2(hex_content)
    
    with open(uf2_path, "wb") as f:
        f.write(uf2_content)
    
    print(f"UF2 Generated: {uf2_path}")

# Hook into PlatformIO
try:
    Import("env")
    env.AddPostAction("$BUILD_DIR/${PROGNAME}.hex", after_build)
except ImportError:
    # Standalone testing
    pass
