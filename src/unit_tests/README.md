# Unit Tests

Unit tests for the rm_vision_project modules using Google Test (GTest).

## Tested Modules

| Test File | Module | What's Tested |
|-----------|--------|---------------|
| `test_crc.cpp` | `rm_serial_driver` | CRC16 checksum computation, verification, and append |
| `test_packet.cpp` | `rm_serial_driver` | Packet struct layout, `toVector`/`fromVector` serialization |
| `test_strip.cpp` | `img_processing` | Strip geometry: point sorting, endpoint calculation, line fitting |

## Prerequisites

```bash
sudo apt-get install -y libgtest-dev cmake libopencv-dev libeigen3-dev
cd /usr/src/gtest && sudo cmake . && sudo make && sudo cp lib/*.a /usr/lib/
```

## Build & Run

```bash
cd src/unit_tests
mkdir build && cd build
cmake ..
make -j$(nproc)

# Run all tests
./test_crc
./test_packet
./test_strip

# Or use the combined target
make run_all_tests
```

## Coverage Analysis

These tests target the modules with **zero prior test coverage**:

- **`rm_serial_driver/src/crc.cpp`** — Pure computational CRC16 functions critical for serial communication integrity.
- **`rm_serial_driver/include/rm_serial_driver/packet.hpp`** — Packet serialization/deserialization used for MCU communication.
- **`img_processing/src/strip.cpp`** — Geometric algorithms for light strip detection in armor plate recognition.
