# AGENTS.md - Coding Guidelines for XRP Robotics Project

This document provides guidelines for coding agents working on the XRP Robotics MicroPython codebase. The project consists of Python files for the XRP platform, focusing on robotics control, radar integration, and OLED display management.

## Project Overview

- **Language**: MicroPython (Python 3.x compatible subset)
- **Platform**: XRP Robotics (SparkFun) with ESP32-S3
- **Key Components**: Drivetrain, IMU, Radar (HLK-LD2450), OLED display (SH1107), Encoder (Seesaw)
- **Libraries**: XRPLib, machine, sh1107, seesaw, custom XRPRadar

## Build Commands

Since this is a MicroPython project without standard build tools, building involves cross-compiling to .mpy files for deployment.

### Compile to .mpy (MicroPython bytecode)

```bash
# Compile main script
mpy-cross xrp.py

# Compile radar module
mpy-cross XRPRadar.py

# Batch compile all Python files
find . -name "*.py" -exec mpy-cross {} \;
```

### Deploy to Device

```bash
# Using ampy (Adafruit MicroPython tool)
ampy -p /dev/ttyACM0 put xrp.mpy /main.py
ampy -p /dev/ttyACM0 put XRPRadar.mpy /XRPRadar.py

# Or using rshell
rshell -p /dev/ttyACM0 cp xrp.mpy /pyboard/main.py
rshell -p /dev/ttyACM0 cp XRPRadar.mpy /pyboard/XRPRadar.py
```

### Full Build and Deploy Script

```bash
#!/bin/bash
echo "Building XRP Robotics firmware..."
mpy-cross xrp.py && mpy-cross XRPRadar.py
echo "Deploying to device..."
ampy -p /dev/ttyACM0 put xrp.mpy /main.py
ampy -p /dev/ttyACM0 put XRPRadar.mpy /XRPRadar.py
echo "Build complete."
```

## Lint Commands

Use Python linting tools adapted for MicroPython constraints.

### Pylint (with MicroPython profile)

```bash
# Basic lint
pylint --disable=import-error,unused-import xrp.py XRPRadar.py

# With MicroPython-specific disables
pylint --disable=import-error,unused-import,no-member,too-few-public-methods xrp.py XRPRadar.py
```

### Flake8

```bash
# Lint with line length 100 (suitable for small displays)
flake8 --max-line-length=100 xrp.py XRPRadar.py
```

### Pycodestyle

```bash
# Check style
pycodestyle --max-line-length=100 xrp.py XRPRadar.py
```

### Black (Code Formatting)

```bash
# Format code
black --line-length=100 xrp.py XRPRadar.py

# Check if formatting needed
black --check --line-length=100 xrp.py XRPRadar.py
```

### Combined Lint and Format

```bash
#!/bin/bash
echo "Linting and formatting..."
black --line-length=100 xrp.py XRPRadar.py
flake8 --max-line-length=100 xrp.py XRPRadar.py
pylint --disable=import-error,unused-import,no-member xrp.py XRPRadar.py
```

## Test Commands

Testing on MicroPython is limited; focus on unit tests for logic and manual testing on device.

### Run Tests (if unittest is available)

```bash
# Run all tests
python -m unittest discover -s . -p "*test*.py"

# Run specific test file
python -m unittest test_radar.py

# Run single test method
python -m unittest test_radar.TestRadar.test_parse_report
```

### Manual Testing on Device

```bash
# Connect to REPL
screen /dev/ttyACM0 115200

# Or using mpremote
mpremote repl

# Run tests interactively
import xrp
# Test functions manually
```

### Single Test Execution

```bash
# If using pytest
pytest test_radar.py::TestRadar::test_parse_report -v

# Or with unittest
python -c "import unittest; from test_radar import TestRadar; suite = unittest.TestLoader().loadTestsFromTestCase(TestRadar); unittest.TextTestRunner().run(suite)"
```

### Test Coverage

```bash
# Generate coverage report
coverage run -m unittest discover
coverage report -m
coverage html
```

## Code Style Guidelines

### General Principles

- Write clean, readable code optimized for embedded constraints (limited RAM, processing power)
- Prioritize reliability and safety for robotics applications
- Use MicroPython best practices (avoid CPython-only features)
- Comment complex logic, especially sensor data processing and safety checks

### Imports

```python
# Standard library first
import time
import math

# MicroPython specific
import machine
from machine import Pin, UART

# Third-party (alphabetical)
import sh1107
import seesaw

# Local modules
from XRPRadar import XRPRadar
```

- Group imports: standard, MicroPython, third-party, local
- Use explicit imports: `from machine import Pin` instead of `import machine`
- Avoid wildcard imports (`from module import *`)

### Formatting

- **Line length**: 100 characters max (readable on small screens)
- **Indentation**: 4 spaces (standard Python)
- **Blank lines**: 1 between functions/methods, 2 between classes
- **Trailing commas**: Use in multi-line structures for cleaner diffs
- **String quotes**: Double quotes for consistency, single for inner strings

```python
# Good
def calculate_distance(x, y):
    """Calculate Euclidean distance."""
    return math.sqrt(x**2 + y**2)

# Avoid
def calculate_distance(x,y):return math.sqrt(x**2+y**2)
```

### Naming Conventions

- **Variables**: snake_case (`radar_distance`, `current_threshold`)
- **Functions**: snake_case (`get_radar_distance()`, `draw_distance_bar()`)
- **Classes**: PascalCase (`XRPRadar`, `RadarFollower`)
- **Constants**: UPPER_SNAKE_CASE (`CURRENT_THRESHOLD = 20`)
- **Modules**: lowercase or snake_case (`xrp.py`, `xrp_radar.py`)

### Types and Type Hints

MicroPython has limited type hint support; use when possible for documentation.

```python
from typing import Tuple, Optional

def parse_radar_report(self) -> Optional[Tuple[float, ...]]:
    """Parse radar data, return tuple of target data or None."""
    # Implementation
```

### Error Handling

- Use try-except for hardware operations that might fail
- Log errors to console or display (avoid silent failures)
- Provide fallback behavior for sensor failures

```python
try:
    seesaw_device.set_led(128, 0, 0)
except Exception as e:
    print(f"LED error: {e}")  # Log but continue
```

### Comments and Docstrings

- Use docstrings for all public functions/classes
- Comment complex logic, especially sensor data interpretation
- TODO comments for future improvements

```python
def safety_drive(left_eff, right_eff, target_distance):
    """
    Drive with safety checks using radar distance.

    Args:
        left_eff: Left motor effort (-1.0 to 1.0)
        right_eff: Right motor effort (-1.0 to 1.0)
        target_distance: Target distance in encoder units

    Returns:
        None
    """
    # Implementation with inline comments for steps
```

### Performance Considerations

- Minimize memory allocation in loops
- Use global variables sparingly; prefer local scope
- Pre-calculate constants where possible
- Profile code on device using `time.ticks_ms()`

### Safety and Reliability

- Implement software shields for robot safety
- Validate sensor data before use
- Add timeouts to prevent infinite loops
- Test edge cases (no sensor data, invalid inputs)

### Code Structure

- Separate concerns: sensor reading, data processing, display, control
- Use classes for complex modules (e.g., XRPRadar)
- Keep functions short and focused
- Avoid deep nesting; use early returns

### OLED Display Guidelines

- Screen size: 128x128 pixels
- Text height: ~8 pixels per line
- Ensure all text fits without overlap (max ~15 lines)
- Use consistent Y spacing (9 pixels between lines)
- Truncate long messages: `msg[:20]`

### Version Control

- Commit frequently with descriptive messages
- Use feature branches for new functionality
- Test on hardware before merging
- Document hardware-specific changes

### Debugging

- Use `print()` for debugging (visible in REPL)
- Avoid debug prints in production code
- Test sensor values manually via REPL

This guide ensures consistent, maintainable code for the XRP Robotics project. Follow these guidelines to contribute effectively.