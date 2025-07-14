# Encoder Controller Enhancements

## Overview
This document describes the implementation of two key enhancements to the EncoderController module:
1. **Dynamic Velocity Adjustment** - Automatic velocity scaling based on encoder rotation speed
2. **Timeout Safety** - Automatic disable after 5 minutes of inactivity

## 1. Dynamic Velocity Adjustment

### Purpose
Provides more intuitive MPG control by automatically adjusting motor velocity based on how fast the operator turns the encoder wheel.

### How It Works
- **Slow Encoder Rotation**: Motor moves at reduced velocity (50-100% of base) for precise positioning
- **Fast Encoder Rotation**: Motor moves at increased velocity (100-300% of base) for rapid positioning
- **Exponential Smoothing**: Prevents jerky movement by smoothing velocity changes over time

### Anti-Jerk Tuning Parameters

#### `ENCODER_VELOCITY_SMOOTHING_FACTOR = 4`
- **Purpose**: Controls how quickly velocity changes respond to encoder speed changes
- **Higher Values (6-10)**: Smoother, slower response - prevents jerkiness but less responsive
- **Lower Values (1-3)**: Faster response - more immediate but potentially jerky
- **Current Setting**: Balanced for smooth yet responsive control

#### `ENCODER_VELOCITY_THRESHOLD_CPS = 2`
- **Purpose**: Defines the boundary between "slow" and "fast" encoder operation
- **Below 2 counts/sec**: Precision mode (reduced velocity 50-100%)
- **Above 2 counts/sec**: Rapid mode (increased velocity 100-300%)
- **Tuning**: Increase for more precision range, decrease for more rapid range

#### Velocity Scaling Curve
```
Encoder Speed (cps) | Velocity Scale | Motor Behavior
0-2                 | 0.5x - 1.0x    | Precision mode
2-12                | 1.0x - 3.0x    | Rapid positioning
12+                 | 3.0x (max)     | Maximum speed
```

### Configuration Constants
```cpp
#define ENCODER_VELOCITY_SMOOTHING_FACTOR 4    // Smoothing (1-10, higher = smoother)
#define ENCODER_MIN_VELOCITY_SCALE 0.5         // Minimum velocity (50% of base)
#define ENCODER_MAX_VELOCITY_SCALE 3.0         // Maximum velocity (300% of base)
#define ENCODER_VELOCITY_THRESHOLD_CPS 2       // Precision/rapid boundary
```

## 2. Timeout Safety

### Purpose
Automatically disables encoder control after 5 minutes of inactivity to prevent accidental motor movement and reduce wear.

### How It Works
- **Activity Tracking**: Monitors last encoder movement timestamp
- **Periodic Checking**: Checks timeout status every 10 seconds (not every loop cycle)
- **Automatic Disable**: Gracefully disables MPG control with warning message
- **Status Display**: Shows remaining timeout in encoder status

### Configuration Constants
```cpp
#define ENCODER_TIMEOUT_MS 300000              // 5 minutes timeout
#define ENCODER_ACTIVITY_CHECK_INTERVAL_MS 10000  // Check every 10 seconds
```

## Standard MPG Multipliers

### Command Arguments (Industry Standard)
| Command | Internal Constant | Movement | Use Case |
|---------|------------------|----------|----------|
| `encoder multiplier 1` | `MULTIPLIER_X1_SCALED` | 0.1mm/count | Fine positioning |
| `encoder multiplier 10` | `MULTIPLIER_X10_SCALED` | 1.0mm/count | General use |
| `encoder multiplier 100` | `MULTIPLIER_X100_SCALED` | 10.0mm/count | Rapid movement |

### Examples
```
encoder multiplier 1     ← Fine control (0.1mm per encoder count)
encoder multiplier 10    ← General control (1.0mm per encoder count) [DEFAULT]
encoder multiplier 100   ← Rapid control (10.0mm per encoder count)
```

## Implementation Details

### New Global Variables
```cpp
unsigned long lastEncoderActivity;    // Last movement timestamp
unsigned long lastTimeoutCheck;       // Last timeout check timestamp
float currentVelocityScale;           // Current velocity multiplier
int32_t smoothedEncoderVelocity;      // Smoothed encoder speed (cps)
```

### Enhanced Status Output
- Shows dynamic velocity scaling when active (e.g., "Dynamic: 2.1x")
- Displays remaining timeout in seconds
- Provides encoder velocity diagnostics

### Diagnostic Logging
- Real-time velocity scaling information during movement
- Timeout warnings with clear safety messaging
- Encoder velocity measurements for tuning

## Benefits

### Dynamic Velocity Adjustment
✅ **Intuitive Control**: Matches user expectations from professional CNC systems
✅ **Dual-Purpose**: Single encoder serves both precision and rapid positioning
✅ **Smooth Operation**: Anti-jerk smoothing prevents motor oscillation
✅ **Configurable**: Easy tuning via constants for different user preferences

### Timeout Safety
✅ **Safety**: Prevents accidental motor movement from forgotten active MPG
✅ **Equipment Protection**: Reduces motor wear from unintended operation
✅ **Energy Efficiency**: Automatic disable reduces system load
✅ **User Awareness**: Clear status indication of remaining time

## Tuning Recommendations

### For Smoother Response (Less Jerky)
- Increase `ENCODER_VELOCITY_SMOOTHING_FACTOR` to 6-8
- Reduce `ENCODER_MAX_VELOCITY_SCALE` to 2.0-2.5
- Increase `ENCODER_VELOCITY_THRESHOLD_CPS` to 3-4

### For More Responsive Control
- Decrease `ENCODER_VELOCITY_SMOOTHING_FACTOR` to 2-3
- Keep current `ENCODER_MAX_VELOCITY_SCALE` at 3.0
- Keep current `ENCODER_VELOCITY_THRESHOLD_CPS` at 2

### For Different Timeout Periods
- Shorter timeout: Reduce `ENCODER_TIMEOUT_MS` to 120000 (2 minutes)
- Longer timeout: Increase `ENCODER_TIMEOUT_MS` to 600000 (10 minutes)

## Compatibility
- ✅ Fully compatible with existing Commands.cpp interface
- ✅ No changes required to external function calls
- ✅ Backward compatible with all existing functionality
- ✅ Maintains all safety checks and error handling

## Testing Recommendations
1. Test slow encoder rotation for precision positioning
2. Test fast encoder rotation for rapid movement
3. Verify smooth transitions between speed ranges
4. Confirm timeout triggers after 5 minutes of inactivity
5. Check status display shows correct velocity scaling and timeout
