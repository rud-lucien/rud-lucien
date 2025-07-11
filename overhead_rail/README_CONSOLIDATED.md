# Overhead Rail System - Dual-Rail Motor Control System

A comprehensive Arduino-based control system for dual overhead rail transportation using ClearCore motor drivers. This system provides complete motor control, positioning, homing, pneumatic valve control, sensor monitoring, and manual operation capabilities for two independent rail systems.

## Table of Contents
- [System Overview](#system-overview)
- [Hardware Configuration](#hardware-configuration)
- [Quick Start](#quick-start)
- [Motor Control](#motor-control)
- [Pneumatic Valve Control](#pneumatic-valve-control)
- [Sensor System](#sensor-system)
- [Manual Control (MPG/Handwheel)](#manual-control-mpghandwheel)
- [Logging System](#logging-system)
- [Velocity Configuration](#velocity-configuration)
- [Command Interface](#command-interface)
- [Safety Features](#safety-features)
- [Position Reference](#position-reference)
- [API Reference](#api-reference)
- [File Structure](#file-structure)
- [Dependencies](#dependencies)

---

## System Overview

### Key Features
- ✅ **Dual-motor support** with independent control (Rail 1: 8.2m, Rail 2: 1m)
- ✅ **Automatic homing** with hardstop detection
- ✅ **Position-based movements** with predefined locations
- ✅ **Manual jogging** with safety limits
- ✅ **Load-aware velocity** selection (loaded vs empty carriage)
- ✅ **Pneumatic valve control** with position validation
- ✅ **Comprehensive sensor monitoring** for carriages and labware
- ✅ **Manual Pulse Generator (MPG)** handwheel control
- ✅ **Real-time logging** with color-coded output
- ✅ **Advanced fault management** with automatic recovery
- ✅ **Emergency stop** monitoring and response
- ✅ **Ethernet interface** for remote control and monitoring

---

## Hardware Configuration

### Motors and Rails
- **Rail 1**: 8.2m travel distance with 5 defined positions (ClearCore ConnectorM0)
- **Rail 2**: 1m travel distance with 3 defined positions (ClearCore ConnectorM1)
- **Homing**: Hardstop detection with automatic offset positioning

### Pneumatic System
- **Cylinder Control**: Extend/retract pneumatic drive
- **Position Validation**: Sensor feedback for position confirmation
- **Pressure Monitoring**: Real-time pressure monitoring with warnings

### Sensors
- **Carriage Detection**: Optical sensors at each work cell and handoff position
- **Labware Detection**: Sensors to detect presence of plates/labware
- **Pressure Monitoring**: Pneumatic system pressure feedback
- **CCIO-8 Integration**: Expandable I/O for additional sensors

### Manual Control
- **CL-ENCRD-DFIN Encoder Adapter**: For handwheel integration
- **Quadrature Encoder**: Physical handwheel for manual positioning
- **Variable Precision**: 0.1mm, 1.0mm, or 10.0mm per handwheel count

### Safety Systems
- **Emergency Stop**: Hardware E-stop monitoring with immediate shutdown
- **Travel Limits**: Software-enforced position boundaries
- **Fault Detection**: Comprehensive motor and system fault monitoring

---

## Quick Start

### Basic Initialization
```cpp
#include "MotorController.h"
#include "Sensors.h"
#include "ValveController.h"
#include "Logging.h"

void setup() {
    Serial.begin(115200);
    
    // Initialize all systems
    initMotorSystem();
    initSensorSystem(true);    // true = CCIO board available
    initValveSystem(true);     // true = CCIO board available
    initEncoderControl(true, false);
    
    // Home both motors
    initiateHomingSequenceAll();
    while (!isAllHomingComplete()) {
        checkAllHomingProgress();
        handleEStop();
        delay(10);
    }
    
    // Enable logging
    logging.logInterval = 250;  // Log every 250ms
}

void loop() {
    handleEStop();              // Critical safety monitoring
    checkAllHomingProgress();   
    processAllFaultClearing();  
    updateAllSensors();
    processEncoderInput();
    
    // Periodic logging
    if (logging.logInterval > 0 && 
        waitTimeReached(millis(), logging.previousLogTime, logging.logInterval)) {
        logging.previousLogTime = millis();
        logSystemState();
    }
}
```

### Basic Movement Examples
```cpp
// Move to predefined positions
moveToPositionFromCurrent(1, RAIL1_WC2_PICKUP_DROPOFF_POS, true);  // Rail 1 to WC2, loaded
moveToPositionFromCurrent(2, RAIL2_WC3_PICKUP_DROPOFF_POS, false); // Rail 2 to WC3, empty

// Move to specific coordinates
moveToPositionMm(1, 3700.0, true);    // Rail 1 to 3700mm, loaded carriage
moveToPositionMm(2, 750.0, false);    // Rail 2 to 750mm, empty carriage

// Pneumatic control
extendValve();                         // Extend pneumatic cylinder
retractValve();                        // Retract pneumatic cylinder
```

---

## Motor Control

### System Initialization and Safety

#### Core Functions
```cpp
initMotorSystem();                     // Initialize both motors
handleEStop();                         // Monitor E-stop (call in main loop)
isEStopActive();                       // Check E-stop status
isMotorReady(int rail);               // Check motor readiness
isMotorMoving(int rail);              // Check if motor is moving
getMotorPositionMm(int rail);         // Get current position in mm
stopMotion(int rail);                 // Stop specific motor
stopAllMotion();                      // Emergency stop all motors
```

### Position-Based Movement

#### Predefined Positions
```cpp
// Move to named positions with load consideration
moveToPositionFromCurrent(int rail, PositionTarget toPos, bool carriageLoaded);

// Examples:
moveToPositionFromCurrent(1, RAIL1_HOME_POS, false);
moveToPositionFromCurrent(1, RAIL1_WC2_PICKUP_DROPOFF_POS, true);
moveToPositionFromCurrent(2, RAIL2_WC3_PICKUP_DROPOFF_POS, false);
```

**Available Positions:**
- **Rail 1**: `HOME`, `WC2_PICKUP_DROPOFF`, `WC1_PICKUP_DROPOFF`, `STAGING`, `HANDOFF`
- **Rail 2**: `HOME`, `HANDOFF`, `WC3_PICKUP_DROPOFF`

#### Coordinate-Based Movement
```cpp
// Move to specific coordinates in millimeters
moveToPositionMm(int rail, double positionMm, bool carriageLoaded);
moveRelativeManual(int rail, double relativeMm, bool carriageLoaded);

// Examples:
moveToPositionMm(1, 3700.0, true);      // Rail 1 to 3700mm, loaded
moveRelativeManual(2, 50.0, false);     // Rail 2 forward 50mm, empty
```

### Homing Operations

#### Individual and Dual Motor Homing
```cpp
// Individual motor homing
initiateHomingSequence(int rail);
checkHomingProgress(int rail);
isHomingInProgress(int rail);
isHomingComplete(int rail);

// Dual motor homing
initiateHomingSequenceAll();
checkAllHomingProgress();
isAllHomingComplete();
```

### Manual Jogging

#### Jog Operations
```cpp
// Basic jogging with configurable parameters
jogMotor(int rail, bool direction, double customIncrement, bool carriageLoaded);

// Jog configuration
setJogIncrement(int rail, double increment);
setJogSpeed(int rail, int speedRpm);
getJogIncrement(int rail);
getJogSpeed(int rail);
```

### Fault Management

#### Fault Detection and Recovery
```cpp
// Fault management
clearMotorFaults(int rail);
processAllFaultClearing();
hasMotorFault(int rail);
isFaultClearingInProgress(int rail);
clearMotorFaultWithStatus(int rail);   // Immediate status return
```

---

## Pneumatic Valve Control

### Basic Valve Operations
```cpp
// Valve control
extendValve();                         // Extend pneumatic cylinder
retractValve();                        // Retract pneumatic cylinder
getValvePosition();                    // Get current valve state

// Position validation
isCylinderRetracted();                 // Hardware sensor feedback
isCylinderExtended();                  // Hardware sensor feedback
isCylinderActuallyRetracted();         // Validated position
isCylinderActuallyExtended();          // Validated position
validateValvePosition();               // Position/sensor match check
```

### Safety Features
- **Sensor Validation**: Hardware feedback confirms valve position
- **Position Mismatch Detection**: Alerts when commanded position doesn't match sensors
- **Pressure Monitoring**: Ensures sufficient system pressure for operation

---

## Sensor System

### Carriage and Labware Detection
```cpp
// Carriage position sensors
isCarriageAtWC1();                     // Rail 1 - Work Cell 1
isCarriageAtWC2();                     // Rail 1 - Work Cell 2  
isCarriageAtWC3();                     // Rail 2 - Work Cell 3
isCarriageAtRail1Handoff();            // Rail 1 - Handoff position
isCarriageAtRail2Handoff();            // Rail 2 - Handoff position

// Labware detection sensors
isLabwarePresentAtWC1();               // Plate at Work Cell 1
isLabwarePresentAtWC2();               // Plate at Work Cell 2
isLabwarePresentAtWC3();               // Plate at Work Cell 3
isLabwarePresentAtHandoff();           // Plate at handoff position

// System monitoring
updateAllSensors();                    // Update all sensor readings
getPressurePsi();                      // Current system pressure
isPressureSufficient();                // Pressure adequacy check
isPressureWarningLevel();              // Low pressure warning
```

### Sensor Integration
- **CCIO-8 Expansion**: Support for additional I/O via CCIO boards
- **Real-time Updates**: Continuous sensor monitoring in main loop
- **Error Detection**: Sensor fault detection and reporting

---

## Manual Control (MPG/Handwheel)

### Encoder System Setup
```cpp
// Initialize encoder system
initEncoderControl(bool swapDirection, bool indexInverted);

// Enable/disable control
enableEncoderControl(int rail);        // Control specific rail
disableEncoderControl();               // Disable handwheel control
isEncoderControlActive();              // Check if active
getActiveEncoderRail();                // Get controlled rail (1, 2, or 0)
```

### Precision Control
```cpp
// Set precision multipliers
setEncoderMultiplier(0.1);             // x1: Fine (0.1mm per count)
setEncoderMultiplier(1.0);             // x10: Medium (1.0mm per count)
setEncoderMultiplier(10.0);            // x100: Coarse (10.0mm per count)

// Velocity control
setEncoderVelocity(150);               // Set movement speed (50-400 RPM)
getEncoderVelocity();                  // Get current velocity setting
```

### Encoder Features
- **Real-time Response**: 50Hz update rate for smooth control
- **Travel Limit Protection**: Automatic boundary enforcement
- **Quadrature Error Detection**: Automatic error recovery
- **Rail Switching**: Easy switching between Rail 1 and Rail 2
- **Safety Integration**: Auto-disable on faults or E-stop

### Encoder Status and Diagnostics
```cpp
printEncoderStatus();                  // Comprehensive status display
hasQuadratureError();                  // Check for encoder errors
clearQuadratureError();                // Reset and clear errors
processEncoderInput();                 // Process handwheel input (call in loop)
```

---

## Logging System

### Real-Time System Monitoring

The logging system provides comprehensive, color-coded monitoring of the entire dual-rail system with ANSI terminal colors for enhanced readability.

#### Logged Parameters
- **Valve States**: Pneumatic cylinder position with sensor validation
- **Sensors**: All carriage and labware sensors for both rails
- **Motor Status**: State, homing status, HLFB for both rails
- **Position Data**: Current position, targets, travel information
- **Velocity Data**: Current velocity, limits, utilization percentage
- **System Status**: E-stop, air pressure, network client connections
- **MPG Status**: Manual pulse generator state and active rail

#### Color Coding System
- 🟢 **Green**: Normal/healthy states (homed, sensors active, sufficient pressure)
- 🟡 **Yellow**: Active states (moving, extended, not homed)
- 🔴 **Red**: Problem states (faults, low pressure, E-stop triggered)
- 🔵 **Cyan**: Section headers for easy scanning
- ⚠️ **Red [!]**: Critical position/sensor mismatches

### Command Interface

#### Basic Logging Commands
```bash
log on [interval]        # Enable periodic logging (100-60000ms, default: 250ms)
log off                  # Disable periodic logging
log now                  # Log current system state immediately
```

#### History and Diagnostics
```bash
log history              # Show complete operation log history
log errors               # Show only errors and warnings
log last [count]         # Show last N entries (default: 10, max: 50)
log stats                # Show log buffer statistics and status
log help                 # Display detailed help information
```

#### Example Log Output
```
[LOG] Valves: Cylinder=RETRACTED | Sensors: R1-WC1=ABSENT, R1-WC1-Lab=ABSENT, R1-WC2=PRESENT, R1-WC2-Lab=PRESENT, R1-HANDOFF=ABSENT, R2-WC3=PRESENT, R2-WC3-Lab=ABSENT, R2-HANDOFF=ABSENT, HANDOFF-Lab=ABSENT | System: E-Stop=RELEASED, Pressure=45.25 PSI, Clients=1 | R1-Motor: State=IDLE, Homed=YES, HLFB=ASSERTED | R1-Position: 3700.00mm (54824 counts), Target=None, LastTarget=5700.00mm | R1-Velocity: 0.0RPM, Limits: 800RPM/2500RPM/s | R2-Motor: State=MOVING, Homed=YES, HLFB=NOT_ASSERTED | R2-Position: 650.00mm (9635 counts), Target=Moving..., LastTarget=None | R2-Velocity: 250.0RPM (42%), Limits: 600RPM/2500RPM/s | MPG: ON x10 (10.00mm/rot) on Rail 1
```

### Memory Optimization

The logging system uses memory-efficient patterns:
- **PROGMEM Storage**: Format strings stored in flash memory
- **sprintf_P()**: Flash-based string formatting
- **Streaming Output**: Direct printing without large buffers
- **Modular Architecture**: Separate functions for each log section

---

## Velocity Configuration

### Automatic Velocity Selection

The system automatically selects appropriate velocities based on multiple factors:

#### 1. Rail-Specific Load-Dependent Velocities
For normal positioning movements where labware presence affects speed:

```cpp
// Rail 1 velocities
RAIL1_LOADED_CARRIAGE_VELOCITY_RPM    = 325 RPM    // Conservative with load
RAIL1_EMPTY_CARRIAGE_VELOCITY_RPM     = 800 RPM    // Faster when empty

// Rail 2 velocities  
RAIL2_LOADED_CARRIAGE_VELOCITY_RPM    = 250 RPM    // Conservative with load
RAIL2_EMPTY_CARRIAGE_VELOCITY_RPM     = 600 RPM    // Faster when empty

// Access via function
int32_t velocity = getCarriageVelocityRpm(rail, carriageLoaded);
```

#### 2. Homing Operation Velocities
Conservative, precise movement during homing sequences:
```cpp
HOME_APPROACH_VELOCITY_RPM = 40 RPM    // Slow, precise homing speed
```

#### 3. Jogging Operation Velocities
Manual control with user-configurable speeds:
```cpp
// Default jog speeds (user-configurable)
RAIL1_DEFAULT_JOG_SPEED_RPM = 200 RPM
RAIL2_DEFAULT_JOG_SPEED_RPM = 150 RPM

// Default jog increments
RAIL1_DEFAULT_JOG_INCREMENT_MM = 10.0 mm
RAIL2_DEFAULT_JOG_INCREMENT_MM = 5.0 mm
```

#### 4. Encoder/MPG Operation Velocities
Manual pulse generator control with configurable speeds:
```cpp
ENCODER_MIN_VELOCITY_RPM     = 50 RPM     // Minimum handwheel speed
ENCODER_MAX_VELOCITY_RPM     = 400 RPM    // Maximum handwheel speed  
ENCODER_DEFAULT_VELOCITY_RPM = 200 RPM    // Default handwheel speed
```

### Velocity Selection Logic

#### Positioning Functions
```cpp
// Use load-dependent rail-specific speeds
int32_t velocity = getCarriageVelocityRpm(rail, carriageLoaded);
```

#### Homing Functions
```cpp
// Use fixed conservative homing speed
int32_t velocity = rpmToPps(HOME_APPROACH_VELOCITY_RPM);
```

#### Jogging Functions
```cpp
// Use user-configurable jog speed
int32_t velocity = rpmToPps(getJogSpeedRef(rail));
```

#### Encoder/MPG Functions
```cpp
// Use encoder module's velocity management
int32_t velocity = rpmToPps(currentVelocityRpm);
```

---

## Command Interface

### Motor Control Commands
```bash
# Homing operations
home 1                   # Home Rail 1
home 2                   # Home Rail 2  
home all                 # Home both rails

# Position movements
move 1 wc2 loaded        # Rail 1 to WC2 with loaded carriage
move 2 wc3 empty         # Rail 2 to WC3 with empty carriage
move 1 3700.5 loaded     # Rail 1 to 3700.5mm with loaded carriage

# Manual jogging
jog 1 forward 50         # Rail 1 forward 50mm
jog 2 backward 10        # Rail 2 backward 10mm

# Status and diagnostics
status 1                 # Rail 1 status
status all               # Both rails status
position 1               # Rail 1 position info
```

### Pneumatic Control Commands
```bash
# Valve control
valve extend             # Extend pneumatic cylinder
valve retract            # Retract pneumatic cylinder
valve status             # Show valve position and sensors
```

### Sensor Commands
```bash
# Sensor monitoring
sensors                  # Show all sensor readings
pressure                 # Show system pressure
```

### MPG/Encoder Commands
```bash
# Encoder control
mpg enable 1             # Enable handwheel control for Rail 1
mpg enable 2             # Enable handwheel control for Rail 2
mpg disable              # Disable handwheel control
mpg precision 1          # Set fine precision (0.1mm/count)
mpg precision 10         # Set medium precision (1.0mm/count)
mpg precision 100        # Set coarse precision (10.0mm/count)
mpg velocity 150         # Set handwheel velocity to 150 RPM
mpg status               # Show encoder status
```

---

## Safety Features

### Emergency Stop System
- **Hardware Integration**: Direct E-stop input monitoring
- **Immediate Response**: Instant motion halt and motor disable
- **State Preservation**: System maintains fault state until manually cleared
- **Recovery Process**: Structured fault clearing and re-enable sequence

### Travel Limits
- **Rail 1**: 0-8000mm (software enforced)
- **Rail 2**: 0-1000mm (software enforced)
- **Boundary Checking**: Prevents movement beyond safe limits
- **Position Validation**: Continuous position monitoring

### Fault Protection
- **Motor Alert Detection**: Comprehensive motor fault monitoring
- **Automatic Recovery**: Structured fault clearing procedures
- **Move Prevention**: Blocks operations during fault conditions
- **Status Reporting**: Detailed fault information and recovery status

### Sensor Validation
- **Position Confirmation**: Hardware sensors validate pneumatic positions
- **Mismatch Detection**: Alerts when sensors don't match expected states
- **Labware Detection**: Prevents unsafe operations without proper load detection

---

## Position Reference

### Rail 1 Positions (8200mm rail, 8000mm usable)

| Position | Name | Distance | Description |
|----------|------|----------|-------------|
| `RAIL1_HOME_POS` | Home | 0mm | Reference/start position |
| `RAIL1_WC2_PICKUP_DROPOFF_POS` | Work Cell 2 | 3700mm | Pickup/dropoff at WC2 |
| `RAIL1_WC1_PICKUP_DROPOFF_POS` | Work Cell 1 | 5700mm | Pickup/dropoff at WC1 |
| `RAIL1_STAGING_POS` | Staging | 7500mm | Intermediate staging area |
| `RAIL1_HANDOFF_POS` | Handoff | 8000mm | Transfer to Rail 2 |

### Rail 2 Positions (1000mm rail)

| Position | Name | Distance | Description |
|----------|------|----------|-------------|
| `RAIL2_HOME_POS` | Home | 0mm | Reference/start position |
| `RAIL2_HANDOFF_POS` | Handoff | 500mm | Transfer from Rail 1 |
| `RAIL2_WC3_PICKUP_DROPOFF_POS` | Work Cell 3 | 900mm | Pickup/dropoff at WC3 |

### Coordinate System
- **Origin**: Home position (0mm) for each rail
- **Direction**: Positive direction away from motor
- **Resolution**: ~0.135mm per motor step (7400 pulses/mm)
- **Accuracy**: ±0.1mm positioning accuracy

---

## API Reference

### Core System Functions

#### Initialization
```cpp
void initMotorSystem();                // Initialize motor controllers
void initSensorSystem(bool hasCCIO);   // Initialize sensor system
void initValveSystem(bool hasCCIO);    // Initialize pneumatic system
void initEncoderControl(bool swap, bool invert);  // Initialize handwheel
```

#### Safety and Monitoring
```cpp
void handleEStop();                    // Emergency stop monitoring
bool isEStopActive();                  // Check E-stop status
void updateAllSensors();               // Update all sensor readings
void processEncoderInput();            // Process handwheel input
```

#### Movement and Positioning
```cpp
bool moveToPositionFromCurrent(int rail, PositionTarget pos, bool loaded);
bool moveToPositionMm(int rail, double positionMm, bool loaded);
bool moveRelativeManual(int rail, double relativeMm, bool loaded);
bool jogMotor(int rail, bool direction, double increment, bool loaded);
```

#### Status and Information
```cpp
bool isMotorReady(int rail);          // Motor readiness check
bool isMotorMoving(int rail);         // Movement status
double getMotorPositionMm(int rail);  // Current position
bool isHomingComplete(int rail);      // Homing status
```

### Sensor Functions
```cpp
// Carriage detection
bool isCarriageAtWC1();
bool isCarriageAtWC2();
bool isCarriageAtWC3();
bool isCarriageAtRail1Handoff();
bool isCarriageAtRail2Handoff();

// Labware detection
bool isLabwarePresentAtWC1();
bool isLabwarePresentAtWC2();
bool isLabwarePresentAtWC3();
bool isLabwarePresentAtHandoff();

// System monitoring
uint16_t getPressurePsi();
bool isPressureSufficient();
bool isPressureWarningLevel();
```

### Valve Control Functions
```cpp
void extendValve();                   // Extend pneumatic cylinder
void retractValve();                  // Retract pneumatic cylinder
ValvePosition getValvePosition();     // Get valve state
bool isCylinderRetracted();           // Hardware sensor check
bool isCylinderExtended();            // Hardware sensor check
bool validateValvePosition();         // Position validation
```

### Logging Functions
```cpp
void logSystemState();                // Log complete system state
void printValveSection();             // Log valve status
void printSensorSection();            // Log sensor states
void printMotorSection(int rail);     // Log motor status
void printSystemSection();            // Log system status
```

---

## File Structure

```
overhead_rail/
├── overhead_rail.ino              # Main Arduino sketch
├── README.md                      # This documentation
│
├── Motor Control
│   ├── MotorController.h          # Motor control header
│   ├── MotorController.cpp        # Motor control implementation
│   ├── EncoderController.h        # Handwheel/MPG header
│   └── EncoderController.cpp      # Handwheel/MPG implementation
│
├── Pneumatics & Sensors
│   ├── ValveController.h          # Pneumatic valve control header
│   ├── ValveController.cpp        # Pneumatic valve implementation
│   ├── Sensors.h                  # Sensor system header
│   └── Sensors.cpp                # Sensor system implementation
│
├── Communication & Control
│   ├── CommandController.h        # Command processing header
│   ├── CommandController.cpp      # Command processing implementation
│   ├── Commands.h                 # Command definitions header
│   ├── Commands.cpp               # Command implementations
│   ├── EthernetController.h       # Network communication header
│   └── EthernetController.cpp     # Network communication implementation
│
├── Logging & Utilities
│   ├── Logging.h                  # Logging system header
│   ├── Logging.cpp                # Logging system implementation
│   ├── LogHistory.h               # Operation history header
│   ├── LogHistory.cpp             # Operation history implementation
│   ├── OutputManager.h            # Console output management header
│   ├── OutputManager.cpp          # Console output implementation
│   └── Utils.h                    # Utility functions header
│   └── Utils.cpp                  # Utility functions implementation
│
├── Configuration
│   ├── PositionConfig.h           # Position configuration header
│   └── PositionConfig.cpp         # Position configuration implementation
│
└── Documentation
    ├── LOGGING_README.md          # Logging system documentation
    ├── VELOCITY_USAGE_GUIDE.md    # Velocity configuration guide
    └── OverheadRail_SystemDiagram.pdf  # System diagram
```

---

## Dependencies

### Hardware Requirements
- **ClearCore Controller Board** by Teknic
- **ClearPath Motors** (2x) with appropriate configuration
- **CL-ENCRD-DFIN Encoder Adapter Board** (for handwheel control)
- **Quadrature Encoder/Handwheel** (for manual control)
- **CCIO-8 Expansion Board** (optional, for additional I/O)
- **Pneumatic System** with pressure monitoring
- **Sensor Hardware** (optical sensors for position detection)

### Software Requirements
- **Arduino IDE** or **PlatformIO**
- **ClearCore Library** by Teknic
- **Arduino Core** for ClearCore platform

### Network Requirements (Optional)
- **Ethernet Connection** for remote monitoring and control
- **Compatible Terminal** with ANSI color support for enhanced logging display

---

## Getting Started

1. **Hardware Setup**: Connect ClearCore controller, motors, sensors, and optional components
2. **Library Installation**: Install ClearCore library in Arduino IDE
3. **Configuration**: Adjust position constants and sensor assignments in header files
4. **Upload Code**: Compile and upload to ClearCore controller
5. **Homing**: Execute homing sequence for both rails
6. **Testing**: Use command interface to test movement and sensor functionality
7. **Integration**: Integrate with your application-specific control logic

### Initial Setup Checklist
- [ ] ClearCore controller properly wired
- [ ] Motors configured and connected
- [ ] E-stop circuit functional
- [ ] Sensors connected and tested
- [ ] Pneumatic system pressurized
- [ ] Encoder/handwheel connected (if using)
- [ ] Network connection established (if using)
- [ ] Code compiled and uploaded
- [ ] Homing sequence completed
- [ ] Basic movement tests passed

This comprehensive system provides a complete foundation for dual-rail overhead transportation with safety, precision, monitoring, and ease of use. All functions include comprehensive error checking and status reporting for reliable industrial operation.
