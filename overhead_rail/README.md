# OVERHEAD RAIL AUTOMATION SYSTEM

## SYSTEM PURPOSE
This program controls a dual-rail overhead automation system designed for precise labware handling across multiple workcells. The system manages automated transport of laboratory samples, plates, and equipment between three workcells using two independent rail systems with different precision requirements and integrated pneumatic handoff mechanisms.

## CORE FUNCTIONALITY
- **Dual-Rail Precision Motion Control**: Rail 1 (8.2m, high-precision) and Rail 2 (1m, standard precision)
- **Cross-Rail Labware Transfer**: Automated handoff system between Rails 1 and 2
- **Multi-Workcell Integration**: Automated transport to Workcell 1, Workcell 2, and Workcell 3
- **Pneumatic Cylinder Control**: Automated extend/retract for labware handling at Rail 2
- **Dynamic Position Teaching**: Field-adjustable positions with SD card persistence
- **Comprehensive Labware Tracking**: Real-time monitoring of labware location and state
- **Manual Control Interface**: MPG handwheel for precise manual positioning
- **Collision Prevention**: Intelligent collision zone detection and cylinder safety management
- **Network Integration**: Serial and Ethernet interfaces with extensive command documentation
- **Historical Logging**: Comprehensive operation history and error diagnosis capabilities

## REQUIRED HARDWARE COMPONENTS

### 1. CONTROLLER & I/O
- **ClearCore Industrial I/O and Motion Controller** (main controller)
- **CCIO-8 Digital I/O Expansion** (8-point expansion for sensors and pneumatic control)
- **CABLE-RIBBON6** (6 inch ribbon cable for CCIO connection)

### 2. DUAL MOTION SYSTEM

#### Rail 1 (8.2m Long-Distance Rail)
- **NEMA 23 ClearPath-SDSK Model CPM-SDSK-2321S-RLS** (servo motor with integrated drive)
- **8.2 meter linear rail system** with precision positioning capability

**REQUIRED MOTOR CONFIGURATION (using Teknic ClearPath-MSP software):**
- Input Resolution: **3200 pulses per revolution** (high precision for long distance)
- Input Format: Step and Direction
- Torque Limit: 50%
- HLFB Output: ASG-POSITION WITH MEASURED TORQUE
- Homing Configuration:
  * Homing Mode: Normal
  * Homing Style: User seeks home; ClearPath ASG signals when homing is complete
  * Homing Occurs: Upon every Enable
  * Homing Direction: **CCW (toward home position)**
  * Homing Torque Limit: 40%
  * Speed (RPM): 100.00
  * Accel/Decel (RPM/s): 2,500
  * Precision Homing: Use Precision Homing (enabled)
  * Home Offset Move Distance: 5mm from hardstop

#### Rail 2 (1m Precision Rail)
- **NEMA 23 ClearPath-SDSK Model CPM-SDSK-2321S-RLS** (servo motor with integrated drive)
- **1 meter linear rail system** with standard precision positioning

**REQUIRED MOTOR CONFIGURATION (using Teknic ClearPath-MSP software):**
- Input Resolution: **800 pulses per revolution** (standard precision for short distance)
- Input Format: Step and Direction
- Torque Limit: 50%
- HLFB Output: ASG-POSITION WITH MEASURED TORQUE
- Homing Configuration:
  * Homing Mode: Normal
  * Homing Style: User seeks home; ClearPath ASG signals when homing is complete
  * Homing Occurs: Upon every Enable
  * Homing Direction: **CW (away from home position)**
  * Homing Torque Limit: 40%
  * Speed (RPM): 100.00
  * Accel/Decel (RPM/s): 2,000
  * Precision Homing: Use Precision Homing (enabled)
  * Home Offset Move Distance: 5mm from hardstop

### 3. PNEUMATIC HANDOFF SYSTEM
- **Pneumatic cylinder** for Rail 2 labware pickup/dropoff operations
- **Pressure sensor** for pneumatic system monitoring (minimum pressure threshold)
- **Solenoid valves** for cylinder extend/retract control
- **Compressed air supply system** with adequate capacity for automation cycles

### 4. SENSOR SYSTEM
- **Labware detection sensors** at each workcell position
- **Handoff position sensors** for cross-rail transfer detection
- **Home position sensors** for each rail reference
- **Pressure monitoring sensor** for pneumatic system validation

### 5. FEEDBACK & CONTROL
- **CL-ENCDR-DFIN Encoder Input Adapter** (for MPG handwheel manual control)
- **MPG handwheel encoder** (manual positioning interface for both rails)
- **Position feedback systems** integrated with ClearPath motors

### 6. POWER SYSTEM
- **IPC-5 DC Power Supply** (350/500W, 75VDC output for motor power)
- **POWER4-STRIP DC Bus Distribution Strip** (power distribution)
- **24VDC supply** for logic, sensors, and pneumatics

### 7. SAFETY SYSTEMS
- **Emergency stop (E-stop) circuit** with normally closed contacts
- **Collision zone monitoring** for Rail 1/Rail 2 interaction safety
- **Pressure monitoring system** with automatic fault detection
- **Cross-rail safety interlocks** preventing simultaneous collision zone access
## SYSTEM LAYOUT & POSITIONS

### Rail 1 (8.2m Long-Distance Rail)
- **Home Position**: 0mm (reference position)
- **Staging Position**: 150mm (intermediate staging area)
- **Handoff Position**: 35mm (transfer point to Rail 2)
- **Workcell 2**: 3700mm (sample processing station)
- **Workcell 1**: 5700mm (primary analysis station)

### Rail 2 (1m Precision Rail)
- **Home Position**: 0mm (reference position)
- **Workcell 3**: 95mm (specialized handling station)
- **Handoff Position**: 900mm (transfer point from Rail 1)

### Collision Zone Management
- **Collision Zone**: 500-700mm on Rail 1 (where Rails 1 and 2 can interfere)
- **Automatic Cylinder Management**: Cylinder retracts automatically when Rail 1 enters collision zone
- **Safety Interlocks**: Prevents simultaneous Rail 1 and Rail 2 access to handoff area

## COMMUNICATION INTERFACES
- **Serial (USB)**: Direct command interface and diagnostics (115200 baud)
- **Ethernet**: Remote command interface and monitoring with configurable IP
- **Comprehensive Help System**: Type "help" for available commands, "help <command>" for specific usage
- **Command Categories**: Rail control, system management, positioning, labware automation

## USAGE

### OPERATIONAL MODES

#### A. AUTOMATED MODE (Default)
- **Cross-Rail Automation**: Automatic labware transfer between Rails 1 and 2
- **Collision Avoidance**: Intelligent coordination prevents rail interference
- **Labware Tracking**: Persistent state tracking across all workcells
- **Pneumatic Integration**: Automatic cylinder control for labware handling
- **Safety Monitoring**: Continuous pressure, position, and collision zone monitoring

#### B. MANUAL MODE
- **Individual Rail Control**: Direct command control for each rail independently
- **Manual Positioning**: MPG handwheel encoder for precise positioning
- **Component Testing**: Individual control of pneumatics, sensors, and motors
- **Maintenance Operations**: Step-by-step control for troubleshooting and calibration

### BASIC SETUP AND INITIALIZATION

### BASIC SETUP AND INITIALIZATION

#### 1. Motor Configuration
Configure both ClearPath motors using Teknic ClearPath-MSP software:
- Rail 1: 3200 pulses/rev, CCW homing, 2500 RPM/s accel
- Rail 2: 800 pulses/rev, CW homing, 2000 RPM/s accel
- Load configurations to motor memory and perform auto-tuning

#### 2. System Startup
Connect via Serial or Ethernet and run these commands:
- `system,state` - Check overall system status
- `rail1,init` - Initialize Rail 1 motor
- `rail2,init` - Initialize Rail 2 motor
- `system,home` - Home both rails sequentially
- `system,state` - Verify all systems ready

#### 3. System Validation
- `rail1,status` - Check Rail 1 motor status
- `rail2,status` - Check Rail 2 motor status
- `system,state` - Verify all sensor readings and pressure
- `labware,audit` - Validate labware tracking state

### POSITION TEACHING AND CONFIGURATION

### POSITION TEACHING AND CONFIGURATION

#### Rail-Specific Position Teaching
**Rail 1 Position Teaching:**
- `encoder,enable,rail1` - Enable handwheel for Rail 1
- Use handwheel to position Rail 1 precisely
- `teach,rail1,wc1` - Teach Workcell 1 position
- `teach,rail1,wc2` - Teach Workcell 2 position
- `teach,rail1,staging` - Teach staging position
- `teach,rail1,handoff` - Teach handoff position

**Rail 2 Position Teaching:**
- `encoder,enable,rail2` - Enable handwheel for Rail 2
- Use handwheel to position Rail 2 precisely
- `teach,rail2,wc3` - Teach Workcell 3 position
- `teach,rail2,handoff` - Teach handoff position

**Verify Taught Positions:**
- `teach,status` - Show all current positions
- `rail1,move-wc1,no-labware` - Test Rail 1 movements
- `rail2,move-wc3,no-labware` - Test Rail 2 movements

#### Alternative Teaching Methods
**Direct Positioning Method:**
- `rail1,move-mm-to,5700,no-labware` - Position Rail 1 to 5700mm
- `teach,rail1,wc1` - Teach current position as WC1

**Incremental Positioning Method:**
- `jog,rail1,increment,1` - Set 1mm jog increment
- `jog,rail1,+` - Jog forward
- `jog,rail1,-` - Jog backward
- `teach,rail1,wc1` - Teach final position

### MANUAL OPERATION COMMANDS

### MANUAL OPERATION COMMANDS

#### Rail Movement Control
**Absolute Positioning:**
- `rail1,move-wc1,no-labware` - Move Rail 1 to Workcell 1
- `rail1,move-wc2,with-labware` - Move Rail 1 to Workcell 2 with labware
- `rail2,move-wc3,no-labware` - Move Rail 2 to Workcell 3

**Relative Positioning:**
- `rail1,move-rel,100,no-labware` - Move Rail 1 forward 100mm
- `rail2,move-rel,-50,no-labware` - Move Rail 2 backward 50mm

**Direct Millimeter Positioning:**
- `rail1,move-mm-to,3700,no-labware` - Move Rail 1 to 3700mm position
- `rail2,move-mm-to,95,with-labware` - Move Rail 2 to 95mm position

#### Pneumatic Control
- `rail2,extend` - Extend Rail 2 cylinder
- `rail2,retract` - Retract Rail 2 cylinder

#### Manual Positioning Interface
- `encoder,enable,rail1` - Enable handwheel control for Rail 1
- `encoder,enable,rail2` - Enable handwheel control for Rail 2
- `encoder,disable` - Return to automated control
- `encoder,multiplier,5` - Set handwheel sensitivity (1, 10, or 100)
- `encoder,velocity,50` - Set maximum handwheel velocity

### AUTOMATED OPERATION COMMANDS

### AUTOMATED OPERATION COMMANDS

#### High-Level Automation
- `goto,wc1,with-labware` - Automated movement to Workcell 1
- `goto,wc2,no-labware` - Automated movement to Workcell 2  
- `goto,wc3,with-labware` - Automated movement to Workcell 3

#### Labware Management
- `labware,status` - Display current labware tracking state
- `labware,audit` - Automatically validate and fix labware state
- `labware,reset` - Reset labware tracking to known state

#### System Control
- `system,state` - Comprehensive system status display
- `system,home` - Sequential homing of both rails
- `system,reset` - Clear operational state for clean automation

### DIAGNOSTICS AND TROUBLESHOOTING

### DIAGNOSTICS AND TROUBLESHOOTING

#### Help System
- `help` - Display all available commands
- `rail1,help` - Rail 1 specific commands
- `rail2,help` - Rail 2 specific commands
- `goto,help` - Cross-rail automation commands
- `teach,help` - Position teaching system
- `encoder,help` - Manual control interface
- `system,help` - System management commands

#### Status and Monitoring
- `system,state` - Overall system readiness assessment
- `rail1,status` - Detailed Rail 1 motor status
- `rail2,status` - Detailed Rail 2 motor status
- `network,status` - Ethernet connection information
- `teach,status` - Position configuration status

#### Logging and History
- `log,on,250` - Enable logging every 250ms
- `log,off` - Disable periodic logging
- `log,now` - Log current system state immediately
- `log,history` - View complete operation log
- `log,errors` - View only error entries
- `log,last,20` - View last 20 log entries
- `log,stats` - View logging statistics

#### Fault Management
- `rail1,clear-fault` - Clear Rail 1 motor faults
- `rail2,clear-fault` - Clear Rail 2 motor faults
- `rail1,abort` - Emergency stop Rail 1
- `rail2,abort` - Emergency stop Rail 2
- `system,reset` - System-wide fault recovery

## POSITION TEACHING SYSTEM

### Factory Default Positions
The system includes hardcoded factory default positions used when no taught positions exist:

**Rail 1 Defaults:**
- Home: 0mm
- Staging: 150mm  
- Handoff: 35mm
- Workcell 2: 3700mm
- Workcell 1: 5700mm

**Rail 2 Defaults:**
- Home: 0mm
- Workcell 3: 95mm
- Handoff: 900mm

### Taught Position System
- **User-Defined Positions**: Override factory defaults with field-adjustable positions
- **SD Card Persistence**: Automatically saved to SD card for power cycle survival
- **Automatic Loading**: Taught positions loaded at startup if SD card present
- **Rail-Specific Teaching**: Each rail can have independently taught positions

### Position Priority Hierarchy
1. **Taught Positions** (highest priority) - User-defined positions
2. **SD Card Positions** (medium priority) - Loaded at startup
3. **Factory Defaults** (lowest priority) - Fallback values

### Teaching Workflow
1. **Initialize System**: `system,home` to establish reference
2. **Position Motor**: Use handwheel, direct positioning, or jog commands
3. **Capture Position**: `teach,rail1,wc1` (captures current position)
4. **Auto-Save**: Position automatically saved to SD card
5. **Test Position**: `rail1,move-wc1,no-labware` to verify accuracy
6. **Configuration Management**: `teach,status` to view all positions

## TYPICAL WORKFLOWS

### Initial System Setup
Power up system and connect via Serial (115200 baud) or Ethernet:
- `system,state` - Check system readiness
- `rail1,init` - Initialize Rail 1
- `rail2,init` - Initialize Rail 2  
- `system,home` - Home both rails
- `system,state` - Verify all systems ready

### Position Teaching (Handwheel Method)
- `system,home` - Establish reference
- `encoder,enable,rail1` - Enable handwheel for Rail 1
- Manually position using handwheel
- `teach,rail1,wc1` - Capture Workcell 1 position
- `rail1,move-wc1,no-labware` - Test taught position
- Repeat for other positions
- `teach,status` - Verify all positions

### Daily Operation Startup
System automatically loads taught positions from SD card:
- `system,state` - Verify system readiness
- `labware,audit` - Validate labware tracking
- Begin automated operations

### Cross-Rail Automation
- `goto,wc1,with-labware` - Automated Rail 1 → WC1 with labware handling
- `goto,wc3,with-labware` - Automated cross-rail transfer Rail 1 → Rail 2 → WC3
- `goto,wc2,no-labware` - Automated Rail 1 → WC2 without labware

### Maintenance and Troubleshooting
- `encoder,enable,rail1` - Enable manual control
- Perform maintenance operations
- `rail1,clear-fault` - Clear any faults
- `system,reset` - Reset to clean state
- `encoder,disable` - Return to automated mode

### Error Diagnosis
- `log,errors` - View recent errors
- `rail1,status` - Check Rail 1 specific status
- `rail2,status` - Check Rail 2 specific status
- `system,state` - Overall system assessment

## SAFETY FEATURES

### Emergency Stop Integration
- **Hardware E-Stop**: Immediately stops all motion and pneumatic operations
- **Software Monitoring**: Continuous E-stop state checking during operations
- **Safe Recovery**: Systematic restart procedures after E-stop events

### Collision Prevention
- **Collision Zone Detection**: Automatic detection when Rail 1 enters Rail 2 interaction zone
- **Cylinder Safety**: Automatic cylinder retraction during collision zone movements
- **Cross-Rail Interlocks**: Prevents simultaneous access to handoff positions

### Pressure Monitoring
- **Continuous Monitoring**: Real-time pressure sensor reading and validation
- **Automatic Fault Detection**: System alerts when pressure drops below operational threshold
- **Safe Operation Enforcement**: Pneumatic operations blocked during insufficient pressure

### Position Validation
- **Travel Limit Enforcement**: Software limits prevent over-travel on both rails
- **Position Verification**: Automatic validation of motor position after movements
- **Homing Verification**: Comprehensive homing sequence validation with timeout protection

## NETWORK CONFIGURATION

### Ethernet Interface
- **Default IP Configuration**: Static IP assignment (configurable)
- **Remote Command Interface**: Full command set available via network
- **Status Monitoring**: Real-time system monitoring via network connection
- **Multiple Client Support**: Simultaneous Serial and Ethernet connections

### Network Commands
- `network,status` - Show current network configuration
- `network,disconnect` - Safely disconnect network clients

## ADVANCED FEATURES

### Smart Homing
- **Distance-Based Optimization**: Faster homing when motor position is known
- **Precision Approach**: Automatic switching to precision mode near home
- **Rail-Specific Parameters**: Optimized homing speeds and distances for each rail

### Labware Automation
- **Persistent State Tracking**: Maintains labware location across power cycles
- **Cross-Rail Coordination**: Intelligent handoff between Rail 1 and Rail 2
- **Automatic Validation**: Self-correcting labware state with audit function

### Velocity Optimization
- **Load-Aware Speeds**: Different velocities for loaded vs. unloaded carriages
- **Distance-Based Deceleration**: Smooth deceleration for precision positioning
- **Rail-Specific Tuning**: Optimized motion profiles for each rail's characteristics

## TROUBLESHOOTING GUIDE

### Common Issues and Solutions

#### Motor Not Initializing
Check motor power and connections:
- `rail1,clear-fault` - Clear any existing faults
- `rail1,init` - Retry initialization
- `system,state` - Verify system status

#### Homing Failures
Ensure rails are clear of obstructions:
- `system,reset` - Clear any fault states
- `system,home` - Retry homing sequence

#### Pneumatic System Issues
Check air supply and pressure:
- `system,state` - View pressure status
- `rail2,retract` - Manually test cylinder operation

#### Position Teaching Problems
- `teach,reset` - Return to factory defaults
- `teach,status` - Verify position configuration
- Re-teach positions using handwheel method

#### Network Connection Issues
- `network,status` - Check current network state
- `network,disconnect` - Reset network connections
- Check Ethernet cable and IP configuration

### Error Code Reference
- **E-STOP_ACTIVE**: Emergency stop circuit engaged - check E-stop button and wiring
- **MOTOR_FAULT**: Motor alerts present - check motor power, connections, and configuration
- **INSUFFICIENT_PRESSURE**: Pneumatic pressure below threshold - check air supply
- **HOMING_TIMEOUT**: Homing sequence exceeded time limit - check for obstructions
- **POSITION_OUT_OF_RANGE**: Commanded position exceeds travel limits - verify position values
- **COLLISION_ZONE_VIOLATION**: Unsafe rail interaction detected - check cylinder position

## MAINTENANCE REQUIREMENTS

### Regular Maintenance
- **Daily**: Visual inspection of rails, verify system,state shows all systems ready
- **Weekly**: Check pneumatic pressure, verify position accuracy
- **Monthly**: Verify taught positions, test emergency stop function
- **Quarterly**: Motor auto-tuning verification, comprehensive system calibration

### Calibration Procedures
- **Position Teaching**: Re-teach positions when mechanical adjustments are made
- **Motor Tuning**: Re-run ClearPath auto-tuning after significant mechanical changes
- **Pressure Calibration**: Verify pressure sensor readings and thresholds

NOTE: This dual-rail system provides high-precision automation with intelligent cross-rail coordination. The rail-specific motor configurations (Rail 1: 3200 pulses/rev, Rail 2: 800 pulses/rev) optimize performance for each rail's specific requirements - high precision for the long 8.2m Rail 1, and standard precision for the short 1m Rail 2.
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

NOTE: This dual-rail system provides high-precision automation with intelligent cross-rail coordination. The rail-specific motor configurations (Rail 1: 3200 pulses/rev, Rail 2: 800 pulses/rev) optimize performance for each rail's specific requirements - high precision for the long 8.2m Rail 1, and standard precision for the short 1m Rail 2.
