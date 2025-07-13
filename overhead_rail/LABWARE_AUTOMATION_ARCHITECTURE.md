# Labware Automation Architecture

## 🎯 **Overview**

The overhead rail system provides automated labware handling through intelligent goto commands and comprehensive state tracking. The system automatically enables after rail homing and provides operation statistics for cycling tests.

---

## 📋 **Command Structure**

### **Primary Commands**

#### **1. `goto` Command - Automated Pickup/Delivery**
```
goto <location> <labware-status>
```

**Examples:**
- `goto wc1 no-labware` - Pickup labware from WC1
- `goto wc2 with-labware` - Deliver labware to WC2
- `goto wc3 no-labware` - Pickup labware from WC3

**Features:**
- Automatic cross-rail transfers (WC1/WC2 ↔ WC3)
- Comprehensive preflight safety validation
- Collision zone management for Rail 2
- Labware state consistency checking

#### **2. `labware` Command - State Management**
```
labware <action>
```

**Sub-commands:**
- `labware status` - Display current state and operation statistics
- `labware audit` - Validate and fix labware state via sensor readings
- `labware reset` - Clear all tracking and reset operation counters

---

## 🏗️ **System Architecture**

### **Automatic Enablement**
- **Goto commands automatically enabled** after homing both rails
- **Each rail homing** updates labware state from sensors
- **Manual audit available** for complex validation if needed
- **Disabled when dual labware conflicts** exist

### **Labware State Tracking**

#### **Rail 1 (Long Rail) - Checkpoint-Based Tracking**
- **Sensors**: WC1, WC2, handoff position sensors
- **Confidence**: HIGH (real-time), MEDIUM (recent sensor), LOW (inferred)
- **Tracking**: Between sensor checkpoints, state is inferred

#### **Rail 2 (Short Rail) - Continuous Tracking**  
- **Sensor**: Carriage-mounted sensor (always knows state)
- **Confidence**: Always HIGH (real-time)
- **Tracking**: Continuous labware presence detection

#### **Operation Statistics**
- **Pickup Counter**: Successful `no-labware` → work cell movements
- **Delivery Counter**: Successful `with-labware` → work cell movements
- **Cross-Rail Counter**: Successful Rail 1 ↔ Rail 2 handoff transfers
- **Time Tracking**: Human-readable time since last operations
- **Uptime**: Time since counters were reset

---

## 🚀 **How It Works**

### **Automatic Homing Integration**
1. **Home Rails**: `rail1 home` and `rail2 home` 
2. **Sensor Reading**: Automatically reads labware sensors during homing
3. **State Update**: Updates labware tracking based on sensor data
4. **Auto-Enable**: Enables goto commands if no conflicts detected
5. **Ready**: System ready for automated operations

### **Goto Command Flow**
1. **Parse Command**: Location (WC1/WC2/WC3) and status (with-labware/no-labware)
2. **Preflight Checks**: Safety validation, rail status, labware conflicts
3. **Smart Routing**: Automatic cross-rail transfers when needed
4. **Execute Movement**: Use existing rail movement functions
5. **Update Counters**: Track successful pickup/delivery operations
6. **Report Status**: Completion messages and statistics

### **Cross-Rail Intelligence**
- **WC1/WC2 → WC3**: Automatically transfers via handoff if needed
- **WC3 → WC1/WC2**: Automatically transfers via handoff if needed  
- **Collision Avoidance**: Rail 2 cylinder management for safety
- **State Consistency**: Cross-validation between rails

---

## 🛡️ **Error Handling & Recovery**

### **Automatic Recovery**
- **Homing Recovery**: Re-home affected rail to update state
- **Sensor Validation**: `labware audit` reads sensors for ground truth
- **Conflict Resolution**: Clear error messages with specific solutions

### **Operation Statistics Integration**
- **Counters Reset**: `labware reset` clears all counters and timestamps
- **Status Display**: Shows operation counts and time since last work
- **Cycling Test Support**: Perfect for automated test monitoring

---

## � **Current Implementation Status**

### **✅ Fully Implemented**

#### **Command Infrastructure**
- ✅ Complete `labware` command with status/audit/reset/help
- ✅ Complete `goto` command with all location combinations
- ✅ Binary search command parsing for efficiency
- ✅ Comprehensive help documentation

#### **Automatic Enablement System**  
- ✅ Homing integration - automatic labware detection
- ✅ Sensor reading during rail homing operations
- ✅ Automatic goto command enablement when possible
- ✅ Dual labware conflict detection and prevention

#### **Operation Tracking System**
- ✅ Pickup/delivery/cross-rail counters with timestamps
- ✅ Human-readable time formatting (e.g., "2 hours 15 minutes ago")
- ✅ Time since last work activity tracking
- ✅ Integration with labware status display
- ✅ Counter reset functionality

#### **Movement Integration**
- ✅ Integration with existing rail movement functions
- ✅ Counter increments on successful operations
- ✅ Cross-rail transfer tracking via handoff controller
- ✅ Smart routing for WC1/WC2 ↔ WC3 transfers

#### **State Management**
- ✅ Complete labware state structures and tracking
- ✅ Confidence level system (HIGH/MEDIUM/LOW/UNKNOWN)
- ✅ Sensor integration and validation
- ✅ Human-readable status display

---

## 🎯 **Usage Summary**

### **Daily Operation**
1. **Startup**: Home both rails (`rail1 home`, `rail2 home`)
2. **Auto-Ready**: Goto commands automatically enabled
3. **Operations**: Use `goto wc1 no-labware`, `goto wc2 with-labware`, etc.
4. **Monitoring**: Check `labware status` for operation statistics
5. **Recovery**: Use `labware audit` if issues arise

### **Cycling Tests**
- **Start Fresh**: `labware reset` to clear counters
- **Run Cycles**: Automated goto commands track all operations
- **Monitor Progress**: `labware status` shows counts and timing
- **Validate**: Operation counters confirm test completion

### **Error Recovery**
- **Simple Issues**: Re-home affected rail
- **Complex Issues**: `labware audit` for sensor validation
- **Nuclear Option**: `labware reset` to start completely fresh

---

**The system provides a robust, automated labware handling solution with comprehensive operation tracking, perfect for both manual operations and automated cycling tests.**
