# Labware Automation Architecture

## 🎯 **Overview**

This document defines the complete architecture for automated labware handling commands in the overhead rail system. The design focuses on **simplicity, safety, and reliability** through intelligent automation with minimal user decision-making.

---

## 📋 **Command Structure**

### **Primary Commands**

#### **1. `goto` Command - Automated Pickup/Delivery**
```
goto <location> <labware-status>
```

**Examples:**
- `goto WC1 no-labware` - Go to WC1 for pickup operation
- `goto WC2 with-labware` - Go to WC2 for delivery operation
- `goto WC3 no-labware` - Go to WC3 for pickup operation

#### **2. `labware` Command - State Management**
```
labware <action>
```

**Sub-commands:**
- `labware status` - Display current labware tracking state
- `labware audit` - Automatically validate and fix labware state
- `labware reset` - Clear all labware tracking (nuclear option)

---

## 🏗️ **System Architecture**

### **Labware State Tracking**

#### **Rail 1 (Long Rail ~8m) - Checkpoint-Based Tracking**
```cpp
struct Rail1LabwareState {
    bool hasLabware;              // Current labware status
    Location lastKnownLocation;   // WC1, WC2, or HANDOFF
    Location labwareSource;       // Where did this labware originate?
    bool validated;               // Confirmed by sensor reading?
    bool uncertainDueToFault;     // State uncertain after fault?
    unsigned long lastValidated;  // Timestamp of last sensor confirmation
};
```

**Available Sensors:** WC1, WC2, Rail1-Handoff (fixed position sensors)

#### **Rail 2 (Short Rail) - Continuous Tracking**
```cpp
struct Rail2LabwareState {
    bool hasLabware;              // Real-time sensor reading
    Location labwareSource;       // Where did this labware originate?
    unsigned long lastValidated;  // Always current (continuous sensor)
};
```

**Available Sensors:** Carriage-mounted (always knows state)

#### **Global System State**
```cpp
struct SystemLabwareState {
    Rail1LabwareState rail1;
    Rail2LabwareState rail2;
    bool automationEnabled;       // Can execute goto commands?
    bool dualLabwareConflict;     // Both rails have labware?
};
```

---

## 🚀 **`goto` Command Implementation**

### **Command Processing Flow**

#### **1. Command Parsing**
```cpp
goto <location> <labware-status>
```
- **Location**: WC1, WC2, WC3, handoff
- **Labware Status**: `no-labware` (pickup) or `with-labware` (delivery)

#### **2. Operation Type Detection**
```cpp
if (labware_status == "no-labware") {
    operation = PICKUP;
} else if (labware_status == "with-labware") {
    operation = DELIVERY;
}
```

#### **3. Rail Selection Logic**

**For Pickup Operations (`no-labware`):**
- **WC1/WC2** → Rail 1 (only rail that can reach these locations)
- **WC3** → Rail 2 (only rail that can reach this location)

**For Delivery Operations (`with-labware`):**
- **Check which rail has labware** → Use that rail
- **Validate destination is reachable** by the rail with labware
- **Error if no rail has labware** or **multiple rails have labware**

#### **4. Pre-Movement Validation**

**Pickup Operations:**
```cpp
1. Verify destination has labware (sensor check)
2. Verify selected rail is empty (state check)
3. Verify rail can reach destination (geometry check)
4. Verify no dual labware conflict
```

**Delivery Operations:**
```cpp
1. Verify source rail has labware (state check)
2. Verify destination is empty (sensor check when possible)
3. Verify source rail can reach destination (geometry check)
4. Resolve dual labware conflicts
```

#### **5. Movement Execution**

**Pickup Flow:**
```cpp
1. Move to destination at no-labware speeds
2. Arrive and confirm labware presence (sensor)
3. Report "READY_FOR_PICKUP" status
4. Wait for labware loading (manual or automated)
5. Update state: rail.hasLabware = true, rail.labwareSource = destination
6. Report "PICKUP_COMPLETE"
```

**Delivery Flow:**
```cpp
1. Move to destination at with-labware speeds
2. Arrive and confirm labware still present on carriage
3. Complete delivery (manual or automated)
4. Confirm labware transferred to destination (sensor when available)
5. Update state: rail.hasLabware = false
6. Report "DELIVERY_COMPLETE"
```

---

## 🛡️ **Error Handling & Recovery**

### **Error Categories**

#### **1. Pre-Movement Errors (Immediate Rejection)**

**Dual Labware Conflict:**
```
ERROR: DUAL_LABWARE_CONFLICT
- Rail 1: HAS_LABWARE (from WC1)
- Rail 2: HAS_LABWARE (from WC3)
- Cannot execute delivery - multiple sources available
- Resolution: Complete one delivery before starting another
```

**No Labware Available:**
```
ERROR: NO_LABWARE_AVAILABLE
- Rail 1: NO_LABWARE
- Rail 2: NO_LABWARE
- Cannot deliver to WC2 - no source labware
- Resolution: Execute pickup operation first
```

**Unreachable Destination:**
```
ERROR: UNREACHABLE_DESTINATION
- Rail 1 has labware but cannot reach WC3
- Rail 2 has no labware to deliver
- Resolution: Transfer via handoff or deliver Rail 1 labware elsewhere
```

#### **2. Movement Faults (Recovery Required)**

**Motor Fault During Movement:**
```
FAULT_DETECTED: Motor fault during WC1→WC2 movement
- Position: 4200mm (between destinations)
- Labware Status: UNCERTAIN
- Recovery Options:
  1. labware audit  - Automatic sensor validation
  2. labware reset  - Clear state and start fresh
```

**E-Stop During Movement:**
```
ESTOP_RECOVERY_NEEDED: E-stop activated during delivery
- Last Known: Moving to WC2 with labware
- Current Status: STOPPED at unknown position
- Labware Status: UNCERTAIN
- Recovery Required: Use 'labware audit' after E-stop cleared
```

### **Recovery Strategies**

#### **Smart Recovery: `labware audit`**

**Process:**
```cpp
1. Check Rail 1 current position
2. Calculate nearest safe sensor location (WC1 or WC2 only)
3. Move to sensor location at with-labware speeds (conservative)
4. Read sensor and determine actual labware state
5. Update system state based on sensor reality
6. Clear uncertainty flags
7. Report validated state and enable automation
```

**Example Flow:**
```
LABWARE_AUDIT_INITIATED: Analyzing Rail 1 position and state
MOVING_TO_WC2_FOR_VALIDATION: Nearest sensor location (using safe speeds)
ARRIVED_AT_WC2: Reading labware sensor
SENSOR_READING_COMPLETE: Labware detected at WC2
LABWARE_STATE_UPDATED: Rail 1 has labware at WC2 (sensor confirmed)
AUDIT_COMPLETE: System ready for automation commands
```

**Safety Features:**
- **Always moves at with-labware speeds** (protects potential labware)
- **Only moves to WC1/WC2** (avoids handoff collision zone)
- **Uses actual sensor readings** (ground truth validation)
- **Updates all tracking state** (comprehensive recovery)

#### **Nuclear Recovery: `labware reset`**

**Process:**
```cpp
1. Clear all labware tracking state
2. Mark all rail states as UNKNOWN
3. Disable automation until audit complete
4. Require manual state establishment
5. Log reset event for debugging
```

**Example Flow:**
```
NUCLEAR_RESET_INITIATED: Clearing all labware tracking state
LABWARE_STATE_CLEARED: All rails marked as unknown
AUTOMATION_DISABLED: Manual rail commands only until audit
SYSTEM_REQUIRES_AUDIT: Use 'labware audit' to establish current state
RESET_COMPLETE: Ready for manual operation or audit
```

---

## 🔧 **State Validation & Sensor Integration**

### **Sensor Validation Points**

#### **Rail 1 Checkpoint Validation:**
- **WC1 Sensor**: Validates labware presence/absence during operations
- **WC2 Sensor**: Validates labware presence/absence during operations
- **Handoff Sensor**: Used during Rail 1↔Rail 2 transfers only

#### **Rail 2 Continuous Validation:**
- **Carriage Sensor**: Real-time labware presence (high confidence)
- **WC3 Sensor**: Validates pickup/delivery operations
- **Handoff Sensor**: Cross-validates transfers with Rail 1

### **Confidence Levels**
```cpp
enum ConfidenceLevel {
    HIGH,     // Rail 2 carriage sensor, recent sensor reading
    MEDIUM,   // Rail 1 at sensor checkpoint
    LOW,      // Rail 1 between checkpoints (inferred state)
    UNKNOWN   // State uncertain due to fault/reset
};
```

### **State Update Triggers**
1. **Successful movement completion** at sensor locations
2. **Sensor readings** during operations
3. **Manual audit commands** with sensor validation
4. **Cross-rail validation** during handoff operations
5. **Fault recovery** with sensor confirmation

---

## 📊 **User Experience & Messaging**

### **Status Reporting**

#### **`labware status` Output:**
```
LABWARE_SYSTEM_STATUS: Current tracking state
============================================
Rail 1: HAS_LABWARE (from WC1, validated at WC2) - confidence: MEDIUM
Rail 2: NO_LABWARE (sensor confirmed) - confidence: HIGH
WC1: NO_LABWARE (sensor confirmed)
WC2: HAS_LABWARE (sensor confirmed)  
WC3: NO_LABWARE (sensor confirmed)

System Status: READY for automation
Last Validation: 14:32:15 (2 minutes ago)
```

#### **During Uncertain State:**
```
LABWARE_SYSTEM_STATUS: Recovery required
========================================
Rail 1: UNCERTAIN (fault during WC1→WC2 movement) - confidence: UNKNOWN
Rail 2: NO_LABWARE (sensor confirmed) - confidence: HIGH
WC1: NO_LABWARE (sensor confirmed)
WC2: UNKNOWN (destination of interrupted delivery)
WC3: NO_LABWARE (sensor confirmed)

System Status: AUTOMATION_DISABLED
Recovery Options:
- labware audit  (recommended: automatic sensor validation)
- labware reset  (nuclear option: clear all state)
```

### **Operation Messaging**

#### **Successful Operations:**
```
> goto WC1 no-labware
PICKUP_OPERATION_INITIATED: Moving Rail 1 to WC1 for labware pickup
MOVING_TO_WC1: Using no-labware speeds
ARRIVED_AT_WC1: Labware detected, ready for pickup
LABWARE_PICKUP_READY: Manual loading or automated pickup can proceed
(after pickup)
PICKUP_COMPLETE: Rail 1 now has labware from WC1
```

#### **Error Messages:**
```
> goto WC3 with-labware
ERROR: DUAL_LABWARE_CONFLICT
- Rail 1: HAS_LABWARE (from WC1)
- Rail 2: HAS_LABWARE (from WC3)
- Cannot execute delivery - multiple sources available
- Suggestion: Complete one delivery before starting another
- Use 'labware status' to see current state
```

---

## 🎯 **Implementation Benefits**

### **Safety Features**
✅ **Conservative movement** - Always assumes labware present until proven otherwise  
✅ **Sensor validation** - Ground truth confirmation at every opportunity  
✅ **Collision avoidance** - Smart rail selection prevents conflicts  
✅ **State consistency** - Cross-validation between rails and sensors  
✅ **Fault tolerance** - Graceful degradation with recovery options  

### **User Experience**
✅ **Simple commands** - Two primary commands handle all scenarios  
✅ **Automatic operation** - System makes intelligent decisions  
✅ **Clear feedback** - Descriptive status messages throughout  
✅ **Error guidance** - Specific suggestions for resolution  
✅ **Minimal training** - Intuitive command structure  

### **System Reliability**
✅ **Robust tracking** - Hybrid approach leverages all available sensors  
✅ **Self-correcting** - Audit command validates and fixes discrepancies  
✅ **Audit trail** - Complete logging of all labware operations  
✅ **Conflict prevention** - Pre-validation prevents impossible operations  
✅ **Recovery mechanisms** - Smart and nuclear options for all scenarios  

---

## 🔄 **Future Enhancements**

### **Potential Additions**
- **Automated pickup/delivery** - Integration with robotic systems
- **Multi-labware tracking** - Support for multiple labware types
- **Scheduled operations** - Time-based automated transfers
- **Remote monitoring** - Network-based status and control
- **Advanced diagnostics** - Predictive maintenance based on operation patterns

### **Scalability Considerations**
- **Additional sensors** - More Rail 1 checkpoints if needed
- **Multiple rail systems** - Architecture supports expansion
- **External integration** - API for higher-level automation systems
- **Database logging** - Persistent operation history and analytics

---

## 🚧 **Implementation Status & Next Steps**

### **✅ Completed Components**

#### **Command Infrastructure**
- ✅ **API Integration**: Both `labware` and `goto` commands integrated into command tree
- ✅ **Command Parsing**: Binary search command lookup with efficient argument processing
- ✅ **Help Systems**: Comprehensive help documentation for all commands
- ✅ **Error Handling**: Proper error messages and validation framework

#### **Labware Command (`cmd_labware`)**
- ✅ **Core Structure**: Complete implementation with 4 sub-commands
- ✅ **Status Command**: `labware status` calls `printLabwareSystemStatus()` 
- ✅ **Audit Command**: `labware audit` calls `performLabwareAudit()`
- ✅ **Reset Command**: `labware reset` calls `clearLabwareState()`
- ✅ **Help Command**: Detailed usage instructions and scenarios

#### **Goto Command (`cmd_goto`)**  
- ✅ **Core Structure**: Complete implementation with location/action parsing
- ✅ **Binary Search**: Efficient lookup for locations (WC1/WC2/WC3) and actions (with-labware/no-labware)
- ✅ **Validation Framework**: `performGotoPreflightChecks()` placeholder ready
- ✅ **Function Architecture**: 6 placeholder functions for all WC location combinations:
  - `executeWC1WithLabware()` / `executeWC1NoLabware()`
  - `executeWC2WithLabware()` / `executeWC2NoLabware()`
  - `executeWC3WithLabware()` / `executeWC3NoLabware()`
- ✅ **Help System**: Complete usage documentation with examples

#### **Build System**
- ✅ **Compilation**: All code compiles successfully without errors
- ✅ **Function Declarations**: All function signatures properly declared in Commands.h
- ✅ **Integration**: Clean integration with existing rail control systems

### **🔄 Currently Placeholder (Need Implementation)**

#### **LabwareAutomation.h/cpp Functions (Critical Dependencies)**
```cpp
// State Management Functions - NEED IMPLEMENTATION
void printLabwareSystemStatus();     // Called by labware status
bool performLabwareAudit();          // Called by labware audit  
void clearLabwareState();            // Called by labware reset

// Helper Functions for Goto Command - NEED IMPLEMENTATION
const char* getLocationName(Location loc);  // Used in goto messaging
```

#### **Goto Command Implementation Functions - NEED IMPLEMENTATION**
```cpp
// Preflight Validation - NEED IMPLEMENTATION
bool performGotoPreflightChecks(Location targetLocation, bool hasLabware);

// WC1 Movement Functions - NEED IMPLEMENTATION  
bool executeWC1WithLabware();   // Delivery to WC1
bool executeWC1NoLabware();     // Pickup from WC1

// WC2 Movement Functions - NEED IMPLEMENTATION
bool executeWC2WithLabware();   // Delivery to WC2  
bool executeWC2NoLabware();     // Pickup from WC2

// WC3 Movement Functions - NEED IMPLEMENTATION
bool executeWC3WithLabware();   // Delivery to WC3
bool executeWC3NoLabware();     // Pickup from WC3
```

### **📋 Implementation Roadmap (Easiest → Most Difficult)**

#### **Phase 1: Foundation Functions (EASY - 30 mins)**
**Goal**: Get basic labware commands functional
1. **Create `LabwareAutomation.h`**: Define Location enum and basic structures
2. **Implement `getLocationName()`**: Simple string lookup function
3. **Implement `clearLabwareState()`**: Basic state reset function
4. **Implement `printLabwareSystemStatus()`**: Basic status display (can start with placeholders)

**Result**: `labware status`, `labware reset`, and `labware help` will be fully functional

#### **Phase 2: Preflight Framework (MODERATE - 45 mins)**
**Goal**: Get goto command validation working
1. **Implement `performGotoPreflightChecks()`**: Basic validation framework
   - Check if rails are homed
   - Validate location is reachable
   - Basic safety checks (E-stop, motor ready)
2. **Enhance error messaging**: Add specific failure reasons
3. **Test goto command flow**: Verify all parsing and validation works

**Result**: `goto` commands will parse correctly and provide meaningful preflight feedback

#### **Phase 3: Simple Movement Implementation (MODERATE - 1 hour)**
**Goal**: Implement one working goto movement as template
1. **Choose `executeWC1NoLabware()`** as first implementation
   - Use existing `moveRail1CarriageToWC1(false)` function
   - Add pre/post movement logging
   - Add basic state updates
2. **Test end-to-end flow**: `goto WC1 no-labware`
3. **Create template**: Use as pattern for other 5 functions

**Result**: One complete goto operation working, template for others

#### **Phase 4: Complete WC1/WC2 Operations (MODERATE - 1.5 hours)**
**Goal**: Complete Rail 1 operations (WC1 and WC2)
1. **Implement remaining WC1/WC2 functions**:
   - `executeWC1WithLabware()` - Use `moveRail1CarriageToWC1(true)`
   - `executeWC2NoLabware()` - Use `moveRail1CarriageToWC2(false)`  
   - `executeWC2WithLabware()` - Use `moveRail1CarriageToWC2(true)`
2. **Add state tracking updates**: Update labware state after movements
3. **Test all Rail 1 operations**: Verify WC1/WC2 goto commands work

**Result**: Complete WC1 and WC2 automated operations functional

#### **Phase 5: WC3 Operations (CHALLENGING - 2 hours)**
**Goal**: Complete Rail 2 operations (WC3)  
1. **Implement WC3 functions**:
   - `executeWC3NoLabware()` - Use `moveRail2CarriageToWC3(false)`
   - `executeWC3WithLabware()` - Use `moveRail2CarriageToWC3(true)`
2. **Handle Rail 2 specifics**: Pneumatic cylinder, different sensors
3. **Cross-rail validation**: Ensure Rail 1/Rail 2 coordination
4. **Test WC3 operations**: Verify goto WC3 commands work

**Result**: Complete WC3 automated operations functional

#### **Phase 6: Advanced State Management (CHALLENGING - 3 hours)**
**Goal**: Implement intelligent labware tracking and audit
1. **Design state structures**: Rail1LabwareState, Rail2LabwareState, SystemLabwareState
2. **Implement `performLabwareAudit()`**:
   - Move to nearest sensor location
   - Read actual sensor state  
   - Update tracking based on ground truth
   - Handle uncertainty resolution
3. **Enhance `printLabwareSystemStatus()`**: Show real tracking state, confidence levels
4. **Add state persistence**: Maintain state across operations

**Result**: Full labware state tracking and audit recovery functional

#### **Phase 7: Advanced Preflight Validation (MOST CHALLENGING - 4 hours)**
**Goal**: Implement comprehensive operation validation
1. **Enhance `performGotoPreflightChecks()`**:
   - Check for dual labware conflicts
   - Validate pickup/delivery logic consistency
   - Cross-reference sensor states
   - Collision zone analysis
2. **Add operation-specific validation**:
   - Pickup operations: Verify labware present at destination
   - Delivery operations: Verify rail has labware, destination is clear
3. **Implement error recovery suggestions**: Context-specific guidance
4. **Add comprehensive logging**: Full audit trail

**Result**: Production-ready automated labware handling with full safety validation

### **🎯 Recommended Starting Point**

**Begin with Phase 1** - it will give you immediate functional improvements:
```bash
# First session goals (30 minutes):
1. Create LabwareAutomation.h with basic Location enum
2. Implement getLocationName() helper function  
3. Implement clearLabwareState() placeholder
4. Implement basic printLabwareSystemStatus()
5. Test: labware status, labware reset, labware help
```

This approach ensures steady progress with working functionality at each step, making it easy to pick up development at any phase boundary.

---

**This architecture provides a robust, user-friendly foundation for automated labware handling while maintaining the flexibility to grow with future requirements.**
