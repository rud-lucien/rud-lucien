#include "Logging.h"
#include "Hardware.h" // For global hardware objects and hardware functions
#include "Sensors.h"  // For sensor reading functions
#include "Utils.h"    // For any helper functions
#include <stdio.h>
#include "CommandSession.h"
#include "CommandManager.h"

/************************************************************
 * Logging.cpp
 *
 * This file implements functions declared in Logging.h.
 * It provides functions for logging generic messages and
 * detailed system state.
 *
 * Author: Rud Lucien
 * Date: 2025-04-08
 * Version: 2.0
 ************************************************************/

// ============================================================
// Global Logging Instance
// ============================================================
LoggingManagement logging = {0, 250}; // Default log interval: 250 ms

// ============================================================
// logData()
// ============================================================
void logData(const char *module, const char *message)
{
  Serial.print(F("[LOG] "));
  Serial.print(module);
  Serial.print(F(" - "));
  Serial.println(message);
}

// Returns a diagnostic string based on whether the sensor is dispensing or not.
// If not dispensing, it returns "Not Dispensing". Otherwise, it uses getFlowState().
const char *getFlowDiagString(const FlowSensor &sensor, bool isDispensing)
{
  if (!isDispensing)
  {
    return "Not Dispensing";
  }
  return sensor.isValidReading ? "Valid" : "Invalid";
}

// ============================================================
// logSystemState()
// ============================================================
void logSystemState()
{
  char buffer[600];

  // --- Fan State ---
  char fanState = (digitalRead(fan.relayPin) == HIGH ? '1' : '0');

  // --- Valve States ---
  char rValve1 = (reagentValve1.isOpen ? '1' : '0');
  char rValve2 = (reagentValve2.isOpen ? '1' : '0');
  char rValve3 = (reagentValve3.isOpen ? '1' : '0');
  char rValve4 = (reagentValve4.isOpen ? '1' : '0');

  char mValve1 = (mediaValve1.isOpen ? '1' : '0');
  char mValve2 = (mediaValve2.isOpen ? '1' : '0');
  char mValve3 = (mediaValve3.isOpen ? '1' : '0');
  char mValve4 = (mediaValve4.isOpen ? '1' : '0');

  char wValve1 = (wasteValve1.isOpen ? '1' : '0');
  char wValve2 = (wasteValve2.isOpen ? '1' : '0');
  char wValve3 = (wasteValve3.isOpen ? '1' : '0');
  char wValve4 = (wasteValve4.isOpen ? '1' : '0');

  // --- Proportional Valve Feedback ---
  char pFeedbackStr[8];
  dtostrf(getValveFeedback(proportionalValve), 4, 1, pFeedbackStr);
  float feedback = getValveFeedback(proportionalValve);
  float valvePercent = (proportionalValveMaxFeedback > 0) ? (feedback / proportionalValveMaxFeedback) * 100.0 : 0.0;
  char pPercentStr[8];
  dtostrf(valvePercent, 4, 1, pPercentStr);

  // --- Binary Sensor States ---
  char wsl1 = (readBinarySensor(wasteLineSensors[0]) ? '1' : '0');
  char wsl2 = (readBinarySensor(wasteLineSensors[1]) ? '1' : '0');

  char wbl1 = (readBinarySensor(wasteBottleSensors[0]) ? '1' : '0');
  char wbl2 = (readBinarySensor(wasteBottleSensors[1]) ? '1' : '0');

  char wvs1 = (readBinarySensor(wasteVacuumSensors[0]) ? '1' : '0');
  char wvs2 = (readBinarySensor(wasteVacuumSensors[1]) ? '1' : '0');

  char els1 = (readBinarySensor(enclosureLiquidSensor) ? '1' : '0');

  // --- Bubble Sensors ---
  char bs1 = (readBinarySensor(reagentBubbleSensors[0]) ? '1' : '0');
  char bs2 = (readBinarySensor(reagentBubbleSensors[1]) ? '1' : '0');
  char bs3 = (readBinarySensor(reagentBubbleSensors[2]) ? '1' : '0');
  char bs4 = (readBinarySensor(reagentBubbleSensors[3]) ? '1' : '0');

  // --- Overflow Sensors ---
  char os1 = (readBinarySensor(overflowSensors[0]) ? '1' : '0');
  char os2 = (readBinarySensor(overflowSensors[1]) ? '1' : '0');
  char os3 = (readBinarySensor(overflowSensors[2]) ? '1' : '0');
  char os4 = (readBinarySensor(overflowSensors[3]) ? '1' : '0');

  // --- Pressure Sensor ---
  float pressureValue = readPressure(pressureSensor);
  char pressureStr[8];
  dtostrf(pressureValue, 4, 1, pressureStr);

  // --- Temperature & Humidity ---
  TempHumidity th = readTempHumidity();
  char tempStr[8], humStr[8];
  if (th.valid)
  {
    dtostrf(th.temperature, 4, 1, tempStr);
    dtostrf(th.humidity, 4, 1, humStr);
  }
  else
  {
    strcpy(tempStr, "-1");
    strcpy(humStr, "-1");
  }

  // --- Flow Sensor Data (for 4 sensors) ---
  char f1Rate[8], f1Temp[8], f1Disp[8], f1Total[8], f1Flag[4];
  dtostrf(flow1.isValidReading ? flow1.flowRate : -1, 4, 1, f1Rate);
  dtostrf(flow1.isValidReading ? flow1.temperature : -1, 4, 1, f1Temp);
  dtostrf(flow1.dispenseVolume, 4, 1, f1Disp);
  dtostrf(flow1.totalVolume, 4, 1, f1Total);
  snprintf(f1Flag, sizeof(f1Flag), "%d", flow1.isValidReading ? flow1.highFlowFlag : -1);

  char f2Rate[8], f2Temp[8], f2Disp[8], f2Total[8], f2Flag[4];
  dtostrf(flow2.isValidReading ? flow2.flowRate : -1, 4, 1, f2Rate);
  dtostrf(flow2.isValidReading ? flow2.temperature : -1, 4, 1, f2Temp);
  dtostrf(flow2.dispenseVolume, 4, 1, f2Disp);
  dtostrf(flow2.totalVolume, 4, 1, f2Total);
  snprintf(f2Flag, sizeof(f2Flag), "%d", flow2.isValidReading ? flow2.highFlowFlag : -1);

  char f3Rate[8], f3Temp[8], f3Disp[8], f3Total[8], f3Flag[4];
  dtostrf(flow3.isValidReading ? flow3.flowRate : -1, 4, 1, f3Rate);
  dtostrf(flow3.isValidReading ? flow3.temperature : -1, 4, 1, f3Temp);
  dtostrf(flow3.dispenseVolume, 4, 1, f3Disp);
  dtostrf(flow3.totalVolume, 4, 1, f3Total);
  snprintf(f3Flag, sizeof(f3Flag), "%d", flow3.isValidReading ? flow3.highFlowFlag : -1);

  char f4Rate[8], f4Temp[8], f4Disp[8], f4Total[8], f4Flag[4];
  dtostrf(flow4.isValidReading ? flow4.flowRate : -1, 4, 1, f4Rate);
  dtostrf(flow4.isValidReading ? flow4.temperature : -1, 4, 1, f4Temp);
  dtostrf(flow4.dispenseVolume, 4, 1, f4Disp);
  dtostrf(flow4.totalVolume, 4, 1, f4Total);
  snprintf(f4Flag, sizeof(f4Flag), "%d", flow4.isValidReading ? flow4.highFlowFlag : -1);

  // --- Flow Sensor Fluid Types ---
  const char *f1Type = getFluidTypeString(flow1.fluidType);
  const char *f2Type = getFluidTypeString(flow2.fluidType);
  const char *f3Type = getFluidTypeString(flow3.fluidType);
  const char *f4Type = getFluidTypeString(flow4.fluidType);

  // --- Dispensing State (DS) ---
  char ds1 = (valveControls[0].isDispensing ? '1' : '0');
  char ds2 = (valveControls[1].isDispensing ? '1' : '0');
  char ds3 = (valveControls[2].isDispensing ? '1' : '0');
  char ds4 = (valveControls[3].isDispensing ? '1' : '0');

  // --- Target Volume (TV) ---
  char tv1[8], tv2[8], tv3[8], tv4[8];
  dtostrf(valveControls[0].targetVolume, 4, 1, tv1);
  dtostrf(valveControls[1].targetVolume, 4, 1, tv2);
  dtostrf(valveControls[2].targetVolume, 4, 1, tv3);
  dtostrf(valveControls[3].targetVolume, 4, 1, tv4);

  // --- Priming State (PR) ---
  char pr1 = (valveControls[0].isPriming ? '1' : '0');
  char pr2 = (valveControls[1].isPriming ? '1' : '0');
  char pr3 = (valveControls[2].isPriming ? '1' : '0');
  char pr4 = (valveControls[3].isPriming ? '1' : '0');

  // --- Fill Mode (FM) ---
  char fm1 = (valveControls[0].fillMode ? '1' : '0');
  char fm2 = (valveControls[1].fillMode ? '1' : '0');
  char fm3 = (valveControls[2].fillMode ? '1' : '0');
  char fm4 = (valveControls[3].fillMode ? '1' : '0');

  // --- Trough Drain Status (TDS) ---
  // Trough 1: draining if wasteValve1 and wasteValve3 are open.
  // Trough 2: draining if wasteValve1 is open and wasteValve3 is closed.
  // Trough 3: draining if wasteValve2 and wasteValve4 are open.
  // Trough 4: draining if wasteValve2 is open and wasteValve4 are closed.
  char tds1 = (wasteValve1.isOpen && wasteValve3.isOpen) ? '1' : '0';
  char tds2 = (wasteValve1.isOpen && !wasteValve3.isOpen) ? '1' : '0';
  char tds3 = (wasteValve2.isOpen && wasteValve4.isOpen) ? '1' : '0';
  char tds4 = (wasteValve2.isOpen && !wasteValve4.isOpen) ? '1' : '0';

  // --- Format and Print Log Message ---
  sprintf(buffer,
          "[LOG] F%c, RV%c%c%c%c, MV%c%c%c%c, WV%c%c%c%c, PV,%s, PV%%,%s, "
          "WSL%c%c, WBL%c%c, WVS%c%c, ELS%c, BS%c%c%c%c, OS%c%c%c%c, "
          "PS,%s, T,%s, H,%s, FS1,%s,%s,%s,%s,%s,%s; FS2,%s,%s,%s,%s,%s,%s; "
          "FS3,%s,%s,%s,%s,%s,%s; FS4,%s,%s,%s,%s,%s,%s, DS%c%c%c%c, TV,%s,%s,%s,%s, "
          "PR%c%c%c%c, FM%c%c%c%c, TDS%c%c%c%c",
          // Fan state
          fanState,
          // Reagent valves
          rValve1, rValve2, rValve3, rValve4,
          // Media valves
          mValve1, mValve2, mValve3, mValve4,
          // Waste valves
          wValve1, wValve2, wValve3, wValve4,
          // Proportional valve feedback
          pFeedbackStr, pPercentStr,
          // Waste line sensors
          wsl1, wsl2,
          // Waste bottle sensors
          wbl1, wbl2,
          // Waste vacuum sensors
          wvs1, wvs2,
          // Enclosure liquid sensor
          els1,
          // Bubble sensors
          bs1, bs2, bs3, bs4,
          // Overflow sensors
          os1, os2, os3, os4,
          // Pressure, Temperature, Humidity
          pressureStr, tempStr, humStr,
          // Flow Sensor 1 data
          f1Rate, f1Temp, f1Disp, f1Total, f1Flag, f1Type,
          // Flow Sensor 2 data
          f2Rate, f2Temp, f2Disp, f2Total, f2Flag, f2Type,
          // Flow Sensor 3 data
          f3Rate, f3Temp, f3Disp, f3Total, f3Flag, f3Type,
          // Flow Sensor 4 data
          f4Rate, f4Temp, f4Disp, f4Total, f4Flag, f4Type,
          // Dispensing state for valves (DS)
          ds1, ds2, ds3, ds4,
          // Target volume for valves (TV)
          tv1, tv2, tv3, tv4,
          // Priming state (PR)
          pr1, pr2, pr3, pr4,
          // Fill mode (FM)
          fm1, fm2, fm3, fm4,
          // Trough Drain Status (TDS)
          tds1, tds2, tds3, tds4);

  // --- Build Diagnostic Information ---
  char diagBuffer[128];
  sprintf(diagBuffer,
          ", DIAG: FAM:%s, EERR:%s, GVM1:%s, GVM2:%s, MC1:%s, MC2:%s, MC3:%s, MC4:%s, LF:%lu ms, RC:%d, NET:%s",
          fanAutoMode ? "ON" : "OFF",                    // Fan Auto Mode
          globalEnclosureLiquidError ? "TRUE" : "FALSE", // Enclosure Liquid Error
          globalVacuumMonitoring[0] ? "TRUE" : "FALSE",  // Vacuum Monitoring for bottle 1
          globalVacuumMonitoring[1] ? "TRUE" : "FALSE",  // Vacuum Monitoring for bottle 2
          valveControls[0].manualControl ? "ON" : "OFF", // Manual Control for Trough 1
          valveControls[1].manualControl ? "ON" : "OFF", // Manual Control for Trough 2
          valveControls[2].manualControl ? "ON" : "OFF", // Manual Control for Trough 3
          valveControls[3].manualControl ? "ON" : "OFF", // Manual Control for Trough 4
          logging.logInterval,                           // Logging frequency
          cm_getPendingCommands(),                       // Get number of pending commands
          hasActiveClient ? "CONNECTED" : "NONE");       // Network connection status

  char flowDiag[128];
  sprintf(flowDiag,
          ", FLOW_DIAG: FS1:%s, FS2:%s, FS3:%s, FS4:%s",
          getFlowDiagString(flow1, valveControls[0].isDispensing),
          getFlowDiagString(flow2, valveControls[1].isDispensing),
          getFlowDiagString(flow3, valveControls[2].isDispensing),
          getFlowDiagString(flow4, valveControls[3].isDispensing));

  // Add correction parameters to log - using dtostrf for reliable float formatting
char correctionDiag[160];
char slope1[8], offset1[8], slope2[8], offset2[8];
char slope3[8], offset3[8], slope4[8], offset4[8];

dtostrf(flow1.slopeCorrection, 4, 2, slope1);
dtostrf(flow1.offsetCorrection, 4, 2, offset1);
dtostrf(flow2.slopeCorrection, 4, 2, slope2);
dtostrf(flow2.offsetCorrection, 4, 2, offset2);
dtostrf(flow3.slopeCorrection, 4, 2, slope3);
dtostrf(flow3.offsetCorrection, 4, 2, offset3);
dtostrf(flow4.slopeCorrection, 4, 2, slope4);
dtostrf(flow4.offsetCorrection, 4, 2, offset4);

sprintf(correctionDiag,
        ", FCOR: FS1:%s,%s,%s, FS2:%s,%s,%s, FS3:%s,%s,%s, FS4:%s,%s,%s",
        flow1.useCorrection ? "ON" : "OFF", slope1, offset1,
        flow2.useCorrection ? "ON" : "OFF", slope2, offset2, 
        flow3.useCorrection ? "ON" : "OFF", slope3, offset3,
        flow4.useCorrection ? "ON" : "OFF", slope4, offset4);

  // Append diagnostic info to the main log message
  strncat(buffer, diagBuffer, sizeof(buffer) - strlen(buffer) - 1);
  strncat(buffer, flowDiag, sizeof(buffer) - strlen(buffer) - 1);
  // Append correction info to the main log message
  strncat(buffer, correctionDiag, sizeof(buffer) - strlen(buffer) - 1);

  // --- Print the complete log message ---
  Serial.println(buffer);
}
