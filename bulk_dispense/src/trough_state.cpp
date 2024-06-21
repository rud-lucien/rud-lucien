#include "project_includes.h" // Include the project includes header
#include "trough_state.h"     // Include the project-specific header

bool troughState()
{
    // Read the state of the capacitive sensor
    return digitalRead(TROUGH_STATE_SENSOR);
}
