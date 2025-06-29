#include "EncoderController.h"
#include "Utils.h"
#include "OutputManager.h"

// Initialize control variables
bool encoderControlActive = false;
int32_t lastEncoderPosition = 0;
unsigned long lastEncoderUpdateTime = 0;
float currentMultiplier = MULTIPLIER_X1; // Default to x1 multiplier
bool quadratureError = false;

// Helper function to get the multiplier name
const char *getMultiplierName(float multiplier)
{
    // Use almost-equal comparison for float values
    if (fabs(multiplier - MULTIPLIER_X1) < 0.001f)
        return "1";
    if (fabs(multiplier - MULTIPLIER_X10) < 0.001f)
        return "10";
    if (fabs(multiplier - MULTIPLIER_X100) < 0.001f)
        return "100";

    // Still return something helpful in case of unexpected values
    static char buffer[10];
    sprintf(buffer, "%.1f", multiplier);
    return buffer;
}

// Initialize the encoder interface
void initEncoderControl(bool swapDirection, bool indexInverted)
{
    // Enable the encoder input feature
    EncoderIn.Enable(true);

    // Zero the position to start
    EncoderIn.Position(0);

    // Set the encoder direction
    EncoderIn.SwapDirection(swapDirection);

    // Set the sense of index detection
    EncoderIn.IndexInverted(indexInverted);

    // Reset tracking variables
    lastEncoderPosition = 0;
    lastEncoderUpdateTime = millis();
    currentMultiplier = MULTIPLIER_X1; // Make sure default is set

    Console.serialInfo(F("Manual Pulse Generator (MPG) Handwheel interface initialized"));
}

// Process encoder movement
void processEncoderInput()
{
    if (!encoderControlActive || motorState == MOTOR_STATE_FAULTED)
    {
        return; // Only process when encoder control is active and motor is not faulted
    }

    // Extra safety check - ignore encoder input during automatic operations
    if (motorState == MOTOR_STATE_HOMING || motorState == MOTOR_STATE_MOVING || operationInProgress)
    {
        // Just reset the encoder position to avoid accumulating inputs
        EncoderIn.Position(0);
        lastEncoderPosition = 0;
        return;
    }

    static int32_t accumulatedDelta = 0; // Accumulate encoder movement over time
    static unsigned long lastMoveTime = 0;

    unsigned long currentTime = millis();

    // Read current encoder position first
    int32_t currentEncoderPosition = EncoderIn.Position();
    int32_t encoderDelta = currentEncoderPosition - lastEncoderPosition;

    // If encoder has moved
    if (encoderDelta != 0)
    {

        // Check for quadrature errors after movement detected (less prone to false positives)
        if (EncoderIn.QuadratureError())
        {
            Console.serialError(F("Quadrature error detected in encoder! Disabling control."));

            // The proper way to clear errors is to disable/enable
            EncoderIn.Enable(false);
            EncoderIn.Enable(true);

            // If error persists, disable encoder control
            if (EncoderIn.QuadratureError())
            {
                encoderControlActive = false;
                Console.serialInfo(F("MPG Handwheel control disabled due to persistent error"));
                return;
            }
        }

        // Update accumulated delta
        accumulatedDelta += encoderDelta;

        // Calculate time since last update - CHANGED: using timeDiff() for rollover safety
        unsigned long timeDelta = timeDiff(currentTime, lastEncoderUpdateTime);
        if (timeDelta < ENCODER_DEBOUNCE_MS)
        {
            // Update position but wait to process movement (debounce)
            lastEncoderPosition = currentEncoderPosition;
            return;
        }

        // Wait until we have a significant movement or enough time has passed
        bool shouldMove = false;

        // Either we have accumulated enough movement or waited long enough since last move
        // Increase this value for smoother, less frequent moves
        // CHANGED: using timeoutElapsed() for rollover safety
        if (abs(accumulatedDelta) >= 10 || (timeoutElapsed(currentTime, lastMoveTime, 150) && accumulatedDelta != 0))
        {
            shouldMove = true;
        }

        if (shouldMove && motorState != MOTOR_STATE_MOVING)
        {
            // Only move if we're not already moving
            // Make velocity proportional to encoder speed
            int32_t encoderVel = EncoderIn.Velocity();
            // Scale velocity between min and max based on encoder speed
            int32_t scaledVelocity = map(abs(encoderVel), 0, 500, ENCODER_MIN_VELOCITY, ENCODER_MAX_VELOCITY);

            // Ensure velocity stays within bounds
            if (scaledVelocity < ENCODER_MIN_VELOCITY)
            {
                scaledVelocity = ENCODER_MIN_VELOCITY;
            }
            else if (scaledVelocity > ENCODER_MAX_VELOCITY)
            {
                scaledVelocity = ENCODER_MAX_VELOCITY;
            }

            // Calculate steps to move based on accumulated delta and current multiplier
            float stepsToMoveFloat = accumulatedDelta * currentMultiplier;
            int32_t stepsToMove = (int32_t)stepsToMoveFloat;

            // Make sure we move at least a meaningful amount
            if (stepsToMove == 0 && accumulatedDelta != 0)
            {
                stepsToMove = (accumulatedDelta > 0) ? 1 : -1;
            }

            // Calculate target position in mm
            float targetPositionMm = currentPositionMm + (pulsesToMm(stepsToMove));

            // Check against limits
            if (targetPositionMm < 0 || targetPositionMm > MAX_TRAVEL_MM)
            {
                char msg[200];
                sprintf(msg, "Encoder movement rejected: target position %.2f mm is outside allowed range (0 to %.2f mm)", 
                        targetPositionMm, MAX_TRAVEL_MM);
                Console.serialWarning(msg);
            }
            else
            {
                // Update target tracking in motor controller
                updateMotorTarget(targetPositionMm);

                // When displaying the move message, show the directional step count for user readability
                // We want positive numbers when moving away from home (increasing position)
                int32_t displaySteps;
                if (targetPositionMm > currentPositionMm)
                {
                    // Moving away from home (increasing position) - show positive
                    displaySteps = abs(stepsToMove);
                }
                else
                {
                    // Moving towards home (decreasing position) - show negative
                    displaySteps = -abs(stepsToMove);
                }

                // Update the move message to use displaySteps
                char msg[200];
                sprintf(msg, "MPG Move: %ld steps (x%s, %ldk pps) → %.2f mm", 
                        displaySteps, getMultiplierName(currentMultiplier), 
                        scaledVelocity / 1000, targetPositionMm);
                Console.serialDiagnostic(msg);

                // Set velocity for this move
                MOTOR_CONNECTOR.VelMax(scaledVelocity);

                // Move the motor relative to current position
                MOTOR_CONNECTOR.Move(stepsToMove);

                // Update motor state
                motorState = MOTOR_STATE_MOVING;

                // Reset accumulated delta after movement
                accumulatedDelta = 0;
                lastMoveTime = currentTime;
            }
        }

        // Update tracking variables
        lastEncoderPosition = currentEncoderPosition;
        lastEncoderUpdateTime = currentTime;
    }
}

// Enable or disable encoder control
void enableEncoderControl(bool enable)
{
    // Only allow enabling if motor is both initialized and homed
    if (enable)
    {
        if (!motorInitialized)
        {
            Console.serialError(F("Motor must be initialized before enabling MPG control"));
            return;
        }
        if (!isHomed)
        {
            Console.serialError(F("Motor must be homed before enabling MPG control"));
            Console.serialInfo(F("Use the 'home' command to establish a reference position"));
            return;
        }

        // The proper way to reset the encoder and clear errors
        // is to disable and re-enable it
        EncoderIn.Enable(false);
        EncoderIn.Enable(true);
        EncoderIn.Position(0);

        // Motor is ready, enable encoder control
        encoderControlActive = true;

        // Reset encoder position when enabling
        lastEncoderPosition = 0;
        lastEncoderUpdateTime = millis();

        char msg[200];
        sprintf(msg, "MPG handwheel control enabled - current position: %.2f mm", currentPositionMm);
        Console.serialInfo(msg);
        
        sprintf(msg, "Using multiplier x%s (%.1f)", getMultiplierName(currentMultiplier), currentMultiplier);
        Console.serialInfo(msg);
        
        Console.serialInfo(F("Issue 'encoder,disable' when finished with manual control"));
    }
    else
    {
        encoderControlActive = false;
        Console.serialInfo(F("MPG Handwheel control disabled"));
    }
}

// Set the multiplier (x1, x10, x100)
void setEncoderMultiplier(int multiplier)
{
    char msg[200];
    
    switch (multiplier)
    {
    case 1:
        currentMultiplier = MULTIPLIER_X1;
        sprintf(msg, "MPG multiplier set to x1 (fine control): %.1f", currentMultiplier);
        Console.serialInfo(msg);
        break;
    case 10:
        currentMultiplier = MULTIPLIER_X10;
        sprintf(msg, "MPG multiplier set to x10 (medium control): %.1f", currentMultiplier);
        Console.serialInfo(msg);
        break;
    case 100:
        currentMultiplier = MULTIPLIER_X100;
        sprintf(msg, "MPG multiplier set to x100 (coarse control): %.1f", currentMultiplier);
        Console.serialInfo(msg);
        break;
    default:
        Console.serialError(F("Invalid multiplier. Use 1, 10, or 100"));
        return;
    }

    // Useful diagnostic information
    float mmPerRotation = 100 * currentMultiplier / PULSES_PER_MM;
    sprintf(msg, "One full rotation moves ~%.2f mm", mmPerRotation);
    Console.serialInfo(msg);
}
