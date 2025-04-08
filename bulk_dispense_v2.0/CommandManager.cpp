#include "CommandManager.h"

extern unsigned long networkCommandStartTime;

extern unsigned long serialCommandStartTime;

// Module-level state variables
static StreamSession serialSession = {false, 0, &Serial};
static StreamSession networkSession = {false, 0, nullptr};
static int pendingCommands = 0;

volatile bool commandLineBeingProcessed = false;

// Helper to get session for a stream
static StreamSession *getSessionForStream(Stream *stream)
{
    if (stream == &Serial)
        return &serialSession;
    if (stream == &currentClient)
    {
        networkSession.stream = &currentClient; // Update network stream pointer
        return &networkSession;
    }
    return nullptr;
}

void cm_startSession(Stream *stream)
{
    StreamSession *session = getSessionForStream(stream);
    if (!session)
        return;

    if (!session->active)
    {
        session->active = true;
        session->startTime = millis();

        // Set the appropriate command start time based on stream type
        if (stream == &Serial)
        {
            serialCommandStartTime = millis();
            stream->println(F("[ACTION START]"));
        }
        else if (stream == &currentClient)
        {
            networkCommandStartTime = millis();
            // Send to network client
            stream->println(F("[ACTION START]"));
            // Also mirror to serial console for monitoring
            Serial.println(F("[ACTION START]"));
        }
    }
}

void cm_endSession(Stream *stream)
{
    StreamSession *session = getSessionForStream(stream);
    if (!session || !session->active)
        return;

    // For network commands, print ACTION END
    if (stream == &currentClient && networkCommandStartTime > 0)
    {
        // Print to Serial monitor
        Serial.print(F("[ACTION END] Duration: "));
        Serial.print(millis() - networkCommandStartTime);
        Serial.println(F(" ms"));

        // Also send to network client
        if (hasActiveClient && currentClient.connected())
        {
            currentClient.print(F("[ACTION END] Duration: "));
            currentClient.print(millis() - networkCommandStartTime);
            currentClient.println(F(" ms"));
        }

        networkCommandStartTime = 0;
    }
    // For serial commands, print ACTION END
    else if (stream == &Serial && serialCommandStartTime > 0)
    {
        unsigned long duration = millis() - serialCommandStartTime;
        stream->print(F("[ACTION END] Duration: "));
        stream->print(duration);
        stream->println(F(" ms"));
        serialCommandStartTime = 0;
    }

    // Normal session ending logic
    session->active = false;

    // sendMessage(F("[SESSION ENDED]"), stream, currentClient);

    // For network commands, make sure "[SESSION ENDED]" appears in both streams
    if (stream == &currentClient)
    {
        Serial.println(F("[SESSION ENDED]"));
        if (hasActiveClient && currentClient.connected())
        {
            currentClient.println(F("[SESSION ENDED]"));
        }
    }
    else
    {
        // For serial commands, use sendMessage for normal behavior
        sendMessage(F("[SESSION ENDED]"), stream, currentClient);
    }
}

void cm_registerCommand(void)
{
    pendingCommands++;
}

void cm_commandCompleted(Stream *stream)
{
    StreamSession *session = getSessionForStream(stream);
    bool wasActive = session ? session->active : false;

    if (pendingCommands > 0)
    {
        pendingCommands--;

        // Keep this user-facing message
        sendMessage(F("[DEBUG] Command completed. Pending: "), stream, currentClient, false);
        sendMessage(String(pendingCommands).c_str(), stream, currentClient);
    }

    // End session if needed
    if (pendingCommands <= 0 && !commandLineBeingProcessed && wasActive)
    {
        cm_endSession(stream);
    }
}

bool cm_isSessionActive(void)
{
    return serialSession.active || networkSession.active;
}

int cm_getPendingCommands(void)
{
    return pendingCommands;
}

void cm_abortSession(Stream *stream)
{
    StreamSession *session = getSessionForStream(stream);
    if (!session)
        return;

    // Reset all pending commands
    pendingCommands = 0;

    // Make sure to reset the timing variables
    if (stream == &currentClient)
    {
        // For network client, ensure we reset the network timing
        networkCommandStartTime = 0;

        // Print a special ACTION END message for aborted commands
        if (session->active)
        {
            unsigned long duration = millis() - session->startTime;

            // Send to network client
            stream->print(F("[ACTION END] ABORTED after "));
            stream->print(duration);
            stream->println(F(" ms"));

            // Also mirror to Serial console for monitoring
            Serial.print(F("[ACTION END] ABORTED after "));
            Serial.print(duration);
            Serial.println(F(" ms"));

            session->active = false;
        }

        // Send session ended to both streams
        if (hasActiveClient && currentClient.connected())
        {
            currentClient.println(F("[SESSION ENDED]"));
        }
        Serial.println(F("[SESSION ENDED]"));

        // Skip the sendMessage at the end for network client
        return;
    }
    else if (stream == &Serial)
    {
        // For serial commands, reset the serial timing
        serialCommandStartTime = 0;

        // Print a special ACTION END message for aborted commands
        if (session->active)
        {
            unsigned long duration = millis() - session->startTime;
            stream->print(F("[ACTION END] ABORTED after "));
            stream->print(duration);
            stream->println(F(" ms"));
            session->active = false;
        }
    }

    // Only call sendMessage for Serial stream
    sendMessage(F("[SESSION ENDED]"), stream, currentClient);
}

// Add this function:
void resetCommandTimers()
{
    networkCommandStartTime = 0;
    serialCommandStartTime = 0;
}
