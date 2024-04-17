// OttoDIY Arduino Library project 2024

// SerialCommand (C) 2011-2013 Steven Cogswell

#ifndef SerialCommand_h
#define SerialCommand_h

#include "Arduino.h"

#ifndef SERIALCOMMAND_HARDWAREONLY
#include <SoftwareSerial.h>
#endif

#include <string.h>

#define SERIALCOMMANDBUFFER 35
#define MAXSERIALCOMMANDS   16
#define MAXDELIMETER        2

#define SERIALCOMMANDDEBUG  1
#undef SERIALCOMMANDDEBUG // Comment this out to run the library in debug mode (verbose messages)

class SerialCommand {
  public:
    SerialCommand(); // Constructor
#ifndef SERIALCOMMAND_HARDWAREONLY
    SerialCommand(Stream &SoftSer); // Constructor for using SoftwareSerial objects
#endif

    void clearBuffer(); // Sets the command buffer to all '\0' (nulls)
    char *next(); // returns pointer to next token found in command buffer (for getting arguments to commands)
    void readSerial(); // Main entry point.
    void addCommand(const char *, void(*)()); // Add commands to processing dictionary
    void addDefaultHandler(void (*function)()); // A handler to call when no valid command received.

  private:
    char inChar; // A character read from the serial stream
    char buffer[SERIALCOMMANDBUFFER]; // Buffer of stored characters while waiting for terminator character
    int bufPos; // Current position in the buffer
    char delim[MAXDELIMETER]; // null-terminated list of character to be used as delimeters for tokenizing (default " ")
    char term; // Character that signals end of command (default '\r')
    char *token; // Returned token from the command buffer as returned by strtok_r
    char *last; // State variable used by strtok_r during processing

    typedef struct _callback {
      char command[MAXDELIMETER];
      void (*function)();
    } SerialCommandCallback; // Data structure to hold Command/Handler function key-value pairs

    int numCommand;
    SerialCommandCallback CommandList[MAXSERIALCOMMANDS]; // Actual definition for command/handler array
    void (*defaultHandler)(); // Pointer to the default handler function
    int usingSoftwareSerial; // Used as boolean to see if we're using SoftwareSerial object or not
#ifndef SERIALCOMMAND_HARDWAREONLY
    Stream *_serialPort; // Pointer to a user-created SoftwareSerial object
#endif
};

#endif // SerialCommand_h

