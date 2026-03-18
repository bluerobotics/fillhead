/**
 * @file command_parser.h
 * @brief Command parsing declarations for the Fillhead controller.
 * @details Matches inc/commands.h parser API.
 * @see commands.h for command definitions
 * @see command_parser.cpp for implementations
 */
#pragma once

#include "commands.h"

//==================================================================================================
// Command Parser Functions (from inc/commands.h)
//==================================================================================================

/**
 * @brief Parse a command string and return the corresponding Command enum.
 * @param cmdStr The command string to parse
 * @return The parsed Command enum value, or CMD_UNKNOWN if not recognized
 */
Command parseCommand(const char* cmdStr);

/**
 * @brief Extract parameter string from a command.
 * @param cmdStr The full command string
 * @param cmd The parsed command enum
 * @return Pointer to the parameter substring, or NULL if no parameters
 */
const char* getCommandParams(const char* cmdStr, Command cmd);
