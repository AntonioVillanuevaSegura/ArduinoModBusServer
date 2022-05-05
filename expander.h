//Antonio Villanueva Segura Gestion Bus expander i2c mcp23017
#ifndef EXPANDER_H_
#define EXPANDER_H_

#include <DFRobot_MCP23017.h>

void expanderSetup (DFRobot_MCP23017 *mcp);

void test(DFRobot_MCP23017 *mcp);

byte readPort(DFRobot_MCP23017 *mcp,char port);
void setPort(DFRobot_MCP23017 *mcp,byte *value);

#endif //EXPANDER_H_
