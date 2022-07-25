//Antonio Villanueva Segura Gestion Bus expander i2c mcp23017
#ifndef EXPANDER_H_
#define EXPANDER_H_

#include <DFRobot_MCP23017.h>

void expanderSetup (DFRobot_MCP23017 *mcp);

void test(DFRobot_MCP23017 *mcp);

byte readPort(DFRobot_MCP23017 *mcp,char port);
void setPort(DFRobot_MCP23017 *mcp,byte *value);
//The modbus client activates the relays, then we activate the expander bus outputs physical outputs of the expander bus MCP23017 ...8 Coils or Relays
void setRelays(DFRobot_MCP23017 *mcp,uint8_t *outs );

#endif //EXPANDER_H_
