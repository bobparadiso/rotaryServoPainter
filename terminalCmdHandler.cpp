#include "Terminal.h"
#include "Trace.h"

#define ARM_DRAW_BUILD_VERSION "0.1"

#define CMD_BUF_SIZE 64

extern float targetX, targetY;

//
void terminalCmdHandler(char *cmd)
{
	if (strcmp(cmd, "help") == 0)
	{
		tracef("TRANSMITTER version:%s\r\n", ARM_DRAW_BUILD_VERSION);
		trace("command list:\r\n");
		trace("help\r\n");
	}
	else if (strcmp(cmd, "pos") == 0)
	{
		int16_t x = atoi(strtok(NULL, " \r\n"));
		int16_t y = atoi(strtok(NULL, " \r\n"));
		tracef(F("gotoPos: x:%d y:%d\r\n"), x, y);
		targetX = x;
		targetY = y;
	}
	else if (cmd)
	{
		trace("ERROR\r\n");
	}
}
