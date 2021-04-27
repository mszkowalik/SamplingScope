/*
 * CLI.cpp
 *
 *  Created on: Apr 27, 2021
 *      Author: mszko
 */

#include "CLI.h"

#include <stdint.h>
#include <string.h>


CLI::CLI(println_func_ptr_t print) {
	/* Set buffer ptr to beginning of buf */
	buf_ptr = buf;
	println = print;
    /* Print the CLI prompt. */
	println((char*)cli_prompt);


  }

CLI::~CLI() {

}


cli_status_t CLI::process()
{
	uint8_t argc = 0;
	    char *argv[30];

	    /* Get the first token (cmd name) */
		if(cmd_buf[0] != 0)
		{
			argv[argc] = strtok((char*)cmd_buf, " ");

			/* Walk through the other tokens (parameters) */
			while((argv[argc] != NULL) && (argc < 30))
			{
				argv[++argc] = strtok(NULL, " ");
			}

			/* Search the command table for a matching command, using argv[0]
			 * which is the command name. */
			for(size_t i = 0 ; i < cmd_cnt ; i++)
			{
				if(strcmp(argv[0], cmd_tbl[i].cmd) == 0)
				{
					/* Found a match, execute the associated function. */

					// clear buffer
					cmd_tbl[i].func(argc, argv);
					println((char*)cli_prompt); /* Print the CLI prompt to the user.             */

					for(uint16_t i=0; i< MAX_BUF_SIZE; i++)
					{
						cmd_buf[i] = 0;
					}
					return CLI_OK;
				}
			}

			/* Command not found */
			println((char*)cli_unrecog);
			println((char*)cli_prompt); /* Print the CLI prompt to the user.             */
			for(uint16_t i=0; i< MAX_BUF_SIZE; i++)
			{
				cmd_buf[i] = 0;
			}
		}
	    return CLI_E_CMD_NOT_FOUND;
}


cli_status_t CLI::put(char c)
{
    switch(c)
    {
    case '\r':

        *buf_ptr = '\0';            /* Terminate the msg and reset the msg ptr.      */
        strcpy((char*)cmd_buf,(char*) buf);       /* Copy string to command buffer for processing. */
        buf_ptr = buf;              /* Reset buf_ptr to beginning.                   */
        //cli_print(cli, cli_prompt); /* Print the CLI prompt to the user.             */
        break;

    case '\b':
        /* Backspace. Delete character. */
        if(buf_ptr > buf)
            buf_ptr--;
        break;

    default:
        /* Normal character received, add to buffer. */
        if((buf_ptr - buf) < MAX_BUF_SIZE)
            *buf_ptr++ = c;
        else
            return CLI_E_BUF_FULL;
        break;

    }
    return CLI_E_BUF_FULL;
}
