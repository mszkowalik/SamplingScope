/*
 * CLI.h
 *
 *  Created on: Apr 27, 2021
 *      Author: mszko
 */

#ifndef CLI_CLI_H_
#define CLI_CLI_H_

#include "cli_defs.h"
#include "stm32h7xx.h"


class CLI {
public:
	CLI(println_func_ptr_t print);
	virtual ~CLI();


	cli_status_t process();
	cli_status_t put(char c);


    println_func_ptr_t println; /* Function pointer to user defined println function.      */
    cmd_t *cmd_tbl;             /* Pointer to series of commands which are to be accepted. */
    size_t cmd_cnt;             /* Number of commands in cmd_tbl.                          */

	uint8_t buf[MAX_BUF_SIZE];      /* CLI Rx byte-buffer */
	uint8_t *buf_ptr;               /* Pointer to Rx byte-buffer */

	uint8_t cmd_buf[MAX_BUF_SIZE];  /* CLI command buffer */
	uint8_t *cmd_ptr;               /* Pointer to command buffer */

	const char cli_prompt[4] = ">> ";       /* CLI prompt displayed to the user */
	const char cli_unrecog[30] = "CMD: Command not recognised\r\n";
	const char *cli_error_msg[2] = {
	    "OK",
	    "Command not recognised"
	};

};

#endif /* CLI_CLI_H_ */
