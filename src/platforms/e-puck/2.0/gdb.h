#ifndef GDB_H
#define GDB_H

#include "main.h"

//Event source used to send events to other threads
extern event_source_t gdb_status_event;

//Possible flags of the GDB status event
#define ERROR_FLAG			(1<<0)
#define RUNNING_FLAG		(1<<1)
#define PROGRAMMING_FLAG	(1<<2)
#define IDLE_FLAG			(1<<3)

/**
* Starts the GDB module
*/
void gdbStart(void);

/**
 * @brief 	Used to tell in which state is GDB
 * 			Send an event on gdb_status_event with the specified flags (can be ORed)
 */
void gdbSetFlag(eventflags_t flags);

#endif  /* GDB_H */