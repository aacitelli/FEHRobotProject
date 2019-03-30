#ifndef POSTTEST_H
#define POSTTEST_H

// Imports 
#include <FEHSD.h> // SD Card Functions

// Deinitializing systems at the end of a run 
void deinit()
{
    SD.Printf("Running deinitialization protocols.\r\n");
    SD.CloseLog();
}

#endif