# Earliest Deadline First(EDF) Scheduler Implementation

* This implementaion is based on FreeRTOS and "Implementation and Test of EDF and LLREFSchedulers in FreeRTOS" Thesis.

* The edits discussed from the thesis are implemented to FreeRTOS files downloaded from the official website.
* The implementation is hardware-independant.
* All implementation lies within the "FreeRTOSConfig.h" and the "tasks.c" files, by replacing only these you have an EDF Scheduler running in hand.

* It's tested and verified on a set of tasks on an ARM-based microcontroller, the LPC2129, this is the hardware-dependant part.
* A full detailed report is provided. All calculations included in report are verified in three different ways,
    * By hand calculations.
    * Using SIMSO Offline Simulator.
    * Through KEIL's logic analyzer and hooks.
