/* Library for OSRAM Pictiva OS288048 displays.
 * See the README file for author and licensing information. In case it's
 * missing from your distribution, use the one here as the authoritative
 * version: https://github.com/dariomas/Pictiva_OS288048/blob/master/README.md
 *
 * This library is based on the Adafruit SSD1322 library and modified 
 * for OSRAM Pictiva OS288048 Grayscale OLEDs (SSD0332 drivers)
 * 
 * Thankyou to Leonardo SAMMARTANO for help and support.
 *
 * See the example sketches to learn how to use the library in your code.
 *
 * This is the main include file for the library.
 *
 * ---------------------------------------------------------------------------
 * Copyright (c) 2024 Dariomas
 *
 * MIT license, all text here must be included in any redistribution.
 */

// Version 0.0.1

#pragma once

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#ifdef __has_include

#define INCLUDED_PGMSPACE
#if (__has_include(<avr/pgmspace.h>))
#include <avr/pgmspace.h>
#else
#include <pgmspace.h>
#endif

#if (__has_include(<array>))
#include <array>
#define STD_CAPABLE 1
#else
#define STD_CAPABLE 0
#endif

#endif

#ifndef INCLUDED_PGMSPACE
#include <avr/pgmspace.h>
#define INCLUDED_PGMSPACE
#endif

#include "OS288048.h"
