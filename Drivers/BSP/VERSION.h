/**
 * @file    version.h
 * @brief   Firmware version information
 * @version X.Y.Z where:
 *          - Z (PATCH): Incremented for each bug fix
 *          - Y (MINOR): Incremented when a device/function works correctly, Z reset to 0
 *          - X (MAJOR): Incremented when all tasks completed and stable, Y and Z reset to 0
 */

#ifndef VERSION_H
#define VERSION_H

#define FIRMWARE_VERSION_MAJOR    0
#define FIRMWARE_VERSION_MINOR    1
#define FIRMWARE_VERSION_PATCH    85

#define FIRMWARE_VERSION_STRING   "0.1.85"
#define FIRMWARE_BUILD_DATE       __DATE__
#define FIRMWARE_BUILD_TIME       __TIME__

#endif /* VERSION_H */
