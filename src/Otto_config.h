// OttoDIY Arduino Library project 2024

#ifndef Otto_config_h
#define Otto_config_h

#define BLOCKING      false
#define NON_BLOCKING  true

#define FPS30 33	// 30 frames per second = 33 ms per frame (approximately)

#include "Otto_models.h"
#include "Otto_sound.h"
#include "Otto_mouth.h"
#include "Otto_version.h"

template <typename T> void PROGMEM_readAnything(const T *sce, T &dest) {
    memcpy_P(&dest, sce, sizeof(T));
}

template <typename T> T PROGMEM_getAnything(const T *sce) {
    static T temp;
    memcpy_P(&temp, sce, sizeof(T));
    return temp;
}

#endif // Otto_config_h
