#pragma once

#include "ds/common.hpp"

// Forward-declare SDL types to keep this header SDL-free for clients.
struct SDL_Window;
struct SDL_Renderer;
struct SDL_Texture;

namespace ds {

class NDS;

// Owns the SDL2 window, renderer, textures for both DS screens, input pump,
// and audio output. Only this translation unit (and any file under
// src/frontend/) may include SDL headers.
class Frontend {
public:
    Frontend();
    ~Frontend();

    Frontend(const Frontend&)            = delete;
    Frontend& operator=(const Frontend&) = delete;

    // Initialize SDL subsystems, create window + renderer + textures.
    // Returns false on failure.
    bool init(int scale);

    // Poll SDL events. Returns false when the user requested quit (Esc or
    // window close).
    bool pump_events();

    // Upload the two screens' framebuffers and present.
    // Phase 0: passes null pointers and uses the internal test pattern instead.
    void present(const u32* top_rgba, const u32* bot_rgba);

    // Release all SDL resources.
    void shutdown();

private:
    SDL_Window*   window_    = nullptr;
    SDL_Renderer* renderer_  = nullptr;
    SDL_Texture*  tex_top_   = nullptr;
    SDL_Texture*  tex_bot_   = nullptr;
    int           scale_     = 1;
    bool          initialized_ = false;

    // Phase 0 test pattern buffer — a simple gradient.
    u32 test_pattern_top_[192 * 256] = {};
    u32 test_pattern_bot_[192 * 256] = {};

    void fill_test_pattern();
};

}  // namespace ds
