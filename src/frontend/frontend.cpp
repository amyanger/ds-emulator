#include "frontend/frontend.hpp"

#include <SDL.h>

#include <cstdio>

namespace ds {

static constexpr int kScreenWidth  = 256;
static constexpr int kScreenHeight = 192;
static constexpr int kScreenGap    = 4;  // px between top and bottom

Frontend::Frontend() {
    fill_test_pattern();
}

Frontend::~Frontend() {
    shutdown();
}

void Frontend::fill_test_pattern() {
    // Top: horizontal red-green gradient with a blue grid.
    for (int y = 0; y < kScreenHeight; ++y) {
        for (int x = 0; x < kScreenWidth; ++x) {
            const u8 r = static_cast<u8>(x);
            const u8 g = static_cast<u8>(y);
            const u8 b = ((x % 32) == 0 || (y % 32) == 0) ? 0xFF : 0x20;
            test_pattern_top_[y * kScreenWidth + x] =
                0xFF000000u | (static_cast<u32>(r) << 16) | (static_cast<u32>(g) << 8)
                | static_cast<u32>(b);
        }
    }
    // Bottom: inverse gradient (cyan/yellow with magenta grid).
    for (int y = 0; y < kScreenHeight; ++y) {
        for (int x = 0; x < kScreenWidth; ++x) {
            const u8 r = static_cast<u8>(255 - x);
            const u8 g = static_cast<u8>(255 - y);
            const u8 b = ((x % 32) == 0 || (y % 32) == 0) ? 0xFF : 0xE0;
            test_pattern_bot_[y * kScreenWidth + x] =
                0xFF000000u | (static_cast<u32>(r) << 16) | (static_cast<u32>(g) << 8)
                | static_cast<u32>(b);
        }
    }
}

bool Frontend::init(int scale) {
    scale_ = scale < 1 ? 1 : scale;

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) != 0) {
        std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return false;
    }

    const int win_w = kScreenWidth * scale_;
    const int win_h = (kScreenHeight * 2 + kScreenGap) * scale_;

    window_ = SDL_CreateWindow(
        "ds-emulator",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        win_w, win_h,
        SDL_WINDOW_ALLOW_HIGHDPI);
    if (!window_) {
        std::fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
        return false;
    }

    renderer_ = SDL_CreateRenderer(window_, -1, SDL_RENDERER_ACCELERATED);
    if (!renderer_) {
        std::fprintf(stderr, "SDL_CreateRenderer failed: %s\n", SDL_GetError());
        return false;
    }

    tex_top_ = SDL_CreateTexture(
        renderer_, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
        kScreenWidth, kScreenHeight);
    tex_bot_ = SDL_CreateTexture(
        renderer_, SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
        kScreenWidth, kScreenHeight);
    if (!tex_top_ || !tex_bot_) {
        std::fprintf(stderr, "SDL_CreateTexture failed: %s\n", SDL_GetError());
        return false;
    }

    initialized_ = true;
    return true;
}

bool Frontend::pump_events() {
    SDL_Event ev;
    while (SDL_PollEvent(&ev) != 0) {
        if (ev.type == SDL_QUIT) {
            return false;
        }
        if (ev.type == SDL_KEYDOWN && ev.key.keysym.sym == SDLK_ESCAPE) {
            return false;
        }
    }
    return true;
}

void Frontend::present(const u32* top_rgba, const u32* bot_rgba) {
    if (!initialized_) return;

    const u32* src_top = top_rgba ? top_rgba : test_pattern_top_;
    const u32* src_bot = bot_rgba ? bot_rgba : test_pattern_bot_;

    SDL_UpdateTexture(tex_top_, nullptr, src_top, kScreenWidth * static_cast<int>(sizeof(u32)));
    SDL_UpdateTexture(tex_bot_, nullptr, src_bot, kScreenWidth * static_cast<int>(sizeof(u32)));

    SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 255);
    SDL_RenderClear(renderer_);

    const int w = kScreenWidth  * scale_;
    const int h = kScreenHeight * scale_;

    SDL_Rect dst_top{0, 0, w, h};
    SDL_Rect dst_bot{0, (kScreenHeight + kScreenGap) * scale_, w, h};

    SDL_RenderCopy(renderer_, tex_top_, nullptr, &dst_top);
    SDL_RenderCopy(renderer_, tex_bot_, nullptr, &dst_bot);

    SDL_RenderPresent(renderer_);
}

void Frontend::shutdown() {
    if (tex_top_)  { SDL_DestroyTexture(tex_top_);  tex_top_  = nullptr; }
    if (tex_bot_)  { SDL_DestroyTexture(tex_bot_);  tex_bot_  = nullptr; }
    if (renderer_) { SDL_DestroyRenderer(renderer_); renderer_ = nullptr; }
    if (window_)   { SDL_DestroyWindow(window_);    window_   = nullptr; }
    if (initialized_) {
        SDL_Quit();
        initialized_ = false;
    }
}

}  // namespace ds
