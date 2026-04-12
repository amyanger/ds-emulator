#include "frontend/frontend.hpp"
#include "nds.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

void print_usage(const char* argv0) {
    std::fprintf(stderr,
        "Usage: %s [<rom.nds>] [--scale N] [--frames N]\n"
        "\n"
        "Phase 0: ROM argument is ignored. Window shows a test pattern.\n"
        "Press Esc or close the window to quit.\n"
        "\n"
        "  --scale N   Integer window scale factor (default 2).\n"
        "  --frames N  Run for N frames then exit (default 0 = run until\n"
        "              window closed). Used for non-interactive testing.\n",
        argv0);
}

int parse_scale(int argc, char** argv) {
    for (int i = 1; i < argc - 1; ++i) {
        if (std::strcmp(argv[i], "--scale") == 0) {
            return std::atoi(argv[i + 1]);
        }
    }
    return 2;
}

int parse_frames(int argc, char** argv) {
    for (int i = 1; i < argc - 1; ++i) {
        if (std::strcmp(argv[i], "--frames") == 0) {
            return std::atoi(argv[i + 1]);
        }
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "-h") == 0 || std::strcmp(argv[i], "--help") == 0) {
            print_usage(argv[0]);
            return 0;
        }
    }

    const int scale      = parse_scale(argc, argv);
    const int max_frames = parse_frames(argc, argv);

    ds::NDS      nds;
    ds::Frontend frontend;

    if (!frontend.init(scale)) {
        std::fprintf(stderr, "Failed to initialize frontend.\n");
        return 1;
    }

    std::fprintf(stderr, "ds-emulator Phase 0: window open, showing test pattern.\n");
    std::fprintf(stderr, "Press Esc to quit.\n");

    int frame_count = 0;
    while (frontend.pump_events()) {
        nds.run_frame();                     // advances clock, no-op otherwise
        frontend.present(nullptr, nullptr);  // null = use test pattern
        if (max_frames > 0 && ++frame_count >= max_frames) break;
    }

    frontend.shutdown();
    std::fprintf(stderr, "Clean exit.\n");
    return 0;
}
