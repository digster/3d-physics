// ===========================================================================
//  Chapter 1 — Hello, SDL3
// ---------------------------------------------------------------------------
//  Goal: get a window on screen, run a game loop, and make something move.
//  We bounce a 2D ball under gravity. There is intentionally NO framework here
//  and NO 3D — this chapter is the bare skeleton every later demo is built on:
//
//        initialise → loop { handle input; advance the world; draw } → clean up
//
//  Along the way we meet the two ideas the rest of the course leans on:
//    * the GAME LOOP, and
//    * DELTA TIME — scaling motion by how long the last frame took, so the ball
//      moves at the same real-world speed regardless of frame rate.
//
//  We also deliberately use the *frame's* delta time to move the ball. It works
//  here, but it is subtly fragile (the physics changes if the frame rate does).
//  Chapter 5 explains the problem and fixes it with a fixed timestep — keep the
//  ball's slight behaviour changes under load in mind until then.
//
//  Read the companion page: docs/chapters/01-hello-sdl3.html
// ===========================================================================
#include <SDL3/SDL.h>
#include <cmath>
#include <cstring>
#include <vector>

// --- A filled circle via SDL_RenderGeometry --------------------------------
// SDL has no "draw a disc" call, so we build one from triangles: a fan of
// wedges around the centre. This is also a first look at SDL_RenderGeometry —
// the exact primitive our 3D renderer will lean on in Chapter 4.
static void fillCircle(SDL_Renderer* r, float cx, float cy, float radius,
                       SDL_FColor color) {
    constexpr int kSegments = 48;
    std::vector<SDL_Vertex> verts;
    std::vector<int> indices;

    // Centre vertex (index 0), then one vertex per step around the rim.
    verts.push_back({SDL_FPoint{cx, cy}, color, SDL_FPoint{0, 0}});
    for (int i = 0; i <= kSegments; ++i) {
        const float a = (float(i) / kSegments) * 2.0f * float(M_PI);
        verts.push_back({SDL_FPoint{cx + std::cos(a) * radius, cy + std::sin(a) * radius},
                         color, SDL_FPoint{0, 0}});
    }
    // Each triangle is centre + two neighbouring rim vertices.
    for (int i = 1; i <= kSegments; ++i) {
        indices.push_back(0);
        indices.push_back(i);
        indices.push_back(i + 1);
    }
    SDL_RenderGeometry(r, nullptr, verts.data(), int(verts.size()),
                       indices.data(), int(indices.size()));
}

int main(int argc, char** argv) {
    const bool selftest = (argc > 1 && std::strcmp(argv[1], "--selftest") == 0);

    // --- 1. Initialise SDL and create a window + renderer ------------------
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return 1;
    }
    const int W = 900, H = 600;
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    const SDL_WindowFlags flags = selftest ? SDL_WINDOW_HIDDEN : 0;
    if (!SDL_CreateWindowAndRenderer("Ch1 — Hello SDL3", W, H, flags, &window, &renderer)) {
        SDL_Log("CreateWindowAndRenderer failed: %s", SDL_GetError());
        return 1;
    }
    SDL_SetRenderVSync(renderer, 1);

    // --- 2. The world: a ball with a position and a velocity ---------------
    // Units here are pixels and seconds. Gravity pulls down (+y is down in
    // screen space). Restitution < 1 means the ball loses energy on each bounce.
    float px = W * 0.5f, py = 120.0f;       // position (pixels)
    float vx = 220.0f,   vy = 0.0f;         // velocity (pixels/second)
    const float radius       = 28.0f;
    const float gravity      = 1400.0f;     // pixels/second^2
    const float restitution  = 0.8f;        // bounciness, 0..1

    // --- 3. The game loop --------------------------------------------------
    bool running = true;
    Uint64 prev = SDL_GetPerformanceCounter();
    const double freq = double(SDL_GetPerformanceFrequency());
    int frameCount = 0;

    while (running) {
        // 3a. Handle input / window events.
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_EVENT_QUIT) running = false;
            if (e.type == SDL_EVENT_KEY_DOWN && e.key.key == SDLK_ESCAPE) running = false;
            // Space relaunches the ball from the top with a fresh sideways kick.
            if (e.type == SDL_EVENT_KEY_DOWN && e.key.key == SDLK_SPACE) {
                px = W * 0.5f; py = 120.0f; vx = (vx >= 0 ? -1 : 1) * 240.0f; vy = 0;
            }
        }

        // 3b. Measure how long the previous frame took: this is DELTA TIME.
        const Uint64 now = SDL_GetPerformanceCounter();
        float dt = float((now - prev) / freq);
        prev = now;
        if (dt > 0.05f) dt = 0.05f;   // clamp, so a hitch doesn't teleport the ball

        // 3c. Advance the world. This IS the physics of Chapter 1, in three
        //     lines: gravity changes velocity, velocity changes position.
        vy += gravity * dt;           // a = F/m, here just gravity
        px += vx * dt;                // integrate velocity into position...
        py += vy * dt;                // ...one step of "semi-implicit Euler"

        // 3d. Collide with the four walls: if the ball crosses an edge, put it
        //     back and reflect the velocity component that hit the wall.
        if (py + radius > H) { py = H - radius; vy = -vy * restitution; }
        if (py - radius < 0) { py = radius;     vy = -vy * restitution; }
        if (px + radius > W) { px = W - radius; vx = -vx * restitution; }
        if (px - radius < 0) { px = radius;     vx = -vx * restitution; }

        // 3e. Draw: clear to a dark background, then paint the ball.
        SDL_SetRenderDrawColor(renderer, 26, 28, 36, 255);
        SDL_RenderClear(renderer);
        fillCircle(renderer, px, py, radius, SDL_FColor{0.35f, 0.63f, 0.92f, 1.0f});

        // A one-line HUD, using SDL's built-in debug font (no font files!).
        SDL_SetRenderScale(renderer, 2.0f, 2.0f);
        SDL_SetRenderDrawColor(renderer, 230, 234, 240, 255);
        SDL_RenderDebugText(renderer, 6, 6, "Ch1: Hello SDL3   Space relaunch   Esc quit");
        SDL_SetRenderScale(renderer, 1.0f, 1.0f);

        SDL_RenderPresent(renderer);

        // In self-test mode, run a fixed number of frames then save a snapshot.
        if (selftest && ++frameCount >= 60) {
            SDL_Surface* shot = SDL_RenderReadPixels(renderer, nullptr);
            if (shot) { SDL_SaveBMP(shot, "selftest.bmp"); SDL_DestroySurface(shot); }
            SDL_Log("self-test: ch01 ran %d frames", frameCount);
            running = false;
        }
    }

    // --- 4. Clean up -------------------------------------------------------
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
