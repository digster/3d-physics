// ===========================================================================
//  app.cpp — the framework loop, input handling, HUD, and self-test
// ===========================================================================
#include "app.hpp"

#include "render/debug_draw.hpp"
#include <cstring>   // std::strcmp
#include <cstdio>

namespace p3d {

App::App() : App(Config{}) {}

App::App(Config cfg) : cfg_(std::move(cfg)) {}

App::~App() { shutdownSDL(); }

// ---------------------------------------------------------------------------
//  SDL setup / teardown
// ---------------------------------------------------------------------------
bool App::initSDL(bool hidden) {
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        SDL_Log("SDL_Init failed: %s", SDL_GetError());
        return false;
    }
    const SDL_WindowFlags flags = SDL_WINDOW_RESIZABLE | (hidden ? SDL_WINDOW_HIDDEN : 0);
    if (!SDL_CreateWindowAndRenderer(cfg_.title.c_str(), cfg_.width, cfg_.height,
                                     flags, &window_, &renderer_)) {
        SDL_Log("SDL_CreateWindowAndRenderer failed: %s", SDL_GetError());
        return false;
    }
    // VSync keeps the visible window from tearing; harmless for the self-test.
    SDL_SetRenderVSync(renderer_, 1);
    // Alpha blending lets debug lines and the HUD panel composite nicely.
    SDL_SetRenderDrawBlendMode(renderer_, SDL_BLENDMODE_BLEND);

    r3d_ = std::make_unique<Renderer3D>(renderer_);
    keyState_ = SDL_GetKeyboardState(nullptr);
    return true;
}

void App::shutdownSDL() {
    r3d_.reset();
    if (renderer_) { SDL_DestroyRenderer(renderer_); renderer_ = nullptr; }
    if (window_)   { SDL_DestroyWindow(window_);     window_   = nullptr; }
    SDL_Quit();
}

// ---------------------------------------------------------------------------
//  Input
// ---------------------------------------------------------------------------
void App::processEvents() {
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
        // Let the chapter peek at every event first.
        onEvent(e);

        switch (e.type) {
        case SDL_EVENT_QUIT:
            running_ = false;
            break;

        case SDL_EVENT_KEY_DOWN:
            if (e.key.repeat) break;   // ignore auto-repeat; we want one-shots
            switch (e.key.key) {
            case SDLK_ESCAPE: running_ = false; break;
            case SDLK_SPACE:  paused_ = !paused_; break;
            case SDLK_PERIOD: stepOnce_ = true; break;          // single-step
            case SDLK_R:      simTime_ = 0; onReset(); break;    // reset scene
            case SDLK_W:      r3d_->setWireframe(!r3d_->wireframe()); break;
            case SDLK_H:      showHud_ = !showHud_; break;
            default:
                // Number keys 1..9 pick a scene variant and rebuild.
                if (e.key.key >= SDLK_1 && e.key.key <= SDLK_9) {
                    sceneVariant_ = static_cast<int>(e.key.key - SDLK_0);
                    simTime_ = 0;
                    onReset();
                }
                onKey(e.key.key);
                break;
            }
            break;

        case SDL_EVENT_MOUSE_BUTTON_DOWN:
            if (e.button.button == SDL_BUTTON_LEFT) dragging_ = true;
            break;
        case SDL_EVENT_MOUSE_BUTTON_UP:
            if (e.button.button == SDL_BUTTON_LEFT) dragging_ = false;
            break;
        case SDL_EVENT_MOUSE_MOTION:
            if (dragging_) camera.orbit(e.motion.xrel, e.motion.yrel);
            break;
        case SDL_EVENT_MOUSE_WHEEL:
            camera.zoom(e.wheel.y);
            break;

        default: break;
        }
    }
}

// ---------------------------------------------------------------------------
//  HUD (drawn with SDL's built-in 8px debug font, scaled up for readability)
// ---------------------------------------------------------------------------
void App::drawHud() {
    if (!showHud_) return;

    constexpr float kScale = 2.0f;   // 8px font → 16px on screen
    SDL_SetRenderScale(renderer_, kScale, kScale);

    // Assemble the lines: caption, status, chapter extras, then controls.
    std::vector<std::string> lines;
    lines.push_back(caption());
    {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "t=%.2fs  scene %d  %s%s",
                      static_cast<double>(simTime_), sceneVariant_,
                      paused_ ? "PAUSED" : "running",
                      r3d_->wireframe() ? "  [wire]" : "");
        lines.emplace_back(buf);
    }
    for (const std::string& s : hudLines()) lines.push_back(s);
    lines.emplace_back("");
    lines.emplace_back("drag orbit  wheel zoom  Space pause  . step");
    lines.emplace_back("R reset  1-9 scene  W wire  H hud  Esc quit");

    const float x = 8.0f;
    float y = 8.0f;
    for (const std::string& line : lines) {
        // Cheap drop shadow: draw the text once dark and offset, then bright.
        SDL_SetRenderDrawColor(renderer_, 0, 0, 0, 200);
        SDL_RenderDebugText(renderer_, x + 1, y + 1, line.c_str());
        SDL_SetRenderDrawColor(renderer_, 235, 238, 245, 255);
        SDL_RenderDebugText(renderer_, x, y, line.c_str());
        y += 10.0f;   // one line height (8px glyph + 2px gap) in scaled units
    }

    SDL_SetRenderScale(renderer_, 1.0f, 1.0f);   // restore for 3D drawing
}

// ---------------------------------------------------------------------------
//  The main loop — fixed timestep with an accumulator (see ARCHITECTURE.md)
// ---------------------------------------------------------------------------
int App::run(int argc, char** argv) {
    // --- Parse arguments --------------------------------------------------
    bool selftest = false;
    int  selftestFrames = 40;
    std::string shotPath;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--selftest") == 0) selftest = true;
        else if (std::strcmp(argv[i], "--frames") == 0 && i + 1 < argc) selftestFrames = std::atoi(argv[++i]);
        else if (std::strcmp(argv[i], "--shot") == 0 && i + 1 < argc) shotPath = argv[++i];
        else if (std::strcmp(argv[i], "--scene") == 0 && i + 1 < argc) sceneVariant_ = std::atoi(argv[++i]);
    }

    if (!initSDL(/*hidden=*/selftest)) return 1;
    onStart();
    onReset();   // build the initial scene (honours --scene)

    if (selftest) {
        if (shotPath.empty()) shotPath = "selftest.bmp";
        int code = runSelfTest(selftestFrames, shotPath);
        return code;
    }

    // --- Interactive loop --------------------------------------------------
    const Real dt = cfg_.fixedDt;
    Uint64 prev = SDL_GetPerformanceCounter();
    const double freq = static_cast<double>(SDL_GetPerformanceFrequency());
    Real accumulator = 0;

    while (running_) {
        processEvents();

        // Elapsed wall-clock time since last frame, clamped so that a long
        // stall (dragging the window, a breakpoint) cannot make the physics
        // "spiral of death" by trying to catch up over hundreds of steps.
        const Uint64 now = SDL_GetPerformanceCounter();
        Real frame = static_cast<Real>((now - prev) / freq);
        prev = now;
        if (frame > Real(0.25)) frame = Real(0.25);

        // Consume time in fixed dt chunks. This is the crux of a stable sim.
        if (!paused_) {
            accumulator += frame;
            while (accumulator >= dt) {
                fixedUpdate(dt);
                simTime_ += dt;
                accumulator -= dt;
            }
        } else if (stepOnce_) {
            fixedUpdate(dt);
            simTime_ += dt;
            stepOnce_ = false;
        }

        // How far we are into the next tick, for interpolated rendering.
        const Real alpha = paused_ ? Real(1) : (accumulator / dt);

        // --- Draw ----------------------------------------------------------
        int pw = cfg_.width, ph = cfg_.height;
        SDL_GetCurrentRenderOutputSize(renderer_, &pw, &ph);
        SDL_SetRenderDrawColor(renderer_,
            static_cast<Uint8>(backgroundColor.r * 255),
            static_cast<Uint8>(backgroundColor.g * 255),
            static_cast<Uint8>(backgroundColor.b * 255), 255);
        SDL_RenderClear(renderer_);

        const Real aspect = static_cast<Real>(pw) / static_cast<Real>(ph == 0 ? 1 : ph);
        r3d_->begin(camera.viewMatrix(), camera.projectionMatrix(aspect), pw, ph);
        render(*r3d_, alpha);
        r3d_->end();

        drawHud();
        SDL_RenderPresent(renderer_);
    }
    return 0;
}

// ---------------------------------------------------------------------------
//  Headless self-test: step & render a few frames, save a BMP, exit 0.
//  Used for CI smoke tests and for visually verifying a chapter without
//  standing in front of the window.
// ---------------------------------------------------------------------------
int App::runSelfTest(int frames, const std::string& outPath) {
    const Real dt = cfg_.fixedDt;
    int pw = cfg_.width, ph = cfg_.height;

    for (int i = 0; i < frames; ++i) {
        fixedUpdate(dt);
        simTime_ += dt;

        SDL_GetCurrentRenderOutputSize(renderer_, &pw, &ph);
        SDL_SetRenderDrawColor(renderer_,
            static_cast<Uint8>(backgroundColor.r * 255),
            static_cast<Uint8>(backgroundColor.g * 255),
            static_cast<Uint8>(backgroundColor.b * 255), 255);
        SDL_RenderClear(renderer_);

        const Real aspect = static_cast<Real>(pw) / static_cast<Real>(ph == 0 ? 1 : ph);
        r3d_->begin(camera.viewMatrix(), camera.projectionMatrix(aspect), pw, ph);
        render(*r3d_, Real(1));
        r3d_->end();
        drawHud();
        SDL_RenderPresent(renderer_);
    }

    // Grab the final frame and write it out.
    SDL_Surface* shot = SDL_RenderReadPixels(renderer_, nullptr);
    if (!shot) {
        SDL_Log("self-test: RenderReadPixels failed: %s", SDL_GetError());
        return 1;
    }
    const bool ok = SDL_SaveBMP(shot, outPath.c_str());
    SDL_DestroySurface(shot);
    if (!ok) {
        SDL_Log("self-test: SaveBMP failed: %s", SDL_GetError());
        return 1;
    }
    SDL_Log("self-test: wrote %s (%dx%d, %d frames)", outPath.c_str(), pw, ph, frames);
    return 0;
}

}  // namespace p3d
