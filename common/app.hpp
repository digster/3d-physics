// ===========================================================================
//  app.hpp — the demo framework: window, loop, input, HUD, self-test
// ---------------------------------------------------------------------------
//  Introduced in: Chapter 3 (first demo to use it), refined through Part I
//
//  Every 3D demo from Chapter 3 on is a small class that inherits App and fills
//  in a few hooks. App handles the tedious-but-important scaffolding so each
//  chapter's main.cpp can be almost entirely about the physics:
//
//      * creates the SDL window + renderer and a Renderer3D,
//      * runs a FIXED-TIMESTEP loop (the accumulator pattern) so the physics
//        is deterministic and stable regardless of frame rate,
//      * routes mouse drags to the orbit camera and provides key state,
//      * draws an on-screen HUD and the standard controls,
//      * supports `--selftest`: render a few frames headless, save a BMP, exit.
//
//  A minimal chapter looks like:
//      struct Demo : App {
//          void fixedUpdate(Real dt) override { /* step the sim */ }
//          void render(Renderer3D& r, Real alpha) override { /* draw */ }
//      };
//      int main(int argc, char** argv) { return Demo{}.run(argc, argv); }
// ===========================================================================
#pragma once

#include "math/math.hpp"
#include "render/camera.hpp"
#include "render/renderer3d.hpp"
#include "render/color.hpp"

#include <SDL3/SDL.h>
#include <memory>
#include <string>
#include <vector>

namespace p3d {

class App {
public:
    struct Config {
        std::string title = "3D Physics From Scratch";
        int  width   = 1280;
        int  height  = 720;
        Real fixedDt = Real(1) / Real(120);   // 120 Hz physics tick
    };

    App();                          // uses a default Config
    explicit App(Config cfg);
    virtual ~App();

    // The entry point a chapter's main() calls. Parses args (e.g. --selftest),
    // opens the window, runs the loop, and returns a process exit code.
    int run(int argc, char** argv);

protected:
    // --- Hooks a chapter overrides ----------------------------------------
    // Called once after the window/renderer exist but before the loop starts.
    virtual void onStart() {}
    // One deterministic physics step of length `dt` (== Config::fixedDt).
    virtual void fixedUpdate(Real /*dt*/) {}
    // Draw the scene. `alpha` in [0,1] is the fraction between the previous and
    // current physics states, for smooth rendering between ticks.
    virtual void render(Renderer3D& /*r*/, Real /*alpha*/) {}
    // Raw event access, if a chapter needs something beyond the standard input.
    virtual void onEvent(const SDL_Event& /*e*/) {}
    // A key was pressed this frame (one-shot, not repeated while held).
    virtual void onKey(SDL_Keycode /*key*/) {}
    // The user pressed R, or switched scene variant (1–9): rebuild the scene.
    virtual void onReset() {}
    // Extra lines to show in the HUD (e.g. energy, body count).
    virtual std::vector<std::string> hudLines() const { return {}; }
    // A short chapter title / hint shown at the top of the HUD.
    virtual std::string caption() const { return "3D Physics"; }

    // --- Services available to a chapter ----------------------------------
    OrbitCamera camera;                       // the shared orbit camera
    Color backgroundColor = Color::bytes(26, 28, 36);

    bool keyDown(SDL_Scancode sc) const {     // continuous "is this held?" query
        return keyState_ && keyState_[sc];
    }
    int  sceneVariant() const { return sceneVariant_; }  // 1..9, chosen with number keys
    bool paused() const       { return paused_; }
    Real simTime() const      { return simTime_; }        // accumulated sim seconds
    Real fixedDt() const      { return cfg_.fixedDt; }
    SDL_Renderer* sdlRenderer() const { return renderer_; }
    void requestQuit() { running_ = false; }

private:
    void processEvents();
    void drawHud();
    bool initSDL(bool hidden);
    void shutdownSDL();
    int  runSelfTest(int frames, const std::string& outPath);

    Config cfg_;
    SDL_Window*   window_{nullptr};
    SDL_Renderer* renderer_{nullptr};
    std::unique_ptr<Renderer3D> r3d_;

    const bool* keyState_{nullptr};  // owned by SDL; valid for process lifetime

    // Loop / timing state.
    bool  running_{true};
    bool  paused_{false};
    bool  stepOnce_{false};
    bool  showHud_{true};
    int   sceneVariant_{1};
    Real  simTime_{0};

    // Mouse-drag state for the orbit camera.
    bool  dragging_{false};
};

}  // namespace p3d
