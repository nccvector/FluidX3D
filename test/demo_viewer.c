/*
 * FluidX3D C API Demo - Real-Time SDL2 Viewer
 *
 * Interactive viewer for GPU-rendered simulations on macOS.
 * Uses SDL2 for window management and display.
 *
 * Controls:
 *   ESC / Q     - Quit
 *   SPACE       - Pause/resume simulation
 *   R           - Reset camera rotation
 *   Arrow keys  - Rotate camera
 *   +/-         - Zoom in/out
 *   1-4         - Change visualization mode
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <SDL2/SDL.h>
#include "../lib/fluidx3d.h"

#define PI 3.14159265358979323846f
#define WINDOW_SCALE 0.6f  /* Scale down from 1920x1080 for better performance */

typedef struct {
    float rx, ry;      /* Camera rotation */
    float zoom;        /* Zoom level */
    float fov;         /* Field of view */
    bool paused;       /* Simulation paused */
    int vis_mode;      /* Visualization mode */
} ViewerState;

void handle_input(SDL_Event* event, ViewerState* state, bool* running) {
    if (event->type == SDL_QUIT) {
        *running = false;
    }
    else if (event->type == SDL_KEYDOWN) {
        switch (event->key.keysym.sym) {
            case SDLK_ESCAPE:
            case SDLK_q:
                *running = false;
                break;
            case SDLK_SPACE:
                state->paused = !state->paused;
                printf("Simulation %s\n", state->paused ? "PAUSED" : "RUNNING");
                break;
            case SDLK_r:
                state->rx = 0.5f * PI;
                state->ry = 1.0f * PI;
                state->zoom = 1.0f;
                printf("Camera reset\n");
                break;
            case SDLK_LEFT:
                state->rx -= 0.1f;
                break;
            case SDLK_RIGHT:
                state->rx += 0.1f;
                break;
            case SDLK_UP:
                state->ry -= 0.1f;
                break;
            case SDLK_DOWN:
                state->ry += 0.1f;
                break;
            case SDLK_EQUALS:
            case SDLK_PLUS:
                state->zoom *= 1.1f;
                break;
            case SDLK_MINUS:
                state->zoom /= 1.1f;
                break;
            case SDLK_1:
                state->vis_mode = 1;
                printf("Mode: Solid + Velocity Field\n");
                break;
            case SDLK_2:
                state->vis_mode = 2;
                printf("Mode: Velocity Field Only\n");
                break;
            case SDLK_3:
                state->vis_mode = 3;
                printf("Mode: Solid Only\n");
                break;
            case SDLK_4:
                state->vis_mode = 4;
                printf("Mode: Streamlines\n");
                break;
        }
    }
}

int main(int argc, char* argv[]) {
    printf("=====================================================\n");
    printf("  FluidX3D Real-Time Viewer (SDL2)\n");
    printf("=====================================================\n\n");

    /* Check graphics */
    if (!fx3d_graphics_available()) {
        printf("ERROR: Graphics not available!\n");
        return 1;
    }

    fx3d_u32 frame_w = fx3d_graphics_get_width();
    fx3d_u32 frame_h = fx3d_graphics_get_height();
    int win_w = (int)(frame_w * WINDOW_SCALE);
    int win_h = (int)(frame_h * WINDOW_SCALE);

    printf("Frame: %u x %u, Window: %d x %d\n\n", frame_w, frame_h, win_w, win_h);

    /* Initialize SDL */
    if (SDL_Init(SDL_INIT_VIDEO) < 0) {
        printf("SDL init failed: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "FluidX3D Viewer - Press Q to quit, SPACE to pause",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        win_w, win_h, SDL_WINDOW_SHOWN);
    if (!window) {
        printf("Window creation failed: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!renderer) {
        printf("Renderer creation failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    SDL_Texture* texture = SDL_CreateTexture(renderer,
        SDL_PIXELFORMAT_ARGB8888, SDL_TEXTUREACCESS_STREAMING,
        frame_w, frame_h);
    if (!texture) {
        printf("Texture creation failed: %s\n", SDL_GetError());
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    /* Create simulation */
    printf("Creating simulation...\n");
    const fx3d_u32 nx = 192, ny = 72, nz = 72;
    const fx3d_f32 cylinder_r = 10.0f;
    const fx3d_f32 u_inflow = 0.08f;
    const fx3d_f32 Re = 200.0f;
    const fx3d_f32 nu = (2.0f * cylinder_r) * u_inflow / Re;

    fx3d_lbm lbm = fx3d_lbm_create(nx, ny, nz, nu);
    if (!lbm) {
        printf("Simulation creation failed!\n");
        SDL_DestroyTexture(texture);
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    /* Initialize flow */
    fx3d_float3 center = fx3d_vec3(nx * 0.2f, ny * 0.5f, nz * 0.5f);
    fx3d_u64 N = fx3d_lbm_get_n(lbm);
    fx3d_f32* ux = fx3d_lbm_get_ux_ptr(lbm);
    fx3d_f32* uy = fx3d_lbm_get_uy_ptr(lbm);
    fx3d_u8* flags = fx3d_lbm_get_flags_ptr(lbm);

    for (fx3d_u64 n = 0; n < N; n++) {
        fx3d_u32 x, y, z;
        fx3d_lbm_coordinates(lbm, n, &x, &y, &z);

        fx3d_f32 dx = (fx3d_f32)x - center.x;
        fx3d_f32 dy = (fx3d_f32)y - center.y;
        if (sqrtf(dx*dx + dy*dy) < cylinder_r) {
            flags[n] = FX3D_TYPE_S | FX3D_TYPE_X;
        } else {
            ux[n] = u_inflow;
            uy[n] = 0.001f * sinf(0.1f * z);
        }

        if (x == 0 || x == nx-1 || y == 0 || y == ny-1) {
            flags[n] = FX3D_TYPE_E;
        }
    }

    /* Viewer state */
    ViewerState state = {
        .rx = 0.5f * PI,
        .ry = 1.0f * PI,
        .zoom = 1.2f,
        .fov = 100.0f,
        .paused = false,
        .vis_mode = 1
    };

    /* Initial graphics setup */
    fx3d_graphics_set_visualization_modes(lbm, FX3D_VIS_FLAG_LATTICE | FX3D_VIS_FIELD);
    fx3d_graphics_set_field_mode(lbm, FX3D_FIELD_VELOCITY);
    fx3d_graphics_set_slice_mode(lbm, FX3D_SLICE_Z);
    fx3d_graphics_set_slice_position(lbm, nx/2, ny/2, nz/2);

    printf("\nControls:\n");
    printf("  ESC/Q      - Quit\n");
    printf("  SPACE      - Pause/Resume\n");
    printf("  Arrows     - Rotate camera\n");
    printf("  +/-        - Zoom\n");
    printf("  R          - Reset camera\n");
    printf("  1-4        - Change visualization\n\n");

    /* Main loop */
    bool running = true;
    Uint32 last_time = SDL_GetTicks();
    int frame_count = 0;

    while (running) {
        /* Handle events */
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            handle_input(&event, &state, &running);
        }

        /* Update visualization mode */
        switch (state.vis_mode) {
            case 1:
                fx3d_graphics_set_visualization_modes(lbm, FX3D_VIS_FLAG_LATTICE | FX3D_VIS_FIELD);
                break;
            case 2:
                fx3d_graphics_set_visualization_modes(lbm, FX3D_VIS_FIELD);
                break;
            case 3:
                fx3d_graphics_set_visualization_modes(lbm, FX3D_VIS_FLAG_LATTICE);
                break;
            case 4:
                fx3d_graphics_set_visualization_modes(lbm, FX3D_VIS_FLAG_LATTICE | FX3D_VIS_STREAMLINES);
                break;
        }

        /* Run simulation */
        if (!state.paused) {
            fx3d_lbm_run(lbm, 50);
        }

        /* Update camera */
        fx3d_graphics_set_camera_centered(lbm, state.rx, state.ry, state.fov, state.zoom);

        /* Render frame on GPU */
        fx3d_i32* bitmap = fx3d_graphics_draw_frame(lbm);
        if (bitmap) {
            /* Update texture with GPU-rendered bitmap */
            SDL_UpdateTexture(texture, NULL, bitmap, frame_w * sizeof(int));
            SDL_RenderClear(renderer);
            SDL_RenderCopy(renderer, texture, NULL, NULL);
            SDL_RenderPresent(renderer);
        }

        /* FPS counter */
        frame_count++;
        Uint32 now = SDL_GetTicks();
        if (now - last_time >= 1000) {
            char title[128];
            snprintf(title, sizeof(title),
                "FluidX3D - Step: %llu, FPS: %d, %s",
                (unsigned long long)fx3d_lbm_get_t(lbm),
                frame_count,
                state.paused ? "PAUSED" : "RUNNING");
            SDL_SetWindowTitle(window, title);
            frame_count = 0;
            last_time = now;
        }
    }

    /* Cleanup */
    printf("\nShutting down...\n");
    fx3d_lbm_destroy(lbm);
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    printf("Done!\n");
    return 0;
}
