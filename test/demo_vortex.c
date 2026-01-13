/*
 * FluidX3D C API Demo - Karman Vortex Street
 *
 * ASCII visualization of vortex shedding behind a cylinder
 * Shows real-time flow evolution with color-coded velocity
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>  /* for usleep */
#include "../lib/fluidx3d.h"

/* ANSI color codes for terminal */
#define RESET   "\033[0m"
#define BLACK   "\033[30m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN    "\033[36m"
#define WHITE   "\033[37m"
#define BOLD    "\033[1m"

/* Background colors */
#define BG_BLACK   "\033[40m"
#define BG_RED     "\033[41m"
#define BG_GREEN   "\033[42m"
#define BG_YELLOW  "\033[43m"
#define BG_BLUE    "\033[44m"
#define BG_MAGENTA "\033[45m"
#define BG_CYAN    "\033[46m"
#define BG_WHITE   "\033[47m"

/* Clear screen and move cursor home */
#define CLEAR_SCREEN "\033[2J\033[H"

/* Velocity magnitude to colored character */
void print_colored_char(float vel, float max_vel, int is_solid) {
    if (is_solid) {
        printf(BG_WHITE BLACK "#" RESET);
        return;
    }

    float normalized = vel / max_vel;

    if (normalized < 0.1f) {
        printf(BLUE "." RESET);
    } else if (normalized < 0.25f) {
        printf(CYAN ":" RESET);
    } else if (normalized < 0.4f) {
        printf(GREEN "-" RESET);
    } else if (normalized < 0.55f) {
        printf(GREEN "=" RESET);
    } else if (normalized < 0.7f) {
        printf(YELLOW "+" RESET);
    } else if (normalized < 0.85f) {
        printf(YELLOW "*" RESET);
    } else if (normalized < 0.95f) {
        printf(RED "#" RESET);
    } else {
        printf(RED BOLD "@" RESET);
    }
}

/* Print Y-velocity (for vorticity visualization) */
void print_vortex_char(float vy, float max_vy, int is_solid) {
    if (is_solid) {
        printf(BG_WHITE BLACK "O" RESET);
        return;
    }

    float normalized = vy / max_vy;  /* Can be negative */

    if (normalized > 0.3f) {
        printf(RED BOLD "^" RESET);      /* Strong upward */
    } else if (normalized > 0.15f) {
        printf(RED "+" RESET);           /* Upward */
    } else if (normalized > 0.05f) {
        printf(YELLOW "'" RESET);        /* Slight upward */
    } else if (normalized < -0.3f) {
        printf(CYAN BOLD "v" RESET);     /* Strong downward */
    } else if (normalized < -0.15f) {
        printf(CYAN "-" RESET);          /* Downward */
    } else if (normalized < -0.05f) {
        printf(BLUE "," RESET);          /* Slight downward */
    } else {
        printf(" ");                      /* Near zero */
    }
}

void render_frame(fx3d_lbm lbm, int mode, int frame_num, float drag, float lift) {
    fx3d_u32 nx = fx3d_lbm_get_nx(lbm);
    fx3d_u32 ny = fx3d_lbm_get_ny(lbm);
    fx3d_u64 N = fx3d_lbm_get_n(lbm);

    fx3d_f32* ux = fx3d_lbm_get_ux_ptr(lbm);
    fx3d_f32* uy = fx3d_lbm_get_uy_ptr(lbm);
    fx3d_u8* flags = fx3d_lbm_get_flags_ptr(lbm);

    /* Find max values for normalization */
    float max_vel = 0.001f;
    float max_vy = 0.001f;
    for (fx3d_u64 n = 0; n < N; n++) {
        float vel = sqrtf(ux[n]*ux[n] + uy[n]*uy[n]);
        if (vel > max_vel) max_vel = vel;
        if (fabsf(uy[n]) > max_vy) max_vy = fabsf(uy[n]);
    }

    /* Subsample for terminal */
    int step_x = (nx > 100) ? 2 : 1;
    int step_y = (ny > 50) ? 2 : 1;

    /* Print header */
    printf(CLEAR_SCREEN);
    printf(BOLD CYAN "╔════════════════════════════════════════════════════════════════════════════════╗\n" RESET);
    printf(BOLD CYAN "║" RESET "  " BOLD WHITE "FluidX3D Demo" RESET " - Karman Vortex Street (Re=200)                                  " BOLD CYAN "║\n" RESET);
    printf(BOLD CYAN "╠════════════════════════════════════════════════════════════════════════════════╣\n" RESET);
    printf(BOLD CYAN "║" RESET "  Step: " YELLOW "%6llu" RESET "  |  Drag: " GREEN "%+.4f" RESET "  |  Lift: " MAGENTA "%+.4f" RESET "  |  Mode: %s        " BOLD CYAN "║\n" RESET,
           (unsigned long long)fx3d_lbm_get_t(lbm), drag, lift,
           mode == 0 ? "Velocity " : "Vorticity");
    printf(BOLD CYAN "╚════════════════════════════════════════════════════════════════════════════════╝\n" RESET);

    /* Flow direction indicator */
    printf("  " BOLD "Flow →" RESET "\n");

    /* Top border */
    printf("  ");
    for (fx3d_u32 x = 0; x < nx; x += step_x) printf("─");
    printf("\n");

    /* Render field */
    for (int y = ny - 1; y >= 0; y -= step_y) {
        printf("  ");
        for (fx3d_u32 x = 0; x < nx; x += step_x) {
            fx3d_u64 n = x + (fx3d_u64)y * nx;
            int is_solid = (flags[n] & FX3D_TYPE_S) != 0;

            if (mode == 0) {
                /* Velocity magnitude mode */
                float vel = sqrtf(ux[n]*ux[n] + uy[n]*uy[n]);
                print_colored_char(vel, max_vel, is_solid);
            } else {
                /* Vorticity mode (Y-velocity shows rotation) */
                print_vortex_char(uy[n], max_vy, is_solid);
            }
        }
        printf("\n");
    }

    /* Bottom border */
    printf("  ");
    for (fx3d_u32 x = 0; x < nx; x += step_x) printf("─");
    printf("\n");

    /* Legend */
    if (mode == 0) {
        printf("\n  " BOLD "Velocity:" RESET " ");
        printf(BLUE "." RESET "=low ");
        printf(CYAN ":" RESET " ");
        printf(GREEN "-=" RESET " ");
        printf(YELLOW "+*" RESET " ");
        printf(RED "#@" RESET "=high ");
        printf(BG_WHITE BLACK "#" RESET "=solid\n");
    } else {
        printf("\n  " BOLD "Vorticity:" RESET " ");
        printf(RED "^+" RESET "=CCW ");
        printf(CYAN "v-" RESET "=CW ");
        printf(" =none ");
        printf(BG_WHITE BLACK "O" RESET "=solid\n");
    }
}

int main(int argc, char* argv[]) {
    printf(CLEAR_SCREEN);
    printf(BOLD CYAN "\n  FluidX3D Vortex Street Demo\n" RESET);
    printf("  Initializing GPU simulation...\n\n");

    /* Simulation parameters for nice vortex shedding */
    const fx3d_u32 nx = 200;
    const fx3d_u32 ny = 80;
    const fx3d_u32 nz = 1;
    const fx3d_f32 cylinder_radius = 6.0f;
    const fx3d_f32 u_inflow = 0.08f;
    const fx3d_f32 Re = 200.0f;  /* Good for vortex shedding */
    const fx3d_f32 nu = (2.0f * cylinder_radius) * u_inflow / Re;

    /* Create simulation */
    fx3d_lbm lbm = fx3d_lbm_create(nx, ny, nz, nu);
    if (!lbm) {
        printf(RED "ERROR: Failed to create simulation\n" RESET);
        return 1;
    }

    /* Get pointers */
    fx3d_f32* ux_ptr = fx3d_lbm_get_ux_ptr(lbm);
    fx3d_f32* uy_ptr = fx3d_lbm_get_uy_ptr(lbm);
    fx3d_u8* flags_ptr = fx3d_lbm_get_flags_ptr(lbm);

    /* Cylinder position (left side for long wake) */
    fx3d_float3 center = fx3d_vec3(nx * 0.15f, ny * 0.5f, 0.5f);

    /* Initialize flow field */
    fx3d_u64 N = fx3d_lbm_get_n(lbm);
    for (fx3d_u64 n = 0; n < N; n++) {
        fx3d_u32 x, y, z;
        fx3d_lbm_coordinates(lbm, n, &x, &y, &z);

        float dx = (float)x - center.x;
        float dy = (float)y - center.y;
        float dist = sqrtf(dx*dx + dy*dy);

        if (dist < cylinder_radius) {
            flags_ptr[n] = FX3D_TYPE_S | FX3D_TYPE_X;
        } else {
            /* Add slight perturbation to trigger instability faster */
            float perturbation = 0.001f * sinf(0.1f * y);
            ux_ptr[n] = u_inflow;
            uy_ptr[n] = perturbation;
        }

        /* Boundaries */
        if (x == 0 || x == nx-1 || y == 0 || y == ny-1) {
            flags_ptr[n] = FX3D_TYPE_E;
        }
    }

    printf("  Grid: %ux%u, Re=%.0f, Cylinder R=%.0f\n", nx, ny, Re, cylinder_radius);
    printf("  Running simulation...\n\n");
    usleep(500000);  /* 0.5 second pause */

    /* Main simulation loop */
    int mode = 0;  /* 0 = velocity, 1 = vorticity */
    int frame = 0;

    while (1) {
        /* Run simulation steps */
        fx3d_lbm_run(lbm, 200);

        /* Transfer data from GPU */
        fx3d_lbm_u_read_from_device(lbm);

        /* Get forces */
        fx3d_lbm_update_force_field(lbm);
        fx3d_float3 force = fx3d_lbm_object_force(lbm, FX3D_TYPE_S | FX3D_TYPE_X);

        /* Switch visualization mode every 20 frames */
        if (frame % 40 == 20) {
            mode = 1 - mode;
        }

        /* Render */
        render_frame(lbm, mode, frame, force.x, force.y);

        frame++;

        /* Small delay for animation */
        usleep(10000);  /* 10ms */

        /* Stop after enough frames to see vortices */
        if (frame > 50) {
            printf("\n  " BOLD GREEN "Demo complete!" RESET " Vortex street should be visible.\n\n");
            break;
        }
    }

    fx3d_lbm_destroy(lbm);
    return 0;
}
