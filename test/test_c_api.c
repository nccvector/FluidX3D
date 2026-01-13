/*
 * FluidX3D C API Test
 *
 * Test raw pointer access for efficient visualization
 * Includes ASCII velocity magnitude visualization
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "../lib/fluidx3d.h"

/* ASCII characters for velocity magnitude (low to high) */
const char* VELOCITY_CHARS = " .:-=+*#%@";

/* Print ASCII visualization of a Z-slice of velocity magnitude */
void print_velocity_slice(fx3d_lbm lbm, fx3d_u32 z_slice) {
    fx3d_u32 nx = fx3d_lbm_get_nx(lbm);
    fx3d_u32 ny = fx3d_lbm_get_ny(lbm);
    fx3d_u64 N = fx3d_lbm_get_n(lbm);

    /* Get raw pointers for fast access */
    fx3d_f32* ux = fx3d_lbm_get_ux_ptr(lbm);
    fx3d_f32* uy = fx3d_lbm_get_uy_ptr(lbm);
    fx3d_f32* uz = fx3d_lbm_get_uz_ptr(lbm);
    fx3d_u8* flags = fx3d_lbm_get_flags_ptr(lbm);

    if (!ux || !uy || !uz || !flags) {
        printf("ERROR: Raw pointer access not available (multi-GPU?)\n");
        return;
    }

    /* Find max velocity for normalization */
    fx3d_f32 max_vel = 0.001f;
    for (fx3d_u64 n = 0; n < N; n++) {
        fx3d_f32 vel = sqrtf(ux[n]*ux[n] + uy[n]*uy[n] + uz[n]*uz[n]);
        if (vel > max_vel) max_vel = vel;
    }

    /* Print slice (subsample for terminal width) */
    int step_x = (nx > 80) ? (nx / 80) : 1;
    int step_y = (ny > 40) ? (ny / 40) : 1;

    printf("\n  Velocity magnitude at z=%u (max=%.4f):\n", z_slice, max_vel);
    printf("  ");
    for (fx3d_u32 x = 0; x < nx; x += step_x) printf("-");
    printf("\n");

    for (int y = ny - 1; y >= 0; y -= step_y) {
        printf("  ");
        for (fx3d_u32 x = 0; x < nx; x += step_x) {
            fx3d_u64 n = x + ((fx3d_u64)y + (fx3d_u64)z_slice * ny) * nx;

            if (flags[n] & FX3D_TYPE_S) {
                /* Solid cell */
                printf("#");
            } else {
                /* Fluid cell - show velocity magnitude */
                fx3d_f32 vel = sqrtf(ux[n]*ux[n] + uy[n]*uy[n] + uz[n]*uz[n]);
                int idx = (int)(9.0f * vel / max_vel);
                if (idx > 9) idx = 9;
                if (idx < 0) idx = 0;
                printf("%c", VELOCITY_CHARS[idx]);
            }
        }
        printf("\n");
    }

    printf("  ");
    for (fx3d_u32 x = 0; x < nx; x += step_x) printf("-");
    printf("\n");
    printf("  Legend: ' '=0  '.'=low  '@'=high  '#'=solid\n");
}

int main(int argc, char* argv[]) {
    printf("=====================================================\n");
    printf("  FluidX3D C API Test - Raw Pointer Access + ASCII\n");
    printf("=====================================================\n\n");

    printf("Library version: %s\n\n", fx3d_version());

    /* Simulation parameters */
    const fx3d_u32 nx = 160;
    const fx3d_u32 ny = 80;
    const fx3d_u32 nz = 1;  /* 2D slice for ASCII visualization */
    const fx3d_f32 cylinder_radius = 8.0f;
    const fx3d_f32 u_inflow = 0.1f;
    const fx3d_f32 Re = 200.0f;  /* Karman vortex street regime */
    const fx3d_f32 nu = (2.0f * cylinder_radius) * u_inflow / Re;

    printf("Grid: %u x %u x %u (2D slice)\n", nx, ny, nz);
    printf("Cylinder radius: %.1f, Re=%.0f\n", cylinder_radius, Re);
    printf("This should produce Karman vortex street!\n\n");

    /* Create simulation */
    printf("Creating simulation...\n");
    fx3d_lbm lbm = fx3d_lbm_create(nx, ny, nz, nu);
    if (!lbm) {
        printf("ERROR: Failed to create simulation\n");
        return 1;
    }

    /* Check raw pointer availability */
    printf("Multi-GPU mode: %s\n", fx3d_lbm_is_multi_gpu(lbm) ? "YES" : "NO");
    printf("Raw pointers available: %s\n\n",
           fx3d_lbm_get_ux_ptr(lbm) ? "YES" : "NO");

    /* Cylinder center */
    fx3d_float3 center = fx3d_vec3(nx * 0.25f, ny * 0.5f, 0.5f);

    /* Initialize using raw pointers (much faster than per-cell API) */
    printf("Initializing with raw pointer access...\n");

    fx3d_u64 N = fx3d_lbm_get_n(lbm);
    fx3d_f32* ux_ptr = fx3d_lbm_get_ux_ptr(lbm);
    fx3d_f32* uy_ptr = fx3d_lbm_get_uy_ptr(lbm);
    fx3d_u8* flags_ptr = fx3d_lbm_get_flags_ptr(lbm);

    if (!ux_ptr || !uy_ptr || !flags_ptr) {
        printf("ERROR: Could not get raw pointers\n");
        fx3d_lbm_destroy(lbm);
        return 1;
    }

    fx3d_u64 solid_count = 0;
    for (fx3d_u64 n = 0; n < N; n++) {
        fx3d_u32 x, y, z;
        fx3d_lbm_coordinates(lbm, n, &x, &y, &z);

        /* Cylinder */
        fx3d_f32 dx = (fx3d_f32)x - center.x;
        fx3d_f32 dy = (fx3d_f32)y - center.y;
        if (sqrtf(dx*dx + dy*dy) < cylinder_radius) {
            flags_ptr[n] = FX3D_TYPE_S | FX3D_TYPE_X;
            solid_count++;
        } else {
            /* Inflow velocity */
            ux_ptr[n] = u_inflow;
            uy_ptr[n] = 0.0f;
        }

        /* Boundaries */
        if (x == 0 || x == nx-1 || y == 0 || y == ny-1) {
            flags_ptr[n] = FX3D_TYPE_E;
        }
    }
    printf("Solid cells: %llu\n\n", (unsigned long long)solid_count);

    /* Run simulation with ASCII visualization */
    printf("Running simulation with ASCII visualization...\n");
    printf("(Flow direction: left -> right)\n");

    const int iterations = 6;
    const fx3d_u64 steps_per_iter = 500;

    for (int i = 0; i < iterations; i++) {
        fx3d_lbm_run(lbm, steps_per_iter);

        /* Read velocity data from GPU to CPU */
        fx3d_lbm_u_read_from_device(lbm);

        printf("\n--- Step %llu ---", (unsigned long long)fx3d_lbm_get_t(lbm));
        print_velocity_slice(lbm, 0);

        /* Also print drag force */
        fx3d_lbm_update_force_field(lbm);
        fx3d_float3 force = fx3d_lbm_object_force(lbm, FX3D_TYPE_S | FX3D_TYPE_X);
        printf("  Drag: %.6f, Lift: %.6f\n", force.x, force.y);
    }

    /* Cleanup */
    printf("\nDone!\n");
    fx3d_lbm_destroy(lbm);

    return 0;
}
