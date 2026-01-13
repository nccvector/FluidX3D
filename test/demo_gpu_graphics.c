/*
 * FluidX3D C API Demo - GPU-Accelerated Graphics
 *
 * This demo shows how to use the GPU rendering pipeline via the C API.
 * Frames are rendered on GPU and written directly to PNG files.
 * This is ~250x more efficient than CPU-side visualization for large grids.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../lib/fluidx3d.h"

#define PI 3.14159265358979323846f

int main(int argc, char* argv[]) {
    printf("=====================================================\n");
    printf("  FluidX3D C API Demo - GPU-Accelerated Graphics\n");
    printf("=====================================================\n\n");

    printf("Library version: %s\n\n", fx3d_version());

    /* Check graphics availability */
    if (!fx3d_graphics_available()) {
        printf("ERROR: Graphics support not available!\n");
        printf("Rebuild library with GRAPHICS enabled in defines.hpp\n");
        return 1;
    }

    printf("Graphics available: YES\n");
    printf("Frame size: %u x %u\n\n", fx3d_graphics_get_width(), fx3d_graphics_get_height());

    /* Simulation parameters - 3D cylinder for nice visualization */
    const fx3d_u32 nx = 256;
    const fx3d_u32 ny = 96;
    const fx3d_u32 nz = 96;
    const fx3d_f32 cylinder_radius = 12.0f;
    const fx3d_f32 u_inflow = 0.08f;
    const fx3d_f32 Re = 200.0f;
    const fx3d_f32 nu = (2.0f * cylinder_radius) * u_inflow / Re;

    printf("Creating 3D simulation...\n");
    printf("  Grid: %u x %u x %u (%.1f million cells)\n", nx, ny, nz,
           (float)(nx * ny * nz) / 1e6f);
    printf("  Cylinder radius: %.1f, Re=%.0f\n", cylinder_radius, Re);

    /* Create simulation */
    fx3d_lbm lbm = fx3d_lbm_create(nx, ny, nz, nu);
    if (!lbm) {
        printf("ERROR: Failed to create simulation\n");
        return 1;
    }

    /* Cylinder center */
    fx3d_float3 center = fx3d_vec3(nx * 0.2f, ny * 0.5f, nz * 0.5f);

    /* Initialize flow field */
    printf("Initializing flow field...\n");
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

        /* Cylinder (infinite in Z) */
        fx3d_f32 dx = (fx3d_f32)x - center.x;
        fx3d_f32 dy = (fx3d_f32)y - center.y;
        if (sqrtf(dx*dx + dy*dy) < cylinder_radius) {
            flags_ptr[n] = FX3D_TYPE_S | FX3D_TYPE_X;
            solid_count++;
        } else {
            /* Uniform inflow with small perturbation */
            ux_ptr[n] = u_inflow;
            uy_ptr[n] = 0.001f * sinf(0.1f * z);
        }

        /* Equilibrium boundaries */
        if (x == 0 || x == nx-1 || y == 0 || y == ny-1) {
            flags_ptr[n] = FX3D_TYPE_E;
        }
    }
    printf("  Solid cells: %llu\n\n", (unsigned long long)solid_count);

    /* Configure visualization */
    printf("Configuring GPU graphics...\n");

    /* Show solid boundary and velocity field */
    fx3d_graphics_set_visualization_modes(lbm, FX3D_VIS_FLAG_LATTICE | FX3D_VIS_FIELD);

    /* Set field mode to velocity */
    fx3d_graphics_set_field_mode(lbm, FX3D_FIELD_VELOCITY);

    /* Set up slice visualization - show XY plane at center */
    fx3d_graphics_set_slice_mode(lbm, FX3D_SLICE_Z);
    fx3d_graphics_set_slice_position(lbm, nx/2, ny/2, nz/2);

    /* Set initial camera position - orbit view */
    fx3d_graphics_set_camera_centered(lbm, 0.5f * PI, 1.0f * PI, 100.0f, 1.0f);

    /* Run simulation and render frames */
    printf("\nRunning simulation and rendering frames...\n");
    printf("Output directory: ./output/\n\n");

    /* Create output directory */
    system("mkdir -p output");

    const int num_frames = 20;
    const fx3d_u64 steps_per_frame = 200;

    for (int frame = 0; frame < num_frames; frame++) {
        /* Run simulation steps */
        fx3d_lbm_run(lbm, steps_per_frame);

        /* Rotate camera slowly */
        float angle = 0.5f * PI + 0.1f * frame;
        fx3d_graphics_set_camera_centered(lbm, angle, 1.0f * PI, 100.0f, 1.2f);

        /* Render frame on GPU */
        fx3d_i32* bitmap = fx3d_graphics_draw_frame(lbm);
        if (!bitmap) {
            printf("ERROR: Failed to render frame %d\n", frame);
            continue;
        }

        /* Write to PNG file */
        char filename[256];
        snprintf(filename, sizeof(filename), "output/frame_%04d", frame);
        fx3d_graphics_write_frame_png(lbm, filename);

        /* Get forces */
        fx3d_lbm_update_force_field(lbm);
        fx3d_float3 force = fx3d_lbm_object_force(lbm, FX3D_TYPE_S | FX3D_TYPE_X);

        printf("  Frame %3d: step=%6llu, drag=%.5f, lift=%.5f -> %s.png\n",
               frame, (unsigned long long)fx3d_lbm_get_t(lbm),
               force.x, force.y, filename);
    }

    /* Final summary */
    printf("\n=====================================================\n");
    printf("  Demo Complete!\n");
    printf("=====================================================\n");
    printf("\nRendered %d frames to output/*.png\n", num_frames);
    printf("Total simulation steps: %llu\n", (unsigned long long)fx3d_lbm_get_t(lbm));
    printf("\nTo view frames, open output/frame_0000.png etc.\n");
    printf("To create video: ffmpeg -i output/frame_%%04d.png -c:v libx264 video.mp4\n\n");

    /* Cleanup */
    fx3d_lbm_destroy(lbm);
    return 0;
}
