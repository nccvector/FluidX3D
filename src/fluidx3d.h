/*
 * FluidX3D C API
 *
 * A C-style wrapper around the FluidX3D C++ library for use with
 * C, Zig, Rust, Python (ctypes), and other languages via FFI.
 *
 * Usage:
 *   1. Include this header in your C/Zig/etc. code
 *   2. Link against libfluidx3d.so (Linux), libfluidx3d.dylib (macOS), or fluidx3d.dll (Windows)
 *   3. Make sure OpenCL runtime is available on the system
 */

#ifndef FLUIDX3D_H
#define FLUIDX3D_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 * Type Definitions
 * ============================================================================ */

typedef uint8_t   fx3d_u8;
typedef uint16_t  fx3d_u16;
typedef uint32_t  fx3d_u32;
typedef uint64_t  fx3d_u64;
typedef int32_t   fx3d_i32;
typedef int64_t   fx3d_i64;
typedef float     fx3d_f32;
typedef double    fx3d_f64;

/* Opaque handle types */
typedef struct fx3d_lbm_t*   fx3d_lbm;
typedef struct fx3d_mesh_t*  fx3d_mesh;
typedef struct fx3d_units_t* fx3d_units;

/* 3D vector type */
typedef struct fx3d_float3 {
    fx3d_f32 x, y, z;
} fx3d_float3;

/* 3x3 matrix type (row-major) */
typedef struct fx3d_float3x3 {
    fx3d_f32 m[9]; /* [0,1,2] = row 0, [3,4,5] = row 1, [6,7,8] = row 2 */
} fx3d_float3x3;

/* Cell type flags */
#define FX3D_TYPE_S  0x01  /* Solid boundary (stationary or moving) */
#define FX3D_TYPE_E  0x02  /* Equilibrium boundary (inflow/outflow) */
#define FX3D_TYPE_T  0x04  /* Temperature boundary */
#define FX3D_TYPE_F  0x08  /* Fluid */
#define FX3D_TYPE_I  0x10  /* Interface (free surface) */
#define FX3D_TYPE_G  0x20  /* Gas (free surface) */
#define FX3D_TYPE_X  0x40  /* Reserved marker X (for force calculation) */
#define FX3D_TYPE_Y  0x80  /* Reserved marker Y (for force calculation) */

/* Visualization modes */
#define FX3D_VIS_FLAG_LATTICE   0x01
#define FX3D_VIS_FLAG_SURFACE   0x02
#define FX3D_VIS_FIELD          0x04
#define FX3D_VIS_STREAMLINES    0x08
#define FX3D_VIS_Q_CRITERION    0x10
#define FX3D_VIS_PHI_RASTERIZE  0x20
#define FX3D_VIS_PHI_RAYTRACE   0x40
#define FX3D_VIS_PARTICLES      0x80

/* ============================================================================
 * LBM Simulation API
 * ============================================================================ */

/**
 * Create a new LBM simulation.
 *
 * @param nx Grid size in X direction
 * @param ny Grid size in Y direction
 * @param nz Grid size in Z direction
 * @param nu Kinematic viscosity in lattice units
 * @return Handle to the LBM simulation, or NULL on failure
 */
fx3d_lbm fx3d_lbm_create(fx3d_u32 nx, fx3d_u32 ny, fx3d_u32 nz, fx3d_f32 nu);

/**
 * Create a new LBM simulation with volume force.
 *
 * @param nx Grid size in X direction
 * @param ny Grid size in Y direction
 * @param nz Grid size in Z direction
 * @param nu Kinematic viscosity in lattice units
 * @param fx Force per volume in X direction
 * @param fy Force per volume in Y direction
 * @param fz Force per volume in Z direction
 * @return Handle to the LBM simulation, or NULL on failure
 */
fx3d_lbm fx3d_lbm_create_with_force(fx3d_u32 nx, fx3d_u32 ny, fx3d_u32 nz,
                                     fx3d_f32 nu, fx3d_f32 fx, fx3d_f32 fy, fx3d_f32 fz);

/**
 * Destroy an LBM simulation and free all resources.
 *
 * @param lbm Handle to the LBM simulation
 */
void fx3d_lbm_destroy(fx3d_lbm lbm);

/**
 * Run the simulation for a specified number of time steps.
 *
 * @param lbm Handle to the LBM simulation
 * @param steps Number of time steps to run (0 = run indefinitely)
 */
void fx3d_lbm_run(fx3d_lbm lbm, fx3d_u64 steps);

/**
 * Reset the simulation to initial state.
 *
 * @param lbm Handle to the LBM simulation
 */
void fx3d_lbm_reset(fx3d_lbm lbm);

/**
 * Update macroscopic fields (rho, u, T) from distribution functions.
 *
 * @param lbm Handle to the LBM simulation
 */
void fx3d_lbm_update_fields(fx3d_lbm lbm);

/* ============================================================================
 * Grid Information
 * ============================================================================ */

fx3d_u32 fx3d_lbm_get_nx(fx3d_lbm lbm);
fx3d_u32 fx3d_lbm_get_ny(fx3d_lbm lbm);
fx3d_u32 fx3d_lbm_get_nz(fx3d_lbm lbm);
fx3d_u64 fx3d_lbm_get_n(fx3d_lbm lbm);  /* Total number of cells */
fx3d_u64 fx3d_lbm_get_t(fx3d_lbm lbm);  /* Current time step */
fx3d_f32 fx3d_lbm_get_nu(fx3d_lbm lbm); /* Kinematic viscosity */

/* ============================================================================
 * Cell Data Access
 * ============================================================================ */

/**
 * Convert 3D coordinates to linear index.
 */
fx3d_u64 fx3d_lbm_index(fx3d_lbm lbm, fx3d_u32 x, fx3d_u32 y, fx3d_u32 z);

/**
 * Convert linear index to 3D coordinates.
 */
void fx3d_lbm_coordinates(fx3d_lbm lbm, fx3d_u64 n, fx3d_u32* x, fx3d_u32* y, fx3d_u32* z);

/**
 * Get/set cell flag at index n.
 */
fx3d_u8 fx3d_lbm_get_flags(fx3d_lbm lbm, fx3d_u64 n);
void fx3d_lbm_set_flags(fx3d_lbm lbm, fx3d_u64 n, fx3d_u8 flags);

/**
 * Get/set density at index n.
 */
fx3d_f32 fx3d_lbm_get_rho(fx3d_lbm lbm, fx3d_u64 n);
void fx3d_lbm_set_rho(fx3d_lbm lbm, fx3d_u64 n, fx3d_f32 rho);

/**
 * Get/set velocity at index n.
 */
fx3d_float3 fx3d_lbm_get_u(fx3d_lbm lbm, fx3d_u64 n);
void fx3d_lbm_set_u(fx3d_lbm lbm, fx3d_u64 n, fx3d_float3 u);

/**
 * Get velocity components at index n.
 */
fx3d_f32 fx3d_lbm_get_ux(fx3d_lbm lbm, fx3d_u64 n);
fx3d_f32 fx3d_lbm_get_uy(fx3d_lbm lbm, fx3d_u64 n);
fx3d_f32 fx3d_lbm_get_uz(fx3d_lbm lbm, fx3d_u64 n);
void fx3d_lbm_set_ux(fx3d_lbm lbm, fx3d_u64 n, fx3d_f32 ux);
void fx3d_lbm_set_uy(fx3d_lbm lbm, fx3d_u64 n, fx3d_f32 uy);
void fx3d_lbm_set_uz(fx3d_lbm lbm, fx3d_u64 n, fx3d_f32 uz);

/* ============================================================================
 * Bulk Data Transfer (for efficient initialization)
 * ============================================================================ */

/**
 * Copy flags array to device. Call after setting all flags.
 */
void fx3d_lbm_flags_write_to_device(fx3d_lbm lbm);

/**
 * Copy flags array from device to host.
 */
void fx3d_lbm_flags_read_from_device(fx3d_lbm lbm);

/**
 * Copy rho array to device.
 */
void fx3d_lbm_rho_write_to_device(fx3d_lbm lbm);

/**
 * Copy rho array from device to host.
 */
void fx3d_lbm_rho_read_from_device(fx3d_lbm lbm);

/**
 * Copy u (velocity) array to device.
 */
void fx3d_lbm_u_write_to_device(fx3d_lbm lbm);

/**
 * Copy u (velocity) array from device to host.
 */
void fx3d_lbm_u_read_from_device(fx3d_lbm lbm);

/* ============================================================================
 * Force Field API (requires FORCE_FIELD extension)
 * ============================================================================ */

/**
 * Update force field on solid boundaries.
 * Must be called before object_force() or object_torque().
 */
void fx3d_lbm_update_force_field(fx3d_lbm lbm);

/**
 * Get center of mass of all cells marked with flag_marker.
 *
 * @param lbm Handle to the LBM simulation
 * @param flag_marker Flag combination to match (e.g., FX3D_TYPE_S | FX3D_TYPE_X)
 * @return Center of mass position
 */
fx3d_float3 fx3d_lbm_object_center_of_mass(fx3d_lbm lbm, fx3d_u8 flag_marker);

/**
 * Get total force on all cells marked with flag_marker.
 *
 * @param lbm Handle to the LBM simulation
 * @param flag_marker Flag combination to match (e.g., FX3D_TYPE_S | FX3D_TYPE_X)
 * @return Total force vector in lattice units
 */
fx3d_float3 fx3d_lbm_object_force(fx3d_lbm lbm, fx3d_u8 flag_marker);

/**
 * Get total torque on all cells marked with flag_marker about rotation_center.
 *
 * @param lbm Handle to the LBM simulation
 * @param rotation_center Center point for torque calculation
 * @param flag_marker Flag combination to match (e.g., FX3D_TYPE_S | FX3D_TYPE_X)
 * @return Total torque vector in lattice units
 */
fx3d_float3 fx3d_lbm_object_torque(fx3d_lbm lbm, fx3d_float3 rotation_center, fx3d_u8 flag_marker);

/* ============================================================================
 * Moving Boundaries API (requires MOVING_BOUNDARIES extension)
 * ============================================================================ */

/**
 * Update moving boundary markers. Call after changing solid velocities.
 */
void fx3d_lbm_update_moving_boundaries(fx3d_lbm lbm);

/* ============================================================================
 * Mesh API
 * ============================================================================ */

/**
 * Load a mesh from an STL file.
 *
 * @param path Path to the STL file
 * @param scale Scale factor (1.0 = no scaling)
 * @return Handle to the mesh, or NULL on failure
 */
fx3d_mesh fx3d_mesh_load_stl(const char* path, fx3d_f32 scale);

/**
 * Load a mesh from an STL file with positioning.
 *
 * @param path Path to the STL file
 * @param box_size Size of the simulation box (for auto-scaling)
 * @param center Target center position
 * @param size Target size (longest dimension)
 * @return Handle to the mesh, or NULL on failure
 */
fx3d_mesh fx3d_mesh_load_stl_positioned(const char* path, fx3d_float3 box_size,
                                         fx3d_float3 center, fx3d_f32 size);

/**
 * Destroy a mesh and free resources.
 */
void fx3d_mesh_destroy(fx3d_mesh mesh);

/**
 * Get mesh center.
 */
fx3d_float3 fx3d_mesh_get_center(fx3d_mesh mesh);

/**
 * Get mesh bounding box min point.
 */
fx3d_float3 fx3d_mesh_get_pmin(fx3d_mesh mesh);

/**
 * Get mesh bounding box max point.
 */
fx3d_float3 fx3d_mesh_get_pmax(fx3d_mesh mesh);

/**
 * Get number of triangles in mesh.
 */
fx3d_u32 fx3d_mesh_get_triangle_count(fx3d_mesh mesh);

/**
 * Translate mesh by offset.
 */
void fx3d_mesh_translate(fx3d_mesh mesh, fx3d_float3 offset);

/**
 * Scale mesh by factor.
 */
void fx3d_mesh_scale(fx3d_mesh mesh, fx3d_f32 factor);

/**
 * Rotate mesh by rotation matrix.
 */
void fx3d_mesh_rotate(fx3d_mesh mesh, fx3d_float3x3 rotation);

/**
 * Set mesh center point.
 */
void fx3d_mesh_set_center(fx3d_mesh mesh, fx3d_float3 center);

/* ============================================================================
 * Voxelization API
 * ============================================================================ */

/**
 * Voxelize a mesh onto the LBM grid (stationary solid).
 *
 * @param lbm Handle to the LBM simulation
 * @param mesh Handle to the mesh
 * @param flag Cell flag to set (e.g., FX3D_TYPE_S or FX3D_TYPE_S | FX3D_TYPE_X)
 */
void fx3d_lbm_voxelize_mesh(fx3d_lbm lbm, fx3d_mesh mesh, fx3d_u8 flag);

/**
 * Voxelize a mesh with linear and rotational velocity (moving solid).
 *
 * @param lbm Handle to the LBM simulation
 * @param mesh Handle to the mesh
 * @param flag Cell flag to set
 * @param rotation_center Center of rotation
 * @param linear_velocity Linear velocity vector
 * @param rotational_velocity Angular velocity vector (rad/timestep)
 */
void fx3d_lbm_voxelize_mesh_moving(fx3d_lbm lbm, fx3d_mesh mesh, fx3d_u8 flag,
                                    fx3d_float3 rotation_center,
                                    fx3d_float3 linear_velocity,
                                    fx3d_float3 rotational_velocity);

/**
 * Remove a voxelized mesh from the LBM grid.
 */
void fx3d_lbm_unvoxelize_mesh(fx3d_lbm lbm, fx3d_mesh mesh, fx3d_u8 flag);

/* ============================================================================
 * Unit Conversion API
 * ============================================================================ */

/**
 * Get the global units object.
 */
fx3d_units fx3d_units_get(void);

/**
 * Set up unit conversion: length, velocity, density in both LBM and SI units.
 *
 * @param units Units handle
 * @param lbm_x Length in LBM units
 * @param lbm_u Velocity in LBM units
 * @param lbm_rho Density in LBM units (usually 1.0)
 * @param si_x Length in SI units (meters)
 * @param si_u Velocity in SI units (m/s)
 * @param si_rho Density in SI units (kg/m^3)
 */
void fx3d_units_set_m_kg_s(fx3d_units units,
                            fx3d_f32 lbm_x, fx3d_f32 lbm_u, fx3d_f32 lbm_rho,
                            fx3d_f32 si_x, fx3d_f32 si_u, fx3d_f32 si_rho);

/* SI to LBM conversions */
fx3d_f32 fx3d_units_x(fx3d_units units, fx3d_f32 si_x);        /* Length */
fx3d_f32 fx3d_units_u(fx3d_units units, fx3d_f32 si_u);        /* Velocity */
fx3d_f32 fx3d_units_rho(fx3d_units units, fx3d_f32 si_rho);    /* Density */
fx3d_f32 fx3d_units_nu(fx3d_units units, fx3d_f32 si_nu);      /* Kinematic viscosity */
fx3d_f32 fx3d_units_F(fx3d_units units, fx3d_f32 si_F);        /* Force */
fx3d_f32 fx3d_units_f(fx3d_units units, fx3d_f32 si_f);        /* Force per volume */
fx3d_u64 fx3d_units_t(fx3d_units units, fx3d_f32 si_t);        /* Time */

/* LBM to SI conversions */
fx3d_f32 fx3d_units_si_x(fx3d_units units, fx3d_f32 lbm_x);    /* Length */
fx3d_f32 fx3d_units_si_u(fx3d_units units, fx3d_f32 lbm_u);    /* Velocity */
fx3d_f32 fx3d_units_si_rho(fx3d_units units, fx3d_f32 lbm_rho);/* Density */
fx3d_f32 fx3d_units_si_nu(fx3d_units units, fx3d_f32 lbm_nu);  /* Kinematic viscosity */
fx3d_f32 fx3d_units_si_F(fx3d_units units, fx3d_f32 lbm_F);    /* Force */
fx3d_f32 fx3d_units_si_f(fx3d_units units, fx3d_f32 lbm_f);    /* Force per volume */
fx3d_f32 fx3d_units_si_t(fx3d_units units, fx3d_u64 lbm_t);    /* Time */
fx3d_f32 fx3d_units_si_p(fx3d_units units, fx3d_f32 lbm_p);    /* Pressure */
fx3d_f32 fx3d_units_si_M(fx3d_units units, fx3d_f32 lbm_M);    /* Torque */

/* Derived quantities */
fx3d_f32 fx3d_units_nu_from_Re(fx3d_units units, fx3d_f32 Re, fx3d_f32 x, fx3d_f32 u);
fx3d_f32 fx3d_units_Re(fx3d_units units, fx3d_f32 x, fx3d_f32 u, fx3d_f32 nu);

/* ============================================================================
 * Geometric Shape Helpers
 * ============================================================================ */

/**
 * Check if point (x,y,z) is inside a sphere.
 */
bool fx3d_shape_sphere(fx3d_u32 x, fx3d_u32 y, fx3d_u32 z,
                        fx3d_float3 center, fx3d_f32 radius);

/**
 * Check if point (x,y,z) is inside a cylinder.
 */
bool fx3d_shape_cylinder(fx3d_u32 x, fx3d_u32 y, fx3d_u32 z,
                          fx3d_float3 center, fx3d_float3 axis, fx3d_f32 radius);

/**
 * Check if point (x,y,z) is inside a cuboid.
 */
bool fx3d_shape_cuboid(fx3d_u32 x, fx3d_u32 y, fx3d_u32 z,
                        fx3d_float3 center, fx3d_float3 size);

/**
 * Check if point (x,y,z) is on the positive side of a plane.
 */
bool fx3d_shape_plane(fx3d_u32 x, fx3d_u32 y, fx3d_u32 z,
                       fx3d_float3 point, fx3d_float3 normal);

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

/**
 * Create an identity rotation matrix.
 */
fx3d_float3x3 fx3d_rotation_identity(void);

/**
 * Create a rotation matrix for rotation around X axis.
 */
fx3d_float3x3 fx3d_rotation_x(fx3d_f32 angle_radians);

/**
 * Create a rotation matrix for rotation around Y axis.
 */
fx3d_float3x3 fx3d_rotation_y(fx3d_f32 angle_radians);

/**
 * Create a rotation matrix for rotation around Z axis.
 */
fx3d_float3x3 fx3d_rotation_z(fx3d_f32 angle_radians);

/**
 * Create a rotation matrix for rotation around arbitrary axis.
 */
fx3d_float3x3 fx3d_rotation_axis(fx3d_float3 axis, fx3d_f32 angle_radians);

/**
 * Create a float3 vector.
 */
static inline fx3d_float3 fx3d_vec3(fx3d_f32 x, fx3d_f32 y, fx3d_f32 z) {
    fx3d_float3 v = {x, y, z};
    return v;
}

/**
 * Get the simulation box size as float3.
 */
fx3d_float3 fx3d_lbm_get_size(fx3d_lbm lbm);

/**
 * Get the simulation box center as float3.
 */
fx3d_float3 fx3d_lbm_get_center(fx3d_lbm lbm);

/* ============================================================================
 * VTK Export API
 * ============================================================================ */

/**
 * Export density field to VTK file.
 */
void fx3d_lbm_rho_write_vtk(fx3d_lbm lbm, const char* path);

/**
 * Export velocity field to VTK file.
 */
void fx3d_lbm_u_write_vtk(fx3d_lbm lbm, const char* path);

/**
 * Export flags field to VTK file.
 */
void fx3d_lbm_flags_write_vtk(fx3d_lbm lbm, const char* path);

/* ============================================================================
 * Version Info
 * ============================================================================ */

/**
 * Get FluidX3D C API version string.
 */
const char* fx3d_version(void);

#ifdef __cplusplus
}
#endif

#endif /* FLUIDX3D_H */
