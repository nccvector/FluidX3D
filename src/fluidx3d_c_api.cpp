/*
 * FluidX3D C API Implementation
 *
 * This file implements the C wrapper around the FluidX3D C++ library.
 */

#include "fluidx3d.h"
#include "lbm.hpp"
#include "shapes.hpp"
#include "units.hpp"
#include "utilities.hpp"

#include <cstring>
#include <cmath>

/* ============================================================================
 * Internal Helpers
 * ============================================================================ */

static inline float3 to_float3(fx3d_float3 v) {
    return float3(v.x, v.y, v.z);
}

static inline fx3d_float3 from_float3(const float3& v) {
    fx3d_float3 r = {v.x, v.y, v.z};
    return r;
}

static inline float3x3 to_float3x3(fx3d_float3x3 m) {
    return float3x3(
        m.m[0], m.m[1], m.m[2],
        m.m[3], m.m[4], m.m[5],
        m.m[6], m.m[7], m.m[8]
    );
}

static inline fx3d_float3x3 from_float3x3(const float3x3& m) {
    fx3d_float3x3 r;
    r.m[0] = m.xx; r.m[1] = m.xy; r.m[2] = m.xz;
    r.m[3] = m.yx; r.m[4] = m.yy; r.m[5] = m.yz;
    r.m[6] = m.zx; r.m[7] = m.zy; r.m[8] = m.zz;
    return r;
}

/* ============================================================================
 * LBM Simulation API
 * ============================================================================ */

extern "C" {

fx3d_lbm fx3d_lbm_create(fx3d_u32 nx, fx3d_u32 ny, fx3d_u32 nz, fx3d_f32 nu) {
    try {
        LBM* lbm = new LBM(nx, ny, nz, nu);
        return reinterpret_cast<fx3d_lbm>(lbm);
    } catch (...) {
        return nullptr;
    }
}

fx3d_lbm fx3d_lbm_create_with_force(fx3d_u32 nx, fx3d_u32 ny, fx3d_u32 nz,
                                     fx3d_f32 nu, fx3d_f32 fx, fx3d_f32 fy, fx3d_f32 fz) {
    try {
        LBM* lbm = new LBM(nx, ny, nz, nu, fx, fy, fz);
        return reinterpret_cast<fx3d_lbm>(lbm);
    } catch (...) {
        return nullptr;
    }
}

void fx3d_lbm_destroy(fx3d_lbm lbm) {
    if (lbm) {
        delete reinterpret_cast<LBM*>(lbm);
    }
}

void fx3d_lbm_run(fx3d_lbm lbm, fx3d_u64 steps) {
    if (lbm) {
        LBM* l = reinterpret_cast<LBM*>(lbm);
        if (steps == 0) {
            l->run();
        } else {
            l->run(steps);
        }
    }
}

void fx3d_lbm_reset(fx3d_lbm lbm) {
    if (lbm) {
        reinterpret_cast<LBM*>(lbm)->reset();
    }
}

void fx3d_lbm_update_fields(fx3d_lbm lbm) {
    if (lbm) {
        reinterpret_cast<LBM*>(lbm)->update_fields();
    }
}

/* ============================================================================
 * Grid Information
 * ============================================================================ */

fx3d_u32 fx3d_lbm_get_nx(fx3d_lbm lbm) {
    return lbm ? reinterpret_cast<LBM*>(lbm)->get_Nx() : 0;
}

fx3d_u32 fx3d_lbm_get_ny(fx3d_lbm lbm) {
    return lbm ? reinterpret_cast<LBM*>(lbm)->get_Ny() : 0;
}

fx3d_u32 fx3d_lbm_get_nz(fx3d_lbm lbm) {
    return lbm ? reinterpret_cast<LBM*>(lbm)->get_Nz() : 0;
}

fx3d_u64 fx3d_lbm_get_n(fx3d_lbm lbm) {
    return lbm ? reinterpret_cast<LBM*>(lbm)->get_N() : 0;
}

fx3d_u64 fx3d_lbm_get_t(fx3d_lbm lbm) {
    return lbm ? reinterpret_cast<LBM*>(lbm)->get_t() : 0;
}

fx3d_f32 fx3d_lbm_get_nu(fx3d_lbm lbm) {
    return lbm ? reinterpret_cast<LBM*>(lbm)->get_nu() : 0.0f;
}

fx3d_float3 fx3d_lbm_get_size(fx3d_lbm lbm) {
    if (lbm) {
        return from_float3(reinterpret_cast<LBM*>(lbm)->size());
    }
    return fx3d_vec3(0.0f, 0.0f, 0.0f);
}

fx3d_float3 fx3d_lbm_get_center(fx3d_lbm lbm) {
    if (lbm) {
        return from_float3(reinterpret_cast<LBM*>(lbm)->center());
    }
    return fx3d_vec3(0.0f, 0.0f, 0.0f);
}

/* ============================================================================
 * Cell Data Access
 * ============================================================================ */

fx3d_u64 fx3d_lbm_index(fx3d_lbm lbm, fx3d_u32 x, fx3d_u32 y, fx3d_u32 z) {
    return lbm ? reinterpret_cast<LBM*>(lbm)->index(x, y, z) : 0;
}

void fx3d_lbm_coordinates(fx3d_lbm lbm, fx3d_u64 n, fx3d_u32* x, fx3d_u32* y, fx3d_u32* z) {
    if (lbm && x && y && z) {
        reinterpret_cast<LBM*>(lbm)->coordinates(n, *x, *y, *z);
    }
}

fx3d_u8 fx3d_lbm_get_flags(fx3d_lbm lbm, fx3d_u64 n) {
    return lbm ? reinterpret_cast<LBM*>(lbm)->flags[n] : 0;
}

void fx3d_lbm_set_flags(fx3d_lbm lbm, fx3d_u64 n, fx3d_u8 flags) {
    if (lbm) {
        reinterpret_cast<LBM*>(lbm)->flags[n] = flags;
    }
}

fx3d_f32 fx3d_lbm_get_rho(fx3d_lbm lbm, fx3d_u64 n) {
    return lbm ? reinterpret_cast<LBM*>(lbm)->rho[n] : 0.0f;
}

void fx3d_lbm_set_rho(fx3d_lbm lbm, fx3d_u64 n, fx3d_f32 rho) {
    if (lbm) {
        reinterpret_cast<LBM*>(lbm)->rho[n] = rho;
    }
}

fx3d_float3 fx3d_lbm_get_u(fx3d_lbm lbm, fx3d_u64 n) {
    if (lbm) {
        LBM* l = reinterpret_cast<LBM*>(lbm);
        return fx3d_vec3(l->u.x[n], l->u.y[n], l->u.z[n]);
    }
    return fx3d_vec3(0.0f, 0.0f, 0.0f);
}

void fx3d_lbm_set_u(fx3d_lbm lbm, fx3d_u64 n, fx3d_float3 u) {
    if (lbm) {
        LBM* l = reinterpret_cast<LBM*>(lbm);
        l->u.x[n] = u.x;
        l->u.y[n] = u.y;
        l->u.z[n] = u.z;
    }
}

fx3d_f32 fx3d_lbm_get_ux(fx3d_lbm lbm, fx3d_u64 n) {
    return lbm ? reinterpret_cast<LBM*>(lbm)->u.x[n] : 0.0f;
}

fx3d_f32 fx3d_lbm_get_uy(fx3d_lbm lbm, fx3d_u64 n) {
    return lbm ? reinterpret_cast<LBM*>(lbm)->u.y[n] : 0.0f;
}

fx3d_f32 fx3d_lbm_get_uz(fx3d_lbm lbm, fx3d_u64 n) {
    return lbm ? reinterpret_cast<LBM*>(lbm)->u.z[n] : 0.0f;
}

void fx3d_lbm_set_ux(fx3d_lbm lbm, fx3d_u64 n, fx3d_f32 ux) {
    if (lbm) {
        reinterpret_cast<LBM*>(lbm)->u.x[n] = ux;
    }
}

void fx3d_lbm_set_uy(fx3d_lbm lbm, fx3d_u64 n, fx3d_f32 uy) {
    if (lbm) {
        reinterpret_cast<LBM*>(lbm)->u.y[n] = uy;
    }
}

void fx3d_lbm_set_uz(fx3d_lbm lbm, fx3d_u64 n, fx3d_f32 uz) {
    if (lbm) {
        reinterpret_cast<LBM*>(lbm)->u.z[n] = uz;
    }
}

/* ============================================================================
 * Bulk Data Transfer
 * ============================================================================ */

void fx3d_lbm_flags_write_to_device(fx3d_lbm lbm) {
    if (lbm) {
        reinterpret_cast<LBM*>(lbm)->flags.write_to_device();
    }
}

void fx3d_lbm_flags_read_from_device(fx3d_lbm lbm) {
    if (lbm) {
        reinterpret_cast<LBM*>(lbm)->flags.read_from_device();
    }
}

void fx3d_lbm_rho_write_to_device(fx3d_lbm lbm) {
    if (lbm) {
        reinterpret_cast<LBM*>(lbm)->rho.write_to_device();
    }
}

void fx3d_lbm_rho_read_from_device(fx3d_lbm lbm) {
    if (lbm) {
        reinterpret_cast<LBM*>(lbm)->rho.read_from_device();
    }
}

void fx3d_lbm_u_write_to_device(fx3d_lbm lbm) {
    if (lbm) {
        reinterpret_cast<LBM*>(lbm)->u.write_to_device();
    }
}

void fx3d_lbm_u_read_from_device(fx3d_lbm lbm) {
    if (lbm) {
        reinterpret_cast<LBM*>(lbm)->u.read_from_device();
    }
}

/* ============================================================================
 * Force Field API
 * ============================================================================ */

void fx3d_lbm_update_force_field(fx3d_lbm lbm) {
#ifdef FORCE_FIELD
    if (lbm) {
        reinterpret_cast<LBM*>(lbm)->update_force_field();
    }
#endif
}

fx3d_float3 fx3d_lbm_object_center_of_mass(fx3d_lbm lbm, fx3d_u8 flag_marker) {
#ifdef FORCE_FIELD
    if (lbm) {
        return from_float3(reinterpret_cast<LBM*>(lbm)->object_center_of_mass(flag_marker));
    }
#endif
    return fx3d_vec3(0.0f, 0.0f, 0.0f);
}

fx3d_float3 fx3d_lbm_object_force(fx3d_lbm lbm, fx3d_u8 flag_marker) {
#ifdef FORCE_FIELD
    if (lbm) {
        return from_float3(reinterpret_cast<LBM*>(lbm)->object_force(flag_marker));
    }
#endif
    return fx3d_vec3(0.0f, 0.0f, 0.0f);
}

fx3d_float3 fx3d_lbm_object_torque(fx3d_lbm lbm, fx3d_float3 rotation_center, fx3d_u8 flag_marker) {
#ifdef FORCE_FIELD
    if (lbm) {
        return from_float3(reinterpret_cast<LBM*>(lbm)->object_torque(to_float3(rotation_center), flag_marker));
    }
#endif
    return fx3d_vec3(0.0f, 0.0f, 0.0f);
}

/* ============================================================================
 * Moving Boundaries API
 * ============================================================================ */

void fx3d_lbm_update_moving_boundaries(fx3d_lbm lbm) {
#ifdef MOVING_BOUNDARIES
    if (lbm) {
        reinterpret_cast<LBM*>(lbm)->update_moving_boundaries();
    }
#endif
}

/* ============================================================================
 * Mesh API
 * ============================================================================ */

fx3d_mesh fx3d_mesh_load_stl(const char* path, fx3d_f32 scale) {
    try {
        Mesh* mesh = read_stl(string(path), scale);
        return reinterpret_cast<fx3d_mesh>(mesh);
    } catch (...) {
        return nullptr;
    }
}

fx3d_mesh fx3d_mesh_load_stl_positioned(const char* path, fx3d_float3 box_size,
                                         fx3d_float3 center, fx3d_f32 size) {
    try {
        Mesh* mesh = read_stl(string(path), to_float3(box_size), to_float3(center), size);
        return reinterpret_cast<fx3d_mesh>(mesh);
    } catch (...) {
        return nullptr;
    }
}

void fx3d_mesh_destroy(fx3d_mesh mesh) {
    if (mesh) {
        delete reinterpret_cast<Mesh*>(mesh);
    }
}

fx3d_float3 fx3d_mesh_get_center(fx3d_mesh mesh) {
    if (mesh) {
        return from_float3(reinterpret_cast<Mesh*>(mesh)->get_center());
    }
    return fx3d_vec3(0.0f, 0.0f, 0.0f);
}

fx3d_float3 fx3d_mesh_get_pmin(fx3d_mesh mesh) {
    if (mesh) {
        return from_float3(reinterpret_cast<Mesh*>(mesh)->pmin);
    }
    return fx3d_vec3(0.0f, 0.0f, 0.0f);
}

fx3d_float3 fx3d_mesh_get_pmax(fx3d_mesh mesh) {
    if (mesh) {
        return from_float3(reinterpret_cast<Mesh*>(mesh)->pmax);
    }
    return fx3d_vec3(0.0f, 0.0f, 0.0f);
}

fx3d_u32 fx3d_mesh_get_triangle_count(fx3d_mesh mesh) {
    return mesh ? reinterpret_cast<Mesh*>(mesh)->triangle_number : 0;
}

void fx3d_mesh_translate(fx3d_mesh mesh, fx3d_float3 offset) {
    if (mesh) {
        reinterpret_cast<Mesh*>(mesh)->translate(to_float3(offset));
    }
}

void fx3d_mesh_scale(fx3d_mesh mesh, fx3d_f32 factor) {
    if (mesh) {
        reinterpret_cast<Mesh*>(mesh)->scale(factor);
    }
}

void fx3d_mesh_rotate(fx3d_mesh mesh, fx3d_float3x3 rotation) {
    if (mesh) {
        reinterpret_cast<Mesh*>(mesh)->rotate(to_float3x3(rotation));
    }
}

void fx3d_mesh_set_center(fx3d_mesh mesh, fx3d_float3 center) {
    if (mesh) {
        reinterpret_cast<Mesh*>(mesh)->set_center(to_float3(center));
    }
}

/* ============================================================================
 * Voxelization API
 * ============================================================================ */

void fx3d_lbm_voxelize_mesh(fx3d_lbm lbm, fx3d_mesh mesh, fx3d_u8 flag) {
    if (lbm && mesh) {
        reinterpret_cast<LBM*>(lbm)->voxelize_mesh_on_device(
            reinterpret_cast<Mesh*>(mesh), flag);
    }
}

void fx3d_lbm_voxelize_mesh_moving(fx3d_lbm lbm, fx3d_mesh mesh, fx3d_u8 flag,
                                    fx3d_float3 rotation_center,
                                    fx3d_float3 linear_velocity,
                                    fx3d_float3 rotational_velocity) {
    if (lbm && mesh) {
        reinterpret_cast<LBM*>(lbm)->voxelize_mesh_on_device(
            reinterpret_cast<Mesh*>(mesh), flag,
            to_float3(rotation_center),
            to_float3(linear_velocity),
            to_float3(rotational_velocity));
    }
}

void fx3d_lbm_unvoxelize_mesh(fx3d_lbm lbm, fx3d_mesh mesh, fx3d_u8 flag) {
    if (lbm && mesh) {
        reinterpret_cast<LBM*>(lbm)->unvoxelize_mesh_on_device(
            reinterpret_cast<Mesh*>(mesh), flag);
    }
}

/* ============================================================================
 * Unit Conversion API
 * ============================================================================ */

fx3d_units fx3d_units_get(void) {
    return reinterpret_cast<fx3d_units>(&units);
}

void fx3d_units_set_m_kg_s(fx3d_units u,
                            fx3d_f32 lbm_x, fx3d_f32 lbm_u, fx3d_f32 lbm_rho,
                            fx3d_f32 si_x, fx3d_f32 si_u, fx3d_f32 si_rho) {
    if (u) {
        reinterpret_cast<Units*>(u)->set_m_kg_s(lbm_x, lbm_u, lbm_rho, si_x, si_u, si_rho);
    }
}

fx3d_f32 fx3d_units_x(fx3d_units u, fx3d_f32 si_x) {
    return u ? reinterpret_cast<Units*>(u)->x(si_x) : si_x;
}

fx3d_f32 fx3d_units_u(fx3d_units u, fx3d_f32 si_u) {
    return u ? reinterpret_cast<Units*>(u)->u(si_u) : si_u;
}

fx3d_f32 fx3d_units_rho(fx3d_units u, fx3d_f32 si_rho) {
    return u ? reinterpret_cast<Units*>(u)->rho(si_rho) : si_rho;
}

fx3d_f32 fx3d_units_nu(fx3d_units u, fx3d_f32 si_nu) {
    return u ? reinterpret_cast<Units*>(u)->nu(si_nu) : si_nu;
}

fx3d_f32 fx3d_units_F(fx3d_units u, fx3d_f32 si_F) {
    return u ? reinterpret_cast<Units*>(u)->F(si_F) : si_F;
}

fx3d_f32 fx3d_units_f(fx3d_units u, fx3d_f32 si_f) {
    return u ? reinterpret_cast<Units*>(u)->f(si_f) : si_f;
}

fx3d_u64 fx3d_units_t(fx3d_units u, fx3d_f32 si_t) {
    return u ? reinterpret_cast<Units*>(u)->t(si_t) : (fx3d_u64)si_t;
}

fx3d_f32 fx3d_units_si_x(fx3d_units u, fx3d_f32 lbm_x) {
    return u ? reinterpret_cast<Units*>(u)->si_x(lbm_x) : lbm_x;
}

fx3d_f32 fx3d_units_si_u(fx3d_units u, fx3d_f32 lbm_u) {
    return u ? reinterpret_cast<Units*>(u)->si_u(lbm_u) : lbm_u;
}

fx3d_f32 fx3d_units_si_rho(fx3d_units u, fx3d_f32 lbm_rho) {
    return u ? reinterpret_cast<Units*>(u)->si_rho(lbm_rho) : lbm_rho;
}

fx3d_f32 fx3d_units_si_nu(fx3d_units u, fx3d_f32 lbm_nu) {
    return u ? reinterpret_cast<Units*>(u)->si_nu(lbm_nu) : lbm_nu;
}

fx3d_f32 fx3d_units_si_F(fx3d_units u, fx3d_f32 lbm_F) {
    return u ? reinterpret_cast<Units*>(u)->si_F(lbm_F) : lbm_F;
}

fx3d_f32 fx3d_units_si_f(fx3d_units u, fx3d_f32 lbm_f) {
    return u ? reinterpret_cast<Units*>(u)->si_f(lbm_f) : lbm_f;
}

fx3d_f32 fx3d_units_si_t(fx3d_units u, fx3d_u64 lbm_t) {
    return u ? reinterpret_cast<Units*>(u)->si_t(lbm_t) : (fx3d_f32)lbm_t;
}

fx3d_f32 fx3d_units_si_p(fx3d_units u, fx3d_f32 lbm_p) {
    return u ? reinterpret_cast<Units*>(u)->si_p(lbm_p) : lbm_p;
}

fx3d_f32 fx3d_units_si_M(fx3d_units u, fx3d_f32 lbm_M) {
    return u ? reinterpret_cast<Units*>(u)->si_M(lbm_M) : lbm_M;
}

fx3d_f32 fx3d_units_nu_from_Re(fx3d_units u, fx3d_f32 Re, fx3d_f32 x, fx3d_f32 vel) {
    return u ? reinterpret_cast<Units*>(u)->nu_from_Re(Re, x, vel) : (x * vel / Re);
}

fx3d_f32 fx3d_units_Re(fx3d_units u, fx3d_f32 x, fx3d_f32 vel, fx3d_f32 nu) {
    return u ? reinterpret_cast<Units*>(u)->Re(x, vel, nu) : (x * vel / nu);
}

/* ============================================================================
 * Geometric Shape Helpers
 * ============================================================================ */

bool fx3d_shape_sphere(fx3d_u32 x, fx3d_u32 y, fx3d_u32 z,
                        fx3d_float3 center, fx3d_f32 radius) {
    return sphere(x, y, z, to_float3(center), radius);
}

bool fx3d_shape_cylinder(fx3d_u32 x, fx3d_u32 y, fx3d_u32 z,
                          fx3d_float3 center, fx3d_float3 axis, fx3d_f32 radius) {
    return cylinder(x, y, z, to_float3(center), to_float3(axis), radius);
}

bool fx3d_shape_cuboid(fx3d_u32 x, fx3d_u32 y, fx3d_u32 z,
                        fx3d_float3 center, fx3d_float3 size) {
    return cuboid(x, y, z, to_float3(center), to_float3(size));
}

bool fx3d_shape_plane(fx3d_u32 x, fx3d_u32 y, fx3d_u32 z,
                       fx3d_float3 point, fx3d_float3 normal) {
    return plane(x, y, z, to_float3(point), to_float3(normal));
}

/* ============================================================================
 * Utility Functions
 * ============================================================================ */

fx3d_float3x3 fx3d_rotation_identity(void) {
    return from_float3x3(float3x3(1.0f));
}

fx3d_float3x3 fx3d_rotation_x(fx3d_f32 angle) {
    return from_float3x3(float3x3(float3(1.0f, 0.0f, 0.0f), angle));
}

fx3d_float3x3 fx3d_rotation_y(fx3d_f32 angle) {
    return from_float3x3(float3x3(float3(0.0f, 1.0f, 0.0f), angle));
}

fx3d_float3x3 fx3d_rotation_z(fx3d_f32 angle) {
    return from_float3x3(float3x3(float3(0.0f, 0.0f, 1.0f), angle));
}

fx3d_float3x3 fx3d_rotation_axis(fx3d_float3 axis, fx3d_f32 angle) {
    return from_float3x3(float3x3(to_float3(axis), angle));
}

/* ============================================================================
 * VTK Export API
 * ============================================================================ */

void fx3d_lbm_rho_write_vtk(fx3d_lbm lbm, const char* path) {
    if (lbm) {
        reinterpret_cast<LBM*>(lbm)->rho.write_device_to_vtk(path ? string(path) : "");
    }
}

void fx3d_lbm_u_write_vtk(fx3d_lbm lbm, const char* path) {
    if (lbm) {
        reinterpret_cast<LBM*>(lbm)->u.write_device_to_vtk(path ? string(path) : "");
    }
}

void fx3d_lbm_flags_write_vtk(fx3d_lbm lbm, const char* path) {
    if (lbm) {
        reinterpret_cast<LBM*>(lbm)->flags.write_device_to_vtk(path ? string(path) : "");
    }
}

/* ============================================================================
 * Version Info
 * ============================================================================ */

const char* fx3d_version(void) {
    return "FluidX3D C API v1.0.0";
}

} /* extern "C" */
