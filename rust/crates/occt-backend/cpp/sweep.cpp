#include "bridge.hpp"

#include <BRepPrimAPI_MakeCylinder.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <TopoDS_Shape.hxx>

#include <cmath>
#include <new>

struct umlcad_occt_shape { TopoDS_Shape value; };

extern "C" int32_t umlcad_occt_sweep_linear_circular(
    double start_x, double start_y, double start_z,
    double end_x, double end_y, double end_z,
    double radius,
    double normal_x, double normal_y, double normal_z,
    umlcad_occt_shape** out_shape) {
    if (out_shape == nullptr) return UMLCAD_OCCT_INVALID_ARGUMENT;
    *out_shape = nullptr;
    const double values[] = {start_x, start_y, start_z, end_x, end_y, end_z, radius, normal_x, normal_y, normal_z};
    for (double value : values) {
        if (!std::isfinite(value)) return UMLCAD_OCCT_INVALID_ARGUMENT;
    }
    if (radius <= 0.0) return UMLCAD_OCCT_INVALID_ARGUMENT;
    const double dx = end_x - start_x;
    const double dy = end_y - start_y;
    const double dz = end_z - start_z;
    const double length = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (!std::isfinite(length) || length <= 0.0) return UMLCAD_OCCT_INVALID_ARGUMENT;
    const double direction_norm = length;
    const double nx = normal_x;
    const double ny = normal_y;
    const double nz = normal_z;
    const double normal_norm = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (!std::isfinite(normal_norm) || normal_norm <= 0.0) return UMLCAD_OCCT_INVALID_ARGUMENT;
    const double orthogonality = std::abs((nx * dx + ny * dy + nz * dz) / (normal_norm * direction_norm));
    if (!std::isfinite(orthogonality) || orthogonality > 1e-9) return UMLCAD_OCCT_INVALID_ARGUMENT;
    try {
        const gp_Pnt origin(start_x, start_y, start_z);
        const gp_Dir direction(dx, dy, dz);
        const gp_Ax2 axis(origin, direction);
        const TopoDS_Shape cylinder = BRepPrimAPI_MakeCylinder(axis, radius, length).Shape();
        if (cylinder.IsNull()) return UMLCAD_OCCT_CONSTRUCTION_FAILED;
        auto* result = new (std::nothrow) umlcad_occt_shape{cylinder};
        if (result == nullptr) return UMLCAD_OCCT_INTERNAL_ERROR;
        *out_shape = result;
        return UMLCAD_OCCT_OK;
    } catch (...) {
        return UMLCAD_OCCT_INTERNAL_ERROR;
    }
}
