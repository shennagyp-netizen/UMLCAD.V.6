#include "bridge.hpp"

#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <cmath>
#include <new>

struct umlcad_occt_shape {
    TopoDS_Shape value;
};

namespace {

bool hasSolid(const TopoDS_Shape& shape) {
    for (TopExp_Explorer explorer(shape, TopAbs_SOLID); explorer.More(); explorer.Next()) {
        return true;
    }
    return false;
}

bool isFiniteValue(double value) {
    return std::isfinite(value);
}

bool finiteAxis(double x, double y, double z) {
    return isFiniteValue(x) && isFiniteValue(y) && isFiniteValue(z)
        && (x != 0.0 || y != 0.0 || z != 0.0);
}

bool solidBoundaryIsEdgeManifold(const TopoDS_Shape& shape) {
    if (!hasSolid(shape)) {
        return false;
    }

    TopTools_IndexedDataMapOfShapeListOfShape edge_to_faces;
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edge_to_faces);

    if (edge_to_faces.IsEmpty()) {
        return false;
    }

    for (int index = 1; index <= edge_to_faces.Extent(); ++index) {
        const TopTools_ListOfShape& incident_faces = edge_to_faces.FindFromIndex(index);
        if (incident_faces.Extent() != 2) {
            return false;
        }
    }

    return true;
}

}

extern "C" int32_t umlcad_occt_box(
    double width,
    double depth,
    double height,
    umlcad_occt_shape** out_shape) {
    if (out_shape == nullptr) {
        return UMLCAD_OCCT_INVALID_ARGUMENT;
    }
    *out_shape = nullptr;

    if (!(width > 0.0) || !(depth > 0.0) || !(height > 0.0)) {
        return UMLCAD_OCCT_INVALID_ARGUMENT;
    }

    try {
        const TopoDS_Shape box = BRepPrimAPI_MakeBox(width, depth, height).Shape();
        if (box.IsNull()) {
            return UMLCAD_OCCT_CONSTRUCTION_FAILED;
        }

        auto* result = new (std::nothrow) umlcad_occt_shape{box};
        if (result == nullptr) {
            return UMLCAD_OCCT_INTERNAL_ERROR;
        }

        *out_shape = result;
        return UMLCAD_OCCT_OK;
    } catch (...) {
        return UMLCAD_OCCT_INTERNAL_ERROR;
    }
}

extern "C" int32_t umlcad_occt_cylinder(
    double radius,
    double height,
    umlcad_occt_shape** out_shape) {
    if (out_shape == nullptr) {
        return UMLCAD_OCCT_INVALID_ARGUMENT;
    }
    *out_shape = nullptr;

    if (!(radius > 0.0) || !(height > 0.0)) {
        return UMLCAD_OCCT_INVALID_ARGUMENT;
    }

    try {
        const TopoDS_Shape cylinder = BRepPrimAPI_MakeCylinder(radius, height).Shape();
        if (cylinder.IsNull()) {
            return UMLCAD_OCCT_CONSTRUCTION_FAILED;
        }

        auto* result = new (std::nothrow) umlcad_occt_shape{cylinder};
        if (result == nullptr) {
            return UMLCAD_OCCT_INTERNAL_ERROR;
        }

        *out_shape = result;
        return UMLCAD_OCCT_OK;
    } catch (...) {
        return UMLCAD_OCCT_INTERNAL_ERROR;
    }
}

extern "C" int32_t umlcad_occt_shape_clone(
    const umlcad_occt_shape* input,
    umlcad_occt_shape** out_shape) {
    if (input == nullptr || out_shape == nullptr) {
        return UMLCAD_OCCT_INVALID_ARGUMENT;
    }
    *out_shape = nullptr;

    try {
        if (input->value.IsNull()) {
            return UMLCAD_OCCT_NULL_SHAPE;
        }

        auto* result = new (std::nothrow) umlcad_occt_shape{input->value};
        if (result == nullptr) {
            return UMLCAD_OCCT_INTERNAL_ERROR;
        }

        *out_shape = result;
        return UMLCAD_OCCT_OK;
    } catch (...) {
        return UMLCAD_OCCT_INTERNAL_ERROR;
    }
}

extern "C" int32_t umlcad_occt_shape_translate(
    const umlcad_occt_shape* input,
    double dx,
    double dy,
    double dz,
    umlcad_occt_shape** out_shape) {
    if (input == nullptr || out_shape == nullptr) {
        return UMLCAD_OCCT_INVALID_ARGUMENT;
    }
    *out_shape = nullptr;

    try {
        if (input->value.IsNull()) {
            return UMLCAD_OCCT_NULL_SHAPE;
        }

        gp_Trsf transformation;
        transformation.SetTranslation(gp_Vec(dx, dy, dz));

        BRepBuilderAPI_Transform transformer(input->value, transformation, true);
        if (!transformer.IsDone()) {
            return UMLCAD_OCCT_TRANSFORM_FAILED;
        }

        const TopoDS_Shape translated = transformer.Shape();
        if (translated.IsNull()) {
            return UMLCAD_OCCT_TRANSFORM_FAILED;
        }

        auto* result = new (std::nothrow) umlcad_occt_shape{translated};
        if (result == nullptr) {
            return UMLCAD_OCCT_INTERNAL_ERROR;
        }

        *out_shape = result;
        return UMLCAD_OCCT_OK;
    } catch (...) {
        return UMLCAD_OCCT_INTERNAL_ERROR;
    }
}

extern "C" int32_t umlcad_occt_shape_rotate(
    const umlcad_occt_shape* input,
    double axis_x,
    double axis_y,
    double axis_z,
    double angle_radians,
    umlcad_occt_shape** out_shape) {
    if (input == nullptr || out_shape == nullptr) {
        return UMLCAD_OCCT_INVALID_ARGUMENT;
    }
    *out_shape = nullptr;

    if (!finiteAxis(axis_x, axis_y, axis_z) || !isFiniteValue(angle_radians)) {
        return UMLCAD_OCCT_INVALID_ARGUMENT;
    }

    try {
        if (input->value.IsNull()) {
            return UMLCAD_OCCT_NULL_SHAPE;
        }

        const gp_Ax1 axis(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(axis_x, axis_y, axis_z));
        gp_Trsf transformation;
        transformation.SetRotation(axis, angle_radians);

        BRepBuilderAPI_Transform transformer(input->value, transformation, true);
        if (!transformer.IsDone()) {
            return UMLCAD_OCCT_TRANSFORM_FAILED;
        }

        const TopoDS_Shape rotated = transformer.Shape();
        if (rotated.IsNull()) {
            return UMLCAD_OCCT_TRANSFORM_FAILED;
        }

        auto* result = new (std::nothrow) umlcad_occt_shape{rotated};
        if (result == nullptr) {
            return UMLCAD_OCCT_INTERNAL_ERROR;
        }

        *out_shape = result;
        return UMLCAD_OCCT_OK;
    } catch (...) {
        return UMLCAD_OCCT_INTERNAL_ERROR;
    }
}

extern "C" int32_t umlcad_occt_shape_bounding_box(
    const umlcad_occt_shape* input,
    double* out_bounds) {
    if (input == nullptr || out_bounds == nullptr) {
        return UMLCAD_OCCT_INVALID_ARGUMENT;
    }

    for (int index = 0; index < 6; ++index) {
        out_bounds[index] = 0.0;
    }

    try {
        if (input->value.IsNull()) {
            return UMLCAD_OCCT_NULL_SHAPE;
        }

        Bnd_Box box;
        box.SetGap(0.0);
        BRepBndLib::AddOptimal(input->value, box, Standard_False, Standard_False);
        if (box.IsVoid()) {
            return UMLCAD_OCCT_TRANSFORM_FAILED;
        }

        double min_x;
        double min_y;
        double min_z;
        double max_x;
        double max_y;
        double max_z;
        box.Get(min_x, min_y, min_z, max_x, max_y, max_z);

        if (!isFiniteValue(min_x) || !isFiniteValue(min_y) || !isFiniteValue(min_z)
            || !isFiniteValue(max_x) || !isFiniteValue(max_y) || !isFiniteValue(max_z)) {
            return UMLCAD_OCCT_INTERNAL_ERROR;
        }

        out_bounds[0] = min_x;
        out_bounds[1] = min_y;
        out_bounds[2] = min_z;
        out_bounds[3] = max_x;
        out_bounds[4] = max_y;
        out_bounds[5] = max_z;
        return UMLCAD_OCCT_OK;
    } catch (...) {
        return UMLCAD_OCCT_INTERNAL_ERROR;
    }
}

extern "C" int32_t umlcad_occt_shape_topology_counts(
    const umlcad_occt_shape* input,
    uint32_t* out_counts) {
    if (input == nullptr || out_counts == nullptr) {
        return UMLCAD_OCCT_INVALID_ARGUMENT;
    }

    for (int index = 0; index < 5; ++index) {
        out_counts[index] = 0;
    }

    try {
        if (input->value.IsNull()) {
            return UMLCAD_OCCT_NULL_SHAPE;
        }

        const TopAbs_ShapeEnum kinds[] = {
            TopAbs_SOLID,
            TopAbs_SHELL,
            TopAbs_FACE,
            TopAbs_EDGE,
            TopAbs_VERTEX,
        };

        for (int index = 0; index < 5; ++index) {
            TopTools_IndexedMapOfShape unique_shapes;
            TopExp::MapShapes(input->value, kinds[index], unique_shapes);
            out_counts[index] = static_cast<uint32_t>(unique_shapes.Extent());
        }

        return UMLCAD_OCCT_OK;
    } catch (...) {
        return UMLCAD_OCCT_INTERNAL_ERROR;
    }
}

extern "C" int32_t umlcad_occt_shape_validate(
    const umlcad_occt_shape* input,
    int32_t* valid,
    int32_t* manifold) {
    if (input == nullptr || valid == nullptr || manifold == nullptr) {
        return UMLCAD_OCCT_INVALID_ARGUMENT;
    }

    *valid = 0;
    *manifold = 0;

    try {
        if (input->value.IsNull()) {
            return UMLCAD_OCCT_NULL_SHAPE;
        }

        const BRepCheck_Analyzer analyzer(input->value, true);
        *valid = analyzer.IsValid() ? 1 : 0;
        *manifold = solidBoundaryIsEdgeManifold(input->value) ? 1 : 0;
        return UMLCAD_OCCT_OK;
    } catch (...) {
        return UMLCAD_OCCT_INTERNAL_ERROR;
    }
}

extern "C" void umlcad_occt_shape_delete(umlcad_occt_shape* shape) {
    delete shape;
}
