#include "bridge.hpp"

#include <BRepBuilderAPI_Transform.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

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
        *manifold = hasSolid(input->value) ? 1 : 0;
        return UMLCAD_OCCT_OK;
    } catch (...) {
        return UMLCAD_OCCT_INTERNAL_ERROR;
    }
}

extern "C" void umlcad_occt_shape_delete(umlcad_occt_shape* shape) {
    delete shape;
}
