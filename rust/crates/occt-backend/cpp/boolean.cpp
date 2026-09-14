#include "bridge.hpp"

#include <BRepAlgoAPI_Fuse.hxx>
#include <TopoDS_Shape.hxx>

#include <new>

struct umlcad_occt_shape {
    TopoDS_Shape value;
};

extern "C" int32_t umlcad_occt_fuse(
    const umlcad_occt_shape* left,
    const umlcad_occt_shape* right,
    umlcad_occt_shape** out_shape) {
    if (left == nullptr || right == nullptr || out_shape == nullptr) {
        return UMLCAD_OCCT_INVALID_ARGUMENT;
    }
    *out_shape = nullptr;

    try {
        if (left->value.IsNull() || right->value.IsNull()) {
            return UMLCAD_OCCT_NULL_SHAPE;
        }

        BRepAlgoAPI_Fuse fuse(left->value, right->value);
        fuse.Build();
        if (!fuse.IsDone()) {
            return UMLCAD_OCCT_CONSTRUCTION_FAILED;
        }

        const TopoDS_Shape result_shape = fuse.Shape();
        if (result_shape.IsNull()) {
            return UMLCAD_OCCT_CONSTRUCTION_FAILED;
        }

        auto* result = new (std::nothrow) umlcad_occt_shape{result_shape};
        if (result == nullptr) {
            return UMLCAD_OCCT_INTERNAL_ERROR;
        }

        *out_shape = result;
        return UMLCAD_OCCT_OK;
    } catch (...) {
        return UMLCAD_OCCT_INTERNAL_ERROR;
    }
}
