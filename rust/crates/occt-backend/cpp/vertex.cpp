#include "bridge.hpp"
#include <BRep_Tool.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Shape.hxx>
#include <cmath>
#include <new>

struct umlcad_occt_shape { TopoDS_Shape value; };

namespace {
bool finite(double value) { return std::isfinite(value); }

bool descriptor(const TopoDS_Shape& shape, const TopoDS_Shape& vertex, double* values) {
    if (values == nullptr || vertex.IsNull() || vertex.ShapeType() != TopAbs_VERTEX) return false;
    const gp_Pnt point = BRep_Tool::Pnt(TopoDS::Vertex(vertex));
    const double x = point.X(), y = point.Y(), z = point.Z();
    if (!finite(x) || !finite(y) || !finite(z)) return false;

    TopTools_IndexedDataMapOfShapeListOfShape vertex_to_edges;
    TopExp::MapShapesAndAncestors(shape, TopAbs_VERTEX, TopAbs_EDGE, vertex_to_edges);
    const int edge_index = vertex_to_edges.FindIndex(vertex);
    const uint32_t edge_use_count = edge_index == 0 ? 0U : static_cast<uint32_t>(vertex_to_edges.FindFromIndex(edge_index).Extent());
    if (edge_use_count == 0) return false;

    TopTools_IndexedDataMapOfShapeListOfShape vertex_to_faces;
    TopExp::MapShapesAndAncestors(shape, TopAbs_VERTEX, TopAbs_FACE, vertex_to_faces);
    const int face_index = vertex_to_faces.FindIndex(vertex);
    const uint32_t face_use_count = face_index == 0 ? 0U : static_cast<uint32_t>(vertex_to_faces.FindFromIndex(face_index).Extent());
    if (face_use_count == 0) return false;

    values[0] = x; values[1] = y; values[2] = z;
    values[3] = static_cast<double>(edge_use_count);
    values[4] = static_cast<double>(face_use_count);
    return true;
}
}

extern "C" int32_t umlcad_occt_shape_vertex_descriptor_count(const umlcad_occt_shape* input, uint32_t* out_count) {
    if (input == nullptr || out_count == nullptr) return UMLCAD_OCCT_INVALID_ARGUMENT;
    *out_count = 0;
    try {
        if (input->value.IsNull()) return UMLCAD_OCCT_NULL_SHAPE;
        for (TopExp_Explorer explorer(input->value, TopAbs_VERTEX); explorer.More(); explorer.Next()) ++(*out_count);
        return UMLCAD_OCCT_OK;
    } catch (...) { return UMLCAD_OCCT_INTERNAL_ERROR; }
}

extern "C" int32_t umlcad_occt_shape_vertex_descriptors(const umlcad_occt_shape* input, double* out_values, uint32_t capacity) {
    if (input == nullptr || out_values == nullptr) return UMLCAD_OCCT_INVALID_ARGUMENT;
    try {
        if (input->value.IsNull()) return UMLCAD_OCCT_NULL_SHAPE;
        uint32_t index = 0;
        for (TopExp_Explorer explorer(input->value, TopAbs_VERTEX); explorer.More(); explorer.Next()) {
            if (index >= capacity) return UMLCAD_OCCT_INVALID_ARGUMENT;
            if (!descriptor(input->value, explorer.Current(), out_values + (static_cast<size_t>(index) * 5U))) return UMLCAD_OCCT_INTERNAL_ERROR;
            ++index;
        }
        return UMLCAD_OCCT_OK;
    } catch (...) { return UMLCAD_OCCT_INTERNAL_ERROR; }
}
