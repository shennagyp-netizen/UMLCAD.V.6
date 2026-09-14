#pragma once

#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct umlcad_occt_shape umlcad_occt_shape;

typedef enum umlcad_occt_status {
    UMLCAD_OCCT_OK = 0,
    UMLCAD_OCCT_INVALID_ARGUMENT = 1,
    UMLCAD_OCCT_NULL_SHAPE = 2,
    UMLCAD_OCCT_CONSTRUCTION_FAILED = 3,
    UMLCAD_OCCT_TRANSFORM_FAILED = 4,
    UMLCAD_OCCT_INTERNAL_ERROR = 5
} umlcad_occt_status;

int32_t umlcad_occt_box(
    double width,
    double depth,
    double height,
    umlcad_occt_shape** out_shape);

int32_t umlcad_occt_shape_clone(
    const umlcad_occt_shape* input,
    umlcad_occt_shape** out_shape);

int32_t umlcad_occt_shape_translate(
    const umlcad_occt_shape* input,
    double dx,
    double dy,
    double dz,
    umlcad_occt_shape** out_shape);

int32_t umlcad_occt_shape_rotate(
    const umlcad_occt_shape* input,
    double axis_x,
    double axis_y,
    double axis_z,
    double angle_radians,
    umlcad_occt_shape** out_shape);

int32_t umlcad_occt_shape_bounding_box(
    const umlcad_occt_shape* input,
    double* out_bounds);

int32_t umlcad_occt_shape_validate(
    const umlcad_occt_shape* input,
    int32_t* valid,
    int32_t* manifold);

void umlcad_occt_shape_delete(umlcad_occt_shape* shape);

#ifdef __cplusplus
}
#endif
