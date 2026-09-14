#include "bridge.hpp"
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Pnt.hxx>
#include <cmath>
#include <new>

struct umlcad_occt_shape { TopoDS_Shape value; };

namespace {
bool validSection(const double* xy,uint32_t count,double z){
    if(!xy||count<3||!std::isfinite(z)) return false;
    for(uint32_t i=0;i<count;++i){if(!std::isfinite(xy[2*i])||!std::isfinite(xy[2*i+1])) return false;}
    for(uint32_t i=0;i<count;++i){uint32_t j=(i+1)%count;if(xy[2*i]==xy[2*j]&&xy[2*i+1]==xy[2*j+1]) return false;}
    long double area=0.0L;for(uint32_t i=0;i<count;++i){uint32_t j=(i+1)%count;area+=static_cast<long double>(xy[2*i])*xy[2*j+1]-static_cast<long double>(xy[2*j])*xy[2*i+1];}
    return std::abs(area)>1e-18L;
}
TopoDS_Wire makeWire(const double* xy,uint32_t count,double z,bool& done){
    BRepBuilderAPI_MakePolygon builder;for(uint32_t i=0;i<count;++i)builder.Add(gp_Pnt(xy[2*i],xy[2*i+1],z));builder.Close();done=builder.IsDone();return done?builder.Wire():TopoDS_Wire();
}
}

extern "C" int32_t umlcad_occt_loft_polygons(const double* lower_xy,uint32_t lower_count,double lower_z,const double* upper_xy,uint32_t upper_count,double upper_z,umlcad_occt_shape** out_shape){
    if(out_shape==nullptr||lower_xy==nullptr||upper_xy==nullptr)return UMLCAD_OCCT_INVALID_ARGUMENT;*out_shape=nullptr;
    if(lower_count!=upper_count||lower_count<3||lower_z==upper_z||!validSection(lower_xy,lower_count,lower_z)||!validSection(upper_xy,upper_count,upper_z))return UMLCAD_OCCT_INVALID_ARGUMENT;
    try{
        bool lower_done=false,upper_done=false;TopoDS_Wire lower=makeWire(lower_xy,lower_count,lower_z,lower_done);TopoDS_Wire upper=makeWire(upper_xy,upper_count,upper_z,upper_done);
        if(!lower_done||!upper_done)return UMLCAD_OCCT_CONSTRUCTION_FAILED;
        BRepOffsetAPI_ThruSections loft(true,true);loft.AddWire(lower);loft.AddWire(upper);loft.Build();if(!loft.IsDone())return UMLCAD_OCCT_CONSTRUCTION_FAILED;
        TopoDS_Shape shape=loft.Shape();if(shape.IsNull())return UMLCAD_OCCT_CONSTRUCTION_FAILED;
        auto* result=new(std::nothrow) umlcad_occt_shape{shape};if(!result)return UMLCAD_OCCT_INTERNAL_ERROR;*out_shape=result;return UMLCAD_OCCT_OK;
    }catch(...){return UMLCAD_OCCT_INTERNAL_ERROR;}
}
