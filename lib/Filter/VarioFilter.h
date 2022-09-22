// VarioFilter.h
//

#ifndef __VARIO_FILTER_H__
#define __VARIO_FILTER_H__


///////////////////////////////////////////////////////////////////////////////////////////////
// interface IVarioFilter

struct IVarioFilter
{
    virtual void update(float altitude, float va, float* altitudeFiltered, float* vv) = 0;
    virtual void reset(float altitude) = 0;
};



///////////////////////////////////////////////////////////////////////////////////////////////
//

IVarioFilter* CreateVarioFilter();


#endif // __VARIO_FILTER_H__
