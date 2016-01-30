//////////////////////////////////////////////////////////////////////////////
// Name:        SVGPathSegList.h
// Author:      Alex Thuering
// Created:     2005/09/27
// RCS-ID:      $Id: SVGPathSegList.h,v 1.4 2005/11/07 17:33:15 ntalex Exp $
// Copyright:   (c) 2005 Alex Thuering
// Licence:     wxWindows licence
// Notes:       generated by genList.py
//////////////////////////////////////////////////////////////////////////////

#ifndef WX_SVG_PATH_SEG_LIST_H
#define WX_SVG_PATH_SEG_LIST_H

#include "SVGPathSeg.h"
#include <wx/dynarray.h>
WX_DECLARE_OBJARRAY(wxSVGPathSeg, wxSVGPathSegListBase);

class wxSVGPathSegList: public wxSVGPathSegListBase
{
  public:
    wxSVGPathSegList() {}
    wxSVGPathSegList(const wxSVGPathSegList& src) { DoCopy(src); }
    wxSVGPathSegList& operator=(const wxSVGPathSegList& src)
    { Clear(); DoCopy(src); return *this; }
    
    wxString GetValueAsString() const;
    void SetValueAsString(const wxString& value);
    
  protected:
    void DoCopy(const wxSVGPathSegList& src);
};

#endif // WX_SVG_PATH_SEG_LIST_H
