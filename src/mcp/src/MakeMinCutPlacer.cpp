// Copyright (c) 2021, The Regents of the University of California
// All rights reserved.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "mcp/MakeMinCutPlacer.h"

#include "mcp/MinCutPlacer.h"
#include "odb/db.h"
#include "gui/gui.h"
#include "gpl/graphicsImpl.h"
#include "gpl/graphicsNone.h"
#include "utl/decode.h"

extern "C" {
extern int Mcp_Init(Tcl_Interp* interp);
}

namespace mcp {

// Tcl files encoded into strings.
extern const char* mcp_tcl_inits[];


void initMinCutPlacer(Tcl_Interp* tcl_interp)
{
  Mcp_Init(tcl_interp);
  utl::evalTclInit(tcl_interp, mcp::mcp_tcl_inits);
}

void initMinCutPlacerGraphics(MinCutPlacer* min_cut_placer, utl::Logger* log)
{
  if (gui::Gui::get() == nullptr) {
    min_cut_placer->setGraphicsInterface(gpl::GraphicsNone());
  } else {
    min_cut_placer->setGraphicsInterface(gpl::GraphicsImpl(log));
  }
}

}  // namespace mcp
