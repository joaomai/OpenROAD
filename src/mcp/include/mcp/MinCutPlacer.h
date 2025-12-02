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

#pragma once

#include <random>

#include "gpl/AbstractGraphics.h"
#include "gpl/placerBase.h"
#include "odb/db.h"
#include "gpl/Replace.h"

namespace mcp {

using InstanceGraph = std::unordered_map<gpl::Instance*, std::unordered_map<gpl::Instance*, int>>;
using InstanceVec = std::vector<gpl::Instance*>;
using InstanceMap = std::unordered_map<gpl::Instance*, int>;

class MinCutPlacer
{
 public:
  MinCutPlacer(odb::dbDatabase* db, utl::Logger* logger);
  ~MinCutPlacer();
  
  void clear();
  bool initPlacer();
  void setGraphicsInterface(const gpl::AbstractGraphics& graphics);
  void randomPlace(int threads);
  void KernighanLinPlacement(int threads, bool compact);

 private:
  struct Move { gpl::Instance *a, *b; int gain; };
  struct Partition { InstanceMap part; int cuts; };

  int KLCountCut(const InstanceGraph& adj, const InstanceVec& vertices, const InstanceMap& part);
  Partition KLPartitioner(const InstanceGraph& adj, const InstanceVec& vertices);
  void KLRecursion(const InstanceGraph& adj, const InstanceVec& vertices, const odb::Rect& region, int depth, std::vector<std::pair<int, int>>& pos);

  odb::dbDatabase* db_;
  utl::Logger* log_;
  std::unique_ptr<gpl::AbstractGraphics> graphics_;

  std::shared_ptr<gpl::PlacerBaseCommon> pbc_;
  std::vector<std::shared_ptr<gpl::PlacerBase>> pbVec_;

  std::mt19937 gen_;
  std::uniform_int_distribution<> distribution_;

  std::unordered_map<gpl::Instance*, int> inst_to_index_;
  bool compact_ = false;
};

}  // namespace mcp
