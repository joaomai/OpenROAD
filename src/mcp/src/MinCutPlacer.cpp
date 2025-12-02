#include "mcp/MinCutPlacer.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <random>
#include <unordered_map>
#include <utility>
#include <vector>

#include "gpl/placerBase.h"
#include "gpl/nesterovBase.h"
#include "gpl/graphicsImpl.h"
#include "gpl/graphicsNone.h"
#include "gui/gui.h"
#include "odb/geom.h"
#include "sta/StaMain.hh"
#include "odb/util.h"
#include "utl/Logger.h"


namespace sta {
// Tcl files encoded into strings.
extern const char* mincutplacer_tcl_inits[];
}  // namespace sta

namespace mcp {

using utl::MCP;

extern "C" {
extern int Mcp_Init(Tcl_Interp* interp);
}

MinCutPlacer::MinCutPlacer(odb::dbDatabase* db, utl::Logger* logger) : db_(db), log_(logger)
{
  std::mt19937 gen(0);
  gen_ = gen;
  std::uniform_int_distribution<> distribution(0);
  distribution_ = distribution;
  clear();
}

MinCutPlacer::~MinCutPlacer()
{
  clear();
}

bool MinCutPlacer::initPlacer()
{
  if (pbc_) {
    log_->warn(MCP, 2, "Placer already initialized.");
    return true;
  }
  // Init PlacerBaseCommon
  gpl::PlacerBaseVars pb_vars;
  pbc_ = std::make_shared<gpl::PlacerBaseCommon>(db_, pb_vars, log_);
  if (pbc_->placeInsts().size() == 0) {
    log_->warn(MCP, 1, "No placeable instances - skipping placement.");
    return false;
  }

  // Init PlacerBase
  pbVec_.push_back(std::make_shared<gpl::PlacerBase>(db_, pbc_, log_));
  for (auto pd : db_->getChip()->getBlock()->getPowerDomains()) {
    if (pd->getGroup()) {
      pbVec_.push_back(
          std::make_shared<gpl::PlacerBase>(db_, pbc_, log_, pd->getGroup()));
    }
  }
  return true;
}

void MinCutPlacer::setGraphicsInterface(const gpl::AbstractGraphics& graphics)
{
  graphics_ = graphics.MakeNew(log_);
}


inline int MinCutPlacer::KLCountCut(const InstanceGraph& adj, const InstanceVec& vertices, const InstanceMap& part) 
{
  int cut = 0;
  for (auto& u : vertices) {
    for (auto& v : vertices) {
      if (adj.at(u).contains(v)) {
        cut += (part.at(u) != part.at(v)) * adj.at(u).at(v);
      }
    }
  }
  return cut / 2;
}

MinCutPlacer::Partition MinCutPlacer::KLPartitioner(const InstanceGraph& adj, const InstanceVec& vertices)
{
  const int n = vertices.size();
  InstanceMap part;

  std::vector<std::pair<int64_t, gpl::Instance*>> area_index;
  for (auto& inst : vertices) {
    area_index.emplace_back(inst->area(), inst);
  }

  std::ranges::sort(area_index);
  for (int i = 0; i < n; i++) {
    part[area_index[i].second] = i % 2;
  }

  bool improved = true;
  int bestCut = KLCountCut(adj, vertices, part);

  while (improved) {
    improved = false;

    InstanceMap locked;
    InstanceMap D;

    for (auto& u : vertices) {
      for (auto& v : vertices) {
        if (adj.at(u).contains(v)) {
          D[u] += (part.at(u) == part.at(v)) ? -1 : 1;
        }
      }
    }

    std::vector<Move> moves;

    for (int step = 0; step < n / 2; step++) {
      gpl::Instance* bestA = nullptr;
      gpl::Instance* bestB = nullptr;
      int bestGain = std::numeric_limits<int>::min();

      for (auto& a : vertices) {
        if (!locked[a] && part[a] == 0) {
          for (auto& b : vertices) {
            if (!locked[b] && part[b] == 1) {
              int gain = D[a] + D[b] - (2*(adj.at(a).contains(b) ? adj.at(a).at(b) : 0));

              if (gain > bestGain) {
                bestGain = gain;
                bestA = a;
                bestB = b;
              }
            }
          }
        }
      }

      if (bestA == nullptr) {
        break;
      }

      // Lock and record move
      locked[bestA] = locked[bestB] = true;
      moves.push_back({bestA, bestB, bestGain});

      int partA_old = part[bestA];
      int partB_old = part[bestB];
      part[bestA] = 1;
      part[bestB] = 0;
      // update D values
      for (auto& [u, map] : adj) if (!locked[u]) {
          for (auto& [v, _] : map) {
              if (v == bestA) D[u] += (part[u] == partA_old ? 2 : -2);
              if (v == bestB) D[u] += (part[u] == partB_old ? 2 : -2);
          }
      }
    }

    // compute cumulative gains
    std::vector<int> G(moves.size());
    G[0] = moves[0].gain;
    for (int i = 1; i < moves.size(); i++) {
        G[i] = G[i - 1] + moves[i].gain;
    }

    int k = std::ranges::max_element(G) - G.begin();
    if (G[k] <= 0) {
      improved = false;
      for (int i = moves.size()-1; i >= 0; i--) {
        std::swap(part[moves[i].a], part[moves[i].b]);
      }
    } else {
      improved = true;
      for (int i = moves.size()-1; i > k; i--) 
      {
        std::swap(part[moves[i].a], part[moves[i].b]);
      }
      bestCut = KLCountCut(adj, vertices, part);
    }
  }

  return {.part=part, .cuts=bestCut};
}

void MinCutPlacer::KLRecursion(const InstanceGraph& adj, const InstanceVec& vertices, const odb::Rect& region, int depth, std::vector<std::pair<int, int>>& pos)
{
  if (vertices.empty()) return;
  log_->report("Partition depth = {} vertices = {} rect {}", depth, vertices.size(), region);
  
  if (vertices.size() == 1) {
      vertices[0]->dbSetLocation(region.xMin(), region.yMin());
      vertices[0]->dbSetPlaced();
      return;
  }

  Partition part = KLPartitioner(adj, vertices);

  int64_t area_a = 0;
  InstanceVec A, B;
  for (auto& inst : vertices) {
      if (part.part[inst] == 0) { 
        A.push_back(inst); 
        area_a += inst->area();
      }
      else { 
        B.push_back(inst); 
      }
  }
  
  bool vertical = (depth % 2 == 0);
  odb::Rect rA, rB;
  if (vertical) {
    int bissect_location = 0;
    if (false || compact_) {
      bissect_location = region.xMin() + (area_a / region.dy());
    } else {
      bissect_location = region.xCenter();
    }
    if (graphics_) {
      graphics_->drawLine(odb::Line(bissect_location, region.yMin(), bissect_location, region.yMax()));
    }
    rA = odb::Rect(
        region.xMin(), region.yMin(), bissect_location, region.yMax());
    rB = odb::Rect(
        bissect_location, region.yMin(), region.xMax(), region.yMax());
  } else {
    int bissect_location = 0;
    if (false || compact_) {
      bissect_location = region.yMin() + (area_a / region.dx());
    } else {
      bissect_location = region.yCenter();
    }
    if (graphics_) {
      graphics_->drawLine(odb::Line(region.xMin(), bissect_location, region.xMax(), bissect_location));
    }
    rA = odb::Rect(
        region.xMin(), region.yMin(), region.xMax(), bissect_location);
    rB = odb::Rect(
        region.xMin(), bissect_location, region.xMax(), region.yMax());
  }
  graphics_->cellPlot(true);
  KLRecursion(adj, A, rA, depth + 1, pos);
  graphics_->cellPlot(true);
  KLRecursion(adj, B, rB, depth + 1, pos);
  graphics_->cellPlot(true);
}

void MinCutPlacer::KernighanLinPlacement(int threads, bool compact) {
  compact_ = compact;
  if (!initPlacer()) {
    return;
  }

  auto& insts = pbc_->placeInsts();
  int n = insts.size();
  
  for (int i = 0; i < n; i++) {
    inst_to_index_[insts[i]] = i;
  }

  InstanceGraph adj;

  for (auto& net : pbc_->getNets()) {
    auto& pins = net->getPins();

    for (int i = 0; i < pins.size(); i++) {
      if (pins[i]->isBTerm()) continue;

      for (int j = i+1; j < pins.size(); j++) { 
        if (pins[j]->isBTerm()) continue;

        adj[pins[i]->getInstance()][pins[j]->getInstance()] += 1;
        adj[pins[j]->getInstance()][pins[i]->getInstance()] += 1;
      }
    }
  }

  std::vector<std::pair<int, int>> pos(n);

  odb::Rect region = db_->getChip()->getBlock()->getCoreArea();

  float ar = region.dx()/static_cast<float>(region.dy());
  if (compact_) {
    int64_t total_area = 0;
    for (auto& inst : insts) {
      total_area += inst->area();
    }
    int side_x = std::sqrt(total_area*ar);
    int side_y = total_area / side_x;
    odb::Rect new_region = {region.xCenter() - (side_x/2),
                            region.yCenter() - (side_y/2),
                            region.xCenter() + (side_x/2),
                            region.yCenter() + (side_y/2)};
    region = new_region;
  }
  KLRecursion(adj, pbc_->placeInsts(), region, 0, pos);

  // #pragma omp parallel for num_threads(threads)
  // for (int i = 0; i < n; i++) {
    // auto& inst = insts[i];
    // inst->dbSetLocation(pos[i].first, pos[i].second);
  //   inst->dbSetPlaced();
  // }

  auto block = db_->getChip()->getBlock();
  odb::WireLengthEvaluator eval(block);
  log_->report("Final HPWL: {} {}", block->dbuToMicrons(pbc_->getHpwl()), block->dbuToMicrons(eval.hpwl()));
}

void MinCutPlacer::randomPlace(int threads)
{
  debugPrint(log_,
             MCP,
             "random_place",
             7,
             "random_place: number of threads {}",
             threads);
  if (!initPlacer()) {
    return;
  }

  odb::dbBlock* block = db_->getChip()->getBlock();
  odb::Rect core = block->getCoreArea();

  auto& insts = pbc_->placeInsts();
  int n_inst = insts.size();
  int core_x = core.dx();
  int core_y = core.dy();
  int core_x_min = core.xMin();
  int core_y_min = core.yMin();
  
  std::vector<int> pos_x(n_inst), pos_y(n_inst);
  
  std::ranges::generate(
      pos_x, [&]() { return distribution_(gen_); });
  std::ranges::generate(
      pos_y, [&]() { return distribution_(gen_); });

  #pragma omp parallel for num_threads(threads)
  for (int i = 0; i < n_inst; i++) {
    auto& inst = insts[i];
    inst->dbSetLocation(
      core_x_min + (pos_x[i] % (core_x-inst->dx())), 
      core_y_min + (pos_y[i] % (core_y-inst->dy()))
    );
    inst->dbSetPlaced();
  }
}

void MinCutPlacer::clear()
{
  // clear instances
  pbc_.reset();
  pbVec_.clear();
}

}  // namespace mcp
