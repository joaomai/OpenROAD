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

inline int MinCutPlacer::KLCountCut(const Graph& adj, const std::vector<int>& part) 
{
  int cut = 0;
  for (int u = 0; u < adj.size(); ++u) {
    for (int v : adj[u]) {
      cut += (part[u] != part[v]);
    }
  }
  return cut / 2;
}

MinCutPlacer::Partition MinCutPlacer::KLPartitioner(const Graph& adj, const std::vector<int>& map_to_orig)
{
  const int n = adj.size();
  std::vector<int> part(n);

  std::vector<std::pair<int64_t, int>> area_index;
  for (int i = 0; i < n; i++) {
    area_index.emplace_back((pbc_->placeInsts()[map_to_orig[i]]->area()), i);;
  }
  std::ranges::sort(area_index);
  for (int i = 0; i < n; i++) {
    part[area_index[i].second] = i % 2;
  }

  bool improved = true;
  int bestCut = KLCountCut(adj, part);

  while (improved) {
    improved = false;

    std::vector<bool> locked(n, false);
    std::vector<int> D(n, 0);

    for (int v = 0; v < n; v++) {
      for (int u : adj[v]) {
        if (part[u] == part[v]) {
          D[v] -= 1;
        } else {
          D[v] += 1;
        }
      }
    }

    std::vector<Move> moves;

    for (int step = 0; step < n / 2; step++) {
      int bestA = -1, bestB = -1;
      int bestGain = std::numeric_limits<int>::min();

      for (int a = 0; a < n; a++) {
        if (!locked[a] && part[a] == 0) {
          for (int b = 0; b < n; b++) {
            if (!locked[b] && part[b] == 1) {
              int w = 0;
              if (std::ranges::find(adj[a], b) != adj[a].end()) {
                w = 1;
              }
              int gain = D[a] + D[b] - (2*w);


              if (gain > bestGain) {
                bestGain = gain;
                bestA = a;
                bestB = b;
              }
            }
          }
        }
      }

      if (bestA == -1) {
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
      for (int v = 0; v < n; v++) if (!locked[v]) {
          for (int x : adj[v]) {
              if (x == bestA) D[v] += (part[v] == partA_old ? 2 : -2);
              if (x == bestB) D[v] += (part[v] == partB_old ? 2 : -2);
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
      bestCut = KLCountCut(adj, part);
    } 
  }

  return {.part=part, .cuts=bestCut};
}

void MinCutPlacer::KLRecursion(const Graph& adj, const std::vector<int>& vertices, const odb::Rect& region, int depth, std::vector<std::pair<int, int>>& pos)
{
  if (vertices.empty()) return;
  log_->report("Partition depth = {} vertices = {} rect {}", depth, vertices.size(), region);
  
  if (vertices.size() == 1) {
      pos[vertices[0]] = {
          region.xMin(),
          region.yMin()};
      return;
  }

  Graph sub(vertices.size());
  std::vector<int> mapToOrig(vertices.size());
  std::vector<int> mapFromOrig(adj.size(), -1);

  for (int i = 0; i < vertices.size(); i++) {
      mapToOrig[i] = vertices[i];
      mapFromOrig[vertices[i]] = i;
  }

  for (int i = 0; i < vertices.size(); i++) {
      int uOrig = mapToOrig[i];
      for (int vOrig : adj[uOrig]) {
          if (mapFromOrig[vOrig] != -1) { sub[i].push_back(mapFromOrig[vOrig]); }
      }
  }

  Partition part = KLPartitioner(sub, mapToOrig);

  int64_t area_a = 0;
  std::vector<int> A, B;
  for (int i = 0; i < sub.size(); i++) {
      if (part.part[i] == 0) { 
        A.push_back(mapToOrig[i]); 
        area_a += pbc_->placeInsts()[mapToOrig[i]]->area();
      }
      else { 
        B.push_back(mapToOrig[i]); 
      }
  }
  
  bool vertical = (depth % 2 == 0);
  odb::Rect rA, rB;
  if (vertical) {
    int bissect_location = 0;
    if (compact_) {
      bissect_location = region.xMin() + (area_a / region.dy());
    } else {
      bissect_location = region.xCenter();
    }
    rA = odb::Rect(
        region.xMin(), region.yMin(), bissect_location, region.yMax());
    rB = odb::Rect(
        bissect_location, region.yMin(), region.xMax(), region.yMax());
  } else {
    int bissect_location = 0;
    if (compact_) {
      bissect_location = region.yMin() + (area_a / region.dx());
    } else {
      bissect_location = region.yCenter();
    }
    rA = odb::Rect(
        region.xMin(), region.yMin(), region.xMax(), bissect_location);
    rB = odb::Rect(
        region.xMin(), bissect_location, region.xMax(), region.yMax());
  }

  KLRecursion(adj, A, rA, depth + 1, pos);
  KLRecursion(adj, B, rB, depth + 1, pos);
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

  std::vector<std::set<int>> adj_set(n);

  for (auto& net : pbc_->getNets()) {
    auto& pins = net->getPins();

    for (int i = 0; i < pins.size(); i++) {
      if (pins[i]->isBTerm()) continue;

      for (int j = i+1; j < pins.size(); j++) { 
        if (pins[j]->isBTerm()) continue;

        adj_set[inst_to_index_[pins[i]->getInstance()]].insert(inst_to_index_[pins[j]->getInstance()]);
        adj_set[inst_to_index_[pins[j]->getInstance()]].insert(inst_to_index_[pins[i]->getInstance()]);
      }
    }
  }

  Graph adj(n);
  for (int i = 0; i < n; i++) {
    adj[i] = std::vector<int>(adj_set[i].begin(), adj_set[i].end());
  }

  std::vector<std::pair<int, int>> pos(n);
  std::vector<int> vertices(n);
  std::iota(vertices.begin(), vertices.end(), 0);

  odb::Rect region = db_->getChip()->getBlock()->getCoreArea();

  if (compact_) {
    int64_t total_area = 0;
    for (auto& inst : insts) {
      total_area += inst->area();
    }
    int side = std::sqrt(total_area) / 2;

    odb::Rect new_region = {region.xCenter() - side,
                            region.yCenter() - side,
                            region.xCenter() + side,
                            region.yCenter() + side};
    region = new_region;
  }
  KLRecursion(adj, vertices, region, 0, pos);

  #pragma omp parallel for num_threads(threads)
  for (int i = 0; i < n; i++) {
    auto& inst = insts[i];
    inst->dbSetLocation(pos[i].first, pos[i].second);
    inst->dbSetPlaced();
  }

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
