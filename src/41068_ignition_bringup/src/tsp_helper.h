// tsp_helper.h
#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include <limits>

struct V3 { double x,y,z; };

inline double dist3(const V3& a, const V3& b){
  const double dx=a.x-b.x, dy=a.y-b.y, dz=a.z-b.z;
  return std::sqrt(dx*dx + dy*dy + dz*dz);
}

// Nearest-neighbor starting from nearest to 'start'
inline std::vector<V3> nn_from_start(const V3& start, std::vector<V3> pts){
  std::vector<V3> route;
  if (pts.empty()) return route;
  size_t k=0; double best=std::numeric_limits<double>::infinity();
  for (size_t i=0;i<pts.size();++i){ double d=dist3(start,pts[i]); if(d<best){best=d;k=i;} }
  route.push_back(pts[k]); pts.erase(pts.begin()+k);
  while(!pts.empty()){
    const auto& last = route.back();
    size_t j=0; double bd=std::numeric_limits<double>::infinity();
    for(size_t i=0;i<pts.size();++i){ double d=dist3(last,pts[i]); if(d<bd){bd=d;j=i;} }
    route.push_back(pts[j]); pts.erase(pts.begin()+j);
  }
  return route;
}

// 2-opt for OPEN path (no return to start)
inline void two_opt_open(const V3& start, std::vector<V3>& r){
  if (r.size()<3) return;
  bool improved=true;
  while(improved){
    improved=false;
    for(size_t i=0;i+2<r.size();++i){
      for(size_t j=i+2;j<r.size()+1;++j){
        const V3 a = (i==0)? start : r[i-1];
        const V3 b = r[i];
        const V3 c = r[j-1];
        const V3* d = (j<r.size())? &r[j] : nullptr;
        const double before = dist3(a,b) + (d? dist3(c,*d):0.0);
        const double after  = dist3(a,c) + (d? dist3(b,*d):0.0);
        if (after + 1e-9 < before){
          std::reverse(r.begin()+static_cast<long>(i), r.begin()+static_cast<long>(j));
          improved=true;
        }
      }
    }
  }
}

// Main entry: returns ordered waypoints (open path)
inline std::vector<V3> tsp_order_open(const V3& start, const std::vector<V3>& points){
  auto route = nn_from_start(start, points);
  two_opt_open(start, route);
  return route;
}
