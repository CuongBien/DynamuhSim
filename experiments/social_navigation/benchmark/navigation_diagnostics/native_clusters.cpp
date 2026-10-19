// Offline clustering of a recorded ParticleCloud with the installed AMCL PF library.
// No ROS node, sensor update, motion update, random sampling or replay.
#include <iomanip>
#include <iostream>
extern "C" {
#include "nav2_amcl/pf/pf.hpp"
}
int main() {
  std::cout << std::setprecision(17);
  int count;
  while (std::cin >> count) {
    if (count <= 0 || count > 20000) return 2;
    pf_t *pf = pf_alloc(count, count, 0., 0., nullptr);
    auto *set = &pf->sets[pf->current_set];
    pf_kdtree_clear(set->kdtree);
    for (int i=0; i<count; ++i) {
      auto &s = set->samples[i];
      std::cin >> s.pose.v[0] >> s.pose.v[1] >> s.pose.v[2] >> s.weight;
      pf_kdtree_insert(set->kdtree, s.pose, s.weight);
    }
    if (!std::cin) return 3;
    pf_cluster_stats(pf, set);
    std::cout << set->cluster_count;
    for (int c=0; c<set->cluster_count; ++c) {
      const auto &v=set->clusters[c];
      std::cout << " " << v.weight << " " << v.mean.v[0] << " " << v.mean.v[1] << " " << v.mean.v[2];
    }
    std::cout << "\n";
    pf_free(pf);
  }
}
