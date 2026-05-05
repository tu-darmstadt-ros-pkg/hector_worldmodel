#ifndef HECTOR_WORLD_MODEL_DB_SCAN_CLUSTERER_HPP
#define HECTOR_WORLD_MODEL_DB_SCAN_CLUSTERER_HPP

#include <clustering/clusterer_base.hpp>

#define FAILURE ( -3 )
#define NOISE ( -2 )
#define UNCLASSIFIED ( -1 )
#define SUCCESS 0
#define CORE_POINT 1
#define BORDER_POINT 2

namespace hector_world_model
{
typedef struct Point_ {
  Eigen::Vector3d position;
  int clusterID; // clustered ID
} Point;

class DBScanClusterer : public DetectionClusterer
{
public:
  DBScanClusterer( std::atomic<int> &latest_marker_id, const std::shared_ptr<WorldModel> &node );

  ~DBScanClusterer() noexcept override = default;

private:
  void fit() override;
  void processNewDetections() override;

  void processClusteringResults();

  void promoteObjectCandidates();

  int run_db_scan();
  int expandCluster( Point &point, int clusterID );
  std::vector<int> calculateCluster( const Point &source_point ) const;
  inline bool isWithinEpsilon( const Point &p1, const Point &p2 ) const;
  static inline double d( const Point &p1, const Point &p2 );

  std::vector<Point> clustering_data_;
  int min_neighbours_;
  double epsilon_;
};
} // namespace hector_world_model

#endif
