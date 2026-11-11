// Independent offline oracle against the source geometry, not the fused map.
#include "falcon_map_render/render_core.h"
#include <fstream>
#include <iostream>
#include <iomanip>
#include <limits>
#include <stdexcept>

int main(int argc, char **argv) {
  try {
    if (argc != 21) throw std::invalid_argument("Usage: check_clearance MAP SCALE T_m_w[16] XYZ_FILE RADIUS");
    Eigen::Isometry3d transform=Eigen::Isometry3d::Identity();
    for (int i=0;i<16;++i) transform.matrix()(i/4,i%4)=std::stod(argv[3+i]);
    falcon_map_render::Scene scene;
    scene.load(argv[1],std::stod(argv[2]),transform);
    const double radius=std::stod(argv[20]);
    if (!std::isfinite(radius) || radius<=0) throw std::invalid_argument("Invalid vehicle radius");
    std::ifstream stream(argv[19]);
    if (!stream) throw std::runtime_error("Cannot open XYZ samples");
    Eigen::Vector3d previous, point;
    size_t samples=0, unsafe=0;
    double minimum=std::numeric_limits<double>::infinity();
    Eigen::Vector3d minimum_point=Eigen::Vector3d::Zero();
    size_t minimum_segment=0;
    while (stream >> point.x() >> point.y() >> point.z()) {
      if (!point.allFinite()) throw std::invalid_argument("Nonfinite sample");
      if (samples == 0) { minimum=scene.clearance(point); minimum_point=point; unsafe += minimum<radius; }
      else {
        const double length=(point-previous).norm();
        const int count=std::max(1,static_cast<int>(std::ceil(length/0.025)));
        // Distance is 1-Lipschitz: subtract half the sample spacing to bound
        // clearance over the complete recorded polyline, including thin walls.
        for (int i=0;i<=count;++i) {
          const Eigen::Vector3d p=previous+(point-previous)*(double(i)/count);
          const double clearance=scene.clearance(p)-length/(2.0*count);
          if (clearance<minimum) {minimum=clearance; minimum_point=p; minimum_segment=samples;}
          unsafe += clearance<radius;
        }
      }
      previous=point; ++samples;
    }
    if (!stream.eof() || samples<2) throw std::runtime_error("Invalid or insufficient XYZ samples");
    std::cout << std::setprecision(10) << "{\"clearance_lower_bound\":" << minimum
              << ",\"unsafe_samples\":" << unsafe << ",\"radius\":" << radius
              << ",\"source_is_mesh\":" << (scene.mesh()?"true":"false") << "}\n";
    std::cerr << "Minimum at segment " << minimum_segment << ", position " << minimum_point.transpose() << '\n';
    return unsafe ? 2 : 0;
  } catch (const std::exception &e) { std::cerr << e.what() << '\n'; return 1; }
}
