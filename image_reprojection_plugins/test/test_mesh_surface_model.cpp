#include <gtest/gtest.h>

#include <image_reprojection_plugins/MeshStamped.h>
#include <image_reprojection_plugins/mesh_surface_model.hpp>

#include <opencv2/core/core.hpp>

namespace irp = image_reprojection_plugins;

// the global random number generator
cv::RNG g_rng(std::time(NULL));

geometry_msgs::Point randomPoint() {
  geometry_msgs::Point p;
  p.x = g_rng.uniform(-1., 1.);
  p.y = g_rng.uniform(-1., 1.);
  p.z = g_rng.uniform(-1., 1.);
  return p;
}

// generate mesh having exactly one random triangle
irp::MeshStamped randomMesh() {
  irp::MeshStamped mesh;
  mesh.mesh.vertices.push_back(randomPoint());
  mesh.mesh.vertices.push_back(randomPoint());
  mesh.mesh.vertices.push_back(randomPoint());
  shape_msgs::MeshTriangle triangle;
  triangle.vertex_indices[0] = 0;
  triangle.vertex_indices[1] = 1;
  triangle.vertex_indices[2] = 2;
  mesh.mesh.triangles.push_back(triangle);
  return mesh;
}

// pick <vertex_id>-th vertex of <triangle_id>-th triangle in the given mesh
cv::Vec3f triangleVertex(const irp::MeshStamped &mesh, const int triangle_id, const int vertex_id) {
  const geometry_msgs::Point &vertex(
      mesh.mesh.vertices[mesh.mesh.triangles[triangle_id].vertex_indices[vertex_id]]);
  return cv::Vec3f(vertex.x, vertex.y, vertex.z);
}

// generate intersections to 0th triangle of the given mesh
void randomIntersection(const cv::Size &dsize, const irp::MeshStamped &mesh, cv::Vec3f &ray_origin,
                        cv::Mat &ray_direction, cv::Mat &intersection, cv::Mat &mask) {
  ray_direction.create(dsize, CV_32FC3);
  intersection.create(dsize, CV_32FC3);
  mask.create(dsize, CV_8UC1);

  ray_origin[0] = g_rng.uniform(-1., 1.);
  ray_origin[1] = g_rng.uniform(-1., 1.);
  ray_origin[2] = g_rng.uniform(-1., 1.);

  const cv::Vec3f v0(triangleVertex(mesh, 0, 0));
  const cv::Vec3f e1(triangleVertex(mesh, 0, 1) - v0);
  const cv::Vec3f e2(triangleVertex(mesh, 0, 2) - v0);
  for (int x = 0; x < dsize.width; ++x) {
    for (int y = 0; y < dsize.height; ++y) {
      //
      const double b1(g_rng.uniform(-0.5, 1.5)), b2(g_rng.uniform(-0.5, 1.5));
      const cv::Vec3f p(v0 + b1 * e1 + b2 * e2);
      intersection.at< cv::Vec3f >(y, x) = p;
      //
      const double d_scale(g_rng.uniform(-1., 2.));
      ray_direction.at< cv::Vec3f >(y, x) = d_scale * (p - ray_origin);
      //
      mask.at< unsigned char >(y, x) = (b1 >= 0 && b2 >= 0 && b1 + b2 <= 1 && d_scale >= 0) ? 1 : 0;
    }
  }
}

TEST(Intersection, random10000) {
  // initialize tested model with random mesh
  irp::MeshSurfaceModel model;
  const irp::MeshStamped mesh(randomMesh());
  model.update(mesh);

  // generate truth data at random
  const cv::Size size(100, 100);
  cv::Vec3f ray_origin;
  cv::Mat ray_direction, true_intersection, true_mask;
  randomIntersection(size, mesh, ray_origin, ray_direction, true_intersection, true_mask);

  // calculate intersections using tested model
  cv::Mat intersection, mask(cv::Mat::ones(ray_direction.size(), CV_8UC1));
  model.intersection(ray_origin, ray_direction, intersection, mask);

  // compare results from tested model and expected results
  for (int x = 0; x < mask.size().width; ++x) {
    for (int y = 0; y < mask.size().height; ++y) {
      // comapre mask values
      const unsigned char m(mask.at< unsigned char >(y, x)),
          tm(true_mask.at< unsigned char >(y, x));
      EXPECT_TRUE((m != 0 && tm != 0) || (m == 0 && tm == 0));
      // compare intersection points
      if (m != 0) {
        const cv::Vec3f i(intersection.at< cv::Vec3f >(y, x)),
            ti(true_intersection.at< cv::Vec3f >(y, x));
        EXPECT_NEAR(i[0], ti[0], 0.001);
        EXPECT_NEAR(i[1], ti[1], 0.001);
        EXPECT_NEAR(i[2], ti[2], 0.001);
      }
    }
  }
}

int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}