#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Geometry>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_urdf/mesh.h>
#include "tesseract_urdf_common_unit.h"

TEST(TesseractURDFUnit, parse_mesh)
{
  {
    std::string str = "<mesh filename=\"package://tesseract_support/meshes/sphere_p25m.stl\" scale=\"1 2 1\" extra=\"0 "
                      "0 0\"/>";
    std::vector<tesseract_geometry::Mesh::Ptr> geom;
    auto status = runTest<std::vector<tesseract_geometry::Mesh::Ptr>>(geom, str, "mesh", locateResource, true);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(geom.size() == 1);
    EXPECT_TRUE(geom[0]->getTriangleCount() == 80);
    EXPECT_TRUE(geom[0]->getVerticeCount() == 42);
    EXPECT_NEAR(geom[0]->getScale()[0], 1, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[1], 2, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[2], 1, 1e-5);
  }

  {
    std::string str = "<mesh filename=\"package://tesseract_support/meshes/sphere_p25m.stl\"/>";
    std::vector<tesseract_geometry::Mesh::Ptr> geom;
    auto status = runTest<std::vector<tesseract_geometry::Mesh::Ptr>>(geom, str, "mesh", locateResource, true);
    EXPECT_TRUE(*status);
    EXPECT_TRUE(geom.size() == 1);
    EXPECT_TRUE(geom[0]->getTriangleCount() == 80);
    EXPECT_TRUE(geom[0]->getVerticeCount() == 42);
    EXPECT_NEAR(geom[0]->getScale()[0], 1, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[1], 1, 1e-5);
    EXPECT_NEAR(geom[0]->getScale()[2], 1, 1e-5);
  }

  {
    std::string str = "<mesh filename=\"abc\" scale=\"1 2 1\"/>";
    std::vector<tesseract_geometry::Mesh::Ptr> geom;
    auto status = runTest<std::vector<tesseract_geometry::Mesh::Ptr>>(geom, str, "mesh", locateResource, true);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<mesh filename=\"package://tesseract_support/meshes/sphere_p25m.stl\" scale=\"1 a 1\"/>";
    std::vector<tesseract_geometry::Mesh::Ptr> geom;
    auto status = runTest<std::vector<tesseract_geometry::Mesh::Ptr>>(geom, str, "mesh", locateResource, true);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<mesh filename=\"package://tesseract_support/meshes/sphere_p25m.stl\" scale=\"1 2 1 3\"/>";
    std::vector<tesseract_geometry::Mesh::Ptr> geom;
    auto status = runTest<std::vector<tesseract_geometry::Mesh::Ptr>>(geom, str, "mesh", locateResource, true);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<mesh scale=\"1 2 1\"/>";
    std::vector<tesseract_geometry::Mesh::Ptr> geom;
    auto status = runTest<std::vector<tesseract_geometry::Mesh::Ptr>>(geom, str, "mesh", locateResource, true);
    EXPECT_FALSE(*status);
  }

  {
    std::string str = "<mesh />";
    std::vector<tesseract_geometry::Mesh::Ptr> geom;
    auto status = runTest<std::vector<tesseract_geometry::Mesh::Ptr>>(geom, str, "mesh", locateResource, true);
    EXPECT_FALSE(*status);
  }
}
