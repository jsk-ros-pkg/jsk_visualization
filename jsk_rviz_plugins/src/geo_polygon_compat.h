// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef JSK_RVIZ_PLUGINS_GEO_POLYGON_COMPAT_H_
#define JSK_RVIZ_PLUGINS_GEO_POLYGON_COMPAT_H_

// jsk_recognition_utils::Polygon of jsk_recognition_utils/geo/polygon.h,
// which is not ported to ROS 2: on ROS 2, the part of it PolygonArrayDisplay
// uses (fromROSMsg(), getVertices(), getNormal() and decomposeToTriangles()),
// with the ear clipping of jsk_recognition_utils' pcl::EarClippingPatched
// (src/pcl/ear_clipping_patched.cpp) written out on Eigen, so that the
// triangles are the same as on ROS 1.
#include "rviz_compat.h"
#if ROS_VERSION_MAJOR != 1
#include <array>
#include <memory>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <geometry_msgs/msg/polygon.hpp>

namespace jsk_recognition_utils
{
  typedef std::vector<Eigen::Vector3f,
                      Eigen::aligned_allocator<Eigen::Vector3f> > Vertices;

  class Polygon
  {
  public:
    typedef std::shared_ptr<Polygon> Ptr;

    explicit Polygon(const Vertices& vertices):
      vertices_(vertices),
      normal_(Eigen::Vector3f::Zero())
    {
      // jsk_recognition_utils reads vertices[1] and vertices[2] even if
      // there are fewer vertices; PolygonArrayDisplay calls getNormal() and
      // decomposeToTriangles() of polygons with at least 3 vertices only
      if (vertices_.size() >= 3) {
        normal_ = (vertices_[1] - vertices_[0]).cross(
          vertices_[2] - vertices_[0]).normalized().normalized();
      }
    }

    Polygon(const Vertices& vertices, const Eigen::Vector3f& normal):
      vertices_(vertices), normal_(normal)
    {
    }

    static Polygon fromROSMsg(const geometry_msgs::msg::Polygon& polygon)
    {
      Vertices vertices;
      for (size_t i = 0; i < polygon.points.size(); i++) {
        vertices.push_back(Eigen::Vector3f(polygon.points[i].x,
                                           polygon.points[i].y,
                                           polygon.points[i].z));
      }
      return Polygon(vertices);
    }

    size_t getNumVertices() const { return vertices_.size(); }
    Eigen::Vector3f getVertex(size_t i) const { return vertices_[i]; }
    Vertices getVertices() const { return vertices_; }
    Eigen::Vector3f getNormal() const { return normal_; }

    std::vector<Ptr> decomposeToTriangles() const
    {
      std::vector<Ptr> ret;
      if (vertices_.size() == 3) {
        ret.push_back(Ptr(new Polygon(*this)));
        return ret;
      }
      std::vector<std::array<size_t, 3> > triangles;
      triangulate(triangles);
      for (size_t i = 0; i < triangles.size(); i++) {
        Vertices vs(3);
        for (size_t j = 0; j < 3; j++) {
          vs[j] = vertices_[triangles[i][j]];
        }
        ret.push_back(Ptr(new Polygon(vs, normal_)));
      }
      return ret;
    }

  protected:
    // pcl::EarClippingPatched::triangulate()
    void triangulate(std::vector<std::array<size_t, 3> >& output) const
    {
      const size_t n_vertices = vertices_.size();
      if (n_vertices < 3) {
        return;
      }
      std::vector<size_t> remaining_vertices(n_vertices);
      for (size_t v = 0; v < n_vertices; v++) {
        remaining_vertices[v] = v;
      }
      size_t count = triangulateClockwiseVertices(remaining_vertices, output);
      // if the input vertices order is anti-clockwise, it always left a
      // convex polygon and start infinite loops, which means will left more
      // than 3 points.
      if (remaining_vertices.size() < 3) {
        return;
      }
      // the triangles of the first try are dropped (what the erase() of
      // EarClippingPatched does with libstdc++), the vertices reversed
      output.resize(output.size() - count);
      remaining_vertices.resize(n_vertices);
      for (size_t v = 0; v < n_vertices; v++) {
        remaining_vertices[v] = n_vertices - 1 - v;
      }
      triangulateClockwiseVertices(remaining_vertices, output);
    }

    // pcl::EarClippingPatched::triangulateClockwiseVertices()
    size_t triangulateClockwiseVertices(
      std::vector<size_t>& vertices,
      std::vector<std::array<size_t, 3> >& output) const
    {
      size_t count = 0;
      // null_iterations avoids infinite loops if the polygon is not simple.
      for (int u = static_cast<int>(vertices.size()) - 1, null_iterations = 0;
           vertices.size() > 2 && null_iterations < static_cast<int>(vertices.size() * 2);
           ++null_iterations, u = (u + 1) % static_cast<int>(vertices.size())) {
        int v = (u + 1) % static_cast<int>(vertices.size());
        int w = (u + 2) % static_cast<int>(vertices.size());
        if (vertices.size() == 3 || isEar(u, v, w, vertices)) {
          std::array<size_t, 3> triangle = {{vertices[u], vertices[v], vertices[w]}};
          output.push_back(triangle);
          vertices.erase(vertices.begin() + v);
          null_iterations = 0;
          count++;
        }
      }
      return count;
    }

    // pcl::EarClippingPatched::isEar()
    bool isEar(int u, int v, int w, const std::vector<size_t>& vertices) const
    {
      Eigen::Vector3f p_u = vertices_[vertices[u]];
      Eigen::Vector3f p_v = vertices_[vertices[v]];
      Eigen::Vector3f p_w = vertices_[vertices[w]];

      const float eps = 1e-15f;
      Eigen::Vector3f p_vu = p_u - p_v;
      Eigen::Vector3f p_vw = p_w - p_v;

      // 1: Avoid flat triangles and concave vertex
      Eigen::Vector3f cross = p_vu.cross(p_vw);
      if ((cross[2] > 0) || (cross.norm() < eps)) {
        return false;
      }

      // 2: Check if any other vertex is inside the triangle.
      for (int k = 0; k < static_cast<int>(vertices.size()); k++) {
        if ((k == u) || (k == v) || (k == w)) {
          continue;
        }
        if (isInsideTriangle(p_u, p_v, p_w, vertices_[vertices[k]])) {
          return false;
        }
      }

      // 3: Check if the line segment uw lies completely inside the polygon,
      // by the middle point of uw (a simple polygon is supposed).
      Eigen::Vector3f p_mid_uw = (p_u + p_w) / 2.0;
      // avoid double-counting intersection at polygon vertices
      Eigen::Vector3f p_inf = p_mid_uw + (p_v - p_mid_uw) * 1e15f + p_vu * 1e10f;
      int intersect_count = 0;
      for (int i = 0; i < static_cast<int>(vertices.size()); i++) {
        if (intersect(p_mid_uw, p_inf, vertices_[vertices[i]],
                      vertices_[vertices[(i + 1) % static_cast<int>(vertices.size())]])) {
          intersect_count++;
        }
      }
      if (intersect_count % 2 == 0) {
        return false;
      }
      return true;
    }

    // pcl::EarClippingPatched::isInsideTriangle(), with barycentric coordinates
    static bool isInsideTriangle(const Eigen::Vector3f& u,
                                 const Eigen::Vector3f& v,
                                 const Eigen::Vector3f& w,
                                 const Eigen::Vector3f& p)
    {
      Eigen::Vector3f v0 = w - u;
      Eigen::Vector3f v1 = v - u;
      Eigen::Vector3f v2 = p - u;
      float dot00 = v0.dot(v0);
      float dot01 = v0.dot(v1);
      float dot02 = v0.dot(v2);
      float dot11 = v1.dot(v1);
      float dot12 = v1.dot(v2);
      float invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
      float a = (dot11 * dot02 - dot01 * dot12) * invDenom;
      float b = (dot00 * dot12 - dot01 * dot02) * invDenom;
      return (a >= 0) && (b >= 0) && (a + b < 1);
    }

    // pcl::EarClippingPatched::intersect() of the segments p0-p1 and p2-p3
    static bool intersect(const Eigen::Vector3f& p0,
                          const Eigen::Vector3f& p1,
                          const Eigen::Vector3f& p2,
                          const Eigen::Vector3f& p3)
    {
      Eigen::Vector3f a = p1 - p0;
      Eigen::Vector3f b = p3 - p2;
      Eigen::Vector3f c = p2 - p0;
      // Parallel line segments do not intersect each other.
      if (a.cross(b).norm() == 0) {
        return false;
      }
      float s = (c.cross(b)).dot(a.cross(b)) / ((a.cross(b)).norm() * (a.cross(b)).norm());
      float t = (c.cross(a)).dot(a.cross(b)) / ((a.cross(b)).norm() * (a.cross(b)).norm());
      return ((s >= 0 && s <= 1) && (t >= 0 && t <= 1));
    }

    Vertices vertices_;
    Eigen::Vector3f normal_;
  };
}  // namespace jsk_recognition_utils
#else
#include <jsk_recognition_utils/geo/polygon.h>
#endif

#endif  // JSK_RVIZ_PLUGINS_GEO_POLYGON_COMPAT_H_
