#include <jsk_transformable_interactive_marker/transformable_torus.h>
#include <math.h>

#define PI 3.14159265
namespace jsk_transformable_interactive_marker{

  TransformableTorus::TransformableTorus( float radius, float small_radius, int u_div, int v_div, float r, float g, float b, float a, std::string frame, std::string name, std::string description){
    torus_radius_ = radius;
    torus_small_radius_ = small_radius;
    torus_r_ = r;
    torus_g_ = g;
    torus_b_ = b;
    torus_a_ = a;
    marker_.type = visualization_msgs::Marker::TRIANGLE_LIST;

    frame_id_ = frame;
    name_ = name;
    description_ = description;

    u_division_num_ = u_div;
    v_division_num_ = v_div;
  }

  std::vector<geometry_msgs::Point > TransformableTorus::calcurateTriangleMesh(){
    std::vector<geometry_msgs::Point> triangle_mesh;
    float center_x = 0;
    float center_y = 0;
    float u_division_num = u_division_num_;
    float v_division_num = v_division_num_;
    std::vector<std::vector<geometry_msgs::Point> > points_array;
    for (int i = 0; i < u_division_num; i ++){
      std::vector<geometry_msgs::Point> points;
      float target_circle_x = torus_radius_ * cos( ( i / u_division_num) * 2 * PI) ;
      float target_circle_y = torus_radius_ * sin( ( i / u_division_num) * 2 * PI) ;
      for (int j = 0; j < v_division_num; j++){
        geometry_msgs::Point new_point;
        new_point.x = target_circle_x + torus_small_radius_ * cos ( (j / v_division_num) * 2 * PI) * cos( ( i / u_division_num) * 2 * PI);
        new_point.y = target_circle_y + torus_small_radius_ * cos ( (j / v_division_num) * 2 * PI) * sin( ( i / u_division_num) * 2 * PI);
        new_point.z = torus_small_radius_ * sin ( (j / v_division_num) * 2 * PI);
        points.push_back(new_point);
      }
      points_array.push_back(points);
    }

    //create mesh list;
    for(int i = 0; i < u_division_num; i++){
      std::vector<geometry_msgs::Point> target_points = points_array[i];
      float prev_index = i - 1, next_index = i + 1;
      if(prev_index < 0)
        prev_index = u_division_num - 1;
      if(next_index > u_division_num - 1)
        next_index = 0;
      std::vector<geometry_msgs::Point> prev_points = points_array[prev_index];
      std::vector<geometry_msgs::Point> next_points = points_array[next_index];
      for(int j = 0; j < v_division_num; j++){
        float next_point_index = j + 1;
        if( next_point_index > v_division_num - 1)
          next_point_index = 0;
        //first pushes
        triangle_mesh.push_back(target_points[j]);
        triangle_mesh.push_back(next_points[j]);
        triangle_mesh.push_back(target_points[next_point_index]);
        //second pushes
        triangle_mesh.push_back(target_points[j]);
        triangle_mesh.push_back(target_points[next_point_index]);
        triangle_mesh.push_back(prev_points[next_point_index]);
      }
    }

    return triangle_mesh;
  }

  visualization_msgs::Marker TransformableTorus::getVisualizationMsgMarker(){
    marker_.points = calcurateTriangleMesh();
    marker_.color.r = torus_r_;
    marker_.color.g = torus_g_;
    marker_.color.b = torus_b_;
    marker_.color.a = torus_a_;
    return marker_;
  }
}
