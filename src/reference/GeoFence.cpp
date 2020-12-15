#include <uav_ros_control/reference/GeoFence.hpp>
#include <typeinfo>

uav_reference::GeoFence::GeoFence(ros::NodeHandle &nh, std::string filename)
  : _global_to_local(nh)
{
  ROS_INFO_STREAM("Loading GPS constraint points from file:\n" << filename);
  YAML::Node config = YAML::LoadFile(filename);
  YAML::Node constraints_list = config["gps_constraints"];
  ROS_INFO_STREAM("Loaded GPS points:\n" << constraints_list);

  ros::Duration(1).sleep();
  ros::spinOnce();

  ROS_INFO_STREAM("Constraint points converted to local frame:");
  _max_z = 1000;
  _min_z = 0;
  for (YAML::const_iterator ti = constraints_list.begin(); ti != constraints_list.end();
       ++ti) {
    const YAML::Node &constraint = *ti;
    double lat = constraint["lat"].as<double>();
    double lon = constraint["lon"].as<double>();
    double alt = constraint["alt"].as<double>();
    Eigen::Vector3d temp_vector = _global_to_local.toLocal(lat, lon, alt);
    geometry_msgs::Vector3 vertex;
    vertex.x = temp_vector.x();
    vertex.y = temp_vector.y();
    vertex.z = alt;
    if (vertex.z < _max_z) { _max_z = vertex.z; }
    _vertices.push_back(vertex);
    std::cout << "X: " << vertex.x << ", Y: " << vertex.y << ", Z: " << vertex.z
              << std::endl;
  }
  ROS_INFO_STREAM("Maximum height: " << _max_z);
  _vertices.push_back(_vertices[0]);

  // Find centroid of polygon.
  double area = 0;
  double sum_x = 0;
  double sum_y = 0;
  int n = _vertices.size() - 1;
  for (int i = 0; i < n; ++i) {
    area =
      area + (_vertices[i].x * _vertices[i + 1].y - _vertices[i + 1].x * _vertices[i].y);
    sum_x =
      sum_x
      + (_vertices[i].x + _vertices[i + 1].x)
          * (_vertices[i].x * _vertices[i + 1].y - _vertices[i + 1].x * _vertices[i].y);
    sum_y =
      sum_y
      + (_vertices[i].y + _vertices[i + 1].y)
          * (_vertices[i].x * _vertices[i + 1].y - _vertices[i + 1].x * _vertices[i].y);
  }
  area = 1.0 / 2 * area;
  _centroid.x = 1.0 / (6 * area) * sum_x;
  _centroid.y = 1.0 / (6 * area) * sum_y;
  ROS_INFO_STREAM("Centroid of constraints in local frame:\n" << _centroid);

  // Define Publisher
  _pub = nh.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("geofence_out", 1);

  // Define Subscriber
  _sub = nh.subscribe("geofence_in", 1, &uav_reference::GeoFence::referenceCb, this);
}

uav_reference::GeoFence::~GeoFence() {}

void uav_reference::GeoFence::referenceCb(
  const trajectory_msgs::MultiDOFJointTrajectoryPointConstPtr &msg)
{
  geometry_msgs::Vector3 current_position = msg->transforms.front().translation;
  trajectory_msgs::MultiDOFJointTrajectoryPoint new_msg = *msg;

  bool inside_planar = checkInside2D(current_position);

  // If inside specified area, limit the height if necessary and forward the message.
  if (checkInside2D(current_position)) {
    new_msg.transforms.front().translation.z =
      limitValue(current_position.z, _min_z, _max_z);
    _last_valid_position = msg->transforms.front().translation;
    _pub.publish(new_msg);
  }
  // Otherwise, find the closest allowed position and publish that.
  else {
    // std::cout << "outside" << std::endl;
    geometry_msgs::Vector3 new_ref = findClosestPoint(current_position);
    new_ref.z = limitValue(current_position.z, _min_z, _max_z);

    new_msg.transforms.front().translation = new_ref;
    new_msg.velocities.front() = geometry_msgs::Twist();
    new_msg.accelerations.front() = geometry_msgs::Twist();
    _pub.publish(new_msg);
  }
}

geometry_msgs::Vector3 uav_reference::GeoFence::findClosestPoint(
  geometry_msgs::Vector3 current)
{
  int n = _vertices.size() - 1;

  // Find the point on the intersection of polygon segment
  // and line from centroid to current position.
  geometry_msgs::Vector3 new_ref;
  geometry_msgs::Vector3 new_ref_return;
  double min_dist_from_current = INT_MAX;
  for (int i = 0; i < n; ++i) {
    if (lineIntersection(_vertices[i], _vertices[i + 1], _centroid, current, new_ref))
      ;
    {
      if (isPointOnLineSegment(new_ref, _vertices[i], _vertices[i + 1])) {
        double dist_from_current = calcDistance(current, new_ref);
        if (dist_from_current < min_dist_from_current) {
          min_dist_from_current = dist_from_current;
          new_ref_return = new_ref;
        }
      }
    }
  }

  if (min_dist_from_current < INT_MAX) {
    return new_ref_return;
  } else {
    return _last_valid_position;
  }


  // double min_vertex_distance = INT_MAX;
  // 	geometry_msgs::Vector3 min_vertex;
  // 	for (int i=0; i<n; ++i)
  // 	{
  // 		double distance = calcDistance(current, _vertices[i]);
  // 		if (distance < min_vertex_distance)
  // 		{
  // 			min_vertex_distance = distance;
  // 			min_vertex = _vertices[i];
  // 		}
  // 	}

  // // Find the projection on each polygon segment and distances to those points.
  // std::vector< std::pair <geometry_msgs::Vector3, double> > projections;
  // for (int i=0; i<n; ++i)
  // {
  // 	geometry_msgs::Vector3 projection = findProjection(_vertices[i], _vertices[i+1],
  // current); 	if (isPointOnLineSegment(projection, _vertices[i], _vertices[i+1]))
  // 	{
  // 		double distance = calcDistance(current, projection);
  // 		projections.push_back( make_pair(projection, distance));
  // 	}
  // }
  // // Sort the projections by distances. The closest one is our desired reference
  // position. if (projections.size() > 0)
  // {
  // 	std::sort(projections.begin(), projections.end(), compareVectors);
  // 	return projections.front().first;
  // }
  // // If none of the projections are on specified polygon, return the closest vertex.
  // else
  // {

  // 	return min_vertex;
  // }
}


bool uav_reference::GeoFence::checkInside2D(geometry_msgs::Vector3 current)
{
  int wn = 0;// the winding number counter
  int n = _vertices.size() - 1;// number of points in fence polygon

  // std::cout << "Current: " << current << std::endl;
  // loop through all edges of the polygon
  for (int i = 0; i < n; i++) {// edge from V[i] to  V[i+1]
    // std::cout << "Vertex: " << _vertices[i] << std::endl;

    if (_vertices[i].y <= current.y) {// start y <= current.y
      if (_vertices[i + 1].y > current.y)// an upward crossing
        if (isLeft(_vertices[i], _vertices[i + 1], current) > 0)// current left of  edge
          ++wn;// have  a valid up intersect
    } else {// start y > current.y (no test needed)
      if (_vertices[i + 1].y <= current.y)// a downward crossing
        if (isLeft(_vertices[i], _vertices[i + 1], current) < 0)// current right of  edge
          --wn;// have  a valid down intersect
    }
  }
  return wn > 0;
}

// isLeft(): tests if a point is Left|On|Right of an infinite line.
//    Input:  three points P0, P1, and P2
//    Return: >0 for P2 left of the line through P0 and P1
//            =0 for P2  on the line
//            <0 for P2  right of the line
//    See: Algorithm 1 "Area of Triangles and Polygons"
int uav_reference::GeoFence::isLeft(geometry_msgs::Vector3 P0,
  geometry_msgs::Vector3 P1,
  geometry_msgs::Vector3 P2)
{

  return ((P1.x - P0.x) * (P2.y - P0.y) - (P2.x - P0.x) * (P1.y - P0.y));
}

void uav_reference::runDefault(uav_reference::GeoFence &geoFenceObj, ros::NodeHandle &nh)
{
  ros::spin();
}
