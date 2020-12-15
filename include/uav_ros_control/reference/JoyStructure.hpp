#ifndef JOY_STRUCTURE_H
#define JOY_STRUCTURE_H

#include <iostream>
#include <string>

/**
 * Namespace defining Joy message indices structures/
 */
namespace joy_struct {

/**
 * This structure defines all joy indices needed for managing inspection mode.
 */
struct InspectionIndices
{
  /** Index for enabling inspection mode. */
  int INSPECTION_MODE;

  /** Index for enabling the left sequence. */
  int LEFT_SEQUENCE;

  /** Index for enabling the right sequence. */
  int RIGHT_SEQUENCE;

  friend std::ostream &operator<<(std::ostream &out, const InspectionIndices &a)
  {
    out << "InspectionIndices are:\ninspect=" << a.INSPECTION_MODE
        << "\nleft sequence=" << a.LEFT_SEQUENCE
        << "\nright sequence=" << a.RIGHT_SEQUENCE << std::endl;
    return out;
  }
};

/**
 * This structure defines all joy indices needed for the UAV control mode.
 */
struct ControlIndices
{
  /** Movement along the x-axis */
  int AXIS_LINEAR_X;

  /** Movement along the y-axis */
  int AXIS_LINEAR_Y;

  /** Movement along the z-axis */
  int AXIS_LINEAR_Z;

  /** Movement around the z-axis */
  int AXIS_ANGULAR_YAW;

  friend std::ostream &operator<<(std::ostream &out, const ControlIndices &a)
  {
    out << "ControlIndices are:\nlin_x=" << a.AXIS_LINEAR_X
        << "\nlin_y=" << a.AXIS_LINEAR_Y << "\nlin_z=" << a.AXIS_LINEAR_Z
        << "\nang_z=" << a.AXIS_ANGULAR_YAW << std::endl;

    return out;
  }
};

/**
 * This structure defines values which scale the received control inputs.
 */
struct ScaleWeights
{
  /** Scale movement along the x-axis */
  double LINEAR_X;

  /** Scale movement along the y-axis */
  double LINEAR_Y;

  /** Scale movement along the z-axis */
  double LINEAR_Z;

  /** Scale movement around the z-axis */
  double ANGULAR_Z;

  friend std::ostream &operator<<(std::ostream &out, const ScaleWeights &a)
  {
    out << "ScaleWeights are:\nlin_x=" << a.LINEAR_X << "\nlin_y=" << a.LINEAR_Y
        << "\nlin_z=" << a.LINEAR_Z << "\nang_z=" << a.ANGULAR_Z << std::endl;

    return out;
  }
};
}// namespace joy_struct

#endif /* JOY_STRUCTURE_H */