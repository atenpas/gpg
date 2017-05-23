/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Andreas ten Pas
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
 */


#ifndef POINT_LIST_H_
#define POINT_LIST_H_


#include <Eigen/Dense>

#include <vector>

#include <gpg/eigen_utils.h>


/** PointList class
 *
 * This class represents a list of points by storing the points (3 x n matrix) and their surface normals
 * (3 x n matrix). It also keeps information about which camera sees which point by storing the camera origins
 * (3 x k matrix), and for each point, if it is seen or not from each camera (k x n matrix). If point i is seen from
 * camera j, then <cam_source[i,j]> = 1. Otherwise, <cam_source[i,j]> = 0.
 *
 */
class PointList
{
  public:

    /**
     * \brief Default constructor.
     */
    PointList() { }

    /**
     * \brief Construct a list of n points.
     * \param points the points (3 x n)
     * \param normals the surface normals associated with the points (3 x n)
     * \param cam_source the camera source for each point (k x n)
     * \param view_points the origins of the cameras that saw the points (3 x k)
     */
    PointList(const Eigen::Matrix3Xd& points, const Eigen::Matrix3Xd& normals, const Eigen::MatrixXi& cam_source,
      const Eigen::Matrix3Xd& view_points)
      : points_(points), normals_(normals), cam_source_(cam_source), view_points_(view_points) { }

    /**
     * \brief Constructor.
     * \param size number of points
     * \param num_cams number of cameras that observed the points
     */
    PointList(int size, int num_cams);

    /**
     * \brief Slice the point list given a set of indices.
     * \param indices the indices to be sliced
     * \return the point list containing the points given by the indices
     */
    PointList slice(const std::vector<int>& indices) const;

    /**
     * \brief Transform a point list to a robot hand frame.
     * \param centroid the origin of the frame
     * \param rotation the orientation of the frame (3 x 3 rotation matrix)
     * \return the point list transformed into the hand frame
     */
    PointList transformToHandFrame(const Eigen::Vector3d& centroid, const Eigen::Matrix3d& rotation) const;

    /**
     * \brief Rotate a point list.
     * \param rotation the 3 x 3 rotation matrix
     * \return the rotated point list
     */
    PointList rotatePointList(const Eigen::Matrix3d& rotation) const;

    /**
     * \brief Crop the points by the height of the robot hand.
     * \param height the robot hand height
     * \param dim the dimension of the points corresponding to the height
     */
    PointList cropByHandHeight(double height, int dim = 2) const;

    const Eigen::MatrixXi& getCamSource() const
    {
      return cam_source_;
    }

    void setCamSource(const Eigen::MatrixXi& cam_source)
    {
      cam_source_ = cam_source;
    }

    const Eigen::Matrix3Xd& getNormals() const
    {
      return normals_;
    }

    void setNormals(const Eigen::Matrix3Xd& normals)
    {
      normals_ = normals;
    }

    const Eigen::Matrix3Xd& getPoints() const
    {
      return points_;
    }

    void setPoints(const Eigen::Matrix3Xd& points)
    {
      points_ = points;
    }

    int size() const
    {
      return points_.cols();
    }

    const Eigen::Matrix3Xd& getViewPoints() const
    {
      return view_points_;
    }

    void setViewPoints(const Eigen::Matrix3Xd& view_points)
    {
      view_points_ = view_points;
    }


  private:

    Eigen::Matrix3Xd points_; ///< the points (3 x n matrix)
    Eigen::Matrix3Xd normals_; ///< the surface normals of the points (3 x n matrix)
    Eigen::MatrixXi cam_source_; ///< the camera source (k x n matrix of 1s and 0s)
    Eigen::Matrix3Xd view_points_; ///< the camera origins (3 x k matrix)
};


#endif /* POINT_LIST_H_ */
