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


#ifndef GRASP_H_
#define GRASP_H_


// Boost
#include <boost/lexical_cast.hpp>

// Eigen
#include <Eigen/Dense>

// System
#include <iostream>
#include <fstream>
#include <vector>

// custom
#include <gpg/finger_hand.h>


struct GraspPose
{
  Eigen::Vector3d surface_; ///< the centered grasp position on the object surface
  Eigen::Vector3d bottom_; ///< the centered grasp position at the base of the robot hand
  Eigen::Vector3d top_; ///< the centered grasp position between the fingertips of the robot hand
  Eigen::Matrix3d frame_; ///< the local reference frame for this grasp hypothesis
};


struct Label
{
  double score_; ///< the score given by the classifier
  bool full_antipodal_; ///< whether the grasp hypothesis is antipodal
  bool half_antipodal_; ///< whether the grasp hypothesis is indeterminate

  Label() : score_(0.0), full_antipodal_(false), half_antipodal_(false) { }

  Label(double score, bool full_antipodal, bool half_antipodal) : score_(score), full_antipodal_(full_antipodal),
    half_antipodal_(half_antipodal) { }
};


struct Configuration1D
{
  double center_; // 1-D positions relative to the hand frame
  double left_; // 1-D positions relative to the hand frame
  double right_; // 1-D positions relative to the hand frame
  double top_; // 1-D positions relative to the hand frame
  double bottom_; // 1-D positions relative to the hand frame
};


/** GraspHypothesis class
 *
 * \brief Grasp hypothesis data structure
 * 
 * This class stores a single grasp hypothesis.
 * 
 */
class Grasp
{
public:

  /**
   * \brief Default constructor.
   */
  Grasp();

  /**
   * \brief Constructor.
   * \param sample the center of the point neighborhood associated with the grasp
   * \param frame the orientation of the grasp as a rotation matrix
   * \param finger_hand the FingerHand object describing a feasible finger placement
   * \param width the opening width of the robot hand
   */
  Grasp(const Eigen::Vector3d& sample, const Eigen::Matrix3d& frame, const FingerHand& finger_hand,
    double width);

  /**
   * \brief Constructor.
   * \param sample the center of the point neighborhood associated with the grasp
   * \param frame the orientation of the grasp as a rotation matrix
   * \param finger_hand the FingerHand object describing a feasible finger placement
   */
  Grasp(const Eigen::Vector3d& sample, const Eigen::Matrix3d& frame, const FingerHand& finger_hand);

  /**
   * \brief Set properties of the grasp.
   * \param finger_hand the FingerHand object describing a feasible finger placement
   */
  void construct(const FingerHand& finger_hand);

  /**
   * \brief Write a list of grasps to a file.
   * \param filename location of the file
   * \param hands the list of grasps
   */
  void writeHandsToFile(const std::string& filename, const std::vector<Grasp>& hands) const;

  /**
   * \brief Print a description of the grasp hypothesis to the systen's standard output.
   */
  void print() const;

  /**
   * \brief Return the approach vector of the grasp.
   * \return 3x1 grasp approach vector
   */
  const Eigen::Vector3d getApproach() const
  {
    return pose_.frame_.col(0);
  }

  /**
   * \brief Return the binormal of the grasp.
   * \return 3x1 binormal
   */
  const Eigen::Vector3d getBinormal() const
  {
    return pose_.frame_.col(1);
  }

  /**
   * \brief Return the hand axis of the grasp.
   * \return 3x1 hand axis
   */
  const Eigen::Vector3d getAxis() const
  {
    return pose_.frame_.col(2);
  }

  /**
   * \brief Return whether the grasp is antipodal.
   * \return true if the grasp is antipodal, false otherwise
   */
  bool isFullAntipodal() const
  {
    return label_.full_antipodal_;
  }

  /**
   * \brief Return the the centered grasp position at the base of the robot hand.
   * \return 3x1 grasp position at the base of the robot hand
   */
  const Eigen::Vector3d& getGraspBottom() const
  {
    return pose_.bottom_;
  }

  /**
   * \brief Return the grasp position between the end of the finger tips projected onto the back of the hand.
   * \return 3x1 grasp position between the end of the finger tips projected onto the back of the hand
   */
  const Eigen::Vector3d& getGraspSurface() const
  {
    return pose_.surface_;
  }

  /**
   * \brief Return the width of the object contained in the grasp.
   * \return the width of the object contained in the grasp
   */
  double getGraspWidth() const
  {
    return grasp_width_;
  }

  /**
   * \brief Return whether the grasp is indeterminate.
   * \return true if the grasp is indeterminate, false otherwise
   */
  bool isHalfAntipodal() const
  {
    return label_.half_antipodal_;
  }

  /**
   * \brief Set whether the grasp is antipodal.
   * \param b whether the grasp is antipodal
   */
  void setFullAntipodal(bool b)
  {
    label_.full_antipodal_ = b;
  }

  /**
   * \brief Set whether the grasp is indeterminate.
   * \param b whether the grasp is indeterminate
   */
  void setHalfAntipodal(bool b)
  {
    label_.half_antipodal_ = b;
  }

  /**
   * \brief Set the width of the object contained in the grasp.
   * \param w the width of the object contained in the grasp
   */
  void setGraspWidth(double w)
  {
    grasp_width_ = w;
  }

  /**
   * \brief Return the the centered grasp position between the fingertips of the robot hand.
   * \return 3x1 grasp position between the fingertips
   */
  const Eigen::Vector3d& getGraspTop() const
  {
    return pose_.top_;
  }

  /**
   * \brief Set the bottom position of the grasp.
   * \param grasp_bottom the bottom position
   */
  void setGraspBottom(const Eigen::Vector3d& grasp_bottom)
  {
    pose_.bottom_ = grasp_bottom;
  }

  /**
   * \brief Set the bottom position relative to the object.
   * \param grasp_surface the bottom position relative to the object
   */
  void setGraspSurface(const Eigen::Vector3d& grasp_surface)
  {
    pose_.surface_ = grasp_surface;
  }

  /**
   * \brief Set the top position of the grasp.
   * \param grasp_top the top position
   */
  void setGraspTop(const Eigen::Vector3d& grasp_top)
  {
    pose_.top_ = grasp_top;
  }

  /**
   * \brief Get the score of the grasp.
   * \return the score
   */
  double getScore() const
  {
    return label_.score_;
  }

  /**
   * \brief Set the score of the grasp.
   * \param score the score
   */
  void setScore(double score)
  {
    label_.score_ = score;
  }

  /**
   * \brief Return the center of the point neighborhood associated with the grasp.
   * \return the center
   */
  const Eigen::Vector3d& getSample() const
  {
    return sample_;
  }

  /**
   * \brief Return the grasp orientation.
   * \return the orientation of the grasp as a rotation matrix
   */
  const Eigen::Matrix3d& getFrame() const
  {
    return pose_.frame_;
  }

  /**
   * \brief Return the grasp position.
   * \return the position of the grasp
   */
  const Eigen::Vector3d getPosition() const
  {
    return pose_.bottom_;
  }

  double getCenter() const
  {
    return config_1d_.center_;
  }

  double getBottom() const
  {
    return config_1d_.bottom_;
  }

  double getLeft() const
  {
    return config_1d_.left_;
  }

  double getRight() const
  {
    return config_1d_.right_;
  }

  double getTop() const
  {
    return config_1d_.top_;
  }

  /**
   * \brief Return the index of the finger placement.
   * \return the index of the finger placement
   */
  int getFingerPlacementIndex () const
  {
    return finger_placement_index_;
  }


private:

  /**
   * \brief Calculate grasp positions (bottom, top, surface).
   * \param finger_hand the FingerHand object describing a feasible finger placement
   */
  void calculateGraspPositions(const FingerHand& finger_hand);

  /**
   * \brief Convert an Eigen vector to a string.
   * \param v the vector
   * \return the string
   */
  std::string vectorToString(const Eigen::VectorXd& v) const;

protected:

  Eigen::Vector3d sample_; ///< the sample at which the grasp hypothesis was found
  double grasp_width_; ///< the width of object enclosed by the fingers

  GraspPose pose_; ///< the grasp pose
  Label label_; ///< labeling information
  int finger_placement_index_; ///< index of the finger placement that resulted in this grasp
  Configuration1D config_1d_; ///< position of fingers and hand base relative to local hand frame
};

#endif /* GRASP_H_ */
