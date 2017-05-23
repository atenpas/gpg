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


#ifndef GRASP_SET_H_
#define GRASP_SET_H_

// System
#include <vector>

// Boost
#include <boost/functional/hash.hpp>
#include <boost/pool/pool.hpp>
#include <boost/random/normal_distribution.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/unordered_set.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/lagged_fibonacci.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/taus88.hpp>

// Eigen
#include <Eigen/Dense>
#include <unsupported/Eigen/AlignedVector3>

// Custom
#include <gpg/antipodal.h>
#include <gpg/finger_hand.h>
#include <gpg/grasp.h>
#include <gpg/local_frame.h>
#include <gpg/point_list.h>


// The hash and equality functions below are necessary for boost's unordered set.
namespace boost
{
template <>
struct hash<Eigen::Vector3i>
{
    inline size_t operator()(Eigen::Vector3i const& v) const
    {
      std::size_t seed = 0;

      for (int i = 0; i < v.size(); i++)
      {
        boost::hash_combine(seed, v(i));
      }

      return seed;
    }
};
}

struct Vector3iEqual
{
    inline bool operator() (const Eigen::Vector3i& a, const Eigen::Vector3i& b) const
    {
      return a(0) == b(0) && a(1) == b(1) && a(2) == b(2);
    }
};

typedef boost::unordered_set<Eigen::Vector3i, boost::hash<Eigen::Vector3i>, Vector3iEqual,
  std::allocator<Eigen::Vector3i> > Vector3iSet;


/** GraspSet class
 *
 * \brief Calculate grasp sets.
 *
 * This class calculates grasp sets. A grasp set is a set of grasps that are all calculated for the same point
 * neighborhood. Each grasp in the set has a different orientation. The grasps are calculated by a local search over
 * discretized orientations and positions. This discretization is done by fixing two degrees of orientation and
 * rotating the robot hand about a fixed axis.
 *
*/
class GraspSet
{
  public:

    /**
     * \brief Comparator for checking uniqueness of two 3D-vectors.
     */
    struct UniqueVectorComparator
    {
      /**
       * \brief Compares two 3D-vectors for uniqueness.
       * \param a the first 3D-vector
       * \param b the second 3D-vector
       * \return true if they differ in at least one element, false if all elements are equal
       */
      bool operator ()(const Eigen::Vector3i& a, const Eigen::Vector3i& b)
      {
        for (int i = 0; i < a.size(); i++)
        {
          if (a(i) != b(i))
          {
            return a(i) < b(i);
          }
        }

        return false;
      }
    };

    /** Robot hand geometry */
    struct HandGeometry
    {
        double finger_width_; ///< the width of the robot hand fingers
        double outer_diameter_; ///< the maximum robot hand aperture
        double depth_; ///< the hand depth (length of fingers)
        double height_; ///< the hand extends plus/minus this value along the hand axis
        double init_bite_; ///< the minimum object height

        HandGeometry() : finger_width_(0.0), outer_diameter_(0.0), depth_(0.0), height_(0.0), init_bite_(0.0) { }

        HandGeometry(double finger_width, double outer_diameter, double hand_depth, double hand_height,
          double init_bite)
        : finger_width_(finger_width), outer_diameter_(outer_diameter), depth_(hand_depth),
          height_(hand_height), init_bite_(init_bite) {  }
    };

    /**
     * Constructor.
     */
    GraspSet();

    /**
     * \brief Constructor.
     * \param hand_geometry the hand geometry parameters
     * \param angles the orientations to be considered
     * \param rotation_axis the axis about which the orientations are considered
     */
    GraspSet(const HandGeometry& hand_geometry, const Eigen::VectorXd& angles, int rotation_axis);

    /**
     * \brief Calculate a grasp set given a local reference frame.
     * \param point_list the point neighborhood
     * \param local_frame the local reference frame
     */
    void evaluateHypotheses(const PointList& point_list, const LocalFrame& local_frame);

    /**
     * \brief Calculate the set of shadow points for a grasp set.
     * \param point_list the point neighborhood
     * \param shadow_length the length of the shadow
     */
    Eigen::Matrix3Xd calculateShadow4(const PointList& point_list, double shadow_length) const;

    /**
     * \brief Return the grasps contained in this grasp set.
     * \return the grasps contained in this grasp set
     */
    const std::vector<Grasp>& getHypotheses() const
    {
      return hands_;
    }

    /**
     * \brief Set the grasps for this grasp set.
     * \param hands the grasps
     */
    void setHands(const std::vector<Grasp>& hands)
    {
      hands_ = hands;
    }

    /**
     * \brief Return the center of the point neighborhood.
     * \return the center of the point neighborhood
     */
    const Eigen::Vector3d& getSample() const
    {
      return sample_;
    }

    /**
     * \brief Set the center of the point neighborhood.
     * \param sample the center of the point neighborhood
     */
    void setSample(const Eigen::Vector3d& sample)
    {
      sample_ = sample;
    }

    /**
     * \brief Return a list of booleans that indicate for each grasp if it is valid or not.
     * \return the list of booleans
     */
    const Eigen::Array<bool, 1, Eigen::Dynamic>& getIsValid() const
    {
      return is_valid_;
    }

    /**
     * \brief Set, for each grasp, if it is valid or not.
     * \param isValid the list of booleans which indicate if each grasp is valid or not
     */
    void setIsValid(const Eigen::Array<bool, 1, Eigen::Dynamic>& isValid)
    {
      is_valid_ = isValid;
    }

    /**
     * \brief Set a single grasp to be valid or not.
     * \param idx the index of the grasp
     * \param val true if the grasp is valid, false if it is not
     */
    void setIsValidWithIndex(int idx, bool val)
    {
      is_valid_[idx] = val;
    }


  private:

    /**
     * \brief Calculate a single shadow.
     * \param[in] points the list of points for which the shadow is calculated
     * \param[in] shadow_vec the vector that describes the direction of the shadow relative to the camera origin
     * \param[in] num_shadow_points the number of shadow points to be calculated
     * \param[in] voxel_grid_size the size of the voxel grid
     * \param[out] shadow_set the set of shadow points
     */
    void calculateVoxelizedShadowVectorized(const Eigen::Matrix3Xd& points,
      const Eigen::Vector3d& shadow_vec, int num_shadow_points, double voxel_grid_size, Vector3iSet& shadow_set) const;

    /**
     * \brief Create a grasp candidate.
     * \param sample the center of the point neighborhood
     * \param point_list the point neighborhood
     * \param indices_learning the indices of the points used for learning
     * \param hand_frame the orientation of the grasp as a rotation matrix
     * \param finger_hand the FingerHand object that describes valid finger placements
     * \return the grasp
     */
    Grasp createHypothesis(const Eigen::Vector3d& sample, const PointList& point_list,
      const std::vector<int>& indices_learning, const Eigen::Matrix3d& hand_frame, const FingerHand& finger_hand) const;

    /**
     * \brief Label a grasp candidate as a viable grasp or not.
     * \param point_list the point neighborhood associated with the grasp
     * \param finger_hand the FingerHand object that describes valid finger placements
     * \param hand the grasp
     */
    void labelHypothesis(const PointList& point_list, const FingerHand& finger_hand, Grasp& hand) const;

    /**
     * \brief Convert shadow voxels to shadow points.
     * \param voxels the shadow voxels
     * \param voxel_grid_size the size of the voxel grid
     * \return the shadow points
     */
    Eigen::Matrix3Xd shadowVoxelsToPoints(const std::vector<Eigen::Vector3i>& voxels, double voxel_grid_size) const;

    /**
     * \brief Calculate the intersection of two shadows.
     * \param set1 the first shadow
     * \param set2 the second shadow
     * \return the intersection
     */
    Vector3iSet intersection(const Vector3iSet& set1, const Vector3iSet& set2) const;

    /**
     * \brief Generate a random integer.
     * source: http://software.intel.com/en-us/articles/fast-random-number-generator-on-the-intel-pentiumr-4-processor/
     */
    inline int fastrand() const;

    Eigen::Vector3d sample_; ///< the center of the point neighborhood
    std::vector<Grasp> hands_; ///< the grasps contained in this grasp set
    Eigen::Array<bool, 1, Eigen::Dynamic> is_valid_; ///< indicates for each grasp if it is valid or not
    Eigen::VectorXd angles_; ///< the hand orientations to consider in the local search

    HandGeometry hand_geometry_; ///< the robot hand geometry
    int rotation_axis_; ///< the axis about which the hand frame is rotated to generate different orientations

    static int seed_; ///< seed for the random generator in fastrand()

    /** constants for rotation axis */
    static const int ROTATION_AXIS_NORMAL; ///< normal axis of local reference frame
    static const int ROTATION_AXIS_BINORMAL; ///< binormal axis of local reference frame
    static const int ROTATION_AXIS_CURVATURE_AXIS; ///< curvature axis of local reference frame

    static const bool MEASURE_TIME; ///< if runtime is measured
};

#endif /* GRASP_SET_H_ */
