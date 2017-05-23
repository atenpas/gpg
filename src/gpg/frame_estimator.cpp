#include <gpg/frame_estimator.h>


std::vector<LocalFrame> FrameEstimator::calculateLocalFrames(const CloudCamera& cloud_cam,
  const std::vector<int>& indices, double radius, const pcl::KdTreeFLANN<pcl::PointXYZRGBA>& kdtree) const
{
  double t1 = omp_get_wtime();
  std::vector<LocalFrame*> frames;
  frames.resize(indices.size());

#ifdef _OPENMP // parallelization using OpenMP
#pragma omp parallel for num_threads(num_threads_)
#endif
  for (int i = 0; i < indices.size(); i++)
  {
    const pcl::PointXYZRGBA& sample = cloud_cam.getCloudProcessed()->points[indices[i]];
    LocalFrame* frame = calculateFrame(cloud_cam.getNormals(), sample.getVector3fMap().cast<double>(), radius, kdtree);
    frames[i] = frame;
//    frames[i]->print();
  }

  std::vector<LocalFrame> frames_out;
  for (int i = 0; i < frames.size(); i++)
  {
    if (frames[i])
      frames_out.push_back(*frames[i]);
    delete frames[i];
  }
  frames.clear();

  double t2 = omp_get_wtime();
  std::cout << "Estimated " << frames_out.size() << " local reference frames in " << t2 - t1 << " sec.\n";

  return frames_out;
}


std::vector<LocalFrame> FrameEstimator::calculateLocalFrames(const CloudCamera& cloud_cam,
  const Eigen::Matrix3Xd& samples, double radius, const pcl::KdTreeFLANN<pcl::PointXYZRGBA>& kdtree) const
{
  double t1 = omp_get_wtime();
  std::vector<LocalFrame*> frames;
  frames.resize(samples.cols());

#ifdef _OPENMP // parallelization using OpenMP
#pragma omp parallel for num_threads(num_threads_)
#endif
  for (int i = 0; i < samples.cols(); i++)
  {
    LocalFrame* frame = calculateFrame(cloud_cam.getNormals(), samples.col(i), radius, kdtree);
    frames[i] = frame;
//    frames[i]->print();
  }

  // delete empty frames
  std::vector<LocalFrame> frames_out;
  for (int i = 0; i < frames.size(); i++)
  {
    if (frames[i])
      frames_out.push_back(*frames[i]);
    delete frames[i];
  }
  frames.clear();

  double t2 = omp_get_wtime();
  std::cout << "Estimated " << frames_out.size() << " local reference frames in " << t2 - t1 << " sec.\n";

  return frames_out;
}


LocalFrame* FrameEstimator::calculateFrame(const Eigen::Matrix3Xd& normals, const Eigen::Vector3d& sample,
  double radius, const pcl::KdTreeFLANN<pcl::PointXYZRGBA>& kdtree) const
{
  LocalFrame* frame = NULL;
  std::vector<int> nn_indices;
  std::vector<float> nn_dists;
  pcl::PointXYZRGBA sample_pcl = eigenVectorToPcl(sample);

  if (kdtree.radiusSearch(sample_pcl, radius, nn_indices, nn_dists) > 0)
  {
    Eigen::Matrix3Xd nn_normals(3, nn_indices.size());

    for (int i = 0; i < nn_normals.cols(); i++)
    {
      nn_normals.col(i) = normals.col(nn_indices[i]);
    }

    frame = new LocalFrame(sample, -1);
    frame->findAverageNormalAxis(nn_normals);
  }

  return frame;
}


pcl::PointXYZRGBA FrameEstimator::eigenVectorToPcl(const Eigen::Vector3d& v) const
{
  pcl::PointXYZRGBA p;
  p.x = v(0);
  p.y = v(1);
  p.z = v(2);
  return p;
}
