// System
#include <sstream>
#include <string>
#include <vector>

// Custom
#include <gpg/candidates_generator.h>
#include <gpg/hand_search.h>
#include <gpg/config_file.h>


// function to read in a double array from a single line of a configuration file
std::vector<double> stringToDouble(const std::string& str)
{
  std::vector<double> values;
  std::stringstream ss(str);
  double v;

  while (ss >> v)
  {
    values.push_back(v);
    if (ss.peek() == ' ')
    {
      ss.ignore();
    }
  }

  return values;
}


int main(int argc, char* argv[])
{
  // Read arguments from command line.
  if (argc < 3)
  {
    std::cout << "Error: Not enough input arguments!\n\n";
    std::cout << "Usage: generate_candidates [CONFIG_FILE] [PCD_FILE] [NORMALS_FILE]\n\n";
    std::cout << "Generate grasp candidates for a point cloud, PCD_FILE (*.pcd), using parameters from CONFIG_FILE (*.cfg).\n\n";
    std::cout << "[NORMALS_FILE] (optional) contains a surface normal for each point in the cloud (*.csv).\n";
    return (-1);
  }

  // Read parameters from configuration file.
  ConfigFile config_file(argv[1]);

  double finger_width = config_file.getValueOfKey<double>("finger_width", 0.01);
  double hand_outer_diameter  = config_file.getValueOfKey<double>("hand_outer_diameter", 0.12);
  double hand_depth = config_file.getValueOfKey<double>("hand_depth", 0.06);
  double hand_height  = config_file.getValueOfKey<double>("hand_height", 0.02);
  double init_bite  = config_file.getValueOfKey<double>("init_bite", 0.01);

  std::cout << "finger_width: " << finger_width << "\n";
  std::cout << "hand_outer_diameter: " << hand_outer_diameter << "\n";
  std::cout << "hand_depth: " << hand_depth << "\n";
  std::cout << "hand_height: " << hand_height << "\n";
  std::cout << "init_bite: " << init_bite << "\n";

  bool voxelize = config_file.getValueOfKey<bool>("voxelize", true);
  bool remove_outliers = config_file.getValueOfKey<bool>("remove_outliers", false);
  std::string workspace_str = config_file.getValueOfKeyAsString("workspace", "");
  std::string camera_pose_str = config_file.getValueOfKeyAsString("camera_pose", "");
  std::vector<double> workspace = stringToDouble(workspace_str);
  std::vector<double> camera_pose = stringToDouble(camera_pose_str);
  std::cout << "voxelize: " << voxelize << "\n";
  std::cout << "remove_outliers: " << remove_outliers << "\n";
  std::cout << "workspace: " << workspace_str << "\n";
  std::cout << "camera_pose: " << camera_pose_str << "\n";

  int num_samples = config_file.getValueOfKey<int>("num_samples", 1000);
  int num_threads = config_file.getValueOfKey<int>("num_threads", 1);
  double nn_radius = config_file.getValueOfKey<double>("nn_radius", 0.01);
  int num_orientations = config_file.getValueOfKey<int>("num_orientations", 8);
  int rotation_axis = config_file.getValueOfKey<int>("rotation_axis", 2);
  std::cout << "num_samples: " << num_samples << "\n";
  std::cout << "num_threads: " << num_threads << "\n";
  std::cout << "nn_radius: " << nn_radius << "\n";
  std::cout << "num_orientations: " << num_orientations << "\n";
  std::cout << "rotation_axis: " << rotation_axis << "\n";

  bool plot_grasps = config_file.getValueOfKey<bool>("plot_grasps", true);
  bool plot_normals = config_file.getValueOfKey<bool>("plot_normals", false);
  std::cout << "plot_grasps: " << plot_grasps << "\n";
  std::cout << "plot_normals: " << plot_normals << "\n";

  // Create object to generate grasp candidates.
  CandidatesGenerator::Parameters generator_params;
  generator_params.num_samples_ = num_samples;
  generator_params.num_threads_ = num_threads;
  generator_params.plot_normals_ = plot_normals;
  generator_params.plot_grasps_ = plot_grasps;
  generator_params.remove_statistical_outliers_ = remove_outliers;
  generator_params.voxelize_ = voxelize;
  generator_params.workspace_ = workspace;
  HandSearch::Parameters hand_search_params;
  hand_search_params.finger_width_ = finger_width;
  hand_search_params.hand_outer_diameter_ = hand_outer_diameter;
  hand_search_params.hand_depth_ = hand_depth;
  hand_search_params.hand_height_ = hand_height;
  hand_search_params.init_bite_ = init_bite;
  hand_search_params.nn_radius_frames_ = nn_radius;
  hand_search_params.num_orientations_ = num_orientations;
  hand_search_params.num_samples_ = num_samples;
  hand_search_params.num_threads_ = num_threads;
  hand_search_params.rotation_axis_ = rotation_axis;
  CandidatesGenerator candidates_generator(generator_params, hand_search_params);

  // Set the camera pose.
  Eigen::Matrix3Xd view_points(3,1);
  view_points << camera_pose[3], camera_pose[6], camera_pose[9];

  // Create object to load point cloud from file.
  CloudCamera cloud_cam(argv[2], view_points);
  if (cloud_cam.getCloudOriginal()->size() == 0)
  {
    std::cout << "Input point cloud is empty or does not exist!\n";
    return (-1);
  }

  // Load surface normals from file.
  std::cout << argc << "\n";
  if (argc > 3)
  {
    cloud_cam.setNormalsFromFile(argv[3]);
    std::cout << "Loaded surface normals from file.\n";
  }

  // Point cloud preprocessing: voxelize, remove statistical outliers, workspace filter, compute normals, subsample.
  candidates_generator.preprocessPointCloud(cloud_cam);

  // Generate a list of grasp candidates.
  std::vector<Grasp> candidates = candidates_generator.generateGraspCandidates(cloud_cam);

  return 0;
}
