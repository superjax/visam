#include "visam/visam.h"

namespace py = pybind11;

namespace visam
{

visam::visam() :
    parameters_(),
    isam_(parameters_),
    K_(new Cal3_S2(50.0, 50.0, 0.0, 50.0, 50.0))
{
    initialized_ = false;
    newest_initialized_feature_ = 0;
    depth_guess_ = 1.0;
    current_frame_id_ = -1;
}

void visam::imuCallback() {}


void visam::featureCallback(py::list py_feature_ids, py::list py_feature_xy, int frame_id)
{
    // Create a Factor Graph and Values to hold the new data
    NonlinearFactorGraph graph;
    Values initialEstimate;

    // convert python objects to c++ objects
    std::vector<int> feature_ids = py_feature_ids.cast<std::vector<int>>();
    std::vector<py::list> feature_xy_lists = py_feature_xy.cast<std::vector<py::list>>();

    // Create a new node for the camera pose
//    SimpleCamera camera(current_pose_, *K_);

    // Add each feature measurement to the graph
    for (int i = 0; i < feature_xy_lists.size(); i++)
    {
        std::vector<double> feature_xy = feature_xy_lists[i].cast<std::vector<double>>();
        int feature_id = feature_ids[i];
        Point2 feature_location;
        feature_location[0] = feature_xy[0];
        feature_location[1] = feature_xy[1];
        graph.emplace_shared<GenericProjectionFactor<Pose3, Point3, Cal3_S2>>
                (feature_location, measurementNoise, Symbol('x', frame_id), Symbol('l', feature_id), K_);

        // If this is an uninitialized feature, then initialize it
        if(feature_id > newest_initialized_feature_)
        {
            SimpleCamera camera(current_pose_, *K_);
            Point3 guess = camera.backproject(feature_location, depth_guess_);
            initialEstimate.insert<Point3>(Symbol('l', feature_id), guess);

        }
    }

    // Add a guess for the current pose
    initialEstimate.insert(Symbol('x', frame_id), current_pose_);

    // Handle Initialization of Features and Pose Nodes
    if (!initialized_)
    {
        // Prior for pose
        noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.3),Vector3::Constant(0.1)).finished()); // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
        graph.emplace_shared<PriorFactor<Pose3> >(Symbol('x', frame_id), current_pose_, poseNoise);

        // Prior for first feature
        Point2 feature_location;
        int feature_id = feature_ids[0];
        feature_location[0] = feature_xy_lists[0].cast<std::vector<double>>()[0];
        feature_location[1] = feature_xy_lists[0].cast<std::vector<double>>()[1];
        SimpleCamera camera(current_pose_, *K_);
        Point3 guess = camera.backproject(feature_location, depth_guess_);
        noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 1.0);
        graph.emplace_shared<PriorFactor<Point3> >(Symbol('l', feature_id), guess, pointNoise); // add directly to graph
    }

    isam_.update(graph, initialEstimate);

    // update current state (best guess is pose of current frame)
    current_pose_ = isam_.calculateEstimate(Symbol('x', frame_id)).cast<Pose3>();

    current_frame_id_ = frame_id;
}

void visam::update()
{
    if (initialized_)
    {
        isam_.update();
        current_pose_ = isam_.calculateEstimate(Symbol('x', current_frame_id_)).cast<Pose3>();;
    }
}


py::list visam::get_state()
{
    Quaternion quat = current_pose_.rotation().toQuaternion();
    Point3 trans = current_pose_.translation();

    py::list out_list;
    out_list.append(quat.w());
    out_list.append(quat.x());
    out_list.append(quat.y());
    out_list.append(quat.z());
    out_list.append(trans.x());
    out_list.append(trans.y());
    out_list.append(trans.z());
    return out_list;
}

}

PYBIND11_PLUGIN(visam) {
  py::module m("visam", "pybind11 visam plugin");

  py::class_<visam::visam>(m, "visam")
      .def("__init__", [](visam::visam &instance) {
    new (&instance) visam::visam();
  })
  .def("imu", &visam::visam::imuCallback)
      .def("add_features", &visam::visam::featureCallback)
      .def("get_state", &visam::visam::get_state);

  return m.ptr();
}
