#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>
#include <dart/dart.hpp>
#include <typeinfo>

using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::math;

class OperationalSpaceControlWorld : public dart::gui::osg::WorldNode
{
public:
  OperationalSpaceControlWorld(dart::simulation::WorldPtr _world)
    : dart::gui::osg::WorldNode(_world)
  {
    // Extract the relevant pointers
    robot = mWorld->getSkeleton(0);
    mEndEffector = robot->getBodyNode(robot->getNumBodyNodes() - 1);
    mEndEffectorPose.resize(6); // Store (x, y, z, roll, pitch, yaw)

    // Set joint properties
    robot->eachJoint([](dart::dynamics::Joint* joint) {
      joint->setLimitEnforcement(false);
      joint->setDampingCoefficient(0, 0.5);
    });

    // mOffset = Eigen::Vector6d(0.05, 0, 0, 0, 0, 0);
    mOffset = Eigen::Vector3d(1.0, 1.0, 1.0);

    // Create target Frame
    Eigen::Isometry3d tf = mEndEffector->getWorldTransform();
    tf.pretranslate(mOffset);

    mOffset
        = mEndEffector->getWorldTransform().rotation().transpose() * mOffset;

    // Gravity torques
    gravityTorques = robot->getGravityForces();

    // Transposed Jacobian matrix in task space
    jacobian = mEndEffector->getJacobian();
    jacobianTranspose = jacobian.transpose();

    // Inertia matrix in task space
    massMatrix = robot->getMassMatrix();
    taskSpaceInertia = jacobian * massMatrix * jacobianTranspose;

    // Coriolis matrix in task space
    coriolisForces = robot->getCoriolisForces();
    taskSpaceCoriolis = jacobian * coriolisForces.asDiagonal();

    // Define damping and stiffness matrices
    dampingMatrix = Eigen::Matrix6d::Identity() * 1.0; // Example: 10.0 as damping coefficient
    stiffnessMatrix = Eigen::Matrix6d::Identity() * 5.0; // Example: 50.0 as stiffness coefficient

    // Define damping and stiffness matrices
    accelerations = Eigen::Vector6d::Zero();
    error_velocities = Eigen::Vector6d::Zero();
    velocities = Eigen::Vector6d::Zero();
    error_positions = Eigen::Vector6d::Zero();
    positions = Eigen::Vector6d::Ones();
  }

  // Triggered at the beginning of each simulation step
  void customPreStep() override
  {
    // Updating properties
    // Gravity torques
    gravityTorques = robot->getGravityForces();

    // Transposed Jacobian matrix in task space
    jacobian = mEndEffector->getJacobian();
    jacobianTranspose = jacobian.transpose();

    // Inertia matrix in task space
    massMatrix = robot->getMassMatrix();
    taskSpaceInertia = jacobian * massMatrix * jacobianTranspose;

    // Coriolis matrix in task space
    coriolisForces = robot->getCoriolisForces();
    taskSpaceCoriolis = jacobian * coriolisForces.asDiagonal();

    std::cout << "DEBUG1" << std::endl;

    mEndEffectorPose.head<3>() = mEndEffector->getWorldTransform().translation(); // Extract translation (x, y, z)
    mEndEffectorPose.tail<3>() = mEndEffector->getWorldTransform().rotation().eulerAngles(2, 1, 0); // ZYX convention (yaw, pitch, roll)
    std::cout << "End-Effector Position and Orientation (XYZRPY):\n" << mEndEffectorPose.transpose() << std::endl;
    std::cout << "positions:\n" << positions.transpose() << std::endl;

    std::cout << "DEBUG2" << std::endl;

    error_positions = positions.transpose() - mEndEffectorPose.transpose() * 0.001;

    std::cout << "DEBUG3" << std::endl;

    // Interation
    Eigen::MatrixXd cartesianPart = taskSpaceInertia * accelerations.transpose() + taskSpaceCoriolis * velocities.transpose()
         + dampingMatrix * error_velocities.transpose() + stiffnessMatrix * error_positions.transpose();
    std::cout << "DEBUG4" << std::endl;
    
    Eigen::VectorXd mForces = gravityTorques + jacobianTranspose * cartesianPart;

    std::cout << "term1:\n" << taskSpaceInertia * accelerations << std::endl;
    std::cout << "term2:\n" << taskSpaceCoriolis * velocities << std::endl;
    std::cout << "term3:\n" << dampingMatrix * error_velocities << std::endl;
    std::cout << "term4:\n" << stiffnessMatrix * error_positions << std::endl;

    std::cout << "gravityTorques:\n" << gravityTorques << std::endl;
    std::cout << "mForces:\n" << mForces << std::endl;
    std::cout << "error_positions:\n" << error_positions << std::endl;
    // std::cout << "cartesianPart:\n" << cartesianPart << std::endl;
    // std::cout << "mEndEffector:\n" << mEndEffectorPose * mOffset << std::endl;

    robot->setForces(mForces);
    // robot->setForces(mForces * -1.0);
    // robot->setForces(gravityTorques * 1.0);
    // robot->setForces(Eigen::Vector6d(0.0, -3.2, -2.39, 0.0, 0.0, 0.0));
    // robot->setForces(mEndEffectorPose * mOffset);
  }

protected:
  // Triggered when this node gets added to the Viewer
  void setupViewer() override
  {
    if (mViewer) {
    //   dnd->setObstructable(false);
      mViewer->addInstructionText(
          "\nLoading simulation ...\n");
    }
  }

  SkeletonPtr robot;
  BodyNode* mEndEffector;

  Eigen::VectorXd mEndEffectorPose; // Now stores (x, y, z, roll, pitch, yaw)
  Eigen::Vector3d mOffset;
  // Gravity torques
  Eigen::VectorXd gravityTorques;

  // Transposed Jacobian matrix in task space
  Eigen::MatrixXd jacobian;
  Eigen::MatrixXd jacobianTranspose;

  // Inertia matrix in task space
  Eigen::MatrixXd massMatrix;
  Eigen::MatrixXd taskSpaceInertia;

  // Coriolis matrix in task space
  Eigen::VectorXd coriolisForces;
  Eigen::MatrixXd taskSpaceCoriolis;

  // Define damping and stiffness matrices
  Eigen::MatrixXd dampingMatrix;
  Eigen::MatrixXd stiffnessMatrix;

  // Define damping and stiffness matrices
  Eigen::VectorXd accelerations;
  Eigen::VectorXd error_velocities;
  Eigen::VectorXd velocities;
  Eigen::VectorXd error_positions;
  Eigen::VectorXd positions;
  Eigen::VectorXd mForces;
};

int main()
{
  dart::simulation::WorldPtr world(new dart::simulation::World);
  dart::utils::DartLoader loader;

  // Load the robot
  dart::dynamics::SkeletonPtr robot
      = loader.parseSkeleton("dart://sample/urdf/omp/open_manipulator_pro.urdf");
  world->addSkeleton(robot);

  // Rotate the robot so that z is upwards (default transform is not Identity)
  robot->getJoint(0)->setTransformFromParentBodyNode(
      Eigen::Isometry3d::Identity());

  // Load the ground
  dart::dynamics::SkeletonPtr ground
      = loader.parseSkeleton("dart://sample/urdf/KR5/ground.urdf");
  world->addSkeleton(ground);

  // Rotate and move the ground so that z is upwards
  Eigen::Isometry3d ground_tf
      = ground->getJoint(0)->getTransformFromParentBodyNode();
  ground_tf.pretranslate(Eigen::Vector3d(0, 0, 0.5));
  ground_tf.rotate(
      Eigen::AngleAxisd(constantsd::pi() / 2, Eigen::Vector3d(1, 0, 0)));
  ground->getJoint(0)->setTransformFromParentBodyNode(ground_tf);

  // Create an instance of our customized WorldNode
  ::osg::ref_ptr<OperationalSpaceControlWorld> node
      = new OperationalSpaceControlWorld(world);
  node->setNumStepsPerCycle(10);

  // Create the Viewer instance
  dart::gui::osg::Viewer viewer;
  viewer.addWorldNode(node);
  viewer.simulate(true);

  // Print out instructions
  std::cout << viewer.getInstructions() << std::endl;

  // Set up the window to be 640x480 pixels
  viewer.setUpViewInWindow(0, 0, 640, 480);

  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(2.57, 3.14, 1.64),
      ::osg::Vec3(0.00, 0.00, 0.00),
      ::osg::Vec3(-0.24, -0.25, 0.94));
  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Begin the application loop
  viewer.run();
}


// #include <iostream>
// #include <Eigen/Dense>

// int main() {
//     // Define 6x6 matrix and 6D vector
//     Eigen::Matrix<double, 6, 6> matrix6d;
//     Eigen::Matrix<double, 6, 1> vector6d;
    
//     // Initialize the matrix with random values
//     matrix6d.setRandom();
    
//     // Initialize the vector with random values
//     vector6d.setRandom();

//     vector6d = vector6d * 0.0;
//     vector6d[0] = 1.0;

//     // Perform matrix-vector multiplication
//     Eigen::Matrix<double, 6, 1> result = matrix6d * vector6d;

//     // Print results
//     std::cout << "Matrix6d:\n" << matrix6d << "\n\n";
//     std::cout << "Vector6d:\n" << vector6d << "\n\n";
//     std::cout << "Result (Matrix6d * Vector6d):\n" << result << "\n";

//     return 0;
// }
