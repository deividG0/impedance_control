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

    // Set joint properties
    robot->eachJoint([](dart::dynamics::Joint* joint) {
      joint->setLimitEnforcement(false);
      joint->setDampingCoefficient(0, 0.5);
    });

    mOffset = Eigen::Vector3d(0.05, 0, 0);

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
    dampingMatrix = Eigen::Matrix6d::Identity() * 10.0; // Example: 10.0 as damping coefficient
    stiffnessMatrix = Eigen::Matrix6d::Identity() * 50.0; // Example: 50.0 as stiffness coefficient

    // Define damping and stiffness matrices
    accelerations = Eigen::Matrix6d::Identity() * 0.0;
    velocities = Eigen::Matrix6d::Identity() * 0.0;
    positions = Eigen::Matrix6d::Identity() * 0.3;
  }

  // Triggered at the beginning of each simulation step
  void customPreStep() override
  {
    // Eigen::MatrixXd M = mRobot->getMassMatrix();

    // LinearJacobian J = mEndEffector->getLinearJacobian(mOffset);
    // Eigen::MatrixXd pinv_J
    //     = J.transpose()
    //       * (J * J.transpose() + 0.0025 * Eigen::Matrix3d::Identity())
    //             .inverse();

    // LinearJacobian dJ = mEndEffector->getLinearJacobianDeriv(mOffset);
    // Eigen::MatrixXd pinv_dJ
    //     = dJ.transpose()
    //       * (dJ * dJ.transpose() + 0.0025 * Eigen::Matrix3d::Identity())
    //             .inverse();

    // // Eigen::Vector3d e = mTarget->getWorldTransform().translation()
    // //                     - mEndEffector->getWorldTransform() * mOffset;

    // Eigen::Vector3d de = -mEndEffector->getLinearVelocity(mOffset);

    // Eigen::VectorXd Cg = mRobot->getCoriolisAndGravityForces();

    // mForces = M * (pinv_J * mKp * de + pinv_dJ * mKp * e) + Cg
    //           + mKd * pinv_J * mKp * e;

    std::cout << "HERE1??\n";
    // Interation
    Eigen::MatrixXd cartesianPart = taskSpaceInertia * accelerations + taskSpaceCoriolis + (dampingMatrix * velocities + stiffnessMatrix * positions);
    std::cout << "HERE2??\n";

    Eigen::VectorXd mForces = gravityTorques + (jacobianTranspose * cartesianPart).diagonal();
    std::cout << "HERE2??\n";

    std::cout << "mEndEffector:\n" << mEndEffector->getWorldTransform() * mOffset << std::endl;

    // positions = positions - Eigen::Matrix6d::Identity() * 0.001;
    // positions = positions - mEndEffector->getWorldTransform() * mOffset;
    std::cout << "HERE3??\n";

    // robot->setForces(mForces);
    // robot->setForces(Eigen::Vector6d(0.05, 0, 0, 0, 0, 0));
    robot->setForces(Eigen::Vector6d(1.0, 1.0, 1.0, 1.0, 1.0, 1.0));
  }

protected:
  // Triggered when this node gets added to the Viewer
  void setupViewer() override
  {
    if (mViewer) {
    //   dnd->setObstructable(false);
      mViewer->addInstructionText(
          "\nClick and drag the red ball to move the target of the operational "
          "space controller\n");
      mViewer->addInstructionText(
          "Hold key 1 to constrain movements to the x-axis\n");
      mViewer->addInstructionText(
          "Hold key 2 to constrain movements to the y-axis\n");
      mViewer->addInstructionText(
          "Hold key 3 to constrain movements to the z-axis\n");
    }
  }

  SkeletonPtr robot;
  BodyNode* mEndEffector;

  Eigen::Vector3d mOffset;
//   Eigen::Matrix3d mKp;
//   Eigen::MatrixXd mKd;
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
  Eigen::MatrixXd accelerations;
  Eigen::MatrixXd velocities;
  Eigen::MatrixXd positions;
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

  // Add our custom event handler to the Viewer
//   viewer.addEventHandler(new ConstraintEventHandler(node->dnd));
//   viewer.addEventHandler(new ShadowEventHandler(node.get(), &viewer));

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
