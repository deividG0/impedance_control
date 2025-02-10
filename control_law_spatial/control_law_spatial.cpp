#include <dart/gui/osg/osg.hpp>

#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

#include <dart/dart.hpp>

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
    mRobot = mWorld->getSkeleton(0);
    mEndEffector = mRobot->getBodyNode(mRobot->getNumBodyNodes() - 1);

    // Setup gain matrices
    // std::size_t dofs = mEndEffector->getNumDependentGenCoords();
    std::size_t dofs = 6;
    stiffnessMatrix.setZero(dofs, dofs);
    for (std::size_t i = 0; i < dofs; ++i)
      stiffnessMatrix(i, i) = 0.5;

    dampingMatrix.setZero(dofs, dofs);
    for (std::size_t i = 0; i < dofs; ++i)
      dampingMatrix(i, i) = 0.5;

    // Set joint properties
    mRobot->eachJoint([](dart::dynamics::Joint* joint) {
      joint->setLimitEnforcement(false);
      joint->setDampingCoefficient(0, 0.5);
    });

    mOffset = Eigen::Vector3d(0, 0, 0);

    // Create target Frame
    Eigen::Isometry3d tf = mEndEffector->getWorldTransform();
    tf.pretranslate(mOffset);
    mTarget = std::make_shared<SimpleFrame>(Frame::World(), "target", tf);
    ShapePtr ball(new SphereShape(0.025));
    mTarget->setShape(ball);
    mTarget->getVisualAspect(true)->setColor(Eigen::Vector3d(0.9, 0, 0));
    mWorld->addSimpleFrame(mTarget);

    mOffset
        = mEndEffector->getWorldTransform().rotation().transpose() * mOffset;
  }

  // Triggered at the beginning of each simulation step
  void customPreStep() override
  {
    Eigen::VectorXd gravityForces = mRobot->getGravityForces();
    // std::cout << "Gravity forces\n" << gravityForces << std::endl;

    Eigen::MatrixXd J = mEndEffector->getJacobian(mOffset);
    // std::cout << "Jacobian\n" << J << std::endl;

    Eigen::MatrixXd transJ = J.transpose();
    // std::cout << "Jacobian Transposed\n" << transJ << std::endl;

    Eigen::MatrixXd M = mRobot->getMassMatrix();
    // std::cout << "Mass matrix\n" << M << std::endl;

    Eigen::MatrixXd pinv_J
        = J.transpose()
          * (J * J.transpose() + 0.0025 * Eigen::Matrix6d::Identity())
                .inverse();
    // std::cout << "Pseudo Jacobian Inverse\n" << pinv_J << std::endl;

    Eigen::MatrixXd pinv_tJ
        = transJ.transpose()
          * (transJ * transJ.transpose() + 0.0025 * Eigen::Matrix6d::Identity())  // OBSERVATION
                .inverse();
    // std::cout << "Pseudo Transposed Jacobian Inverse\n" << pinv_tJ << std::endl;

    Eigen::MatrixXd taskSpaceInertia = pinv_tJ * M * pinv_J;
    // std::cout << "Inertia matrix\n" << taskSpaceInertia << std::endl;

    Eigen::MatrixXd dJ = mEndEffector->getJacobianSpatialDeriv(mOffset);
    // std::cout << "Derivate Jacobian\n" << dJ << std::endl;

    Eigen::MatrixXd C = computeCoriolisMatrix(mRobot);
    // std::cout << "Coriolis matrix\n" << C << std::endl;

    Eigen::MatrixXd taskSpaceCoriolis = pinv_tJ * (C - M * pinv_J * dJ) * pinv_J;
    // std::cout << "Coriolis matrix task space\n" << taskSpaceCoriolis << std::endl;

    Eigen::Vector6d target = Eigen::Vector6d::Zero();
    target.head(3) = mTarget->getWorldTransform().translation();
    target.tail(3) = Eigen::Vector3d(1.57, 1.57, 1.57);
    std::cout << "Target\n" << target << std::endl;

    Eigen::Vector3d eulerAngles = mEndEffector->getWorldTransform().rotation().eulerAngles(2, 1, 0);

    Eigen::Vector6d endEffector = Eigen::Vector6d::Zero();
    endEffector.head(3) = mEndEffector->getWorldTransform() * mOffset;
    // endEffector.tail(3) = eulerAngles;
    endEffector.tail(3) = Eigen::Vector3d(1.57, 1.57, 1.57);   // OBSERVATION
    std::cout << "End-effector\n" << endEffector << std::endl;

    Eigen::Vector6d e = target - endEffector;

    std::cout << "Position error\n" << e << std::endl;

    Eigen::Vector6d de = mEndEffector->getSpatialVelocity(mOffset);
    // Eigen::Vector6d dde = mEndEffector->getSpatialAcceleration(mOffset);
    Eigen::Vector6d dde = Eigen::Vector6d::Zero();
    // std::cout << "Spatial Accelaration\n" << dde << std::endl;
    // std::cout << "Spatial Velocity\n" << de << std::endl;

    // std::cout << "Inertia term\n" << taskSpaceInertia * (-1)*dde << std::endl;
    // std::cout << "Coriolis term\n" << taskSpaceCoriolis * (-1)*de << std::endl;
    // std::cout << "Damping term\n" << dampingMatrix * (-1)*de << std::endl;
    // std::cout << "Stiffness term\n" << stiffnessMatrix * e << std::endl;

    // dde = Eigen::Vector6d(0.0, 0.0, 0.0);
    Eigen::MatrixXd cartesianPart = taskSpaceInertia * (-1)*dde
                + taskSpaceCoriolis * (-1)*de
                + dampingMatrix * (-1)*de
                + stiffnessMatrix * e;

    mForces = gravityForces + transJ * cartesianPart;

    std::cout << "Forces:\n" << mForces << std::endl;
    // mForces = Eigen::Vector6d::Zero();

    mRobot->setForces(mForces);
  }

  dart::gui::osg::DragAndDrop* dnd;

protected:
  // Triggered when this node gets added to the Viewer
  void setupViewer() override
  {
    if (mViewer) {
      dnd = mViewer->enableDragAndDrop(mTarget.get());
      dnd->setObstructable(false);
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

  Eigen::MatrixXd computeCoriolisMatrix(dart::dynamics::SkeletonPtr robot) {
    const double delta = 1e-6; // Small perturbation
    int dof = robot->getNumDofs();
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(dof, dof);
    
    // Get the current state
    Eigen::VectorXd q = robot->getPositions();
    Eigen::VectorXd dq = robot->getVelocities();
    
    // Compute the mass matrix at the current position
    Eigen::MatrixXd M = robot->getMassMatrix();
    
    // Compute partial derivatives using finite differences
    for (int k = 0; k < dof; ++k) {
        Eigen::VectorXd q_plus = q, q_minus = q;
        q_plus[k] += delta;
        q_minus[k] -= delta;

        // Set perturbed positions
        robot->setPositions(q_plus);
        Eigen::MatrixXd M_plus = robot->getMassMatrix();
        
        robot->setPositions(q_minus);
        Eigen::MatrixXd M_minus = robot->getMassMatrix();
        
        // Reset the robot to the original state
        robot->setPositions(q);
        
        // Compute numerical derivative
        Eigen::MatrixXd dM_dqk = (M_plus - M_minus) / (2.0 * delta);

        // Compute Coriolis matrix using the Christoffel symbols formula
        for (int i = 0; i < dof; ++i) {
            for (int j = 0; j < dof; ++j) {
                C(i, j) += 0.5 * (dM_dqk(i, j) + dM_dqk(j, i) - dM_dqk(j, i)) * dq[k];
            }
        }
    }
    return C;
  }

  SkeletonPtr mRobot;
  BodyNode* mEndEffector;
  SimpleFramePtr mTarget;

  Eigen::Vector3d mOffset;
  Eigen::Matrix6d stiffnessMatrix;
  Eigen::Matrix6d dampingMatrix;
  Eigen::VectorXd mForces;
};

class ConstraintEventHandler : public ::osgGA::GUIEventHandler
{
public:
  ConstraintEventHandler(dart::gui::osg::DragAndDrop* dnd = nullptr) : mDnD(dnd)
  {
    clearConstraints();
    if (mDnD)
      mDnD->unconstrain();
  }

  void clearConstraints()
  {
    for (std::size_t i = 0; i < 3; ++i)
      mConstrained[i] = false;
  }

  virtual bool handle(
      const ::osgGA::GUIEventAdapter& ea, ::osgGA::GUIActionAdapter&) override
  {
    if (nullptr == mDnD) {
      clearConstraints();
      return false;
    }

    bool handled = false;
    switch (ea.getEventType()) {
      case ::osgGA::GUIEventAdapter::KEYDOWN: {
        switch (ea.getKey()) {
          case '1':
            mConstrained[0] = true;
            handled = true;
            break;
          case '2':
            mConstrained[1] = true;
            handled = true;
            break;
          case '3':
            mConstrained[2] = true;
            handled = true;
            break;
        }
        break;
      }

      case ::osgGA::GUIEventAdapter::KEYUP: {
        switch (ea.getKey()) {
          case '1':
            mConstrained[0] = false;
            handled = true;
            break;
          case '2':
            mConstrained[1] = false;
            handled = true;
            break;
          case '3':
            mConstrained[2] = false;
            handled = true;
            break;
        }
        break;
      }

      default:
        return false;
    }

    if (!handled)
      return handled;

    std::size_t constraintDofs = 0;
    for (std::size_t i = 0; i < 3; ++i)
      if (mConstrained[i])
        ++constraintDofs;

    if (constraintDofs == 0 || constraintDofs == 3) {
      mDnD->unconstrain();
    } else if (constraintDofs == 1) {
      Eigen::Vector3d v(Eigen::Vector3d::Zero());
      for (std::size_t i = 0; i < 3; ++i)
        if (mConstrained[i])
          v[i] = 1.0;

      mDnD->constrainToLine(v);
    } else if (constraintDofs == 2) {
      Eigen::Vector3d v(Eigen::Vector3d::Zero());
      for (std::size_t i = 0; i < 3; ++i)
        if (!mConstrained[i])
          v[i] = 1.0;

      mDnD->constrainToPlane(v);
    }

    return handled;
  }

  bool mConstrained[3];

  dart::sub_ptr<dart::gui::osg::DragAndDrop> mDnD;
};

class ShadowEventHandler : public osgGA::GUIEventHandler
{
public:
  ShadowEventHandler(
      OperationalSpaceControlWorld* node, dart::gui::osg::Viewer* viewer)
    : mNode(node), mViewer(viewer)
  {
  }

  bool handle(
      const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) override
  {
    if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN) {
      if (ea.getKey() == 's' || ea.getKey() == 'S') {
        if (mNode->isShadowed())
          mNode->setShadowTechnique(nullptr);
        else
          mNode->setShadowTechnique(
              dart::gui::osg::WorldNode::createDefaultShadowTechnique(mViewer));
        return true;
      }
    }

    // The return value should be 'true' if the input has been fully handled
    // and should not be visible to any remaining event handlers. It should be
    // false if the input has not been fully handled and should be viewed by
    // any remaining event handlers.
    return false;
  }

protected:
  OperationalSpaceControlWorld* mNode;
  dart::gui::osg::Viewer* mViewer;
};

int main()
{
  dart::simulation::WorldPtr world(new dart::simulation::World);
  dart::utils::DartLoader loader;

  // // Load the robot
  // dart::dynamics::SkeletonPtr robot
  //     = loader.parseSkeleton("/home/cimatec/Desktop/DART/dart/data/urdf/omp/open_manipulator_pro.urdf");

  // Load the robot
  dart::dynamics::SkeletonPtr robot
      = loader.parseSkeleton("dart://sample/urdf/KR5/KR5 sixx R650.urdf");
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
  viewer.addEventHandler(new ConstraintEventHandler(node->dnd));
  viewer.addEventHandler(new ShadowEventHandler(node.get(), &viewer));

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
