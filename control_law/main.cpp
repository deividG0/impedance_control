#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <iostream>
#include <Eigen/Dense>

int main() {
    // Initialize the DART world
    auto world = dart::simulation::World::create();

    // Set gravity for the world
    world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));

    // Load the ground plane URDF
    const std::string groundUrdfPath = "dart://sample/urdf/KR5/ground.urdf"; // Replace with the actual path to your ground URDF file
    auto ground = dart::utils::DartLoader().parseSkeleton(groundUrdfPath);
    if (!ground) {
        std::cerr << "Failed to load ground URDF file!" << std::endl;
        return -1;
    }
    world->addSkeleton(ground);

    std::cout << "Ground added to the world!" << std::endl;

    // Load URDF file for the robot
    const std::string urdfPath = "dart://sample/urdf/omp/open_manipulator_pro.urdf";
    dart::utils::DartLoader urdfLoader;
    auto robot = urdfLoader.parseSkeleton(urdfPath);

    if (!robot) {
        std::cerr << "Failed to load URDF file: " << urdfPath << std::endl;
        return -1;
    }

    std::cout << "Robot URDF loaded successfully!" << std::endl;

    // Add robot to the world
    world->addSkeleton(robot);

    // Select the end-effector body node
    auto endEffector = robot->getBodyNode("end_link");

    if (!endEffector) {
        std::cerr << "End effector not found!" << std::endl;
        return -1;
    }

    // Get the generalized coordinates and velocities
    Eigen::VectorXd q = robot->getPositions();
    Eigen::VectorXd dq = robot->getVelocities();

    // Gravity torques
    Eigen::VectorXd gravityTorques = robot->getGravityForces();
    std::cout << "Gravity torques:\n" << gravityTorques << std::endl;

    // Transposed Jacobian matrix in task space
    Eigen::MatrixXd jacobian = endEffector->getJacobian();
    Eigen::MatrixXd jacobianTranspose = jacobian.transpose();
    std::cout << "Jacobian transpose:\n" << jacobianTranspose << std::endl;

    // Inertia matrix in task space
    Eigen::MatrixXd massMatrix = robot->getMassMatrix();
    Eigen::MatrixXd taskSpaceInertia = jacobian * massMatrix * jacobianTranspose;
    std::cout << "Task space inertia matrix:\n" << taskSpaceInertia << std::endl;

    // Coriolis matrix in task space
    Eigen::VectorXd coriolisForces = robot->getCoriolisForces();
    std::cout << "Coriolis forces:\n" << coriolisForces << std::endl;

    Eigen::MatrixXd taskSpaceCoriolis = jacobian * coriolisForces.asDiagonal();
    std::cout << "Task space Coriolis matrix:\n" << taskSpaceCoriolis << std::endl;


    // Define damping and stiffness matrices
    Eigen::MatrixXd dampingMatrix = Eigen::Matrix6d::Identity() * 10.0; // Example: 10.0 as damping coefficient
    Eigen::MatrixXd stiffnessMatrix = Eigen::Matrix6d::Identity() * 50.0; // Example: 50.0 as stiffness coefficient

    std::cout << "Damping matrix:\n" << dampingMatrix << std::endl;
    std::cout << "Stiffness matrix:\n" << stiffnessMatrix << std::endl;

    // Define damping and stiffness matrices
    Eigen::MatrixXd accelerations = Eigen::Matrix6d::Identity() * 0.0;
    Eigen::MatrixXd velocities = Eigen::Matrix6d::Identity() * 0.0;
    Eigen::MatrixXd positions = Eigen::Matrix6d::Identity() * 0.3;

    std::cout << "accelerations matrix:\n" << accelerations << std::endl;
    std::cout << "velocities matrix:\n" << velocities << std::endl;
    std::cout << "positions matrix:\n" << positions << std::endl;

    std::cout << "Iteration loop:\n" << std::endl;
    for (size_t i = 0; i < 10; ++i) {
        Eigen::MatrixXd cartesianPart = taskSpaceInertia * accelerations + taskSpaceCoriolis + (dampingMatrix * velocities + stiffnessMatrix * positions);
        std::cout << "cartesianPart matrix:\n" << cartesianPart << std::endl;
        Eigen::VectorXd mForces = gravityTorques + (jacobianTranspose * cartesianPart).diagonal();
        std::cout << "Iteration " << i + 1 << " - mForces:\n" << mForces << std::endl;

        positions -= positions * static_cast<double>(i) / 10.0;
        std::cout << "Iteration " << i + 1 << " - positions:\n" << positions << std::endl;
    }

    return 0;
}