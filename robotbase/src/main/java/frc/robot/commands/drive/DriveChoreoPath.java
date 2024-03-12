package frc.robot.commands.drive;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CHOREO;
import frc.robot.Robot;

public class DriveChoreoPath extends SequentialCommandGroup {

  private String m_pathName; // The name of the path file
  private ChoreoTrajectory m_traj; // The generated trajectory object
  private Pose2d m_finalTargetPose; // The ending pose of the robot
  private ChoreoTrajectoryState m_startingState; // The starting state of the robot

  /**
   * A utility command to run a Choreo path correctly.
   *
   * @param trajectoryFileName The name of the path file with '.traj' excluded.
   */
  public DriveChoreoPath(String trajectoryFileName) {
    // Overloaded constructor, sets the gyro yaw to zero and pose x, y to starting
    // position
    this(trajectoryFileName, trajectoryFileName, false, false);
  }

  /**
   * @param trajectoryFileName The name of the path file with '.traj' excluded.
   * @param pathName The name of the path, is returned in the toString for the auto command chooser.
   */
  public DriveChoreoPath(String trajectoryFileName, String pathName) {
    // Overloaded constructor, sets the gyro yaw to zero and pose x, y to starting
    // position
    this(trajectoryFileName, pathName, false, false);
  }

  /**
   * @param trajectoryFileName The name of the path file with '.traj' excluded.
   * @param pathName The name of the path, is returned in the toString for the auto command chooser.
   */
  public DriveChoreoPath(String trajectoryFileName, boolean setPoseToStartTrajectory) {
    // Overloaded constructor, sets the gyro yaw to zero and pose x, y to starting
    // position
    this(trajectoryFileName, trajectoryFileName, setPoseToStartTrajectory, false);
  }

  /**
   * A utility command to run a Choreo path correctly.
   *
   * @param trajectoryFileName The name of the path file with '.traj' excluded.
   * @param pathName The name of the path, is returned in the toString for the auto command chooser.
   * @param setPoseToStartTrajectory Whether or not to set the robot pose to the paths starting
   *     trajectory.
   */
  public DriveChoreoPath(
      String trajectoryFileName,
      String pathName,
      boolean setPoseToStartTrajectory,
      boolean setStartRotation) {
    m_traj = Choreo.getTrajectory(trajectoryFileName); // Loads choreo file into trajctory object
    m_finalTargetPose = m_traj.getFinalPose(); // Gets the last pose out of the trajectory
    m_pathName =
        pathName; // Gets the path name to display on the smardashboard via the toString() method
    m_startingState = m_traj.getInitialState(); // Gets the starting pose out of the trajectory
    // A sequential command group to run a single choreo path

    // Determine if the starting pose is flipped or not.
    // The starting pose is flipped if we are on the red alliance
    addCommands(
        new InstantCommand(
            () -> {
              m_startingState =
                  CHOREO.CHOREO_AUTO_MIRROR_PATHS.getAsBoolean()
                      ? m_startingState.flipped()
                      : m_startingState;
            }));

    // Set the gyro yaw to 0 and the pose x, y to the starting position of the path
    if (setPoseToStartTrajectory) {
      addCommands(
          new InstantCommand(() -> Robot.swerve.zeroAll()), // Zero the gyro and reset odometry
          new InstantCommand(
              () ->
                  Robot.swerve.setPose(
                      m_startingState
                          .getPose()))); // Zero the gyro and set pose odomety to x, y of starting
      // path
    }

    // Set the gyro yaw, pose rotation, pose x, and pose y to the position of the
    // starting path
    if (setStartRotation) {
      addCommands(
          new InstantCommand(() -> Robot.swerve.zeroAll()), // Zero the gyro and reset the odometry
          // Zero the gyro and set the pose rotation, x, y all off start pose
          new InstantCommand(() -> Robot.swerve.setPoseAndRotation(m_startingState.getPose())));
    }

    addCommands(
        // Set the drive velocity x, y and angular velocity to the starting state's
        // number
        // This should help the wheels "straighten" up before starting the path
        new InstantCommand(
            () ->
                Robot.swerve.driveFieldRelative(
                    m_startingState.velocityX,
                    m_startingState.velocityY,
                    m_startingState.angularVelocity)),
        // The library provided choreo command
        // Runs the actual path
        Choreo.choreoSwerveCommand(
            m_traj,
            Robot.swerve.getPoseSupplier(),
            Choreo.choreoSwerveController(
                CHOREO.X_CONTROLLER, CHOREO.Y_CONTROLLER, CHOREO.ROTATION_CONTROLLER),
            Robot.swerve.getChassisSpeedsConsumer(),
            CHOREO.CHOREO_AUTO_MIRROR_PATHS,
            Robot.swerve));
  }

  @Override
  public String toString() {
    return m_pathName;
  }
}
