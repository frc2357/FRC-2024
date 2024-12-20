package frc.robot.commands.drive;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CHOREO;
import frc.robot.Robot;

public class DriveChoreoPath extends SequentialCommandGroup {

  private String m_pathName; // The name of the path file
  private ChoreoTrajectory m_traj; // The generated trajectory object
  private ChoreoTrajectoryState m_startingState; // The starting state of the robot

  /**
   * A utility command to run a Choreo path correctly.
   *
   * @param trajectoryFileName The name of the path file with '.traj' excluded.
   */
  public DriveChoreoPath(String trajectoryFileName) {
    // Overloaded constructor, sets the gyro yaw to zero and pose x, y to starting
    // position
    this(trajectoryFileName, false);
  }

  /**
   * @param trajectoryFileName The name of the path file with '.traj' excluded.
   * @param pathName The name of the path, is returned in the toString for the auto command chooser.
   */
  public DriveChoreoPath(String trajectoryFileName, boolean setPoseToStartTrajectory) {
    // Overloaded constructor, sets the gyro yaw to zero and pose x, y to starting
    // position
    this(trajectoryFileName, setPoseToStartTrajectory, false);
  }

  /**
   * A utility command to run a Choreo path correctly.
   *
   * @param trajectoryFileName The name of the path file with '.traj' excluded.
   * @param setPoseToStartTrajectory Whether or not to set the robot pose to the paths starting
   *     trajectory.
   */
  public DriveChoreoPath(
      String trajectoryFileName, boolean setPoseToStartTrajectory, boolean targetLock) {
    m_traj = Choreo.getTrajectory(trajectoryFileName); // Loads choreo file into trajctory object
    m_pathName =
        trajectoryFileName; // Gets the path name to display on the smardashboard via the toString()
    // method
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
      addCommands(new InstantCommand(() -> Robot.swerve.setPose(m_startingState.getPose())));
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
        new InstantCommand(
            () -> System.out.println("[DriveChoreoPath] RUNNING PATH: " + m_pathName)),
        Choreo.choreoSwerveCommand(
            m_traj,
            Robot.swerve::getPose,
            Choreo.choreoSwerveController(
                CHOREO.X_CONTROLLER, CHOREO.Y_CONTROLLER, CHOREO.ROTATION_CONTROLLER),
            targetLock
                ? Robot.swerve.getSpeakerLockChassisSpeedsConsumer()
                : Robot.swerve.getChassisSpeedsConsumer(),
            CHOREO.CHOREO_AUTO_MIRROR_PATHS,
            Robot.swerve));
  }

  @Override
  public String toString() {
    return m_pathName;
  }
}
