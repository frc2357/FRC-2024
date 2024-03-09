package frc.robot.commands.auto;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CHOREO;
import frc.robot.Robot;

public class TestAuto extends SequentialCommandGroup {
  private String m_pathName;
  private ChoreoTrajectory m_traj;
  private Pose2d m_finalTargetPose;
  private ChoreoTrajectoryState m_startingState;

  /**
   * A utility command to run a Choreo path correctly.
   *
   * @param trajectoryFileName       The name of the path file with '.traj'
   *                                 excluded.
   * @param pathName                 The name of the path, is returned in the
   *                                 toString for the auto command chooser.
   * @param setPoseToStartTrajectory Whether or not to set the robot pose to the
   *                                 paths starting
   *                                 trajectory.
   */
  public TestAuto(
      String trajectoryFileName, String pathName, boolean setPoseToStartTrajectory, boolean setStartRotation) {
    m_traj = Choreo.getTrajectory(trajectoryFileName);
    m_finalTargetPose = m_traj.getFinalPose();
    m_pathName = pathName;
    m_startingState = m_traj.getInitialState();

    addCommands(
        new InstantCommand(
            () -> {
              m_startingState = CHOREO.CHOREO_AUTO_MIRROR_PATHS.getAsBoolean()
                  ? m_startingState.flipped()
                  : m_startingState;
            }));
    if (setPoseToStartTrajectory) {
      addCommands(
          new InstantCommand(() -> Robot.swerve.zeroAll()),
          new InstantCommand(() -> Robot.swerve.setPose(m_startingState.getPose())));
    }
    if (setStartRotation) {
      addCommands(
          new InstantCommand(() -> Robot.swerve.zeroAll()),
          new InstantCommand(() -> Robot.swerve.setPoseAndRotation(m_startingState.getPose())));
    }

    // delete
    addCommands(new InstantCommand(() -> System.out.println(Robot.swerve.getPose())));
    addCommands(
        // Set the drive velocity x, y and angular velocity to the starting state's
        // number
        // This should help the wheels "straighten" up before starting the path
        new InstantCommand(
            () -> Robot.swerve.driveFieldRelative(
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
            Robot.swerve),

        // Print out the difference between the robot ending pose and the current pose
        new InstantCommand(
            () -> {
              var pose = Robot.swerve.getPose();
              var poseError = pose.minus(m_finalTargetPose);
              System.out.println("Pose & Error | PathName: " + trajectoryFileName);
              System.out.println("|X: " + pose.getX() + " | Err: " + poseError.getX());
              System.out.println("|Y: " + pose.getY() + " | Err: " + poseError.getY());
              System.out.println(
                  "|Roto: "
                      + pose.getRotation().getRadians()
                      + " | Err: "
                      + poseError.getRotation().getRadians());
            }));

  }

}
