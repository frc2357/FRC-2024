package frc.robot.commands.drive;

import java.util.HashMap;
import java.util.Map;
import java.util.MissingFormatArgumentException;
import java.util.function.BiConsumer;

import com.reduxrobotics.canand.CanandDeviceDetails.Msg;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoLoop;
import choreo.auto.AutoFactory.AutoBindings;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.trajectory.TrajectorySample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CHOREO;
import frc.robot.Robot;

public class DriveChoreoPath extends SequentialCommandGroup {

  private String m_pathName; // The name of the path file
  private Trajectory<SwerveSample> m_traj; // The generated trajectory object
  private TrajectorySample<SwerveSample> m_startingState; // The starting state of the robot
  private Pose2d m_startingPose; // The starting pose of the robot
  private boolean m_targetLock; // Whether or not this specific path should target lock or not
  private AutoFactory m_autoFactory =
   Choreo.createAutoFactory(Robot.swerve, 
   () -> Robot.swerve.getPose(), 
   (Pose2d pose, SwerveSample sample) -> choreoController(pose, sample), 
   CHOREO.CHOREO_AUTO_MIRROR_PATHS, new AutoBindings()); // The choreo auto factory to do all the stuff

  private void choreoController(Pose2d curPose, SwerveSample sample){
    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        new ChassisSpeeds(
            CHOREO.X_CONTROLLER.calculate(curPose.getX(), sample.x) + sample.vx,
            CHOREO.Y_CONTROLLER.calculate(curPose.getY(), sample.y) + sample.vy,
            CHOREO.ROTATION_CONTROLLER.calculate(curPose.getRotation().getRadians(), sample.heading) + sample.omega
        ), curPose.getRotation());
    if(m_targetLock){
      Robot.swerve.autonDriveWithTargetLock(speeds);
    }else{
      Robot.swerve.autonDriveWithFeedForward(speeds);
    }
  }
  /**
   * A utility command to run a Choreo path without using the Trigger API.
   *
   * @param trajectoryFileName The name of the path file with '.traj' excluded.
   */
  public DriveChoreoPath(String trajectoryFileName) {
    // Overloaded constructor, sets the gyro yaw to zero and pose x, y to starting
    // position
    this(trajectoryFileName, false);
    m_autoFactory.trajectoryCommand(trajectoryFileName);
  }

  /**
   * A utility command to run a Choreo path without using the Trigger API.
   * @param trajectoryFileName The name of the path file with '.traj' excluded.
   * @param pathName The name of the path, is returned in the toString for the auto command chooser.
   */
  public DriveChoreoPath(String trajectoryFileName, boolean setPoseToStartTrajectory) {
    // Overloaded constructor, sets the gyro yaw to zero and pose x, y to starting
    // position
    this(trajectoryFileName, setPoseToStartTrajectory, false);
  }

  /**
   * A utility command to run a Choreo path without using the Trigger API.
   *
   * @param trajectoryFileName The name of the path file with '.traj' excluded.
   * @param setPoseToStartTrajectory Whether or not to set the robot pose to the paths starting
   *     trajectory.
   */
  public DriveChoreoPath(
      String trajectoryFileName, boolean setPoseToStartTrajectory, boolean targetLock) {
    m_targetLock = targetLock;
    m_traj = ((Trajectory<SwerveSample>) Choreo.loadTrajectory(trajectoryFileName).orElseThrow()); // Loads choreo file into trajctory object
    m_pathName = trajectoryFileName;
    addCommands(
        new InstantCommand(
          () -> {
            m_startingState = CHOREO.CHOREO_AUTO_MIRROR_PATHS.getAsBoolean()
            ? m_traj.getInitialSample().flipped() // The starting stuff is flipped if we are on the red alliance
            : m_traj.getInitialSample();
            m_startingPose = m_startingState.getPose();
          }
        ));

    // Set the gyro yaw to 0 and the pose x, y to the starting position of the path
    if (setPoseToStartTrajectory) {
      addCommands(new InstantCommand(() -> Robot.swerve.setPose(m_startingPose)));
    }

    addCommands(
        // Set the drive velocity x, y and angular velocity to the starting state's
        // number
        // This should help the wheels "straighten" up before starting the path
        new InstantCommand(
            () ->
                Robot.swerve.driveFieldRelative(
                    m_startingState.getChassisSpeeds().vxMetersPerSecond,
                    m_startingState.getChassisSpeeds().vyMetersPerSecond,
                    m_startingState.getChassisSpeeds().omegaRadiansPerSecond)),
        // The library provided choreo command
        // Runs the actual path
        new InstantCommand(
            () -> System.out.println("[DriveChoreoPath] RUNNING PATH: " + m_pathName)));
        m_autoFactory.trajectory(m_traj, m_autoFactory.newLoop(trajectoryFileName)).cmd();//this should run it in a
        //compatible way, so we can use the trigger API, and not use the trigger API.
  }



  @Override
  public String toString() {
    return m_pathName;
  }
}
