package frc.robot.commands.drive;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
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
  private int m_splitIndex;
  private final AutoFactory m_autoFactory =
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
   * A utility command to start a Choreo path without using the Trigger API.
   *
   * @param trajectoryFileName The name of the path file with '.traj' excluded.
   */
  public DriveChoreoPath(String trajectoryFileName) {
    // Overloaded constructor, sets the gyro yaw to zero and pose x, y to starting
    // position
    this(trajectoryFileName, 0);
  }

  /**
   * A utility command to start a Choreo path without using the Trigger API.
   * @param trajectoryFileName The name of the path file with '.traj' excluded.
   */
  public DriveChoreoPath(String trajectoryFileName, int splitIndex) {
    // Overloaded constructor, sets the gyro yaw to zero and pose x, y to starting
    // position
    this(trajectoryFileName, splitIndex, false);
  }

  /**
   * A utility command to start a Choreo path without using the Trigger API.
   * @param trajectoryFileName The name of the path file with '.traj' excluded.
   */
  public DriveChoreoPath(String trajectoryFileName, int splitIndex, boolean setPoseToStartTrajectory) {
    // Overloaded constructor, sets the gyro yaw to zero and pose x, y to starting
    // position
    this(trajectoryFileName, 0, setPoseToStartTrajectory, false);
  }

  /**
   * A utility command to start a Choreo path without using the Trigger API.<p>
   * You still need to use {@link #noTriggers()} or {@link #withTriggers()} to actually run the path, due to
   * how we handle Choreo
   *
   * @param trajectoryFileName The name of the path file with '.traj' excluded.
   * @param splitIndex The split of the path to use. Must be at least 0.
   * @param setPoseToStartTrajectory Whether or not to set the robot pose to the paths starting
   *     trajectory.
   */
  @SuppressWarnings("unchecked")
  public DriveChoreoPath(
      String trajectoryFileName, int splitIndex, boolean setPoseToStartTrajectory, boolean targetLock) {
    m_targetLock = targetLock;
    m_splitIndex = splitIndex;
    m_pathName = trajectoryFileName.split(".")[0];// removes anything that could messup the code.
    m_traj = ((Trajectory<SwerveSample>) Choreo.loadTrajectory(trajectoryFileName).orElseThrow()); // Loads choreo file into trajctory object
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
                    m_startingState.getChassisSpeeds().omegaRadiansPerSecond))
    );
  }

  /**
   * This runs the path with no triggers, and should allow use of the triggers API in other sections of the path.
   */
  public void noTriggers(){
    addCommands( 
      new InstantCommand(
        () -> System.out.println("[DriveChoreoPath] RUNNING PATH: " + m_pathName)),
        m_autoFactory.trajectory(m_pathName, m_splitIndex, m_autoFactory.voidLoop()).cmd()
    );//this should run it in a
  //compatible way, so we can choose to use the trigger API for some sections, and not for others in a single path.
  }

  public AutoTrajectory withTriggers(){
    addCommands(new InstantCommand(
        () -> System.out.println("[DriveChoreoPath] RUNNING PATH: " + m_pathName)));
    return m_autoFactory.trajectory(m_pathName, m_splitIndex, m_autoFactory.newLoop(m_pathName));
  }

  @Override
  public String toString() {
    return m_pathName;
  }
}
