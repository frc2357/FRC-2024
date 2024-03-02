// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.drive.SetCoastOnDisable;
import frc.robot.controls.CodriverControls;
import frc.robot.controls.DriverControls;
import frc.robot.state.RobotState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.EndAffector;
import frc.robot.subsystems.ExtensionArm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakePhotonCamera;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterPhotonCamera;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command m_setCoastOnDisable;
  private RobotContainer m_robotContainer;

  public static RobotState state;

  public static CommandSwerveDrivetrain swerve;
  public static Shooter shooter;
  public static Pivot pivot;
  public static Intake intake;
  public static Climber climber;

  public static DriverControls driverControls;
  public static CodriverControls codriverControls;

  public static ShooterPhotonCamera shooterCam;
  public static IntakePhotonCamera intakeCam;

  public static EndAffector endAffector;

  public static ExtensionArm extensionArm;

  // {ty, pivotRotations, shooterRPM}
  public static final double[][] shooterCurve = {
    {50, 45, 3000}, // Lower bound
    {7.5, 60, 3000}, // Side Subwoofer
    {11.10, 60, 3000}, // Center Subwoofer
    {-8.87, 35, 4000}, // Podium
    {-15, 27, 4250}, // Stage Apriltag
    {-17.75, 25, 4250}, // Wing line
    {-19, 25, 4250} // Center line (Upper bound)
  };

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.

    shooterCam =
        new ShooterPhotonCamera(
            Constants.SHOOTER_PHOTON_CAMERA.NAME,
            Constants.SHOOTER_PHOTON_CAMERA.ROBOT_TO_CAMERA_TRANSFORM,
            Constants.SHOOTER_PHOTON_CAMERA.HEAD_ON_TOLERANCE);
    intakeCam =
        new IntakePhotonCamera(
            Constants.INTAKE_PHOTON_CAMERA.NAME,
            Constants.INTAKE_PHOTON_CAMERA.ROBOT_TO_CAMERA_TRANSFORM,
            Constants.INTAKE_PHOTON_CAMERA.HEAD_ON_TOLERANCE);

    state = new RobotState();

    // CHANGE TO FALSE TO USE CUBE BOT. DO THIS
    swerve = true ? CompSwerveTunerConstants.DriveTrain : CubeBotTunerConstants.DriveTrain;
    shooter = new Shooter();
    pivot = new Pivot();
    intake = new Intake();
    climber = new Climber();
    endAffector = new EndAffector();
    extensionArm = new ExtensionArm();

    driverControls =
        new DriverControls(
            new XboxController(Constants.CONTROLLER.DRIVE_CONTROLLER_PORT),
            Constants.CONTROLLER.DRIVE_CONTROLLER_DEADBAND);
    codriverControls =
        new CodriverControls(
            new XboxController(Constants.CONTROLLER.CODRIVER_CONTROLLER_PORT),
            Constants.CONTROLLER.CODRIVE_CONTROLLER_DEADBAND);

    m_robotContainer = new RobotContainer();

    m_setCoastOnDisable = new SetCoastOnDisable();
    m_setCoastOnDisable.schedule();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    shooterCam.updateResult();

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    SmartDashboard.putString(
        "Alliance", state.getAlliance() == null ? "None" : state.getAlliance().toString());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_setCoastOnDisable.schedule();
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_setCoastOnDisable.cancel();
    Robot.swerve.configNeutralMode(NeutralModeValue.Brake);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    m_setCoastOnDisable.cancel();
    Robot.swerve.configNeutralMode(NeutralModeValue.Brake);
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
