// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.controls.CodriverControls;
import frc.robot.controls.DriverControls;
import frc.robot.state.RobotState;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public static RobotState state;

  public static CommandSwerveDrivetrain drive;
  public static ShooterSubsystem shooter;
  public static ShooterPivotSubsystem pivot;
  public static IntakeSubsystem intake;
  public static ClimberSubsystem climber;

  public static DriverControls driverControls;
  public static CodriverControls codriverControls;

  public static LimelightSubsystem shooterLimelight;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    state = new RobotState();

    drive = new CommandSwerveDrivetrain(
        Constants.PHEONIX_TUNER.DRIVETRAIN_CONSTANTS,
        new SwerveModuleConstants[] {
            Constants.PHEONIX_TUNER.FRONT_LEFT_MODULE_CONSTANTS,
            Constants.PHEONIX_TUNER.FRONT_RIGHT_MODULE_CONSTANTS,
            Constants.PHEONIX_TUNER.BACK_LEFT_MODULE_CONSTANTS,
            Constants.PHEONIX_TUNER.BACK_RIGHT_MODULE_CONSTANTS
        });

    shooter = new ShooterSubsystem();
    pivot = new ShooterPivotSubsystem();
    intake = new IntakeSubsystem();
    climber = new ClimberSubsystem();

    shooterLimelight = new LimelightSubsystem(Constants.SHOOTER_LIMELIGHT.NAME);

    driverControls = new DriverControls(
        new XboxController(Constants.CONTROLLER.DRIVE_CONTROLLER_PORT),
        Constants.CONTROLLER.DRIVE_CONTROLLER_DEADBAND);
    // codriverControls =
    // new CodriverControls(
    // new XboxController(Constants.CONTROLLER.CODRIVER_CONTROLLER_PORT),
    // Constants.CONTROLLER.CODRIVE_CONTROLLER_DEADBAND);

    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
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
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
