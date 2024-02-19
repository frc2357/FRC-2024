// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ExtensionArmTuningSubsystem;
import frc.robot.subsystems.IntakeTuningSubsystem;
import frc.robot.subsystems.ShooterPivotTuningSubsystem;
import frc.robot.subsystems.ShooterTuningSubsystem;

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
  private XboxController m_controller;

  private static IntakeTuningSubsystem intake;
  private static ShooterTuningSubsystem shooter;
  private static ShooterPivotTuningSubsystem pivot;
  private static ExtensionArmTuningSubsystem arm;

  private JoystickButton m_rightBumper;
  private JoystickButton m_leftBumper;

  private boolean m_intakeInverted = false;
  private boolean m_shooterInverted = false;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_controller = new XboxController(0);

    // intake = new IntakeTuningSubsystem();
    // shooter = new ShooterTuningSubsystem();
    // pivot = new ShooterPivotTuningSubsystem();
    arm = new ExtensionArmTuningSubsystem();

    m_rightBumper = new JoystickButton(m_controller, Button.kRightBumper.value);
    m_leftBumper = new JoystickButton(m_controller, Button.kLeftBumper.value);
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
    arm.update();
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
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // intake.update();
    // shooter.update();
    // pivot.update();
    arm.teleopPeriodic();
  }

  @Override
  public void testInit() {
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // double leftTrigger = m_controller.getLeftTriggerAxis();
    // intake.axisRun(leftTrigger, leftTrigger, m_intakeInverted);

    // double rightTrigger = m_controller.getRightTriggerAxis();
    // shooter.axisRun(rightTrigger, rightTrigger, m_shooterInverted);

    // double rightJoystick = m_controller.getRightY();
    // pivot.axisRun(rightJoystick);

    // m_rightBumper.onTrue(new InstantCommand(() -> m_shooterInverted =
    // !m_shooterInverted));
    // m_leftBumper.onTrue(new InstantCommand(() -> m_intakeInverted =
    // !m_intakeInverted));

    double rightJoystick = m_controller.getRightY();
    arm.axisRun(rightJoystick);

    if(m_leftBumper.getAsBoolean()) {
      arm.resetEncoders();
    }
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
