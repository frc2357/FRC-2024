package frc.robot.controls;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.commands.auto.AutoShoot;
import frc.robot.commands.climber.ClimberAxis;
import frc.robot.commands.climber.ClimberRunThenZero;
import frc.robot.commands.endAffector.EndAffectorAxis;
import frc.robot.commands.extensionArm.ExtensionArmAxis;
import frc.robot.commands.extensionArm.ExtensionArmZero;
import frc.robot.commands.intake.IntakeAxis;
import frc.robot.commands.pivot.PivotAxis;
import frc.robot.commands.shooter.ShooterStepAxis;
import frc.robot.controls.util.AxisInterface;
import frc.robot.controls.util.AxisThresholdTrigger;
import frc.robot.controls.util.RumbleInterface;
import frc.robot.util.Utility;

public class CodriverControls implements RumbleInterface {

  private XboxController m_controller;
  private double m_deadband;

  // Triggers
  public AxisThresholdTrigger m_leftTrigger;
  public AxisThresholdTrigger m_rightTrigger;

  // Buttons
  public JoystickButton m_leftStickButton;
  public JoystickButton m_backButton;
  public JoystickButton m_startButton;
  public JoystickButton m_leftBumper;
  public JoystickButton m_rightBumper;
  public Trigger m_aButton;
  public Trigger m_bButton;
  public Trigger m_xButton;
  public Trigger m_yButton;

  // Dpad
  public POVButton m_upDPad;
  public POVButton m_rightDPad;
  public POVButton m_downDPad;
  public POVButton m_leftDPad;

  private AxisThresholdTrigger m_leftTriggerPre;
  private AxisThresholdTrigger m_leftTriggerFull;
  private AxisThresholdTrigger m_rightTriggerPre;
  private AxisThresholdTrigger m_rightTriggerFull;

  public CodriverControls(XboxController controller, double deadband) {
    m_controller = controller;
    m_deadband = deadband;

    m_rightTriggerPre = new AxisThresholdTrigger(m_controller, Axis.kRightTrigger, 0.05);
    m_rightTriggerFull = new AxisThresholdTrigger(m_controller, Axis.kRightTrigger, 0.75);

    m_leftTriggerPre = new AxisThresholdTrigger(m_controller, Axis.kLeftTrigger, 0.05);
    m_leftTriggerFull = new AxisThresholdTrigger(m_controller, Axis.kLeftTrigger, 0.75);

    // Triggers
    m_leftTrigger = new AxisThresholdTrigger(controller, Axis.kLeftTrigger, .1);
    m_rightTrigger = new AxisThresholdTrigger(controller, Axis.kRightTrigger, .1);

    // Buttons
    m_leftStickButton = new JoystickButton(controller, Button.kLeftStick.value);
    m_backButton = new JoystickButton(controller, Button.kBack.value);
    m_startButton = new JoystickButton(controller, Button.kStart.value);
    m_leftBumper = new JoystickButton(controller, Button.kLeftBumper.value);
    m_rightBumper = new JoystickButton(controller, Button.kRightBumper.value);
    m_aButton = new JoystickButton(controller, Button.kA.value);
    m_bButton = new JoystickButton(controller, Button.kB.value);
    m_xButton = new JoystickButton(controller, Button.kX.value);
    m_yButton = new JoystickButton(controller, Button.kY.value);

    // Dpad
    m_upDPad = new POVButton(controller, 0);
    m_rightDPad = new POVButton(controller, 90);
    m_downDPad = new POVButton(controller, 180);
    m_leftDPad = new POVButton(controller, 270);

    mapControls();
  }

  public double getRightXAxis() {
    double value = m_controller.getRightX();
    return Utility.deadband(value, m_deadband);
  }

  public double getRightYAxis() {
    double value = m_controller.getRightY();
    return Utility.deadband(value, m_deadband);
  }

  public double getRightTriggerAxis() {
    return m_controller.getRightTriggerAxis();
  }

  public double getLeftTriggerAxis() {
    return m_controller.getLeftTriggerAxis();
  }

  public double getLeftStickY() {
    return m_controller.getLeftY();
  }

  private void mapControls() {

    AxisInterface axisRightStickX =
        () -> {
          return getRightXAxis();
        };
    AxisInterface axisRightStickY =
        () -> {
          return getRightYAxis();
        };

    AxisInterface subsystemRollerForwardAxis =
        () -> {
          return getRightTriggerAxis();
        };

    AxisInterface subsystemRollerReverseAxis =
        () -> {
          return -getLeftTriggerAxis();
        };

    Trigger noDPad =
        new Trigger(
                () ->
                    m_upDPad.getAsBoolean()
                        || m_rightDPad.getAsBoolean()
                        || m_downDPad.getAsBoolean()
                        || m_leftDPad.getAsBoolean())
            .negate();

    Trigger rightTriggerPreNoDPad = noDPad.and(m_rightTriggerPre);
    Trigger rightTriggerFullNoDPad = noDPad.and(m_rightTriggerFull);
    Trigger leftTriggerPreNoDPad = noDPad.and(m_leftTriggerPre);
    Trigger leftTriggerFullNoDPad = noDPad.and(m_leftTriggerFull);

    Trigger noLetterButtons = m_aButton.or(m_bButton).or(m_xButton).or(m_yButton).negate();
    Trigger upDPadOnly = m_upDPad.and(noLetterButtons);
    Trigger downDPadOnly = m_downDPad.and(noLetterButtons);
    Trigger leftDPadOnly = m_leftDPad.and(noLetterButtons);
    Trigger rightDPadOnly = m_rightDPad.and(noLetterButtons);

    Trigger upDPadAndA = m_upDPad.and(m_aButton);

    Trigger upDPadAndX = m_upDPad.and(m_xButton);
    Trigger upDPadAndY = m_upDPad.and(m_yButton);
    Trigger upDPadAndB = m_upDPad.and(m_bButton);

    Trigger downDPadAndA = m_downDPad.and(m_aButton);
    Trigger downDPadAndX = m_downDPad.and(m_xButton);
    Trigger downDPadAndY = m_downDPad.and(m_yButton);
    Trigger downDPadAndB = m_downDPad.and(m_bButton);
    Trigger downDPadAndRightTrigger = m_downDPad.and(m_rightTrigger);
    Trigger downDPadAndLeftTrigger = m_downDPad.and(m_leftTrigger);

    Trigger leftDPadAndA = m_leftDPad.and(m_aButton);
    Trigger leftDPadAndX = m_leftDPad.and(m_xButton);
    Trigger leftDPadAndY = m_leftDPad.and(m_yButton);
    Trigger leftDPadAndB = m_leftDPad.and(m_bButton);
    Trigger leftDPadAndRightTrigger = m_leftDPad.and(m_rightTrigger);
    Trigger leftDPadAndLeftTrigger = m_leftDPad.and(m_leftTrigger);

    Trigger rightDPadAndA = m_rightDPad.and(m_aButton);
    Trigger rightDPadAndX = m_rightDPad.and(m_xButton);
    Trigger rightDPadAndY = m_rightDPad.and(m_yButton);
    Trigger rightDPadAndB = m_rightDPad.and(m_bButton);
    Trigger rightDPadAndRightTrigger = m_rightDPad.and(m_rightTrigger);
    Trigger rightDPadAndLeftTrigger = m_rightDPad.and(m_leftTrigger);

    Trigger aButton = m_aButton.and(noDPad);
    Trigger bButton = m_bButton.and(noDPad);
    Trigger yButton = m_yButton.and(noDPad);
    Trigger xButton = m_xButton.and(noDPad);

    Trigger bothBumpers = m_rightBumper.and(m_leftBumper);

    // Alliance selection
    m_rightBumper.onTrue(new InstantCommand(() -> Robot.state.setAlliance(Alliance.Red)));
    m_leftBumper.onTrue(new InstantCommand(() -> Robot.state.setAlliance(Alliance.Blue)));
    bothBumpers.onTrue(new InstantCommand(() -> Robot.state.setAlliance(null)));

    // Shooter - Right DPad
    rightDPadAndRightTrigger.whileTrue(
        new ShooterStepAxis(
            subsystemRollerForwardAxis, Constants.SHOOTER.SHOOTER_AXIS_STEP_INTERVAL));
    rightDPadAndLeftTrigger.whileTrue(
        new ShooterStepAxis(
            subsystemRollerReverseAxis, Constants.SHOOTER.SHOOTER_AXIS_STEP_INTERVAL));

    rightDPadOnly.whileTrue(new PivotAxis(axisRightStickY));
    rightDPadAndY.onTrue(
        new InstantCommand(
            () -> {
              Robot.pivot.setZero();
            }));

    // Intake - Left DPad
    leftDPadAndRightTrigger.whileTrue(new IntakeAxis(subsystemRollerForwardAxis));
    leftDPadAndLeftTrigger.whileTrue(new IntakeAxis(subsystemRollerReverseAxis));

    // Climber - Up DPad
    upDPadAndY.onTrue(
        new InstantCommand(
            () -> {
              Robot.climber.setZero();
            }));
    upDPadAndB.whileTrue(new ClimberRunThenZero());
    upDPadOnly.whileTrue(new ClimberAxis(axisRightStickX, axisRightStickY));

    // Extension/EndAffector - Down DPad
    downDPadOnly.whileTrue(new ExtensionArmAxis(axisRightStickY));
    downDPadAndY.onTrue(
        new InstantCommand(
            () -> {
              Robot.extensionArm.setZero();
            }));
    downDPadAndB.whileTrue(new ExtensionArmZero());

    downDPadAndRightTrigger.whileTrue(new EndAffectorAxis(subsystemRollerForwardAxis));
    downDPadAndLeftTrigger.whileTrue(new EndAffectorAxis(subsystemRollerReverseAxis));

    downDPadAndA.whileTrue(new AutoShoot());

    // Climber - up
    // Intake - left
    // Shooter - right
    // end affector - down

    // Cancel all commands - back (the left one)
    m_backButton.onTrue(
        new InstantCommand(
            () -> {
              CommandScheduler.getInstance().cancelAll();
            }));

    // // Uncomment for tuning wheel diameter
    // m_startButton.onTrue(new CalculateWheelDiameter());
  }

  @Override
  public void setRumble(double intensity) {
    m_controller.setRumble(RumbleType.kBothRumble, intensity);
  }
}
