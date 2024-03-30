package frc.robot.commands.drive;

import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class CalculateWheelDiameter extends Command {
  public static final double SWERVE_WHEEL_DISTANCE_FROM_CENTER = 17.151;
  public static final double SWERVE_GEARING_RATIO = 6.12;
  public static final double TARGET_ROTATIONS = 5;
  public static final double ROTATION_SPEED_RADIANS_PER_SECOND = Math.PI; // 1/2 rotation per second

  public class SwerveInfo {
    private int m_index;
    private StatusSignal<Double> m_swerveCurrentRotations;

    public SwerveInfo(int index) {
      m_index = index;
      m_swerveCurrentRotations = Robot.swerve.getModule(index).getDriveMotor().getPosition();
    }

    public void init() {
      Robot.swerve.getModule(m_index).getDriveMotor().setPosition(0.0);
    }

    public double getRotations() {
      return m_swerveCurrentRotations.getValueAsDouble();
    }
  }

  private double m_lastGyroDegrees;
  private double m_robotRotations;
  private SwerveInfo[] m_swerveInfo = {
    new SwerveInfo(0), new SwerveInfo(1), new SwerveInfo(2), new SwerveInfo(3),
  };

  public CalculateWheelDiameter() {
    addRequirements(Robot.swerve);
  }

  @Override
  public void initialize() {
    m_robotRotations = 0;
    m_lastGyroDegrees = Robot.swerve.getYaw();

    for (int i = 0; i < 4; i++) {
      m_swerveInfo[i].init();
    }
  }

  @Override
  public void execute() {
    double newGyroDegrees = Robot.swerve.getYaw();
    double degreesRotated = newGyroDegrees - m_lastGyroDegrees;
    m_robotRotations += (degreesRotated / 360);
    m_lastGyroDegrees = newGyroDegrees;

    Robot.swerve.driveRobotRelative(0, 0, ROTATION_SPEED_RADIANS_PER_SECOND);
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      System.out.println("[CalculateWheelDiameter] interrupted!");
      return;
    }

    System.out.println("-----------------------------------------------------");
    double travelCircleDiameterInches = (SWERVE_WHEEL_DISTANCE_FROM_CENTER * 2);
    double travelCircleCircumferenceInches = Math.PI * travelCircleDiameterInches;
    double totalLinearTravelInches = travelCircleCircumferenceInches * m_robotRotations;

    for (int i = 0; i < 4; i++) {
      double wheelRotations = Math.abs(m_swerveInfo[i].getRotations()) / SWERVE_GEARING_RATIO;
      double wheelCircumference = totalLinearTravelInches / wheelRotations;
      double wheelDiameter = wheelCircumference / Math.PI;

      System.out.println("Wheel " + i + " radius = " + (wheelDiameter / 2));
    }
    System.out.println("-----------------------------------------------------");
  }

  @Override
  public boolean isFinished() {
    return m_robotRotations >= TARGET_ROTATIONS;
  }
}
