package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Drive extends SubsystemBase {

  private TalonFX m_leftFalconMaster;
  private TalonFX m_rightFalconMaster;

  private TalonFX m_leftFalconFollower;
  private TalonFX m_rightFalconFollower;

  private SysIdRoutine.Config sysidConfig = new SysIdRoutine.Config(
      Volts.of(1).per(Seconds.of(0.5)), // Default ramp rate is acceptable
      Volts.of(10), // Reduce dynamic voltage to 4 to prevent motor brownout
      Seconds.of(5), (state) -> SignalLogger.writeString("state", state.toString()) // Default timeout is acceptable
  );

  private SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(this::voltageDrive, null, this);
  private SysIdRoutine m_sysIdRoutine = new SysIdRoutine(sysidConfig, mechanism);

  public Drive() {

    m_leftFalconMaster = new TalonFX(11, "CANivore");
    m_leftFalconFollower = new TalonFX(15, "CANivore");

    m_rightFalconMaster = new TalonFX(13, "CANivore");
    m_rightFalconFollower = new TalonFX(17, "CANivore");

    var normal = new TalonFXConfiguration();
    normal.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    normal.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    normal.CurrentLimits.StatorCurrentLimit = 60;
    normal.CurrentLimits.SupplyCurrentLimit = 60;
    normal.CurrentLimits.StatorCurrentLimitEnable = true;
    normal.CurrentLimits.SupplyCurrentLimitEnable = true;

    var inverted = new TalonFXConfiguration();
    inverted.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    inverted.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    inverted.CurrentLimits.StatorCurrentLimit = 60;
    inverted.CurrentLimits.SupplyCurrentLimit = 60;
    inverted.CurrentLimits.StatorCurrentLimitEnable = true;
    inverted.CurrentLimits.SupplyCurrentLimitEnable = true;

    m_leftFalconMaster.getConfigurator().apply(inverted);
    m_leftFalconFollower.getConfigurator().apply(normal);
    m_leftFalconFollower.setControl(new Follower(m_leftFalconMaster.getDeviceID(), true));

    m_rightFalconMaster.getConfigurator().apply(normal);
    m_rightFalconFollower.getConfigurator().apply(normal);
    m_rightFalconFollower.setControl(new Follower(m_rightFalconMaster.getDeviceID(), false));

    BaseStatusSignal.setUpdateFrequencyForAll(250,
        m_leftFalconMaster.getPosition(),
        m_leftFalconMaster.getVelocity(),
        m_leftFalconMaster.getMotorVoltage());

    m_leftFalconMaster.optimizeBusUtilization();
  }

  public void voltageDrive(Measure<Voltage> voltage) {
    m_rightFalconMaster.setVoltage(voltage.in(Volts));
    m_leftFalconMaster.setVoltage(voltage.in(Volts));
  }

  public void driveProportionalWithStick(double speed, double turn) {
    speed = -1 * MathUtil.applyDeadband(speed, 0.1);
    turn = MathUtil.applyDeadband(turn, 0.1);
    driveProportional(speed, turn);
  }

  /**
   * 
   * @param speed -1 to 1
   * @param turn  -1 to 1
   */
  public void driveProportional(double speed, double turn) {
    double leftProportion = speed + turn;
    double rightProportion = speed - turn;

    leftProportion = MathUtil.clamp(leftProportion, -1.0, 1.0);

    rightProportion = MathUtil.clamp(rightProportion, -1.0, 1.0);

    m_leftFalconMaster.set(leftProportion);
    m_rightFalconMaster.set(rightProportion);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }
}