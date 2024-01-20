package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterPivotTuningSubsystem extends SubsystemBase {
    private CANSparkMax m_motor;

    private SparkPIDController m_pid;

    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double kFF = 0;
    private double maxRPM = 5700;
    private double maxVel = 5700;
    private double maxAcc = 5700;
    private double allErr = 0;

    private double axisMaxSpeed = 0.25;

    public ShooterPivotTuningSubsystem() {
        m_motor = new CANSparkMax(-1, MotorType.kBrushless);

        m_pid = m_motor.getPIDController();

        configure();
        display();
    }

    public void configure() {
        m_motor.setInverted(false);

        m_motor.enableVoltageCompensation(12);
        m_motor.setIdleMode(IdleMode.kCoast);
        m_motor.setSmartCurrentLimit(40, 40);

        m_pid.setOutputRange(-1, 1);

        updatePIDs();
    }

    public void updatePIDs() {
        m_pid.setP(kP);
        m_pid.setI(kI);
        m_pid.setD(kD);
        m_pid.setFF(kFF);
        m_pid.setSmartMotionMaxVelocity(maxVel, 0);
        m_pid.setSmartMotionMaxAccel(maxAcc, 0);
        m_pid.setSmartMotionAllowedClosedLoopError(allErr, 0);
    }

    public void display() {
        SmartDashboard.putNumber("Shooter Pivot P", kP);
        SmartDashboard.putNumber("Shooter Pivot I", kI);
        SmartDashboard.putNumber("Shooter Pivot D", kD);
        SmartDashboard.putNumber("Shooter Pivot FF", kFF);
        SmartDashboard.putNumber("Shooter Pivot MaxVel", maxVel);
        SmartDashboard.putNumber("Shooter Pivot MaxAcc", maxAcc);
        SmartDashboard.putNumber("Shooter Pivot AllErr", allErr);
        SmartDashboard.putNumber("Shooter Pivot Setpoint", 0);
    }

    public void update() {
        kP = SmartDashboard.getNumber("Shooter Pivot P", 0);
        kI = SmartDashboard.getNumber("Shooter Pivot I", 0);
        kD = SmartDashboard.getNumber("Shooter Pivot D", 0);
        kFF = SmartDashboard.getNumber("Shooter Pivot FF", 0);
        maxVel = SmartDashboard.getNumber("Shooter Pivot MaxVel", maxRPM);
        maxAcc = SmartDashboard.getNumber("Shooter Pivot MaxAcc", maxRPM);
        allErr = SmartDashboard.getNumber("Shooter Pivot AllErr", 1);

        updatePIDs();

        double setpoint, position;
        setpoint = SmartDashboard.getNumber("Shooter Pivot Setpoint", 0);
        m_pid.setReference(setpoint, ControlType.kVelocity);
        position = m_motor.getEncoder().getPosition();

        SmartDashboard.putNumber("Shooter Pivot RPMs", position);
    }

    public void axisRun(double motorPO) {
        m_motor.set(motorPO * axisMaxSpeed);
    }
}
