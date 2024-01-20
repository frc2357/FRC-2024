package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterTuningSubsystem {
    private CANSparkMax m_topMotor;
    private CANSparkMax m_bottomMotor;

    private SparkPIDController m_topPID;
    private SparkPIDController m_bottomPID;

    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double kFF = 0;
    private double maxRPM = 5700;
    private double maxVel = 5700;
    private double maxAcc = 5700;

    public ShooterTuningSubsystem() {
        m_topMotor = new CANSparkMax(-1, MotorType.kBrushless);
        m_bottomMotor = new CANSparkMax(-1, MotorType.kBrushless);

        m_topPID = m_topMotor.getPIDController();
        m_bottomPID = m_bottomMotor.getPIDController();

        configure();
        display();
    }

    public void configure() {
        m_topMotor.setInverted(false);
        m_bottomMotor.setInverted(false);

        m_topMotor.enableVoltageCompensation(12);
        m_topMotor.setIdleMode(IdleMode.kCoast);
        m_topMotor.setSmartCurrentLimit(40, 40);

        m_bottomMotor.enableVoltageCompensation(12);
        m_bottomMotor.setIdleMode(IdleMode.kCoast);
        m_bottomMotor.setSmartCurrentLimit(40, 40);

        m_topPID.setOutputRange(-1, 1);
        m_bottomPID.setOutputRange(-1, 1);

        updatePIDs();
    }

    public void updatePIDs() {
        m_topPID.setP(kP);
        m_topPID.setI(kI);
        m_topPID.setD(kD);
        m_topPID.setFF(kFF);
        m_topPID.setSmartMotionMaxVelocity(maxVel, 0);
        m_topPID.setSmartMotionMaxAccel(maxAcc, 0);

        m_bottomPID.setP(kP);
        m_bottomPID.setI(kI);
        m_bottomPID.setD(kD);
        m_bottomPID.setFF(kFF);
        m_bottomPID.setSmartMotionMaxVelocity(maxVel, 0);
        m_bottomPID.setSmartMotionMaxAccel(maxAcc, 0);
    }

    public void display() {
        SmartDashboard.putNumber("Shooter P", kP);
        SmartDashboard.putNumber("Shooter I", kI);
        SmartDashboard.putNumber("Shooter D", kD);
        SmartDashboard.putNumber("Shooter FF", kFF);
        SmartDashboard.putNumber("Shooter MaxVel", maxVel);
        SmartDashboard.putNumber("Shooter MaxAcc", maxAcc);
        SmartDashboard.putNumber("Shooter Setpoint", 0);
    }

    public void update() {
        kP = SmartDashboard.getNumber("Shooter P", 0);
        kI = SmartDashboard.getNumber("Shooter I", 0);
        kD = SmartDashboard.getNumber("Shooter D", 0);
        kFF = SmartDashboard.getNumber("Shooter FF", 0);
        maxVel = SmartDashboard.getNumber("Shooter MaxVel", maxRPM);
        maxAcc = SmartDashboard.getNumber("Shooter MaxAcc", maxRPM);

        updatePIDs();

        double topSetPoint, bottomSetPoint, topSpeed, bottomSpeed;
        topSetPoint = SmartDashboard.getNumber("Top Shooter Setpoint", 0);
        bottomSetPoint = SmartDashboard.getNumber("Bottom Shooter Setpoint", 0);
        m_topPID.setReference(topSetPoint, ControlType.kVelocity);
        m_bottomPID.setReference(bottomSetPoint, ControlType.kVelocity);
        topSpeed = m_topMotor.getEncoder().getVelocity();
        bottomSpeed = m_bottomMotor.getEncoder().getVelocity();

        SmartDashboard.putNumber("Top Shooter RPMs", topSpeed);
        SmartDashboard.putNumber("Bottom Shooter RPMs", bottomSpeed);
    }

    public void axisRun(double topPO, double bottomPO, boolean reverse) {
        topPO = reverse ? -topPO : topPO;
        bottomPO = reverse ? -bottomPO : bottomPO;

        m_topMotor.set(topPO);
        m_bottomMotor.set(bottomPO);
    }
}