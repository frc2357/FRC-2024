// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;

import edu.wpi.first.math.util.Units;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OPERATOR_CONSTANTS {
    public static final int DRIVER_CONTROLLER_PORT = 0;
  }

  public static class CAN_ID{
    public static final int PIGEON_ID = 5;
    public static final int FRONT_LEFT_DRIVE_MOTOR_ID = 11;
    public static final int FRONT_LEFT_STEER_MOTOR_ID = 12;
    public static final int FRONT_LEFT_ENCODER_ID = 19;

    public static final int FRONT_RIGHT_DRIVE_MOTOR_ID = 13;
    public static final int FRONT_RIGHT_STEER_MOTOR_ID = 14;
    public static final int FRONT_RIGHT_ENCODER_ID = 20;

    public static final int BACK_LEFT_DRIVE_MOTOR_ID = 15;
    public static final int BACK_LEFT_STEER_MOTOR_ID = 16;
    public static final int BACK_LEFT_ENCODER_ID = 21;

    public static final int BACK_RIGHT_DRIVE_MOTOR_ID = 17;
    public static final int BACK_RIGHT_STEER_MOTOR_ID = 18;
    public static final int BACK_RIGHT_ENCODER_ID = 22;
  }
  public static class PHEONIX_TUNER{
    
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    public static final Slot0Configs STEER_GAINS = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.05)
        .withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    public static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
        .withKP(3).withKI(0).withKD(0)
        .withKS(0).withKV(0).withKA(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    public static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    public static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    public static final double SLIP_CURRENT_AMPS = 300.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double SPEED_AT_12_VOLTS = 6.0;

    // Every 1 rotation of the azimuth results in COUPLE_RATIO drive motor turns;
    // This may need to be tuned to your individual robot
    public static final double COUPLE_RATIO = 3.5714285714285716;

    private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
            .withDriveMotorGearRatio(SWERVE.DRIVE_GEAR_RATIO)
            .withSteerMotorGearRatio(SWERVE.STEER_GEAR_RATIO)
            .withWheelRadius(SWERVE.WHEEL_RADIUS_INCHES)
            .withSlipCurrent(SLIP_CURRENT_AMPS)
            .withSteerMotorGains(STEER_GAINS)
            .withDriveMotorGains(DRIVE_GAINS)
            .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
            .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
            .withSpeedAt12VoltsMps(SPEED_AT_12_VOLTS)
            .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
            .withCouplingGearRatio(COUPLE_RATIO)
            .withSteerMotorInverted(SWERVE.STEER_MOTOR_INVERSED);
    
    public static final SwerveModuleConstants FRONT_LEFT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
      CAN_ID.FRONT_LEFT_STEER_MOTOR_ID, CAN_ID.FRONT_LEFT_DRIVE_MOTOR_ID, CAN_ID.FRONT_LEFT_ENCODER_ID, SWERVE.FRONT_LEFT_ENCODER_OFFSET, 
        Units.inchesToMeters(SWERVE.FRONT_LEFT_X_POSITION_INCHES), Units.inchesToMeters(SWERVE.FRONT_LEFT_Y_POSITION_INCHES), SWERVE.INVERT_LEFT_SIDE);
    public static final SwerveModuleConstants FRONT_RIGHT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
      CAN_ID.FRONT_RIGHT_STEER_MOTOR_ID, CAN_ID.FRONT_RIGHT_DRIVE_MOTOR_ID, CAN_ID.FRONT_RIGHT_ENCODER_ID, SWERVE.FRONT_RIGHT_ENCODER_OFFSET, 
        Units.inchesToMeters(SWERVE.FRONT_RIGHT_X_POSITION_INCHES), Units.inchesToMeters(SWERVE.FRONT_RIGHT_Y_POSITION_INCHES), SWERVE.INVERT_RIGHT_SIDE);
    public static final SwerveModuleConstants BACK_LEFT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
      CAN_ID.BACK_LEFT_STEER_MOTOR_ID, CAN_ID.BACK_LEFT_DRIVE_MOTOR_ID, CAN_ID.BACK_LEFT_ENCODER_ID, SWERVE.BACK_LEFT_ENCODER_OFFSET, 
        Units.inchesToMeters(SWERVE.BACK_LEFT_X_POSITION_INCHES), Units.inchesToMeters(SWERVE.BACK_LEFT_Y_POSITION_INCHES), SWERVE.INVERT_LEFT_SIDE);
    public static final SwerveModuleConstants BACK_RIGHT_MODULE_CONSTANTS = ConstantCreator.createModuleConstants(
      CAN_ID.BACK_RIGHT_STEER_MOTOR_ID, CAN_ID.BACK_RIGHT_DRIVE_MOTOR_ID, CAN_ID.BACK_RIGHT_ENCODER_ID, SWERVE.BACK_RIGHT_ENCODER_OFFSET, 
        Units.inchesToMeters(SWERVE.BACK_RIGHT_X_POSITION_INCHES), Units.inchesToMeters(SWERVE.BACK_RIGHT_Y_POSITION_INCHES), SWERVE.INVERT_RIGHT_SIDE);
  
    public static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS = new SwerveDrivetrainConstants()
      .withPigeon2Id(CAN_ID.PIGEON_ID).withCANbusName("");
  }
  public static class SWERVE{
    public static final double MAX_SPEED_METERS_PER_SECOND = 4.7;
    public static final double MAX_ANGULAR_RATE_ROTATIONS_PER_SECOND = Math.PI;

    public static final double DRIVE_GEAR_RATIO = 6.746031746031747;
    public static final double STEER_GEAR_RATIO = 21.428571428571427;
    public static final double WHEEL_RADIUS_INCHES = 4;

    public static final boolean STEER_MOTOR_INVERSED = true;
    public static final boolean INVERT_LEFT_SIDE = false;
    public static final boolean INVERT_RIGHT_SIDE = true;

    // Front Left
    public static final double FRONT_LEFT_ENCODER_OFFSET = 0.077392578125;
    public static final double FRONT_LEFT_X_POSITION_INCHES = 9.375;
    public static final double FRONT_LEFT_Y_POSITION_INCHES = 9.375;

    // Front Right
    public static final double FRONT_RIGHT_ENCODER_OFFSET = 0.473876953125;
    public static final double FRONT_RIGHT_X_POSITION_INCHES = 9.375;
    public static final double FRONT_RIGHT_Y_POSITION_INCHES = -9.375;

    // Back Left
    public static final double BACK_LEFT_ENCODER_OFFSET = 0.2294921875;
    public static final double BACK_LEFT_X_POSITION_INCHES = -9.375;
    public static final double BACK_LEFT_Y_POSITION_INCHES = 9.375;

    // Back Right
    public static final double BACK_RIGHT_ENCODER_OFFSET = 0.37841796875;
    public static final double BACK_RIGHT_X_POSITION_INCHES = -9.375;
    public static final double BACK_RIGHT_Y_POSITION_INCHES = -9.375;
  }
}
