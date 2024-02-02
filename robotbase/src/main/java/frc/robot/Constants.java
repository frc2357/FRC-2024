// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import java.util.function.BooleanSupplier;

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

  public static final class CAN_ID {
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

    public static final int TOP_INTAKE_MOTOR_ID = 23;
    public static final int BOTTOM_INTAKE_MOTOR_ID = 24;

    public static final int TOP_SHOOTER_MOTOR_ID = 25;
    public static final int BOTTOM_SHOOTER_MOTOR_ID = 26;

    public static final int RIGHT_CLIMBER_MOTOR_ID = 27;
    public static final int LEFT_CLIMBER_MOTOR_ID = 28;

    public static final int SHOOTER_PIVOT_MOTOR_ID = 29;
    
    public static final int END_AFFECTOR_MOTOR_ID = 30;

    public static final int TRAP_AMP_ARM_MOTOR_ID = 31;
  }

  public static final class PHEONIX_TUNER {

    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    public static final Slot0Configs STEER_GAINS =
        new Slot0Configs().withKP(100).withKI(0).withKD(0.05).withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    public static final Slot0Configs DRIVE_GAINS =
        new Slot0Configs().withKP(3).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    public static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT =
        ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    public static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT =
        ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    public static final double SLIP_CURRENT_AMPS = 300.0;

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double SPEED_AT_12_VOLTS = 4.7;

    // Every 1 rotation of the azimuth results in COUPLE_RATIO drive motor turns;
    // This may need to be tuned to your individual robot
    public static final double COUPLE_RATIO = 3.5714285714285716;

    private static final SwerveModuleConstantsFactory ConstantCreator =
        new SwerveModuleConstantsFactory()
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

    public static final SwerveModuleConstants FRONT_LEFT_MODULE_CONSTANTS =
        ConstantCreator.createModuleConstants(
            CAN_ID.FRONT_LEFT_STEER_MOTOR_ID,
            CAN_ID.FRONT_LEFT_DRIVE_MOTOR_ID,
            CAN_ID.FRONT_LEFT_ENCODER_ID,
            SWERVE.FRONT_LEFT_ENCODER_OFFSET,
            Units.inchesToMeters(SWERVE.FRONT_LEFT_X_POSITION_INCHES),
            Units.inchesToMeters(SWERVE.FRONT_LEFT_Y_POSITION_INCHES),
            SWERVE.INVERT_LEFT_SIDE);
    public static final SwerveModuleConstants FRONT_RIGHT_MODULE_CONSTANTS =
        ConstantCreator.createModuleConstants(
            CAN_ID.FRONT_RIGHT_STEER_MOTOR_ID,
            CAN_ID.FRONT_RIGHT_DRIVE_MOTOR_ID,
            CAN_ID.FRONT_RIGHT_ENCODER_ID,
            SWERVE.FRONT_RIGHT_ENCODER_OFFSET,
            Units.inchesToMeters(SWERVE.FRONT_RIGHT_X_POSITION_INCHES),
            Units.inchesToMeters(SWERVE.FRONT_RIGHT_Y_POSITION_INCHES),
            SWERVE.INVERT_RIGHT_SIDE);
    public static final SwerveModuleConstants BACK_LEFT_MODULE_CONSTANTS =
        ConstantCreator.createModuleConstants(
            CAN_ID.BACK_LEFT_STEER_MOTOR_ID,
            CAN_ID.BACK_LEFT_DRIVE_MOTOR_ID,
            CAN_ID.BACK_LEFT_ENCODER_ID,
            SWERVE.BACK_LEFT_ENCODER_OFFSET,
            Units.inchesToMeters(SWERVE.BACK_LEFT_X_POSITION_INCHES),
            Units.inchesToMeters(SWERVE.BACK_LEFT_Y_POSITION_INCHES),
            SWERVE.INVERT_LEFT_SIDE);
    public static final SwerveModuleConstants BACK_RIGHT_MODULE_CONSTANTS =
        ConstantCreator.createModuleConstants(
            CAN_ID.BACK_RIGHT_STEER_MOTOR_ID,
            CAN_ID.BACK_RIGHT_DRIVE_MOTOR_ID,
            CAN_ID.BACK_RIGHT_ENCODER_ID,
            SWERVE.BACK_RIGHT_ENCODER_OFFSET,
            Units.inchesToMeters(SWERVE.BACK_RIGHT_X_POSITION_INCHES),
            Units.inchesToMeters(SWERVE.BACK_RIGHT_Y_POSITION_INCHES),
            SWERVE.INVERT_RIGHT_SIDE);

    public static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS =
        new SwerveDrivetrainConstants().withCANbusName("").withPigeon2Id(CAN_ID.PIGEON_ID);
  }

  public static final class SWERVE {
    public static final double MAX_SPEED_METERS_PER_SECOND = 4.57;
    public static final double MAX_ANGULAR_RATE_ROTATIONS_PER_SECOND = Math.PI * 2;

    public static final double DRIVE_GEAR_RATIO = 6.74603175;
    public static final double STEER_GEAR_RATIO = 21.428571428571427;
    public static final double WHEEL_RADIUS_INCHES = 2;

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

    public static final double TRANSLATIONAL_DEADBAND = 0.1;
    public static final double ROTATIONAL_DEADBAND = 0.1;

    public static final double STATIC_FEEDFORWARD_METERS_PER_SECOND = 0.094545;
    public static final double TRANSLATION_RAMP_EXPONENT = 2;
  }

  public static final class CHOREO {
    public static final PIDController X_CONTROLLER = new PIDController(0.15, 0, 0);
    public static final PIDController Y_CONTROLLER = new PIDController(0.15, 0, 0);
    public static final PIDController ROTATION_CONTROLLER = new PIDController(0.6, 0, 0);

    public static final BooleanSupplier CHOREO_AUTO_MIRROR_PATHS =
        new BooleanSupplier() {
          @Override
          public boolean getAsBoolean() {
            return false;
          }
        };
  }

  public static final class CONTROLLER {
    public static final int DRIVE_CONTROLLER_PORT = 0;
    public static final double DRIVE_CONTROLLER_DEADBAND = 0.1;
    public static final int CODRIVER_CONTROLLER_PORT = 1;
    public static final double CODRIVE_CONTROLLER_DEADBAND = 0.1;
    public static final double SWERVE_TRANSLATIONAL_DEADBAND = 0.05;
    public static final double SWERVE_ROTATIONAL_DEADBAND = 0.05;
  }

  public static final class SHOOTER {
    public static final double SHOOTER_AXIS_STEP_INTERVAL = 0.1;

    public static final IdleMode IDLE_MODE = IdleMode.kCoast;

    public static final boolean TOP_MOTOR_INVERTED = false;
    public static final boolean BOTTOM_MOTOR_INVERTED = false;

    public static final int TOP_MOTOR_STALL_LIMIT_AMPS = 40;
    public static final int TOP_MOTOR_FREE_LIMIT_AMPS = 40;

    public static final int BOTTOM_MOTOR_STALL_LIMIT_AMPS = 40;
    public static final int BOTTOM_MOTOR_FREE_LIMIT_AMPS = 40;

    public static final double TOP_MOTOR_P = 0.0;
    public static final double TOP_MOTOR_I = 0.0;
    public static final double TOP_MOTOR_D = 0.0;
    public static final double TOP_MOTOR_FF = 0.0;

    public static final double BOTTOM_MOTOR_P = 0.0;
    public static final double BOTTOM_MOTOR_I = 0.0;
    public static final double BOTTOM_MOTOR_D = 0.0;
    public static final double BOTTOM_MOTOR_FF = 0.0;
  }

  public static final class INTAKE {
    public static final IdleMode IDLE_MODE = IdleMode.kCoast;

    public static final boolean TOP_MOTOR_INVERTED = false;
    public static final boolean BOTTOM_MOTOR_INVERTED = false;

    public static final int TOP_MOTOR_STALL_LIMIT_AMPS = 40;
    public static final int TOP_MOTOR_FREE_LIMIT_AMPS = 40;

    public static final int BOTTOM_MOTOR_STALL_LIMIT_AMPS = 40;
    public static final int BOTTOM_MOTOR_FREE_LIMIT_AMPS = 40;
  }

  public static final class SHOOTER_LIMELIGHT {
    public static final String NAME = "";

    public static final int HUMAN_PIPELINE_INDEX = 0;

    public static final int RED_SPEAKER_APRILTAG_PIPELINE_INDEX = 1; // id 4
    public static final int BLUE_SPEAKER_APRILTAG_PIPELINE_INDEX = 2; // id 7

    public static final boolean IS_PRIMARY_STREAM = false;

    public static final double MOUNTING_ANGLE_DEGREES = 0.0;
    public static final double MOUNTING_HEIGHT_INCHES = 0.0;

    public static final double DEFAULT_RETURN_VALUE = 0.0;

    public static final double HEAD_ON_TOLERANCE = 0.0;

    public static final double TARGET_WIDTH = 0.0;
    public static final double TARGET_HEIGHT = 0.0;

    public static final double TARGET_HEIGHT_FROM_FLOOR = 0.0;
  }

  public static final class SHOOTER_PIVOT {
    public static final boolean MOTOR_INVERTED = false;
    public static final boolean ENCODER_INVERTED = false;

    public static final IdleMode IDLE_MODE = IdleMode.kBrake;

    public static final int MOTOR_STALL_LIMIT_AMPS = 40;
    public static final int MOTOR_FREE_LIMIT_AMPS = 40;

    public static final double PIVOT_P = 0.0;
    public static final double PIVOT_I = 0.0;
    public static final double PIVOT_D = 0.0;
    public static final double PIVOT_FF = 0.0;

    public static final int SMART_MOTION_MAX_VEL_RPM = 0;
    public static final int SMART_MOTION_MIN_VEL_RPM = 0;
    public static final int SMART_MOTION_MAX_ACC_RPM = 0;
    public static final int SMART_MOTION_ALLOWED_ERROR = 0;

    public static final double AXIS_MAX_SPEED = 0.25;

    public static final double ENCODER_POSITION_CONVERSION_FACTOR = 1;
    public static final double ENCODER_VELOCITY_CONVERSION_FACTOR = 1;
  }

  public static final class CLIMBER {
    public static final boolean RIGHT_MOTOR_INVERTED = false;
    public static final boolean LEFT_MOTOR_INVERTED = false;

    public static final int MOTOR_FREE_LIMIT_AMPS = 40;
    public static final int MOTOR_STALL_LIMIT_AMPS = 40;
  }

  public static final class END_AFFECTOR {
    public static final boolean IS_INVERTED = false;

    public static final IdleMode IDLE_MODE = IdleMode.kBrake;

    public static final int MOTOR_FREE_LIMIT_AMPS = 20;
    public static final int MOTOR_STALL_LIMIT_AMPS = 20;

    public static final double AXIS_MAX_SPEED = 0.25;
  }

  public static final class TRAP_AMP_ARM {
    public static final boolean MOTOR_IS_INVERTED = false;

    public static final IdleMode MOTOR_IDLE_MODE = IdleMode.kBrake;

    public static final int MOTOR_STALL_LIMIT_AMPS = 20;
    public static final int MOTOR_FREE_LIMIT_AMPS = 20;

    public static final double MOTOR_PID_P = 1;
    public static final double MOTOR_PID_I = 1;
    public static final double MOTOR_PID_D = 1;
    public static final double MOTOR_PID_FF = 0;

    public static final int SMART_MOTION_MAX_VEL_RPM = 0;
    public static final int SMART_MOTION_MIN_VEL_RPM = 0;
    public static final int SMART_MOTION_MAX_ACC_RPM = 0;
    public static final int SMART_MOTION_ALLOWED_ERROR = 0;

    public static final double AXIS_MAX_SPEED = 0.25;
  }
}
