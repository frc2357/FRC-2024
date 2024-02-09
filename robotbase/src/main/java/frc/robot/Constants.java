// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.controller.PIDController;
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

  public static final class DIGITAL_INPUT {
    public static final int INTAKE_BEAM_BREAK_ID = 0;
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

    public static final double STATIC_FEEDFORWARD_METERS_PER_SECOND = 0.094545;
    public static final double TRANSLATION_RAMP_EXPONENT = 2;

    // Gamepiece tracking
    public static final double AUTO_TRANSLATE_DEBOUNCE_SECONDS = 0.1;
    public static final double PIECE_TRACKING_TRANSLATION_SPEED = 0;
    public static final double PIECE_DEBOUNCE_SECONDS = 0.1;
    public static final double PIECE_TRACKING_ROTATION_TOLERANCE = 0.1;

    public static final double PIECE_TRACKING_MAX_DISTANCE_METERS = 3.0; // In Meters
    public static final double PIECE_TRACKING_SLOW_DOWN_METERS =
        1.0; // Robot goes half speed once passed
    public static final double PIECE_TRACKING_X_METERS_PER_SECOND = 2;

    // Target Lock
    public static final double ROTATION_KP = 0.15;
    public static final double ROTATION_KI = 0.0;
    public static final double ROTATION_KD = 0.0;
    public static final PIDController ROTATION_PID_CONTROLLER =
        new PIDController(ROTATION_KP, ROTATION_KI, ROTATION_KD);

    public static final double TARGET_LOCK_FEED_FORWARD = 0.0;
    public static final double TARGET_LOCK_TOLERANCE = 0.25;

    // Translate to Apriltag
    public static final PIDController APRILTAG_ROTATION_PID_CONTROLLER =
        new PIDController(3, 0, 0.01);
    public static final PIDController APRILTAG_X_TRANSLATION_PID_CONTROLLER =
        new PIDController(0.05, 0, 0);
    public static final PIDController APRILTAG_Y_TRANSLATION_PID_CONTROLLER =
        new PIDController(0.2, 0, 0);

    public static final double APRILTAG_X_TOLERANCE = 1.5;
    public static final double APRILTAG_Y_TOLERANCE = 0.5;
    public static final double APRILTAG_ROTATION_TOLERANCE = .025; // Radians
    public static final double APRILTAG_TY_MAGIC_OFFSET = 12.5;

    public static final double AMP_TX_SETPOINT = 0;
    public static final double AMP_TY_SETPOINT = -10;
    public static final double AMP_ROTATION_SETPOINT = Math.PI / 2;
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
    public static final double DRIVE_CONTROLLER_DEADBAND = 0.025;
    public static final int CODRIVER_CONTROLLER_PORT = 1;
    public static final double CODRIVE_CONTROLLER_DEADBAND = 0.025;
    public static final double SWERVE_TRANSLATIONAL_DEADBAND = 0.0;
    public static final double SWERVE_ROTATIONAL_DEADBAND = 0.0;
  }

  public static final class SHOOTER {
    public static final double TOP_MOTOR_SOURCE_INTAKE_RPMS = 0;
    public static final double BOTTOM_MOTOR_SOURCE_INTAKE_RPMS = 0;

    public static final double SHOOTER_AXIS_STEP_INTERVAL = 0.1;

    public static final IdleMode IDLE_MODE = IdleMode.kCoast;

    public static final boolean TOP_MOTOR_INVERTED = false;
    public static final boolean BOTTOM_MOTOR_INVERTED = false;

    public static final int TOP_MOTOR_STALL_LIMIT_AMPS = 40;
    public static final int TOP_MOTOR_FREE_LIMIT_AMPS = 40;

    public static final int BOTTOM_MOTOR_STALL_LIMIT_AMPS = 40;
    public static final int BOTTOM_MOTOR_FREE_LIMIT_AMPS = 40;

    public static final double TOP_MOTOR_P = 0.0; //TODO: tune shooter motor PIDs
    public static final double TOP_MOTOR_I = 0.0;
    public static final double TOP_MOTOR_D = 0.0;
    public static final double TOP_MOTOR_FF = 0.0;

    public static final double BOTTOM_MOTOR_P = 0.0;
    public static final double BOTTOM_MOTOR_I = 0.0;
    public static final double BOTTOM_MOTOR_D = 0.0;
    public static final double BOTTOM_MOTOR_FF = 0.0;
  }

  public static final class INTAKE {
    public static final double AXIS_MAX_SPEED = 0;

    public static final double TOP_MOTOR_PICKUP_SPEED_PERCENT_OUTPUT = 0;
    public static final double BOTTOM_MOTOR_PICKUP_SPEED_PERCENT_OUTPUT = 0;

    public static final double TOP_MOTOR_SOURCE_INTAKE_SPEED_PERCENT_OUTPUT = 0;
    public static final double BOTTOM_MOTOR_SOURCE_INTAKE_SPEED_PERCENT_OUTPUT = 0;

    public static final double TOP_MOTOR_FEED_SPEED_PERCENT_OUTPUT = 0;
    public static final double BOTTOM_MOTOR_FEED_SPEED_PERCENT_OUTPUT = 0;

    public static final IdleMode IDLE_MODE = IdleMode.kCoast;

    public static final boolean TOP_MOTOR_INVERTED = false;
    public static final boolean BOTTOM_MOTOR_INVERTED = false;

    public static final int TOP_MOTOR_STALL_LIMIT_AMPS = 40; //TODO: Tune intake motor amp limits
    public static final int TOP_MOTOR_FREE_LIMIT_AMPS = 40;

    public static final int BOTTOM_MOTOR_STALL_LIMIT_AMPS = 40;
    public static final int BOTTOM_MOTOR_FREE_LIMIT_AMPS = 40;
  }

  /*
   * TODO: Make Limelight pipelines match the following list
   * 0: Human pipeline
   * 1: Gamepiece pipeline
   * 2: Speaker apriltag pipeline
   * 3: Amp apriltag pipeline
   * 4: Source apriltag pipeline
   * 5: Stage apriltag pipeline
   */

  public static final class SHOOTER_LIMELIGHT {
    public static final String NAME = "limelight-shooter";

    public static final int HUMAN_PIPELINE_INDEX = 0;

    public static final int GAMEPIECE_INDEX = 2;
    public static final int AMP_PIPELINE_INDEX = 3;
    public static final int SPEAKER_PIPELINE_INDEX = 4; // ids 4 and 7

    public static final boolean IS_PRIMARY_STREAM = false;

    public static final double MOUNTING_ANGLE_DEGREES = 0.0; //TODO: tune limelight constants
    public static final double MOUNTING_HEIGHT_INCHES = 0.0;

    public static final double DEFAULT_RETURN_VALUE = 0.0;

    public static final double HEAD_ON_TOLERANCE = 0.0;

    public static final double TARGET_WIDTH = 0.0;
    public static final double TARGET_HEIGHT = 0.0;

    public static final double TARGET_HEIGHT_FROM_FLOOR = 0.0;
  }

  public static final class SHOOTER_PIVOT {
    public static final double INTAKE_FROM_SOURCE_ROTATIONS = 0;

    public static final boolean MOTOR_INVERTED = false;
    public static final boolean ENCODER_INVERTED = false;

    public static final IdleMode IDLE_MODE = IdleMode.kBrake;

    public static final int MOTOR_STALL_LIMIT_AMPS = 40;
    public static final int MOTOR_FREE_LIMIT_AMPS = 40;

    public static final double PIVOT_P = 0.02; //TODO: Tune pivot PID
    public static final double PIVOT_I = 0;
    public static final double PIVOT_D = 0;
    public static final double PIVOT_FF = 0.045;

    public static final double POSITION_ALLOWED_ERROR = 0.1;
    public static final boolean POSITION_PID_WRAPPING_ENABLED = false;

    public static final double AXIS_MAX_SPEED = 0.25;

    public static final double ENCODER_POSITION_CONVERSION_FACTOR = 1;
    public static final double ENCODER_VELOCITY_CONVERSION_FACTOR = 1;
  }

  public static final class CLIMBER {
    public static final boolean RIGHT_MOTOR_INVERTED = false;
    public static final boolean LEFT_MOTOR_INVERTED = false;

    public static final int MOTOR_FREE_LIMIT_AMPS = 40; //TODO: Tune climber amp limits
    public static final int MOTOR_STALL_LIMIT_AMPS = 40;
  }

  public static final class END_AFFECTOR {
    public static final boolean IS_INVERTED = false;

    public static final IdleMode IDLE_MODE = IdleMode.kBrake;

    public static final double INTAKE_SPEED = 0;

    public static final double STOWED_NOTE_AMPERAGE_LIMIT = 0;

    public static final int MOTOR_FREE_LIMIT_AMPS = 20;
    public static final int MOTOR_STALL_LIMIT_AMPS = 20; // TODO: TUNE

    public static final double AXIS_MAX_SPEED = 0.25;
  }

  public static final class TRAP_AMP_ARM {
    public static final boolean MOTOR_IS_INVERTED = false;

    public static final IdleMode MOTOR_IDLE_MODE = IdleMode.kBrake;
    
    //TODO: Tune arm amp limits + PID + smart motion + Zeroing constants
    
    public static final int MOTOR_STALL_LIMIT_AMPS = 20; 
    public static final int MOTOR_FREE_LIMIT_AMPS = 20;

    public static final double MOTOR_PID_P = 0;
    public static final double MOTOR_PID_I = 0;
    public static final double MOTOR_PID_D = 0;
    public static final double MOTOR_PID_FF = 0;

    public static final int SMART_MOTION_MAX_VEL_RPM = 0;
    public static final int SMART_MOTION_MIN_VEL_RPM = 0;
    public static final int SMART_MOTION_MAX_ACC_RPM = 0;
    public static final int SMART_MOTION_ALLOWED_ERROR = 0;

    public static final double AXIS_MAX_SPEED = 0.25;

    public static final boolean ABSOLUTE_ENCODER_IS_INVERTED = false;
    public static final double ABSOLUTE_ENCODER_POSITION_SCALE_FACTOR = 1;
    public static final double ABSOLUTE_ENCODER_VELOCITY_SCALE_FACTOR = 1;

    public static final double HOME_ROTATIONS = 0;
    public static final double NOTE_STOW_ROTATIONS = 0;
    public static final double PRELOAD_ROTATIONS = 0;
    public static final double AMP_SCORE_ROTATIONS = 0;
    public static final double TRAP_SCORE_ROTATIONS = 0;
    public static final double ZERO_SPEED = 0;
    public static final double ZERO_SPEED_STOP_TOLERANCE = 0;
  }
}
