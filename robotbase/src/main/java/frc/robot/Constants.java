// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.function.BooleanSupplier;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

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

    public static final int PIVOT_MOTOR_ID = 29;

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
    public static final double TARGET_LOCK_ROTATION_KP = 0.15;
    public static final double TARGET_LOCK_ROTATION_KI = 0.0;
    public static final double TARGET_LOCK_ROTATION_KD = 0.0;
    public static final PIDController TARGET_LOCK_ROTATION_PID_CONTROLLER =
        new PIDController(
            TARGET_LOCK_ROTATION_KP, TARGET_LOCK_ROTATION_KI, TARGET_LOCK_ROTATION_KD);

    public static final double TARGET_LOCK_FEED_FORWARD = 0.0;
    public static final double TARGET_LOCK_TOLERANCE = 0.25;

    // Translate to Apriltag
    public static final PIDController APRILTAG_ROTATION_PID_CONTROLLER =
        new PIDController(0.03, 0, 0.01);
    public static final PIDController APRILTAG_X_TRANSLATION_PID_CONTROLLER =
        new PIDController(0.05, 0, 0);
    public static final PIDController APRILTAG_Y_TRANSLATION_PID_CONTROLLER =
        new PIDController(0.08, 0, 0);

    public static final double APRILTAG_X_TOLERANCE = 0.5;
    public static final double APRILTAG_Y_TOLERANCE = 0.5;
    public static final double APRILTAG_ROTATION_TOLERANCE = .025; // Radians
    public static final double APRILTAG_TY_MAGIC_OFFSET = 12.5;

    public static final double AMP_TX_SETPOINT = 0;
    public static final double AMP_TY_SETPOINT = 3;
    public static final double AMP_ROTATION_SETPOINT = Math.PI / 2;
  }

  public static final class CHOREO {
    public static final PIDController X_CONTROLLER = new PIDController(2, 0, 0);
    public static final PIDController Y_CONTROLLER = new PIDController(2, 0, 0);
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
    public static final double DRIVE_CONTROLLER_DEADBAND = 0.01;
    public static final int CODRIVER_CONTROLLER_PORT = 1;
    public static final double CODRIVE_CONTROLLER_DEADBAND = 0.025;
    public static final double SWERVE_TRANSLATIONAL_DEADBAND = 0.0;
    public static final double SWERVE_ROTATIONAL_DEADBAND = 0.0;
  }

  public static final class SHOOTER {
    public static final double TOP_MOTOR_SOURCE_INTAKE_RPMS = 0;
    public static final double BOTTOM_MOTOR_SOURCE_INTAKE_RPMS = 0;

    public static final double TOP_MOTOR_FEED_END_AFFECTOR_RPMS = 500;
    public static final double BOTTOM_MOTOR_FEED_END_AFFECTOR_RPMS = 500;

    public static final double SHOOTER_AXIS_STEP_INTERVAL = 0.1;

    public static final double SHOOTER_AXIS_MAX_SPEED = 0.8;

    public static final IdleMode IDLE_MODE = IdleMode.kCoast;

    public static final boolean TOP_MOTOR_INVERTED = false;
    public static final boolean BOTTOM_MOTOR_INVERTED = true;

    public static final double TOP_MOTOR_SUBWOOFER_SHOT_RPMS = 0;
    public static final double BOTTOM_MOTOR_SUBWOOFER_SHOT_RPMS = 0;

    public static final int TOP_MOTOR_STALL_LIMIT_AMPS = 40;
    public static final int TOP_MOTOR_FREE_LIMIT_AMPS = 40;

    public static final int BOTTOM_MOTOR_STALL_LIMIT_AMPS = 40;
    public static final int BOTTOM_MOTOR_FREE_LIMIT_AMPS = 40;

    public static final double RAMP_RATE = 5;

    public static final double TOP_MOTOR_P = 0.0; // TODO: tune shooter motor PIDs
    public static final double TOP_MOTOR_I = 0.0;
    public static final double TOP_MOTOR_D = 0.0;
    public static final double TOP_MOTOR_FF = 0.000195;

    public static final double BOTTOM_MOTOR_P = 0.0;
    public static final double BOTTOM_MOTOR_I = 0.0;
    public static final double BOTTOM_MOTOR_D = 0.0;
    public static final double BOTTOM_MOTOR_FF = 0.00021;
  }

  public static final class INTAKE {
    public static final double AXIS_MAX_SPEED = 0.8;

    public static final double FEED_TO_SHOOTER_TIMEOUT = 0;

    public static final double TOP_MOTOR_PICKUP_SPEED_PERCENT_OUTPUT = .75;
    public static final double BOTTOM_MOTOR_PICKUP_SPEED_PERCENT_OUTPUT = .75;

    public static final double TOP_MOTOR_SLOW_PICKUP_SPEED_PERCENT_OUTPUT = .1;
    public static final double BOTTOM_MOTOR_SLOW_PICKUP_SPEED_PERCENT_OUTPUT = .1;

    public static final double TOP_MOTOR_SOURCE_INTAKE_SPEED_PERCENT_OUTPUT = 0.25;
    public static final double BOTTOM_MOTOR_SOURCE_INTAKE_SPEED_PERCENT_OUTPUT = 0.25;

    public static final double TOP_MOTOR_FEED_SPEED_PERCENT_OUTPUT = 0.75;
    public static final double BOTTOM_MOTOR_FEED_SPEED_PERCENT_OUTPUT = 0.75;

    public static final IdleMode IDLE_MODE = IdleMode.kBrake;

    public static final boolean TOP_MOTOR_INVERTED = false;
    public static final boolean BOTTOM_MOTOR_INVERTED = false;

    public static final int TOP_MOTOR_STALL_LIMIT_AMPS = 60;
    public static final int TOP_MOTOR_FREE_LIMIT_AMPS = 60;

    public static final int BOTTOM_MOTOR_STALL_LIMIT_AMPS = 60;
    public static final int BOTTOM_MOTOR_FREE_LIMIT_AMPS = 60;
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

    public static final double MOUNTING_ANGLE_DEGREES = 0.0; // TODO: tune limelight constants
    public static final double MOUNTING_HEIGHT_INCHES = 0.0;

    public static final double DEFAULT_RETURN_VALUE = 0.0;

    public static final double HEAD_ON_TOLERANCE = 0.0;

    public static final double TARGET_WIDTH = 0.0;
    public static final double TARGET_HEIGHT = 0.0;

    public static final double TARGET_HEIGHT_FROM_FLOOR = 0.0;
  }

  public static final class PIVOT {
    public static final double FEED_TO_END_AFFECTOR_LOCATION = 63.5;
    public static final double MAX_PIVOT_ROTATION = 67.5;
    public static final double MIN_PIVOT_ROTATION = 10;

    public static final double SUBWOOFER_SHOT_ROTATION = 60;
    public static final double SPEAKER_SCORE_FROM_BASE_ROTATION = 0;
    public static final double INTAKE_FROM_SOURCE_ROTATION = 45;
    public static final double DEFAULT_PIVOT_ROTATION = 45; // angle of intake

    public static final boolean MOTOR_INVERTED = false;
    public static final boolean ENCODER_INVERTED = false;

    public static final IdleMode IDLE_MODE = IdleMode.kBrake;

    public static final int MOTOR_STALL_LIMIT_AMPS = 40;
    public static final int MOTOR_FREE_LIMIT_AMPS = 40;

    public static final double POSITION_ALLOWED_ERROR = 1;
    public static final boolean POSITION_PID_WRAPPING_ENABLED = false;

    public static final double AXIS_MAX_SPEED = 0.25;

    public static final double ENCODER_POSITION_CONVERSION_FACTOR = 360;
    public static final double ENCODER_VELOCITY_CONVERSION_FACTOR = 1;
    public static final double ENCODER_ZERO_OFFSET = 201.0478306 - 10;

    // Closed loop - lines with comments were used for arm rotation 2023 and we will
    // probably need for this
    public static final double PIVOT_P = 0.015;
    public static final double PIVOT_I = 0;
    public static final double PIVOT_D = 0;
    public static final double PIVOT_FF = 0.0005; // Barely moving: .000465
  }

  public static final class CLIMBER {
    public static final boolean RIGHT_MOTOR_INVERTED = false;
    public static final boolean LEFT_MOTOR_INVERTED = true;

    public static final int MOTOR_FREE_LIMIT_AMPS = 40; // TODO: Tune climber amp limits
    public static final int MOTOR_STALL_LIMIT_AMPS = 40;
  }

  public static final class END_AFFECTOR {
    public static final boolean IS_INVERTED = false;

    public static final IdleMode IDLE_MODE = IdleMode.kBrake;

    public static final double INTAKE_SPEED = -1;
    public static final double PRELOAD_SPEED = -1;
    public static final double SCORE_SPEED_AMP = 1;
    public static final double SCORE_SPEED_TRAP = 1;

    public static final double SECONDS_TO_SCORE_TRAP = 0;

    public static final double STOWED_NOTE_AMPERAGE_LIMIT = 0;

    public static final int MOTOR_FREE_LIMIT_AMPS = 20;
    public static final int MOTOR_STALL_LIMIT_AMPS = 20; // TODO: TUNE

    public static final double AXIS_MAX_SPEED = 1;
  }

  public static final class EXTENSION_ARM {
    public static final boolean MOTOR_IS_INVERTED = false;
    public static final boolean ENCODER_INVERTED = true;

    public static final IdleMode MOTOR_IDLE_MODE = IdleMode.kBrake;

    // TODO: Tune arm amp limits + PID + smart motion + Zeroing constants

    public static final int MOTOR_STALL_LIMIT_AMPS = 40;
    public static final int MOTOR_FREE_LIMIT_AMPS = 40;

    public static final double MOTOR_PID_P = 0.01;
    public static final double MOTOR_PID_I = 0;
    public static final double MOTOR_PID_D = 0;
    public static final double MOTOR_PID_FF = 0.0001;

    public static final int SMART_MOTION_MAX_VEL_RPM = 6000;
    public static final int SMART_MOTION_MIN_VEL_RPM = 0;
    public static final int SMART_MOTION_MAX_ACC_RPM = 30000;
    public static final double SMART_MOTION_ALLOWED_ERROR = 0.1;

    public static final double AXIS_MAX_SPEED = 0.5;

    public static final double ZERO_SPEED = 0; // TODO: TUNE
    public static final double ZERO_SPEED_STOP_TOLERANCE = 0; // TODO: TUNE

    public static final double HOME_ROTATIONS = 0.1;
    public static final double NOTE_STOW_ROTATIONS = 1.4;
    public static final double AMP_PREPOSE_ROTATIONS = 3.75; // TODO: TUNE
    public static final double AMP_SCORE_ROTATIONS = 6;
    public static final double TRAP_SCORE_ROTATIONS = 0; // TODO: TUNE
  }

  public static final class SCORING {
    public static final double SECONDS_PRELOAD_NOTE = 0.5;

    public static final double SECONDS_AMP_SCORE = 1;
  }

  public static final class PHOTON_VISION {
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public static final PoseStrategy POSE_STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
  }

  public static final class SHOOTER_PHOTON_CAMERA {
    public static final String NAME = "ov9782-shooter";

    public static final int APRIL_TAG_PIPELINE = 0;
    public static final int NEURAL_NETWORK_PIPELINE = 1;
    public static final int POSE_ESTIMATION_PIPELINE = 2;

    public static final int DEFAULT_PIPELINE = 2;

    public static final double HEAD_ON_TOLERANCE = 0;

    public static final double LENS_BEHIND_OF_ROBOT_ORIGIN_INCHES = 8.172;
    public static final double LENS_TO_RIGHT_OF_ROBOT_ORIGIN_INCHES = 8.45;
    public static final double LENS_HEIGHT_FROM_ROBOT_ORIGIN_INCHES = 13.388;
    public static final double LENS_ANGLE_TILTED_UP_DEGREES = 30;
    public static final Transform3d ROBOT_TO_CAMERA_TRANSFORM =
        new Transform3d(
            new Translation3d(
                -Units.inchesToMeters(LENS_BEHIND_OF_ROBOT_ORIGIN_INCHES),
                -Units.inchesToMeters(LENS_TO_RIGHT_OF_ROBOT_ORIGIN_INCHES),
                Units.inchesToMeters(LENS_HEIGHT_FROM_ROBOT_ORIGIN_INCHES)),
            new Rotation3d(0, LENS_ANGLE_TILTED_UP_DEGREES, 0));
  }
}
