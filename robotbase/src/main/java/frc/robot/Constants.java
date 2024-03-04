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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
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
    public static final double TARGET_LOCK_ROTATION_KP = 0.1;
    public static final double TARGET_LOCK_ROTATION_KI = 0.0;
    public static final double TARGET_LOCK_ROTATION_KD = 0.0;
    public static final PIDController TARGET_LOCK_ROTATION_PID_CONTROLLER =
        new PIDController(
            TARGET_LOCK_ROTATION_KP, TARGET_LOCK_ROTATION_KI, TARGET_LOCK_ROTATION_KD);

    public static final double TARGET_LOCK_FEED_FORWARD = 0.0;
    public static final double TARGET_LOCK_TOLERANCE = 1.5;
    public static final double TARGET_LOCK_YAW_SETPOINT =
        0; // PhotonVision is consistently 2 degrees to the right of our target

    // Translate to Apriltag
    public static final PIDController APRILTAG_ROTATION_PID_CONTROLLER =
        new PIDController(5, 0, 0.0);
    public static final double APRILTAG_ROTATION_FEEDFORWARD = 0.00001;
    public static final PIDController APRILTAG_X_TRANSLATION_PID_CONTROLLER =
        new PIDController(0.15, 0, 0);
    public static final PIDController APRILTAG_Y_TRANSLATION_PID_CONTROLLER =
        new PIDController(0.15, 0, 0);

    public static final double APRILTAG_YAW_TOLERANCE = 1;
    public static final double APRILTAG_PITCH_TOLERANCE = 0.5;
    public static final double APRILTAG_ROTATION_TOLERANCE_RADIANS = 0.02;
    public static final double APRILTAG_PITCH_MAGIC_OFFSET = 12.5;
    public static final double APRILTAG_CLOSE_PITCH = 4.0;
    public static final double APRILTAG_CLOSE_YAW_FACTOR = 2.0;

    public static final double AMP_YAW_SETPOINT = 0;
    public static final double AMP_PITCH_SETPOINT = 5;
    public static final double BLUE_AMP_ROTATION_SETPOINT_RADIANS = -Math.PI / 2;
    public static final double RED_AMP_ROTATION_SETPOINT_RADIANS = Math.PI / 2;

    public static final double STAGE_YAW_SETPOINT = 0;
    public static final double STAGE_PITCH_SETPOINT = 15;

    // Tune this during field calibration
    public static final double BLUE_LEFT_STAGE_ROTATION_SETPOINT_RADIANS = 0;
    public static final double BLUE_RIGHT_STAGE_ROTATION_SETPOINT_RADIANS = 0;
    public static final double BLUE_CENTER_STAGE_ROTATION_SETPOINT_RADIANS = 2.17;
    public static final double RED_LEFT_STAGE_ROTATION_SETPOINT_RADIANS = 0;
    public static final double RED_RIGHT_STAGE_ROTATION_SETPOINT_RADIANS = 0;
    public static final double RED_CENTER_STAGE_ROTATION_SETPOINT_RADIANS = 0;

    public static final double TIME_TO_COAST_SECONDS = 5;
  }

  public static final class CHOREO {
    public static final PIDController X_CONTROLLER = new PIDController(2, 0, 0);
    public static final PIDController Y_CONTROLLER = new PIDController(2, 0, 0);
    public static final PIDController ROTATION_CONTROLLER = new PIDController(0.6, 0, 0);

    public static final BooleanSupplier CHOREO_AUTO_MIRROR_PATHS =
        new BooleanSupplier() {
          @Override
          public boolean getAsBoolean() {
            return Robot.state.getAlliance() == Alliance.Red;
          }
        };
  }

  public static final class CONTROLLER {
    public static final int DRIVE_CONTROLLER_PORT = 1;
    public static final double DRIVE_CONTROLLER_DEADBAND = 0.01;
    public static final int CODRIVER_CONTROLLER_PORT = 0;
    public static final double CODRIVE_CONTROLLER_DEADBAND = 0.025;
    public static final double SWERVE_TRANSLATIONAL_DEADBAND = 0.0;
    public static final double SWERVE_ROTATIONAL_DEADBAND = 0.0;
    public static final double DRIVE_RUMBLE_INTENSITY = .5;
    public static final double CODRIVE_RUMBLE_INTENSITY = .5;
    public static final double DRIVE_RUMBLE_SECONDS = 2;
    public static final double CODRIVE_RUMBLE_SECONDS = 2;
  }

  public static final class SHOOTER {
    public static final double SOURCE_INTAKE_RPM = -1500;
    public static final double FEED_END_AFFECTOR_RPM = 1500;
    public static final double DEFAULT_STOWED_RPMS = 2000;

    public static final double SHOOTER_AXIS_STEP_INTERVAL = 0.1;

    public static final double SHOOTER_AXIS_MAX_SPEED = 0.8;

    public static final IdleMode IDLE_MODE = IdleMode.kCoast;

    public static final boolean TOP_MOTOR_INVERTED = false;
    public static final boolean BOTTOM_MOTOR_INVERTED = true;

    public static final int TOP_MOTOR_STALL_LIMIT_AMPS = 40;
    public static final int TOP_MOTOR_FREE_LIMIT_AMPS = 40;

    public static final int BOTTOM_MOTOR_STALL_LIMIT_AMPS = 40;
    public static final int BOTTOM_MOTOR_FREE_LIMIT_AMPS = 40;

    public static final double RAMP_RATE = .25;

    public static final double TOP_MOTOR_P = 0.0;
    public static final double TOP_MOTOR_I = 0.0;
    public static final double TOP_MOTOR_D = 0.0;
    public static final double TOP_MOTOR_FF = 0.00018;

    public static final double BOTTOM_MOTOR_P = 0.0;
    public static final double BOTTOM_MOTOR_I = 0.0;
    public static final double BOTTOM_MOTOR_D = 0.0;
    public static final double BOTTOM_MOTOR_FF = 0.0002;

    public static final double RPM_TOLERANCE = 100;
  }

  public static final class INTAKE {
    public static final double AXIS_MAX_SPEED = 0.8;

    public static final double FEED_TO_SHOOTER_TIMEOUT = 0;
    public static final double FLOOR_INTAKE_REVERSE_TIMEOUT = 0.1;

    public static final double PICKUP_SPEED_PERCENT_OUTPUT = .75;
    public static final double SLOW_PICKUP_SPEED_PERCENT_OUTPUT = .1;
    public static final double REVERSE_FEED_SPEED_PERCENT_OUTPUT = -0.2;
    public static final double FEED_SPEED_PERCENT_OUTPUT = 0.75;

    public static final IdleMode IDLE_MODE = IdleMode.kBrake;

    public static final boolean TOP_MOTOR_INVERTED = false;
    public static final boolean BOTTOM_MOTOR_INVERTED = false;

    public static final int TOP_MOTOR_STALL_LIMIT_AMPS = 60;
    public static final int TOP_MOTOR_FREE_LIMIT_AMPS = 60;

    public static final int BOTTOM_MOTOR_STALL_LIMIT_AMPS = 60;
    public static final int BOTTOM_MOTOR_FREE_LIMIT_AMPS = 60;
  }

  public static final class PIVOT {
    public static final double MAX_PIVOT_ANGLE = 70; // TODO TUNE
    public static final double MIN_PIVOT_ANGLE = 17.5;

    public static final double END_AFFECTOR_PRELOAD_ANGLE = 68;
    public static final double INTAKE_FROM_SOURCE_ANGLE = 55;
    public static final double DEFAULT_PIVOT_ANGLE = 45; // angle of intake

    public static final boolean MOTOR_INVERTED = true;
    public static final boolean ENCODER_INVERTED = false;

    public static final IdleMode IDLE_MODE = IdleMode.kBrake;

    public static final int MOTOR_STALL_LIMIT_AMPS = 40;
    public static final int MOTOR_FREE_LIMIT_AMPS = 40;

    public static final double POSITION_ALLOWED_ERROR = 5;
    public static final boolean POSITION_PID_WRAPPING_ENABLED = false;

    public static final double AXIS_MAX_SPEED = 0.25;

    public static final double ENCODER_POSITION_CONVERSION_FACTOR = 360;
    public static final double ENCODER_VELOCITY_CONVERSION_FACTOR = 1;
    public static final double ENCODER_ZERO_OFFSET = 201.0478306 - 10;

    // Closed loop - lines with comments were used for arm rotation 2023 and we will
    // probably need for this
    public static final double PIVOT_P = 0.0195;
    public static final double PIVOT_I = 0;
    public static final double PIVOT_D = 0;
    public static final double PIVOT_FF = 0.00045; // Barely moving: .000465

    public static final String PREFERENCES_ZERO_OFFSET_KEY = "PivotZeroOffset";
    public static final double ZERO_SPEED = 0.1;
    public static final double ZERO_SPEED_STOP_TOLERANCE = 0.015;
    public static final double ZERO_SPEED_INITIAL_SECONDS = 0.15;
  }

  public static final class CLIMBER {
    public static final boolean RIGHT_MOTOR_INVERTED = false;
    public static final boolean LEFT_MOTOR_INVERTED = true;

    public static final int MOTOR_FREE_LIMIT_AMPS = 40; // TODO: Tune climber amp limits
    public static final int MOTOR_STALL_LIMIT_AMPS = 40;

    public static final int ZERO_MOTOR_FREE_LIMIT_AMPS = 20; // TODO: Tune climber amp limits
    public static final int ZERO_MOTOR_STALL_LIMIT_AMPS = 20;

    public static final double ZERO_SPEED = 0.1;
    public static final double ZERO_SPEED_STOP_TOLERANCE = 50.0;
    public static final double ZERO_SPEED_INITIAL_SECONDS = 0.1;
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

    public static final double MOTOR_PID_P = 0.003;
    public static final double MOTOR_PID_I = 0;
    public static final double MOTOR_PID_D = 0;
    public static final double MOTOR_PID_FF = 0.000005;

    public static final int SMART_MOTION_MAX_VEL_RPM = 6000;
    public static final int SMART_MOTION_MIN_VEL_RPM = 0;
    public static final int SMART_MOTION_MAX_ACC_RPM = 48000;
    public static final double SMART_MOTION_ALLOWED_ERROR = 0.1;

    public static final double AXIS_MAX_SPEED = 0.5;

    public static final double ZERO_SPEED = -0.1;
    public static final double ZERO_SPEED_STOP_TOLERANCE = 0.01;
    public static final double ZERO_SPEED_INITIAL_SECONDS = 0.01;

    public static final double HOME_ROTATIONS = 0.0;
    public static final double NOTE_STOW_ROTATIONS = 1.4;
    public static final double AMP_PREPOSE_ROTATIONS = 3.75; // TODO: TUNE
    public static final double AMP_SCORE_ROTATIONS = 6;
    public static final double TRAP_SCORE_ROTATIONS = 0; // TODO: TUNE
  }

  public static final class LEDS {
    public static final int STRIP_LENGTH = 30;

    public static final Color GREEN = new Color(0, 255, 0);
    public static final Color RED = new Color(255, 0, 0);
    public static final Color YELLOW = new Color(255, 255, 0);
    public static final Color ORANGE = new Color(255, 95, 21);
  }

  public static final class SCORING {
    public static final double SECONDS_PRELOAD_NOTE = 1;

    public static final double SECONDS_AMP_SCORE = 1;
  }

  public static final class PHOTON_VISION {
    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT =
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    public static final PoseStrategy POSE_STRATEGY = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

    public static final double MAX_ANGLE = 45;

    public static final String LOST_CONNECTION_ERROR_MESSAGE =
        "----------\nPHOTON VISION HAS LOST CONNECTION!\nVISION RESULTS WILL NOT BE UPDATED!\n----------";
    public static final String CONNECTION_REGAINED_NOFICATION_MESSAGE =
        "**********\nPhoton Vision has regained connection!\nVision results will now be updated.\n**********";
  }

  public static final class SHOOTER_PHOTON_CAMERA {
    public static final String NAME = "shooter camera";

    public static final int APRIL_TAG_PIPELINE = 0;
    public static final int NEURAL_NETWORK_PIPELINE = 1;
    public static final int POSE_ESTIMATION_PIPELINE = 2;

    public static final int DEFAULT_PIPELINE = 2;

    public static final double HEAD_ON_TOLERANCE = 0;

    public static final double LENS_BEHIND_ROBOT_ORIGIN_INCHES = 6.01824;
    public static final double LENS_TO_RIGHT_OF_ROBOT_ORIGIN_INCHES = 0;
    public static final double LENS_HEIGHT_FROM_ROBOT_ORIGIN_INCHES = 5.38192;
    public static final double LENS_ANGLE_TILTED_UP_DEGREES = 35;
    public static final Transform3d ROBOT_TO_CAMERA_TRANSFORM =
        new Transform3d(
            new Translation3d(
                -Units.inchesToMeters(LENS_BEHIND_ROBOT_ORIGIN_INCHES),
                -Units.inchesToMeters(LENS_TO_RIGHT_OF_ROBOT_ORIGIN_INCHES),
                Units.inchesToMeters(LENS_HEIGHT_FROM_ROBOT_ORIGIN_INCHES)),
            new Rotation3d(0, LENS_ANGLE_TILTED_UP_DEGREES, 0));
    public static final int SPEAKER_TARGET_TIMEOUT_MS = 60;
    public static final int AMP_TARGET_TIMEOUT_MS = 60;
    public static final int STAGE_TARGET_TIMEOUT_MS = 60;
  }

  public static final class INTAKE_PHOTON_CAMERA {
    public static final String NAME = "intake camera";

    public static final int APRIL_TAG_PIPELINE = 0;
    public static final int NEURAL_NETWORK_PIPELINE = 1;
    public static final int POSE_ESTIMATION_PIPELINE = 2;

    public static final int DEFAULT_PIPELINE = 2;

    public static final double HEAD_ON_TOLERANCE = 0;

    public static final double LENS_IN_FRONT_OF_ROBOT_ORIGIN_INCHES = 5.15138;
    public static final double LENS_TO_RIGHT_OF_ROBOT_ORIGIN_INCHES = 0;
    public static final double LENS_HEIGHT_FROM_ROBOT_ORIGIN_INCHES = 23.10292;
    public static final double LENS_ANGLE_TILTED_DOWN_DEGREES = 20;
    public static final Transform3d ROBOT_TO_CAMERA_TRANSFORM =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(LENS_IN_FRONT_OF_ROBOT_ORIGIN_INCHES),
                -Units.inchesToMeters(LENS_TO_RIGHT_OF_ROBOT_ORIGIN_INCHES),
                Units.inchesToMeters(LENS_HEIGHT_FROM_ROBOT_ORIGIN_INCHES)),
            new Rotation3d(0, -LENS_ANGLE_TILTED_DOWN_DEGREES, 0));
    public static final int NOTE_TARGET_TIMEOUT_MS = 80;
  }

  public static final class APRIL_TAG_IDS {

    public static final int BLUE_SOURCE_RIGHT = 1;
    public static final int BLUE_SOURCE_LEFT = 2;

    public static final int RED_SPEAKER_OFFSET = 3;
    public static final int RED_SPEAKER_CENTER = 4;

    public static final int RED_AMP = 5;
    public static final int BLUE_AMP = 6;

    public static final int BLUE_SPEAKER_CENTER = 7;
    public static final int BLUE_SPEAKER_OFFSET = 8;

    public static final int RED_SOURCE_RIGHT = 9;
    public static final int RED_SOURCE_LEFT = 10;

    // stage tags are oriented from looking at the stage from the center line.
    public static final int RED_STAGE_RIGHT = 11;
    public static final int RED_STAGE_LEFT = 12;
    public static final int RED_STAGE_MIDDLE = 13;

    public static final int BLUE_STAGE_MIDDLE = 14;
    public static final int BLUE_STAGE_RIGHT = 15;
    public static final int BLUE_STAGE_LEFT = 16;

    public static final int[] BLUE_STAGE_TAGS =
        new int[] {BLUE_STAGE_LEFT, BLUE_STAGE_MIDDLE, BLUE_STAGE_RIGHT};
    public static final int[] RED_STAGE_TAGS =
        new int[] {RED_STAGE_LEFT, RED_STAGE_MIDDLE, RED_STAGE_RIGHT};

    public static final int[] RIGHT_STAGE_TAGS = new int[] {12, 16};
    public static final int[] LEFT_STAGE_TAGS = new int[] {11, 15};
    public static final int[] CENTER_STAGE_TAGS = new int[] {13, 14};

    public static final int[] STAGE_TAGS =
        new int[] {
          BLUE_STAGE_LEFT,
          BLUE_STAGE_MIDDLE,
          BLUE_STAGE_RIGHT,
          RED_STAGE_LEFT,
          RED_STAGE_MIDDLE,
          RED_STAGE_RIGHT
        };
    public static final int[] SPEAKER_CENTER_TAGS =
        new int[] {BLUE_SPEAKER_CENTER, RED_SPEAKER_CENTER};
    public static final int[] AMP_TAGS = new int[] {BLUE_AMP, RED_AMP};
  }
}
