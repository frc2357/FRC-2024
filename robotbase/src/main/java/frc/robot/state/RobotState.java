package frc.robot.state;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.subsystems.PhotonVisionCamera;

public class RobotState {
  public static enum State {
    INIT, // Robot is initializing
    DISABLED, // Robot is in disabled mode and no alliance is selected
  }

  public static enum IntakeState {
    EMPTY, // Robot has nothing in it
    NOTE_IN_INTAKE,
    NOTE_PAST_BEAM_BREAK,
    NOTE_STOWED,
  };

  public static enum DriveControlState {
    FIELD_RELATIVE, // Manual control of the robot is field relative
    ROBOT_RELATIVE, // Manual control of the robot is robot centric
    TARGET_LOCK
  }

  public static enum AutoClimbState {}

  public static enum ShootingState {}

  public static enum AmpScoreState {
    EMPTY,
    AMP_PRELOAD,
    AMP_PREPOSE
  }

  private Alliance m_alliance;
  private State m_currentState;
  private IntakeState m_currentIntakeState;
  private AmpScoreState m_currentAmpScoreState;
  private DriveControlState m_currentDriveControlState;
  private PhotonVisionCamera m_targetLockCamera;

  public RobotState() {
    m_alliance = null;
    m_currentState = State.INIT;
    m_currentIntakeState = IntakeState.EMPTY;
    m_currentAmpScoreState = AmpScoreState.EMPTY;
    m_currentDriveControlState = DriveControlState.FIELD_RELATIVE;
    m_targetLockCamera = Robot.shooterCam;
  }

  public void setIntakeState(IntakeState state) {
    m_currentIntakeState = state;
  }

  public boolean isIntake(IntakeState state) {
    return m_currentIntakeState == state;
  }

  public void setAmpScoreState(AmpScoreState state) {
    m_currentAmpScoreState = state;
  }

  public boolean isAmpScore(AmpScoreState state) {
    return m_currentAmpScoreState == state;
  }

  public Alliance getAlliance() {
    return m_alliance;
  }

  public void robotInit() {
    setState(State.INIT);
  }

  public void disabledInit() {
    setState(State.DISABLED);
  }

  public void setState(State newState) {
    setCurrentState(newState);
  }

  public boolean isFieldRelative() {
    return m_currentDriveControlState == DriveControlState.FIELD_RELATIVE;
  }

  public boolean isTargetLock() {
    return getDriveControlState() == DriveControlState.TARGET_LOCK;
  }

  public DriveControlState getDriveControlState() {
    return m_currentDriveControlState;
  }

  public void setDriveControlState(DriveControlState driveControlState) {
    setCurrentDriveControlState(driveControlState);
  }

  public void setAlliance(Alliance alliance) {
    m_alliance = alliance;
  }

  private void setCurrentState(State newState) {
    m_currentState = newState;
  }

  private void setCurrentDriveControlState(DriveControlState driveControlState) {
    m_currentDriveControlState = driveControlState;
  }

  // public boolean hasNote() {
  // return m_currentState == State.AMP_PRE_POSE
  // || m_currentState == State.NOTE_STOWED
  // || m_currentState == State.NOTE_PRELOAD;
  // }

  public void setTargetLockCamera(PhotonVisionCamera targetLockCamera) {
    m_targetLockCamera = targetLockCamera;
  }

  public PhotonVisionCamera getTargetLockCamera() {
    return m_targetLockCamera;
  }
}
