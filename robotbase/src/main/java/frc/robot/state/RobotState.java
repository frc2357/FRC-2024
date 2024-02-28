package frc.robot.state;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.subsystems.PhotonVisionCamera;

public class RobotState {
  public static enum State {
    INIT, // Robot is initializing
    DISABLED, // Robot is in disabled mode and no alliance is selected
  }

  public static enum NoteState {
    EMPTY, // Robot has nothing in it
    NOTE_IN_INTAKE,
    NOTE_PAST_BEAM_BREAK,
    NOTE_STOWED,
    END_AFFECTOR_PRELOAD
  };

  public static enum DriveControlState {
    FIELD_RELATIVE, // Manual control of the robot is field relative
    ROBOT_RELATIVE, // Manual control of the robot is robot centric
    TARGET_LOCK
  }

  public static enum AutoClimbState {
  }

  public static enum ShootingState {
    VISION_TARGETING,
    MANUAL
  }

  private Alliance m_alliance;
  private State m_currentState;
  private NoteState m_currentNoteState;
  private ShootingState m_currentShootingState;
  private DriveControlState m_currentDriveControlState;
  private PhotonVisionCamera m_targetLockCamera;

  public RobotState() {
    m_alliance = null;
    m_currentState = State.INIT;
    m_currentNoteState = NoteState.EMPTY;
    m_currentDriveControlState = DriveControlState.FIELD_RELATIVE;
    m_targetLockCamera = Robot.shooterCam;
  }

  public void setNoteState(NoteState state) {
    m_currentNoteState = state;
  }

  public boolean isNote(NoteState state) {
    return m_currentNoteState == state;
  }

  public void setShootingState(ShootingState state) {
    m_currentShootingState = state;
  }

  public boolean isShooting(ShootingState state) {
    return m_currentShootingState == state;
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
