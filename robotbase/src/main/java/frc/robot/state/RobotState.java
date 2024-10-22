package frc.robot.state;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.subsystems.PhotonVisionCamera;

public class RobotState {
  public static enum State {
    INIT, // Robot is initializing
    DISABLED, // Robot is in disabled mode and no alliance is selected
  }

  public static enum DriveControlState {
    FIELD_RELATIVE, // Manual control of the robot is field relative
    ROBOT_RELATIVE, // Manual control of the robot is robot centric
    TARGET_LOCK
  }

  private Alliance m_alliance;
  private State m_currentState;
  private DriveControlState m_currentDriveControlState;
  private boolean m_isClimbing;
  private PhotonVisionCamera m_targetLockCamera;
  private boolean m_isAdjusting;

  public RobotState() {
    m_alliance = null;
    m_currentState = State.INIT;
    m_currentDriveControlState = DriveControlState.FIELD_RELATIVE;
    m_isClimbing = false;
    m_targetLockCamera = Robot.shooterCam;
    m_isAdjusting = false;
  }

  public boolean isRobot(State state) {
    return m_currentState == state;
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
    m_currentDriveControlState = driveControlState;
  }

  public void setAlliance(Alliance alliance) {
    m_alliance = alliance;
  }

  private void setCurrentState(State newState) {
    m_currentState = newState;
  }

  public void setTargetLockCamera(PhotonVisionCamera targetLockCamera) {
    m_targetLockCamera = targetLockCamera;
  }

  public PhotonVisionCamera getTargetLockCamera() {
    return m_targetLockCamera;
  }

  public boolean isClimbing() {
    return m_isClimbing;
  }

  public void setClimbing(boolean isClimbing) {
    m_isClimbing = isClimbing;
  }

  public boolean isAdjusting() {
    return m_isAdjusting;
  }

  public void setAdjusting(boolean isAdjusting) {
    m_isAdjusting = isAdjusting;
  }
}
