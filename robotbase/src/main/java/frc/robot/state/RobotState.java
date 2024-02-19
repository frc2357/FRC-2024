package frc.robot.state;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RobotState {
  public static enum State {
    INIT, // Robot is initializing
    DISABLED, // Robot is in disabled mode and no alliance is selected
    EMPTY, // Robot has nothing in it
    NOTE_STOWED, // Robot has a note in the intake
    NOTE_PRELOAD, // Robot has a note preloaded into the end affector
    AMP_PRE_POSE, // Robot is in position to score in the amp
  };

  public static enum DriveControlState {
    FIELD_RELATIVE, // Manual control of the robot is field relative
    ROBOT_RELATIVE, // Manual control of the robot is robot centric
    TARGET_LOCK
  }

  private Alliance m_alliance;
  private State m_currentState;
  private DriveControlState m_currentDriveControlState;
  private boolean m_zeroed;

  public RobotState() {
    m_alliance = null;
    m_currentState = State.INIT;
    m_currentDriveControlState = DriveControlState.FIELD_RELATIVE;
    m_zeroed = false;
  }

  public Alliance getAlliance() {
    return m_alliance;
  }

  public State getState() {
    return m_currentState;
  }

  public boolean isZeroed() {
    return m_zeroed;
  }

  public void setRobotZeroed(boolean zeroed) {
    setZeroed(zeroed);
  }

  public boolean isInState(State state) {
    return m_currentState == state;
  }

  public void onDriverAllianceSelect(Alliance alliance) {
    setAlliance(alliance);
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

  private void setAlliance(Alliance alliance) {
    m_alliance = alliance;
  }

  private void setZeroed(boolean zeroed) {
    m_zeroed = zeroed;
  }

  private void setCurrentState(State newState) {
    m_currentState = newState;
  }

  private void setCurrentDriveControlState(DriveControlState driveControlState) {
    m_currentDriveControlState = driveControlState;
  }

  public boolean hasNote() {
    return m_currentState == State.AMP_PRE_POSE
        || m_currentState == State.NOTE_STOWED
        || m_currentState == State.NOTE_PRELOAD;
  }
}
