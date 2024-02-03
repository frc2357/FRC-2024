package frc.robot.state;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RobotState {
  public static enum State {
    ROBOT_INIT, // Robot is initializing
    ROBOT_DISABLED, // Robot is in disabled mode and no alliance is selected
  };

  public static enum DriveControlState {
    FIELD_RELATIVE, // Manual control of the robot is field relative
    ROBOT_CENTRIC, // Manual control of the robot is robot centric
    TARGET_LOCK
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
    setState(State.ROBOT_INIT);
  }

  public void disabledInit() {
    setState(State.ROBOT_DISABLED);
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

  private Alliance m_alliance;
  private State m_currentState;
  private DriveControlState m_currentDriveControlState;
  private boolean m_zeroed;

  public RobotState() {
    m_alliance = null;
    m_currentState = State.ROBOT_INIT;
    m_currentDriveControlState = DriveControlState.FIELD_RELATIVE;
    m_zeroed = false;
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
}
