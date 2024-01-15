package frc.robot.state;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RobotState {
    private static final RobotState s_instance = new RobotState();

    public static enum State {
        ROBOT_INIT,                    // Robot is initializing
        ROBOT_DISABLED,                // Robot is in disabled mode and no alliance is selected
    };

    public static enum DriveControlState {
        FIELD_RELATIVE, // Manual control of the robot is field relative
        ROBOT_CENTRIC // Manual control of the robot is robot centric
    }

    public static Alliance getAlliance() {
        return s_instance.m_alliance;
    }

    public static State getState() {
        return s_instance.m_currentState;
    }

    public static boolean isZeroed() {
        return s_instance.m_zeroed;
    }

    public static void setRobotZeroed(boolean zeroed) {
        s_instance.setZeroed(zeroed);
    }

    public static boolean isInState(State state) {
        return s_instance.m_currentState == state;
    }

    public static void onDriverAllianceSelect(Alliance alliance) {
        s_instance.setAlliance(alliance);
    }

    public static void robotInit() {
        setState(State.ROBOT_INIT);
    }

    public static void disabledInit() {
        setState(State.ROBOT_DISABLED);
    }

    public static void setState(State newState) {
        s_instance.setCurrentState(newState);
    }

    public static boolean isFieldRelative() {
        return s_instance.m_currentDriveControlState == DriveControlState.FIELD_RELATIVE;
    }

    public static DriveControlState getDriveControlState() {
        return s_instance.m_currentDriveControlState;
    }

    public static void setDriveControlState(DriveControlState driveControlState) {
        s_instance.setCurrentDriveControlState(driveControlState);
    }

    private Alliance m_alliance;
    private State m_currentState;
    private DriveControlState m_currentDriveControlState;
    private boolean m_zeroed;

    private RobotState() {
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