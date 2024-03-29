package frc.robot.networkTables;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

public class ShooterCurveTuner {
  private SendableChooser<Integer> m_rowChooser;

  public ShooterCurveTuner() {
    m_rowChooser = new SendableChooser<>();

    for (int i = 0; i < Robot.shooterCurve.length; i++) {
      m_rowChooser.addOption("TY Setpoint: " + Robot.shooterCurve[i][0], i);
    }

    m_rowChooser.setDefaultOption("TY Setpoint: " + Robot.shooterCurve[0][0], 0);

    m_rowChooser.onChange((val) -> updateSelectedCurveIndex(val));

    SmartDashboard.putData("Shooter Curve Tuner", m_rowChooser);
  }

  public void updateSelectedCurveIndex(int index) {
    double[] newRow = Robot.shooterCurve[index];
    SmartDashboard.putNumber("Pivot Angle Setpoint", newRow[1]);
    SmartDashboard.putNumber("Top Roller Setpoint", newRow[2]);
    SmartDashboard.putNumber("Yaw Setpoint", newRow[3]);
  }

  public void updateCurveValues() {
    int index = m_rowChooser.getSelected();
    double[] row = Robot.shooterCurve[index];

    double pivotSetpoint = SmartDashboard.getNumber("Pivot Angle Setpoint", row[1]);
    double topShooterSetpoint = SmartDashboard.getNumber("Top Roller Setpoint", row[2]);
    double yawSetpoint = SmartDashboard.getNumber("Yaw Setpoint", row[3]);

    Robot.shooterCurve[index][1] = pivotSetpoint;
    Robot.shooterCurve[index][2] = topShooterSetpoint;
    Robot.shooterCurve[index][3] = yawSetpoint;
  }

  public double[] getSelectedRow() {
    int selected = m_rowChooser.getSelected();
    double[] row = Robot.shooterCurve[selected];
    return row;
  }
}
