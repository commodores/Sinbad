package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * 
 * Add your docs here.
 */
public class LimeLight extends SubsystemBase {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

    private final NetworkTable m_limelightTable;
    private boolean isTargetVisible;
    private double xAngle, yAngle, distance;

    public LimeLight() {
        m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public void periodic() {
        isTargetVisible = m_limelightTable.getEntry("tv").getDouble(0) == 1;
        xAngle = m_limelightTable.getEntry("tx").getDouble(0);
        yAngle = m_limelightTable.getEntry("ty").getDouble(0);
        SmartDashboard.putNumber("X Angle", xAngle);
        SmartDashboard.putNumber("Y Angle", yAngle);
    }

    public boolean isTargetVisible() {
        return isTargetVisible;
    }

    public double getXAngle() {
        return xAngle;
    }

    public double getYAngle() {
        return yAngle;
    }

    public double getDistance() {
        return distance;
    }

    public void turnOnLight() {
        m_limelightTable.getEntry("ledmode").setNumber(1);
    }

    public void turnOffLight() {
        m_limelightTable.getEntry("ledmode").setNumber(0);
    }

}