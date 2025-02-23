package frc.robot.hardware;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static java.lang.Math.tan;

public class LimeLightRunner {

    private final NetworkTable tableTag;
    private final double mountedAngle;


    public LimeLightRunner(double mountedAngle) {
        tableTag = NetworkTableInstance.getDefault().getTable("limelight-tag");
        this.mountedAngle = mountedAngle;
    }

    public void periodic() {
        SmartDashboard.putBoolean("Has Target Tag?", hasTargetTag());
        SmartDashboard.putNumber("X Offset", getTagXOffset());
        SmartDashboard.putNumber("Y Offset", getTagYOffset());
        SmartDashboard.putNumber("% of Image", getTagArea());
        SmartDashboard.putNumber("Distance", Units.metersToInches(getDistance()));
    }


    public void setLight(boolean mode) {
        final double res;
        if (mode) {
            res = 3.0;
        } else {
            res = 1.0;
        }
        tableTag.getEntry("ledMode").setValue(res);
    }


    // TODO: Measure offsets of camera to <front|center|end_effector>
    public int lookupTableRound(double distanceToTag) {
//        int res = Math.round();
//        Units.
        return 0;
    }

    public boolean hasTargetTag() {
        return tableTag.getEntry("tv").getDouble(0.0) > 0.0;
    }


    public double getTagXOffset() {
        return tableTag.getEntry("tx").getDouble(0.0);
    }

    public double getTagYOffset() {
        return tableTag.getEntry("ty").getDouble(0.0);
    }

    public double getTagArea() {
        return tableTag.getEntry("ta").getDouble(0.0);
    }

    public double getDistance() {
        double angleInRadians = Math.toRadians((mountedAngle + getTagYOffset()));
        double distance = 33.75 / tan(angleInRadians); // TODO: update mountedAngle and distance offset after mounting

        if (hasTargetTag()) {
            return distance;
        }

        return Double.NaN;
    }
}
