package frc.robot.hardware;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static java.lang.Math.tan;

public class LimeLightRunner extends SubsystemBase{

    private final NetworkTable tableTag;
    private final double[] camPose;
    public LimeLightRunner() {
        this.tableTag = NetworkTableInstance.getDefault().getTable("limelight-tag");
        this.camPose = tableTag.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Has Target Tag?", hasTargetTag());
        SmartDashboard.putNumber("Horizontal Offset", getXOffset());
        SmartDashboard.putNumber("Distance away", getZOffset());
        SmartDashboard.putNumber("Angle", getThetaOffset());
        SmartDashboard.putNumber("% of Image", getTagArea());
        setLight(hasTargetTag());
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
        // TODO: update after limelight is mounted for accurate reading
        double mountedAngle = 12.5;
        double angleInRadians = Math.toRadians((mountedAngle + getTagYOffset()));
        double distance = 33.75 / tan(angleInRadians); // TODO: update mountedAngle and distance offset after mounting

        if (hasTargetTag()) {
            return distance;
        }

        return Double.NaN;
    }

    public double getXOffset() {
        // tx
        return this.camPose[0];
    }

    public double getZOffset() {
        // ty
        return this.camPose[2];
    } 

    public double getThetaOffset() {
        // yaw
        return this.camPose[4];
    }
}
