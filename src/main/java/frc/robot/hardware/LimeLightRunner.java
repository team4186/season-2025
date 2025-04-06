package frc.robot.hardware;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import static frc.robot.LimelightHelpers.setCameraPose_RobotSpace;
import static java.lang.Math.tan;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class LimeLightRunner extends SubsystemBase {

    private final NetworkTable tableTag;
    private double[] botPoseTargetSpace;
    private double[] botPose;
    private int TagID;
    private final double[] emptyArray;

    public LimeLightRunner() {
        this.emptyArray = new double[] {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        // TODO: rename the cheese name, and finish measuring and update this stuff.
        setCameraPose_RobotSpace("limelight",0.32831,-0.08669,0.21574,0.0, 12.5, 0.0);
        this.tableTag = NetworkTableInstance.getDefault().getTable("limelight");
        this.botPose = emptyArray;
    }


    @Override
    public void periodic() {

        this.botPoseTargetSpace = tableTag.getEntry("botpose_targetspace").getDoubleArray(emptyArray);
        this.botPose = tableTag.getEntry("botpose").getDoubleArray(emptyArray);
        this.TagID = (int) tableTag.getEntry("tid").getInteger(-1);

        SmartDashboard.putBoolean("Limelight_HasTargetTag", hasTargetTag());
        SmartDashboard.putNumber("Limelight_Horizontal_Offset", getXOffset());
        SmartDashboard.putNumber("Limelight_Target_Distance", getZOffset());
        SmartDashboard.putNumber("Limelight_Angle", getThetaOffset());
        SmartDashboard.putNumber("Limelight_%_of_Image", getTagArea());

        SmartDashboard.putNumber("Limelight_X_Offset", getTagXOffset());
        SmartDashboard.putNumber("Limelight_Y_Offset", getTagYOffset());
        SmartDashboard.putNumber("Limelight_Z_Offset", getTagZOffset());
        SmartDashboard.putNumber("TagID", getTagID());
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
            /*
            [0]	use the LED Mode set in the current pipeline
            [1]	force off
            [2]	force blink
            [3]	force on
            */
    }


    public boolean hasTargetTag() {
//        return tableTag.getEntry("tv").getDouble(0.0) > 0.0;
        return tableTag.getEntry("tv").getInteger(0) > 0;

    }


    public double getTagXOffset() {
        return tableTag.getEntry("tx").getDouble(0.0);
    }


    public double getTagYOffset() {
        return tableTag.getEntry("ty").getDouble(0.0);
    }


    public double getTagZOffset() {
        return tableTag.getEntry("tz").getDouble(0.0);
    }


    public double getTagArea() {
        return tableTag.getEntry("ta").getDouble(0.0);
    }




    public double getDistance() {
        // TODO: update after limelight is mounted for accurate reading
        double mountedAngle = 12.5;
        double angleInRadians = Math.toRadians((mountedAngle + getTagYOffset()));
        double distance = 33.75 / tan(angleInRadians); // TODO: update mountedAngle and distance offset after mounting

        if ( hasTargetTag() ) {
            return distance;
        }

        return Double.NaN;
    }

    public int getTagID() {
        return TagID;
    }

    public double getXOffset() {
        // tx
        return botPoseTargetSpace[0];
    }


    public double getZOffset() {
        // ty
        return botPoseTargetSpace[2];
    } 


    public double getThetaOffset() {
        // yaw
        return botPoseTargetSpace[4];
    }

//    public Pose2d getVisionPose() {
//        return new Pose2d(botPose[0], botPose[1], Rotation2d.fromDegrees(botPose[5]));
//    }
}
