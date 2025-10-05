package frc.robot.hardware;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import java.lang.Math;



public class LimeLightRunner extends SubsystemBase {

  private final NetworkTable tableTag;
  private double[] botPoseTargetSpace;
  private double[] botPose;
  private int TagID;
  private final double[] emptyArray;
  private boolean useMegaTag2;
  private double[] LLHelpersBotPoseTargetSpace;


  public LimeLightRunner(boolean useMegaTag2) {
    this.emptyArray = new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // TODO: rename the cheese name, and finish measuring and update this stuff.
    LimelightHelpers.setCameraPose_RobotSpace("limelight",0.32831,-0.08669,0.21574,0.0, 12.5, 0.0);
    this.tableTag = NetworkTableInstance.getDefault().getTable("limelight");
    this.botPose = emptyArray;
    this.useMegaTag2 = useMegaTag2;
    this.LLHelpersBotPoseTargetSpace = LimelightHelpers.getBotPose_TargetSpace("limelight");
  }


  @Override
  public void periodic() {
    this.botPoseTargetSpace = tableTag.getEntry("botpose_targetspace").getDoubleArray(emptyArray);
    this.botPose = tableTag.getEntry("botpose").getDoubleArray(emptyArray);
    this.TagID = (int) tableTag.getEntry("tid").getInteger(-1);
    this.LLHelpersBotPoseTargetSpace = LimelightHelpers.getBotPose_TargetSpace("limelight");

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
      res = 1.0;
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
    double distance = 33.75 / Math.tan(angleInRadians); // TODO: update mountedAngle and distance offset after mounting

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


  //TODO: Limelight localization using Megatag2 from limelight docs
  //TODO: replace varible names with already existing stuff, poseEstimator is new, needs to create poseEstimator object
  //TODO: m_gyro should be just the gyro values. need to import some classes. m_frontLeft and similar are the swervemodules
  //TODO: fix all the red stuff lol
  public Pose2d getVisionPose() {
    return new Pose2d(botPose[0], botPose[1], Rotation2d.fromDegrees(botPose[5]));
  }


  /**
   * Everything below is the same as the above but uses limelighthelpers,
   * which should be less error prone compared to using the networktables API.
   */

  /** botpose_targetspace */
  public double getAngleOffset() {
    return LLHelpersBotPoseTargetSpace[4];
  }


  /** Horizontal offset from crosshair */
  public double getHelperXOffset() {
    return LLHelpersBotPoseTargetSpace[0];
  }


  /** Distance from camera to April tag */
  public double getHelperZOffset() {
    return LLHelpersBotPoseTargetSpace[2];
  }


  /** has target tag or not */
  public boolean getTV(String limelightName) {
    return LimelightHelpers.getTV(limelightName);
  }


//    public void updateEstamateOdometry() {
//        m_poseEstimator.update(
//                .getRotation2d(),
//                new SwerveModulePosition[]{
//                        m_frontLeft.getPosition(),
//                        m_frontRight.getPosition(),
//                        m_backLeft.getPosition(),
//                        m_backRight.getPosition()
//                });
//
//        if (useMegaTag2 == true) {
//            LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
//            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
//            if (Math.abs(m_gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
//            {
//                doRejectUpdate = true;
//            }
//            if (mt2.tagCount == 0) {
//                doRejectUpdate = true;
//            }
//            if (!doRejectUpdate) {
//                m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
//                m_poseEstimator.addVisionMeasurement(
//                        mt2.pose,
//                        mt2.timestampSeconds);
//            }
//        }
//    }
}
