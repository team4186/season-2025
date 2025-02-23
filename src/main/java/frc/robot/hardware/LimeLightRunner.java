//package frc.robot.hardware;
//
//import edu.wpi.first.math.util.Units;
//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//
//public class LimeLightRunner {
//
//    private final NetworkTable tableRing;
//    private final NetworkTable tableTag;
//
//    // Distance to April Tag
//    private double distance;
//
//    private double ringXOffset;
//    private double ringYOffset;
//    private double ringArea;
//
//    private double tagXOffset;
//    private double tagYOffset;
//    private double tagArea;
//
//    public LimeLightRunner(NetworkTable tableRing, NetworkTable tableTag){
//        this.tableRing = tableRing;
//        this.tableTag = tableTag;
//
//        ringXOffset = tableRing.getEntry("tx").getDouble(0.0);
//        ringYOffset = tableRing.getEntry("tx").getDouble(0.0);
//        ringArea = tableRing.getEntry("tx").getDouble(0.0);
//        tagXOffset = tableRing.getEntry("tx").getDouble(0.0);
//        tagYOffset = tableRing.getEntry("tx").getDouble(0.0);
//        tagArea = tableRing.getEntry("tx").getDouble(0.0);
//
//    }
//
//    public void periodic() {
////        SmartDashboard.putBoolean("Has Target Ring?", hasTargetRing);
////        SmartDashboard.putBoolean("Has Target Tag?", hasTargetTag);
////        SmartDashboard.putNumber("X Offset", tagXOffset);
////        SmartDashboard.putNumber("Y Offset", tagYOffset);
////        SmartDashboard.putNumber("% of Image", tagArea);
////        SmartDashboard.putNumber("Distance", Units.metersToInches(distance));
//    }
//
//
//    public void setLight(boolean mode){
//        final double res;
//        if (mode) { res = 3.0; } else { res = 1.0; }
//        tableRing.getEntry("ledMode").setValue( res );
//    }
//
//
//    // TODO: Measure offsets of camera to <front|center|end_effector>
//    public int lookupTableRound(double distanceToTag) {
////        int res = Math.round();
////        Units.
//        return 0;
//    }
//
//    public double getRingXOffset() {
//        return ringXOffset;
//    }
//
//    public double getRingYOffset() {
//        return ringYOffset;
//    }
//
//    public double getRingArea(){
//        return ringArea;
//    }
//
//    public double getTagXOffset(){
//        return tagXOffset;
//    }
//
//    public double getTagYOffset() {
//        return tagYOffset;
//    }
//
//    public double getDistance() {
//        return distance;
//    }
//
//    public double getTagArea() {
//        return tagArea;
//    }
//
////    public boolean getTableRing() { return tableRing; }
//
////    public boolean getTargetTag() {
////        return targetTag;
////    }
//}
