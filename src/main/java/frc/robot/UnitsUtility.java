package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import java.lang.Math;


public final class UnitsUtility {

  private UnitsUtility() {}


  public static double feetToMeters(double feet) {
    return 0.3048 * feet;
  }


  public static double ticksToMeters(double pos, double wheelDiameter, String motorType) {
    if (motorType == "NEO550" || motorType == "NEOVORTEX" || motorType == "NEO") {
      return (pos/42.0) * (Math.PI * wheelDiameter);
    } else {
      throw new IllegalArgumentException();
    }
  }


  public static double ticksToMeters(double pos, double wheelDiameter, double gearRatio) {
    return (pos / (42.0 * gearRatio)) * (Math.PI * wheelDiameter);
  }


//	public static double ticksToDegrees(double pos, int encoderTicks, double gearRatio) {
//  // return ( (pos / (encoderTicks * gearRatio)) * 360) % 360;
//        return ( (encoderTicks/ gearRatio) * 360) % 360;
//	}


  // TODO: use positive and negative degrees for items that do not have full rotations and need to be aligned to zero
  public static double ticksToDegrees(double encoderTicks, double gearRatio) {
    // return ( (pos / (encoderTicks * gearRatio)) * 360) % 360;
    return ( (encoderTicks/ gearRatio) * 360) % 360;
  }


//    public static double ticksToDegrees(double encoderTicks, String motorType) {
//		if (motorType == "NEO550" || motorType == "NEOVORTEX" || motorType == "NEO") {
//            // return (encoderTicks/8.57) % 360;
//            return (((encoderTicks/42.0) * 360) % 360);
//        } else {
//            throw new IllegalArgumentException();
//        }
//    }


  public static double inchesToCentimeters(double inches) {
    return inches * 2.54;
  }


  public static double convertMetric(double value, MetricConversion fromMagnitude, MetricConversion toMagnitude) {
    return value * toMagnitude.metricConversion/fromMagnitude.metricConversion;
  }


  public enum MetricConversion {
    MILI(1000),
    CENTI(100),
    MICRO(1000000),
    DEFAULT(1),
    NANO(1000000000),
    DECI(10),
    KILO(1/1000),
    GIGA(1/1000000000),
    MEGA(1/1000000),
    HECTA(1/100),
    DEKA(1/10);

    private final double metricConversion;

    MetricConversion(double conversion) {
      this.metricConversion = conversion;
    }
  }


  public static double clampSpeed (double value) {
    return Math.max(-1.0, Math.min(1.0, value));
  }


  public static double clampValue(double value, double max, double min) {
    return Math.max(min, Math.min(max, value));
  }


  /** Safe-Handling Beam Break */
  public static boolean isBeamBroken( DigitalInput beamBreak, boolean defaultValue, String systemName ){
    try {
      return beamBreak.get();
    } catch (IllegalStateException e) {
      String msg = systemName + " Beam break error: " + e;
      System.out.println( msg );
      return defaultValue;
    }
  }
}
