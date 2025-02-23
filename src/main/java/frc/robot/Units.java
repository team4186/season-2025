package frc.robot;

// JLR - nit: Does the class have what we need already -> import edu.wpi.first.math.util.Units
// Utility Class
// Ziyao - I'm not sure if they have the TicksToMeters or TicksToDegrees methods.
import java.lang.Math;
public final class Units {
    
    private Units() {}

    public static double FeetToMeters(double feet) {
        return 0.3048 * feet;
    }

    public static double TicksToMeters(double encoderTicks, double wheelDiameter, String motorType) {
    	if (motorType == "NEO550" || motorType == "NEOVORTEX" || motorType == "NEO") {
        	return (encoderTicks/42.0) * (Math.PI * wheelDiameter);
        } else {
		throw new IllegalArgumentException();
		}
    }

	public static double TicksToMeters(double encoderTicks, double wheelDiameter, double gearRatio) {
		return (encoderTicks/(42.0 * gearRatio)) * (Math.PI * wheelDiameter);
	}

	public static double TicksToDegrees(double encoderTicks, double gearRatio) {
		return ((encoderTicks/ gearRatio) * 360) % 360;
	}

    public static double TicksToDegrees(double encoderTicks, String motorType) {
		if (motorType == "NEO550" || motorType == "NEOVORTEX" || motorType == "NEO") {
			return ((encoderTicks/42.0) * 360) % 360;
		} else {
			throw new IllegalArgumentException();
		}
    }
    
    public static double InchesToCentimeters(double inches) {
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

}
