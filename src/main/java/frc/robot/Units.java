package frc.robot;

public final class Units {
    
    private Units() {}

    public static double FeetToMeters(double feet) {
        return 0.3048 * feet;
    }

    public static double TicksToMeters(double encoderTicks, double wheelDiameter, String motorType) {
    	if (motorType == "NEO550" || motorType == "NEOVORTEX" || motorType == "NEO") {
        	return (42.0/wheelDiameter) * encoderTicks;
        } else {
		return 0.0;
	}
    }
    
    public static double TicksToDegrees(double encoderTicks, String motorType) {
	if (motorType == "NEO550" || motorType == "NEOVORTEX" || motorType == "NEO") {
		return (encoderTicks/42.0) * 360;
	} else {
		return 0.0;
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
