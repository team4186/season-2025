package frc.robot;

public class Units {
    public static double FeetToMeters(double feet, double meters) {
        meters = 0.3048 * feet;
        return meters;
    }

    public static double TicksToMeters(double encoderTicks, String motorType) {
        if (motorType == "NEO550") {
            return encoderTicks *
        }
        //
    }
}
