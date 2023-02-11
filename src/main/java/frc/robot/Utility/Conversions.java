package frc.robot.Utility;

public class Conversions {
    
    public static double NeoToMeters(double positionCounts, double circumference, double gearRatio) {
        return positionCounts * (circumference / (gearRatio * 42.0));
    }
}
