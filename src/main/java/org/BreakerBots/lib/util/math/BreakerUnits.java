package frc.robot.BreakerLib.util.math;

public class BreakerUnits {
    public static final double MICROSECONDS_PER_SECOND = 1000000;
    public static final double METERS_PER_SECOND_SQUARED_IN_G = 9.80665;
    public static final double INCHES_PER_SECOND_SQUARED_IN_G = 386.088583;
    public static final double MILLIMETERS_PER_INCH = 25.4;
    public static final double MILLIMETERS_PER_METER = 1000;
    public static final double CENTIMETERS_PER_METER = 100;
    public static final double MILLIMETERS_PER_CENTIMETER = 10;
    public static final double METERS_PER_YARD = 0.9144;
    public static final double INCHES_PER_YARD = 36;
    private static ShortDistanceUnits shortDefaultUnits;
    private static LongDistanceUnits longDefaultUnits;

    

    /**
     * Converts given microseconds to seconds.
     * 
     * @param microseconds The microseconds to convert to seconds.
     * 
     * @return Seconds converted from microseconds.
     */
    public static double microsecondsToSeconds(double microseconds) {
        return microseconds / MICROSECONDS_PER_SECOND;

    }

    /**
     * Converts given seconds to microseconds.
     * 
     * @param seconds The seconds to convert to microseconds.
     * 
     * @return Microseconds converted from seconds.
     */
    public static double secondsToMicroseconds(double seconds) {
        return seconds * MICROSECONDS_PER_SECOND;

    }

    /**
     * Converts given Gs to m/s^2.
     * 
     * @param gForces The Gs to convert to m/s^2.
     * 
     * @return m/s^2 converted from Gs.
     */
    public static double gForceToMetersPerSecondSquared(double gForces) {
        return gForces * METERS_PER_SECOND_SQUARED_IN_G;
    }

    /**
     * Converts given m/s^2 to Gs.
     * 
     * @param metersPerSecondSquared The m/s^2 to convert to Gs.
     * 
     * @return Gs converted from m/s^2.
     */
    public static double metersPerSecondSquaredToGs(double metersPerSecondSquared) {
        return metersPerSecondSquared / METERS_PER_SECOND_SQUARED_IN_G;
    }

    /**
     * Converts given Gs to in/s^2.
     * 
     * @param gForces The Gs to convert to in/s^2.
     * 
     * @return in/s^2 converted from Gs.
     */
    public static double gForceToInchesPerSecondSquared(double gForces) {
        return gForces * INCHES_PER_SECOND_SQUARED_IN_G;
    }

    /**
     * Converts given in/s^2 to Gs.
     * 
     * @param inchesPerSecondSquared The in/s^2 to convert to Gs.
     * 
     * @return Gs converted from in/s^2.
     */
    public static double inchesPerSecondSquaredToGs(double inchesPerSecondSquared) {
        return inchesPerSecondSquared / INCHES_PER_SECOND_SQUARED_IN_G;
    }

    public static double inchesToMeters(double inches) {
        return ((inches * MILLIMETERS_PER_INCH) / MILLIMETERS_PER_METER);
    }

    public static double metersToYards(double meters) {
        return meters / METERS_PER_YARD;
    }
   
    public static double metersToInches(double meters) {
        return metersToYards(meters) * INCHES_PER_YARD;
    }

    public static double centimetersToInches(double centimeters) {
        return ((centimeters * MILLIMETERS_PER_CENTIMETER) / MILLIMETERS_PER_CENTIMETER);
    }

    public static double RPMtoFalconRSU(double rpmVal) {
        double tpm = rpmVal * 2048;
        return tpm / 600;
    }

    public static double falconRSUtoRPM(double rsuVal) {
        double sp100ms = rsuVal / 2048;
        return sp100ms * 600;
    }

    public static double falconVelRsuToRadPerSec(double encoderVelRSU) {
        return BreakerMath.radiansPerTick(2048) * (encoderVelRSU * 10);
    }

    public enum ShortDistanceUnits {
        CENTIMETER,
        MILLIMETER,
        INCH
    }

    public enum LongDistanceUnits {
        METER,
        YARD,
        FOOT
    }

    public static void setDefaultUnits(ShortDistanceUnits shortUnit, LongDistanceUnits longUnit) {
        shortDefaultUnits = shortUnit;
        longDefaultUnits = longUnit;
    }

    public static LongDistanceUnits getLongDefaultUnits() {
        return longDefaultUnits;
    }

    public static ShortDistanceUnits getShortDefaultUnits() {
        return shortDefaultUnits;
    }

    public static double convertShortDistanceToDefaultUnit(ShortDistanceUnits imputUnits, double imputVal) {
        switch (imputUnits) {
            case CENTIMETER:
                switch (getShortDefaultUnits()) {
                    case MILLIMETER:
                        return imputVal * MILLIMETERS_PER_CENTIMETER;
                    case INCH:
                        return centimetersToInches(imputVal);
                    case CENTIMETER:
                    default:
                        return imputVal;
                }
            case MILLIMETER:
                switch (getShortDefaultUnits()) {
                    case CENTIMETER:
                        return imputVal / MILLIMETERS_PER_CENTIMETER;
                    case INCH:
                        return imputVal / MILLIMETERS_PER_INCH;
                    case MILLIMETER:
                    default:
                        return imputVal;
                }
            case INCH:
                switch (getShortDefaultUnits()) {
                    case MILLIMETER:
                        return imputVal * MILLIMETERS_PER_INCH;
                    case CENTIMETER:
                        return imputVal * MILLIMETERS_PER_CENTIMETER;
                    case INCH:
                    default:
                        return imputVal;
                }
            default:
                return imputVal;
        }
    }
}