package frc.robot.BreakerLib.util.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotController;

//easily accessible conversion equations
public class BreakerMath {

    private static double prevTime = 0;

    // Drive logistic curve constants

    /** L constant for logistic curve */
    private static double L = 0.9;
    /** k constant for logistic curve */
    private static double k = 7.5;
    /** x0 constant for logistic curve */
    private static double x0 = 0.6;
    /** Vertical translation for logistic curve */
    private static double b = 0.15;

    /**
     * Constrains an angle value in degrees within +- 360 degrees.
     * 
     * @param deg Angle value in degrees.
     * 
     * @return Angle value within -360 to +360 degrees.
     */
    public static final double angleModulus(double deg) {
        return angleModulus(deg, 360);
    }

    /**
     * Constrains an angle value in degrees within +- desired constraint, in degrees
     * 
     * @param deg        Angle value in degrees.
     * @param constraint Degree value to constrain angle within.
     * 
     * @return Angle value within -constraint to +constraint degrees.
     */
    public static final double angleModulus(double deg, double constraint) {
        return deg % constraint;
    }

    /** Calculates radians per second from rotations per minute. */
    public static final double radPerSecFromRPM(double rpm) {
        return ((rpm / 60) * (2 * Math.PI));
    }

    /** Calculates rotations per minute from radians per second. */
    public static final double rpmFromRadPerSec(double radPerSec) {
        return ((radPerSec * 60) / (2 * Math.PI));
    }

    /**
     * Gets the amount of time in seconds between cycles.
     * 
     * @return Time difference between cycles, in seconds.
     */
    public static double getCycleDiffTime() {
        double curTime = RobotController.getFPGATime(); // In microseconds
        double diffTime = curTime - prevTime;
        prevTime = curTime;
        diffTime = BreakerUnits.microsecondsToSeconds(diffTime); // Value converted to seconds
        return diffTime;
    }

    public static double getHypotenuse(double sideA, double sideB) {
        return Math.sqrt(Math.pow(sideA, 2) + Math.pow(sideB, 2));
    }

    // Not necessary. Also misspelled smh
    // public static double getCircumferenceFromRadus(double radius) {
    // return (2 * radius) * Math.PI;
    // }

    public static double getCircumferenceFromDiameter(double diameter) {
        return diameter * Math.PI;
    }

    public static double getTicksPerRotation(double encoderTicks, double gearRatioTo1) {
        return encoderTicks * gearRatioTo1;
    }

    public static double getTicksPerInch(double encoderTicks, double gearRatioTo1, double wheelDiameter) {
        return getTicksPerRotation(encoderTicks, gearRatioTo1) / getCircumferenceFromDiameter(wheelDiameter);
    }

    /**
     * @param ticks Talon FX encoder ticks.
     * @return distance, in inches.
     */
    public static double ticksToInches(double ticks, double ticksPerInch) {
        return ticks / ticksPerInch;
    }

    /**
     * Returns y when x is fed into pre-determined logistic curve.
     * 
     * @param x Value between -1 and 1.
     * @return Value between -1 and 1.
     */
    public static double driveCurve(double x) {
        double absX = MathUtil.applyDeadband(Math.abs(x), 0.05);
        double y = (Math.signum(x) * L) / (1 + Math.pow(Math.E, -k * (absX - x0))) + b;
        return y;
    }

    public static double rollingAvg(double avg, double newVal) {
        return (avg + newVal) / 2.0;
    }

    public static double getAvg(double lastAvg, double newVal, int cycleCount) {
        return (((lastAvg * (cycleCount - 1)) + newVal) / cycleCount);
    }

    /**
     * Converts a fixed point notiation number into a double precision foating point
     * number
     * 
     * @param FixedPointVal    fixed point number represented as a non-fractional
     *                         integer
     * @param bitsAfterDicimal fixed point notation is generaly represented as Qx.y
     *                         where x
     *                         represents the number of bits before the decimal, and
     *                         y prepresents the number of bits
     *                         after the decimal (EX: Q2.14)
     */
    public static double fixedToFloat(int FixedPointVal, int bitsAfterDicimal) {
        return ((Double.valueOf(FixedPointVal)) / (Math.pow(2, bitsAfterDicimal)));
    }

    /**
     * Converts a fixed point notiation number into a double precision foating point
     * number
     * 
     * @param FixedPointVal    fixed point number represented as a non-fractional
     *                         long integer
     * @param bitsAfterDicimal fixed point notation is generaly represented as Qx.y
     *                         where x
     *                         represents the number of bits before the decimal, and
     *                         y prepresents the number of bits
     *                         after the decimal (EX: Q2.14)
     */
    public static double fixedToFloat(Long FixedPointVal, int bitsAfterDicimal) {
        return ((Double.valueOf(FixedPointVal)) / (Math.pow(2, bitsAfterDicimal)));
    }

    /**
     * Converts a fixed point notiation number into a double precision foating point
     * number
     * 
     * @param FixedPointVal    fixed point number represented as a non-fractional
     *                         short integer
     * @param bitsAfterDicimal fixed point notation is generaly represented as Qx.y
     *                         where x
     *                         represents the number of bits before the decimal, and
     *                         y prepresents the number of bits
     *                         after the decimal (EX: Q2.14)
     */
    public static double fixedToFloat(Short FixedPointVal, int bitsAfterDicimal) {
        return ((Double.valueOf(FixedPointVal)) / (Math.pow(2, bitsAfterDicimal)));
    }

    /**
     * @param encoderTicks Ticks per rotation.
     * @return Rotational position in radians.
     */
    public static double radiansPerTick(double encoderTicks) {
        return (2.0 * Math.PI / encoderTicks);
    }

    /**
     * Checks if two numbers are sufficiently proximate.
     * 
     * @param val1         First number.
     * @param val2         Second number.
     * @param maxDeviation Absolute value difference between val1 and val2
     * 
     * @return true if within deviation, false otherwise.
     */
    public static boolean isRoughlyEqualTo(double val1, double val2, double maxDeviation) {
        return ((val1 <= (val2 + maxDeviation)) && (val1 >= (val2 - maxDeviation)));
    }

    public static Rotation2d getPointAngleRelativeToOtherPoint(Translation2d pointOne, Translation2d pointTwo) {
        double x1 = pointOne.getX();
        double y1 = pointOne.getY();
        double x2 = pointTwo.getX();
        double y2 = pointTwo.getY();
        double startRelAng = Math.atan(x2 - x1 / y2 - y1) * 180 / Math.PI;
        double cor = (y2 - y2 / Math.abs(y2 - y1));
        double finalAng = ((startRelAng * cor) + 90) * (-cor);
        return Rotation2d.fromDegrees(finalAng).rotateBy(Rotation2d.fromDegrees(90));
    }

}
