package frc.robot.BreakerLib.util.math; 

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.BreakerLib.physics.Breaker3AxisForces;
import frc.robot.BreakerLib.physics.BreakerVector2;
import frc.robot.BreakerLib.position.movement.BreakerMovementState2d;

/** Easily accessible  math utilitie class. */
public class BreakerMath {

    private static double prevTime = 0;

    /**
     * Constrains an angle value in degrees within +- 180 degrees.
     * 
     * @param deg Angle value in degrees.
     * 
     * @return Angle value within -180 to +180 degrees.
     */
    public static final double angleModulus(double deg) {
        return angleModulus(deg, 180);
    }

    /**
     * Constrains an angle value in degrees within +- desired constraint, in degrees
     * 
     * @param deg        Angle value in degrees.
     * @param constraint Degree value to constrain angle within. Should be > 0.
     * 
     * @return Angle value within -constraint and +constraint degrees.
     */
    public static final double angleModulus(double deg, double constraint) {
        return MathUtil.inputModulus(deg, -constraint, constraint);
    }

    /**Constrains an angle value in degrees within a minimum and maximum angle.
     * 
     * @param deg Angle value in degrees.
     * @param minAngle Minimum angle value.
     * @param maxAngle Maximum angle value.
     * 
     * @return Angle value between minAngle and maxAngle.
     */
    public static final double angleModulus(double deg, double minAngle, double maxAngle) {
        return MathUtil.inputModulus(deg, minAngle, maxAngle);
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

    public static Rotation2d getPointAngleRelativeToOtherPoint(Translation2d point1, Translation2d point2) {
        double x1 = point1.getX();
        double y1 = point1.getY();
        double x2 = point2.getX();
        double y2 = point2.getY();
        double startRelAng = Math.atan(x2 - x1 / y2 - y1) * 180 / Math.PI;
        double cor = (y2 - y2 / Math.abs(y2 - y1));
        double finalAng = ((startRelAng * cor) + 90) * (-cor);
        return Rotation2d.fromDegrees(finalAng).rotateBy(Rotation2d.fromDegrees(90));
    }

    /**
     * Linearly interpolates between 2 points to find y-val at given x.
     * 
     * @param queryX X-value to interpolate from.
     * @param lowX   X-val of low point.
     * @param highX  X-val of high point.
     * @param lowY   Y-val of low point.
     * @param highY  Y-val of high point.
     * @return Approximate Y-value at given X.
     */
    public static double interpolateLinear(double queryX, double lowX, double highX, double lowY, double highY) {
        return MathUtil.interpolate(lowY, highY, getLerpT(queryX, lowX, highX));
    }

    public static double getLerpT(double query, double low, double high) {
        return (query - low) / (high - low);
    }

    /**
     * Lagrange Polynomial interpolation of a Y value from an X value and a set of
     * known points. https://en.wikipedia.org/wiki/Lagrange_polynomial
     * 
     * @param queryX      X-value to interpolate a Y-value for.
     * @param knownPoints Known points in a 2D space.
     * @return The approximate Y-Value that would corespond to the given X-Value
     */
    public static double interpolateLagrange(double queryX, Translation2d... knownPoints) {
        double result = 0;
        for (int i = 0; i < knownPoints.length; i++) { // Goes through points.
            double term = knownPoints[i].getY(); // Y-value of selected point.
            for (int j = 0; j < knownPoints.length; j++) { // Loops through non-identical points.
                if (j != i) { // Avoids multiplication by 0.
                    // Interpolates between selected and point from data set.
                    term *= (queryX - knownPoints[j].getX()) / (knownPoints[i].getX() - knownPoints[j].getX());
                }
            }
            result += term; // Accumulated interpretation is added.
        }
        return result;
    }

    public static BreakerMovementState2d movementStateFromChassisSpeedsAndPreviousState(Pose2d currentPose,
            ChassisSpeeds speeds, double timeToLastUpdateMiliseconds, BreakerMovementState2d prevMovementState) {
        Breaker3AxisForces acceleration = new Breaker3AxisForces(
                new BreakerVector2(
                        (1000 / timeToLastUpdateMiliseconds) * (speeds.vxMetersPerSecond
                                - prevMovementState.getVelocityComponent().getLinearForces().getMagnatudeX()),
                        (1000 / timeToLastUpdateMiliseconds) * (speeds.vyMetersPerSecond
                                - prevMovementState.getVelocityComponent().getLinearForces().getMagnatudeY())),
                (1000 / timeToLastUpdateMiliseconds)
                        * (speeds.omegaRadiansPerSecond - prevMovementState.getVelocityComponent().getAngularForce()));
        Breaker3AxisForces jerk = new Breaker3AxisForces(
                new BreakerVector2(
                        (1000 / timeToLastUpdateMiliseconds) * (acceleration.getLinearForces().getMagnatudeX()
                                - prevMovementState.getAccelerationComponent().getLinearForces().getMagnatudeY()),
                        (1000 / timeToLastUpdateMiliseconds) * (acceleration.getLinearForces().getMagnatudeY()
                                - prevMovementState.getAccelerationComponent().getLinearForces().getMagnatudeY())),
                (1000 / timeToLastUpdateMiliseconds) * (acceleration.getAngularForce()
                        - prevMovementState.getAccelerationComponent().getAngularForce()));
        return new BreakerMovementState2d(currentPose,
                new Breaker3AxisForces(new BreakerVector2(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond),
                        speeds.omegaRadiansPerSecond),
                acceleration, jerk);
    }

    public static ChassisSpeeds fromRobotRelativeSpeeds(ChassisSpeeds robotRelativeSpeeds, Rotation2d robotAngle) {
        double cos = Math.cos(-robotAngle.getRadians());
        double sin = Math.sin(-robotAngle.getRadians());
        return new ChassisSpeeds(
                (robotRelativeSpeeds.vxMetersPerSecond * cos) - (robotRelativeSpeeds.vyMetersPerSecond * sin),
                (robotRelativeSpeeds.vxMetersPerSecond * sin) + (robotRelativeSpeeds.vyMetersPerSecond * cos),
                robotRelativeSpeeds.omegaRadiansPerSecond);
    }

    public static double getWeightedAvg(double[] valuesToAvg, double[] weights) {
        double numer = 0;
        double denom = 0;
        for (int i = 0; i < valuesToAvg.length; i++) {
            double weaght = i < weights.length ? weights[i] : 1.0;
            numer += valuesToAvg[i] * weaght;
        }

        for (int i = 0; i < valuesToAvg.length; i++) {
            denom += i < weights.length ? weights[i] : 1.0;
        }
        
        return numer / denom;
    }

    public static double root(double num, double root) {
        return Math.pow(num, 1.0 / root);
    }


}
