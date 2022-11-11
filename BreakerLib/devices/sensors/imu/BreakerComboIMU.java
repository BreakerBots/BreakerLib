package frc.robot.BreakerLib.devices.sensors.imu;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.interfaces.Accelerometer.Range;
import frc.robot.BreakerLib.devices.sensors.accelerometer.BreakerGenericAccelerometer;
import frc.robot.BreakerLib.devices.sensors.gyro.BreakerGeneric3AxisGyro;
import frc.robot.BreakerLib.util.power.BreakerPowerManagementConfig;
import frc.robot.BreakerLib.util.power.DevicePowerMode;

/**
 * Custom IMU class for combining a 3-axis gyro and accelerometer into a single
 * IMU object.
 */
public class BreakerComboIMU extends BreakerGenericIMU {

    private BreakerGeneric3AxisGyro gyro;
    private BreakerGenericAccelerometer accelerometer;

    /**
     * Creates a BreakerComboIMU.
     * 
     * @param gyro          Gyroscope to use.
     * @param accelerometer Accelerometer to use.
     */
    public BreakerComboIMU(BreakerGeneric3AxisGyro gyro, BreakerGenericAccelerometer accelerometer) {
        this.gyro = gyro;
        this.accelerometer = accelerometer;
    }

    @Override
    public double getPitchDegrees() {
        return gyro.getPitchDegrees();
    }

    @Override
    public double getYawDegrees() {
        return gyro.getYawDegrees();
    }

    @Override
    public double getRollDegrees() {
        return gyro.getRollDegrees();
    }

    @Override
    public Rotation2d getPitchRotation2d() {
        return gyro.getPitchRotation2d();
    }

    @Override
    public Rotation2d getYawRotation2d() {
        return gyro.getYawRotation2d();
    }

    @Override
    public Rotation2d getRollRotation2d() {
        return gyro.getRollRotation2d();
    }

    @Override
    public Rotation3d getRotation3d() {
        return gyro.getRotation3d();
    }

    @Override
    public double[] getRawAngles() {
        return gyro.getRawAngles();
    }

    @Override
    public double getRawPitch() {
        return gyro.getRawPitch();
    }

    @Override
    public double getRawYaw() {
        return gyro.getRawYaw();
    }

    @Override
    public double getRawRoll() {
        return gyro.getRawRoll();
    }

    @Override
    public void setPitch(double value) {
        gyro.setPitch(value);
    }

    @Override
    public void setYaw(double value) {
        gyro.setYaw(value);
    }

    @Override
    public void setRoll(double value) {
        gyro.setRoll(value);
    }

    @Override
    public double[] getRawGyroRates() {
        return gyro.getRawGyroRates();
    }

    @Override
    public double getRawPitchRate() {
        return gyro.getRawPitchRate();
    }

    @Override
    public double getRawYawRate() {
        return gyro.getRawYawRate();
    }

    @Override
    public double getRawRollRate() {
        return gyro.getRawRollRate();
    }

    @Override
    public double getPitchRate() {
        return gyro.getPitchRate();
    }

    @Override
    public double getYawRate() {
        return gyro.getYawRate();
    }

    @Override
    public double getRollRate() {
        return gyro.getRollRate();
    }

    @Override
    public Rotation3d getRawRotation3d() {
        return gyro.getRawRotation3d();
    }

    @Override
    public void reset() {
        gyro.reset();

    }

    @Override
    public double[] getRawAccelerometerVals() {
        return accelerometer.getRawAccelerometerVals();
    }

    @Override
    public double getRawAccelX() {
        return accelerometer.getRawAccelX();
    }

    @Override
    public double getRawAccelY() {
        return accelerometer.getRawAccelY();
    }

    @Override
    public double getRawAccelZ() {
        return accelerometer.getRawAccelZ();
    }

    @Override
    public DevicePowerMode managePower(BreakerPowerManagementConfig managementConfig) {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void overrideAutomaticPowerManagement(DevicePowerMode manualPowerMode) {
        // TODO Auto-generated method stub

    }

    @Override
    public void returnToAutomaticPowerManagement() {
        // TODO Auto-generated method stub

    }

    @Override
    public boolean isUnderAutomaticControl() {
        // TODO Auto-generated method stub
        return false;
    }

    @Override
    public DevicePowerMode getPowerMode() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void runSelfTest() {
        // TODO Auto-generated method stub

    }

    @Override
    public Quaternion getQuaternion() {
        return gyro.getQuaternion();
    }

    @Override
    public void setRange(Range range) {
        accelerometer.setRange(range);
    }

}
