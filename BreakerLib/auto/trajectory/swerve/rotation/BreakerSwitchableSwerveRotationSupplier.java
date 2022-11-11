// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.swerve.rotation;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.geometry.Rotation2d;

/** A {@link BreakerGenericSwerveRotationSupplier} that can switch between multipul {@link BreakerGenericSwerveRotationSupplier} instances */
public class BreakerSwitchableSwerveRotationSupplier implements BreakerGenericSwerveRotationSupplier {
    private BooleanSupplier boolSwitcher;
    private IntSupplier intSwicher;
    private boolean usesBoolSwicher;
    private BreakerGenericSwerveRotationSupplier[] rotationSuppliers;
    private double curTime = 0;

    /** Creates a new {@link BreakerSwitchableSwerveRotationSupplier} that can swich between two {@link BreakerGenericSwerveRotationSupplier}
     * instance based on the returned value of a BooleanSupplier
     * @param swicher The BooleanSupplier that's returned value is used to switch between the given {@link BreakerGenericSwerveRotationSupplier} instances
     * @param falseRotationSupplier The {@link BreakerGenericSwerveRotationSupplier} instance used while the BooleanSupplier returns false
     * @param trueRotationSupplier The {@link BreakerGenericSwerveRotationSupplier} instance used while the BooleanSupplier returns true
     */
    public BreakerSwitchableSwerveRotationSupplier(BooleanSupplier switcher, BreakerGenericSwerveRotationSupplier falseRotationSupplier, BreakerGenericSwerveRotationSupplier trueRotationSupplier) {
        rotationSuppliers = new BreakerGenericSwerveRotationSupplier[] {falseRotationSupplier, trueRotationSupplier};
        boolSwitcher = switcher;
        usesBoolSwicher = true;
    }

    /** Creates a new {@link BreakerSwitchableSwerveRotationSupplier} that can swich between the elements of a
     *  {@link BreakerGenericSwerveRotationSupplier} array based on the returned index value of a IntSupplier
     *  @param swicher The IntSupplier that's returned value is used to indetify the index of the {@link BreakerGenericSwerveRotationSupplier} element in the array to use
     *  @param rotationSuppliers The {@link BreakerGenericSwerveRotationSupplier} array to switch through
     */
    public BreakerSwitchableSwerveRotationSupplier(IntSupplier switcher, BreakerSwerveRotationSupplier... rotationSuppliers) {
        this.rotationSuppliers = rotationSuppliers;
        intSwicher = switcher;
        usesBoolSwicher = false;
    }

    private BreakerGenericSwerveRotationSupplier getSwitchedSuppier() {
        int index = 0;
        if (usesBoolSwicher) {
            index = boolSwitcher.getAsBoolean() ? 0 : 1;
        } else {
           index = intSwicher.getAsInt(); 
        }
        return rotationSuppliers[index];
    }

    @Override
    public Rotation2d getRotation(double currentTime) {
        return getSwitchedSuppier().getRotation(currentTime);
    }


}
