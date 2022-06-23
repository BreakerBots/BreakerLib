// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.auto.trajectory.swerve.rotation;

import java.util.function.BooleanSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class BreakerSwitchableSwerveRotationSupplier implements BreakerGenericSwerveRotationSupplier {
    private BooleanSupplier boolSwitcher;
    private IntSupplier intSwicher;
    private boolean usesBoolSwicher;
    private BreakerSwerveRotationSupplier[] rotationSuppliers;
    private double curTime = 0;

    public BreakerSwitchableSwerveRotationSupplier(BooleanSupplier switcher, BreakerSwerveRotationSupplier falseRotationSupplier, BreakerSwerveRotationSupplier trueRotationSupplier) {
        rotationSuppliers = new BreakerSwerveRotationSupplier[] {falseRotationSupplier, trueRotationSupplier};
        boolSwitcher = switcher;
        usesBoolSwicher = true;
    }

    public BreakerSwitchableSwerveRotationSupplier(IntSupplier switcher, BreakerSwerveRotationSupplier... rotationSuppliers) {
        this.rotationSuppliers = rotationSuppliers;
        intSwicher = switcher;
        usesBoolSwicher = false;
    }

    private BreakerSwerveRotationSupplier getSwitchedSuppier() {
        int index = 0;
        if (usesBoolSwicher) {
            index = boolSwitcher.getAsBoolean() ? 0 : 1;
        } else {
           index = intSwicher.getAsInt(); 
        }
        return rotationSuppliers[index];
    }

    @Override
    public void setCurrentTime(double currentTime) {
        curTime = currentTime;
    }

    @Override
    public BreakerRotationPoint[] getRotationPoints() {
        return getSwitchedSuppier().getRotationPoints();
    }

    @Override
    public Rotation2d getRotation() {
        getSwitchedSuppier().setCurrentTime(curTime);
        return getSwitchedSuppier().getRotation();
    }


}
