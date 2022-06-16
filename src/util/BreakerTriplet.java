// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util;

import edu.wpi.first.math.Pair;

/** Add your docs here. */
public class BreakerTriplet<L, M, R> {
    private L leftValue;
    private M middleValue;
    private R rightValue;
    public BreakerTriplet(L leftValue, M middleValue, R rightValue) {
        this.leftValue = leftValue;
        this.middleValue = middleValue;
        this.rightValue = rightValue;
    }

    public L getLeft() {
        return leftValue;
    }

    public M getMiddle() {
        return middleValue;
    }

    public R getRight() {
        return rightValue;
    }
}
