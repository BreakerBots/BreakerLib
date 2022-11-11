// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.test.suites;

/** A parameterieed interface that provides a standardinxed metod for retriveing a test suite instance from compatable classes */
public interface BreakerGenericTestSuiteImplementation<T> {
    public abstract T getTestSuite();
}
