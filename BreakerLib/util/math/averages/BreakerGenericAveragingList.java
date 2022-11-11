// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.averages;

import java.util.List;

/** Add your docs here. */
public interface BreakerGenericAveragingList<T> {
    /** adds a new value to the list and returns the new average */
    public abstract T addValue(T valueToAdd);
    public abstract T getAverage();
    public abstract T getAverageBetweenGivenIndexes(int startIndex, int stopIndex);
    public abstract T[] getAsArray();
    public abstract List<T> getBaseList();
    public abstract void clear();
    public abstract void removeValueAtGivenIndex(int index);
    public abstract boolean removeGivenValue(Object value);
}
