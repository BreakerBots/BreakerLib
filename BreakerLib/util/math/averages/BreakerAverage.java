// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.averages;

import java.util.ArrayList;
import java.util.List;

/** Add your docs here. */
public class BreakerAverage implements BreakerGenericAveragingList<Double> {
    private List<Double> list = new ArrayList<>();
    public BreakerAverage() {
        
    }

    @Override
    public Double addValue(Double valueToAdd) {
        list.add(valueToAdd);
        return getAverage();
    }

    @Override
    public Double getAverage() {
        double total = 0;
        for (double val: list) {
            total += val;
        }
        return total /= list.size();
    }

    @Override
    public Double getAverageBetweenGivenIndexes(int startIndex, int stopIndex) {
        double total = 0;
        for (int i = startIndex; i <= stopIndex; i++) {
            total += list.get(i);
        }
        return total /= stopIndex - startIndex;
    }

    @Override
    public Double[] getAsArray() {
        return list.toArray(new Double[list.size()]);
    }

    @Override
    public void clear() {
        list.clear();
    }

    @Override
    public void removeValueAtGivenIndex(int index) {
        list.remove(index);
    }

    @Override
    public boolean removeGivenValue(Object value) {
        return list.remove(value);
    }

    @Override
    public List<Double> getBaseList() {
        return list;
    }

}
