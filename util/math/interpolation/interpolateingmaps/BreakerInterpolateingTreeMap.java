// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.BreakerLib.util.math.interpolation.interpolateingmaps;

import java.util.Map.Entry;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolable;

/** Add your docs here. */
public class BreakerInterpolateingTreeMap<K, V extends BreakerInterpolable<V>> extends java.util.AbstractMap<K, V> implements BreakerGenericInterpolateingMap<K, V> {
    private TreeMap<K, V> indexesAndValues;

    public BreakerInterpolateingTreeMap(TreeMap<K, V> indexesAndValues) {
        this.indexesAndValues = indexesAndValues;
    }

    public BreakerInterpolateingTreeMap() {
        indexesAndValues = new TreeMap<K, V>();
    }

    @Override
    public V getInterpolatedValue(K interpolendValue) {
        Entry<K, V> low = indexesAndValues.floorEntry(interpolendValue);
        Entry<K, V> high = indexesAndValues.ceilingEntry(interpolendValue);

        if (low == null) {
            return high.getValue().getSelf();
        }
        if (high == null) {
            return low.getValue().getSelf();
        }
        if (high.getValue().equals(low.getValue())) {
            return high.getValue().getSelf();
        }

        return high.getValue().interpolate((double) interpolendValue, (double) high.getKey(), high.getValue().getSelf(),
                (double) low.getKey(), low.getValue().getSelf());
    }

    @Override
    public Set<Entry<K, V>> entrySet() {
        return indexesAndValues.entrySet();
    }

    @Override
    public V put(K key, V value) {
        return indexesAndValues.put(key, value);
    }

    @Override
    public void putAll(Map<? extends K, ? extends V> m) {
        indexesAndValues.putAll(m);
    }

    @Override
    public boolean containsKey(Object key) {
        return indexesAndValues.containsKey(key);
    }

    @Override
    public boolean containsValue(Object value) {
        return indexesAndValues.containsValue(value);
    }

    @Override
    public boolean replace(K key, V oldValue, V newValue) {
        return indexesAndValues.replace(key, oldValue, newValue);
    }
}
