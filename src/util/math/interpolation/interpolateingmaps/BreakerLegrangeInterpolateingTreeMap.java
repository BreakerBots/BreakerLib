package frc.robot.BreakerLib.util.math.interpolation.interpolateingmaps;

import java.util.Map.Entry;

import edu.wpi.first.math.geometry.Translation2d;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;

import frc.robot.BreakerLib.util.math.BreakerMath;
import frc.robot.BreakerLib.util.math.interpolation.BreakerInterpolable;

/** A TreeMap of Key Value pairs that uses Legrange Polynomial interpolation */
public class BreakerLegrangeInterpolateingTreeMap<K, V extends BreakerInterpolable<V>>
        extends java.util.AbstractMap<K, V> implements BreakerGenericInterpolateingMap<K, V> {
    private TreeMap<K, V> indexesAndValues;

    public BreakerLegrangeInterpolateingTreeMap(TreeMap<K, V> indexesAndValues) {
        this.indexesAndValues = indexesAndValues;
    }

    public BreakerLegrangeInterpolateingTreeMap() {
        indexesAndValues = new TreeMap<K, V>();
    }

    @Override
    public V getInterpolatedValue(K interpolendValue) {
        V refVal = indexesAndValues.pollFirstEntry().getValue(); // establishes a refrence object of the type to use the
                                                                 // getInterpolatableData() and FromInterpolatableData
                                                                 // methods
        List<List<Double>> interpolatableVals = new ArrayList<>(); // creates a list of lists that represent the
                                                                   // interpolatabe values from which to interpolate the
                                                                   // final result
        double[] interpolatedValArr = new double[refVal.getInterpolatableData().length]; // creates the array that will
                                                                                         // holdthe interpolated values
                                                                                         // from which to create the
                                                                                         // methods result
        for (int i = 0; i < refVal.getInterpolatableData().length; i++) {
            List<Double> intValPortion = new ArrayList<>();
            for (Entry<K, V> ent : indexesAndValues.entrySet()) { // loops through each of the interpolatable valuse
                                                                  // that compose each oboject in the tree map and adds
                                                                  // them to the interpolatableVls list as a list
                intValPortion.add(ent.getValue().getInterpolatableData()[i]);
            }
            interpolatableVals.add(intValPortion);
        }
        int j = 0;
        for (List<Double> listD : interpolatableVals) { // loops through the interpolatableVals list
            Translation2d[] arr = new Translation2d[listD.size()];
            Iterator<K> it = indexesAndValues.keySet().iterator();
            int k = 0;
            while (it.hasNext()) { // creats a seprate Translation2d object for each interpolatableValue component
                                   // of each data point in the TreeMap
                arr[k] = new Translation2d((double) it.next(), listD.get(k));
                k++;
            }
            interpolatedValArr[j] = BreakerMath.interpolateLagrange((double) interpolendValue, arr); // interpolates the
                                                                                                     // translation2d
                                                                                                     // values and
                                                                                                     // creats one of
                                                                                                     // the
                                                                                                     // interpolatabelValues
                                                                                                     // from which to
                                                                                                     // construct the
                                                                                                     // methods return
                                                                                                     // object
            j++;
        }
        return refVal.fromInterpolatableData(interpolatedValArr); // creates a new oject of the Class represented by V,
                                                                  // from the now interpolated interpolatableValues
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