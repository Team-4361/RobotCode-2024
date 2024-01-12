package frc.robot.util.storage;

/**
 * This {@link MutablePair} class is designed to hold two values.
 *
 * @author Eric Gold
 * @since 0.0.0
 */
public class MutablePair<K, V> {
    private K first;
    private V second;

    /** @return The first stored value. */
    public K getFirst() { return this.first; }

    /** @return The second stored value. */
    public V getSecond() { return this.second; }

    /**
     * Sets the first value.
     * @param first The value to set.
     * @return The {@link MutablePair} with the value modified.
     */
    public synchronized MutablePair<K, V> setFirst(K first) {
        this.first = first;
        return this;
    }

    /**
     * Sets the second value.
     * @param second The value to set.
     * @return The {@link MutablePair} with the value modified.
     */
    public synchronized MutablePair<K, V> setSecond(V second) {
        this.second = second;
        return this;
    }

    /**
     * Constructs a new {@link MutablePair}.
     * @param first The first value.
     * @param second The second value.
     */
    public MutablePair(K first, V second) {
        this.first = first;
        this.second = second;
    }

    public static <K,V> MutablePair<K,V> of(K first, V second) {
        return new MutablePair<>(first, second);
    }
}
