package frc.robot.util.storage;

/**
 * This {@link MutableTuple} class is designed to hold three values.
 *
 * @author Eric Gold
 * @since 0.0.0
 */
public class MutableTuple<K, V, P> {
    private K first;
    private V second;
    private P third;

    /** @return The first stored value. */
    public K getFirst() { return this.first; }

    /** @return The second stored value. */
    public V getSecond() { return this.second; }

    /** @return The third stored value. */
    public P getThird() { return this.third; }

    /**
     * Sets the first value.
     * @param first The value to set.
     * @return The {@link MutableTuple} with the value modified.
     */
    public synchronized MutableTuple<K, V, P> setFirst(K first) {
        this.first = first;
        return this;
    }

    /**
     * Sets the second value.
     * @param second The value to set.
     * @return The {@link MutableTuple} with the value modified.
     */
    public synchronized MutableTuple<K, V, P> setSecond(V second) {
        this.second = second;
        return this;
    }

    /**
     * Sets the third value.
     * @param third The value to set.
     * @return The {@link MutableTuple} with the value modified.
     */
    public synchronized MutableTuple<K, V, P> setThird(P third) {
        this.third = third;
        return this;
    }

    /**
     * Constructs a new {@link MutableTuple}.
     * @param first The first value.
     * @param second The second value.
     */
    public MutableTuple(K first, V second, P third) {
        this.first = first;
        this.second = second;
        this.third = third;
    }

    public static <K,V,P> MutableTuple<K,V,P> of(K first, V second, P third) {
        return new MutableTuple<>(first, second, third);
    }
}
