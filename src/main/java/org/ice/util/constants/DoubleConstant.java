package org.ice.util.constants;

/**
 * Functional interface that represents a constant double value. Its primary use is to be implemented by {@code enum}s that need to store constant values.
 */
@FunctionalInterface
public interface DoubleConstant {
    double getValue();
}
