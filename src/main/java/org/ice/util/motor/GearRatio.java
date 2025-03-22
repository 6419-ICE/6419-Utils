package org.ice.util.motor;

/**
 * Builder style utility class for creating conversion factors from gear ratios. All mutating methods return {@code this} for method chaining.
 */
public class GearRatio {

    private double conversionFactor;

    /**
     * Creates a new gear ratio with the given input and output rotations (input:output).
     * @param input the input rotations
     * @param output the output rotations
     */
    public GearRatio(double input, double output) {
        conversionFactor = output/input;
    }

    /**
     * Creates a new gear ratio with the given string representation of a gear ratio
     * @param ratio the gear ratio in terms of {@code "#:#"}
     */
    public GearRatio(String ratio) {
        //funky lil regex
        conversionFactor = parse(ratio);
    }

    /**
     * Adds the given gear ratio to this ratio, as if the gear ratios were applied in sequence.
     * @param other the ratio to append
     * @return itself, for method chaining
     */
    public GearRatio append(GearRatio other) {
        conversionFactor *= other.conversionFactor;
        return this;
    }

    /**
     * Adds the ratio calculated from the given input and output to this ratio, as if the gear ratios were applied in sequence.
     * @param input the input of the ratio to append
     * @param output the output of the ratio to append
     * @return itself, for method chaining
     */
    public GearRatio append(double input, double output) {
        conversionFactor *= output/input;
        return this;
    }

    /**
     * Adds the ratio taken from the given string representation to this ratio, as if the gear ratios were applied in sequence.
     * @param ratio string representation of the ratio to append ({@code #:#})
     * @return itself, for method chaining
     */
    public GearRatio append(String ratio) {
        conversionFactor *= parse(ratio);
        return this;
    }

    /**
     * Returns the calculated conversion factor of the gear ratio
     * @return the conversion factor of the gear ratio
     */
    public double getConversionFactor() {
        return conversionFactor;
    }

    /**
     * Converts the ratio's conversion factor from rotations to degrees
     * @return itself, for method chaining
     */
    public GearRatio inDegrees() {
        conversionFactor /= 360.0;
        return this;
    }
    /**
     * Converts the ratio's conversion factor from rotations to radians
     * @return itself, for method chaining
     */
    public GearRatio inRadians() {
        conversionFactor /= (Math.PI*2.0);
        return this;
    }

    /**
     * Inverts this gear ratio, by setting the conversion factor to 1 over itself.
     * @return itself, for method chaining
     */
    public GearRatio invert() {
        conversionFactor = 1.0/conversionFactor;
        return this;
    }
    /**
     * Applies the gear ratio to the given input. This method's inverse is {@link #removeFrom(double)}
     * @param input the value to be applied to
     * @return the input with the gear ratio applied
     */
    public double applyTo(double input) {
        return input * conversionFactor;
    }

    /**
     * removes the gear ratio from the given input. This method's inverse is {@link #applyTo(double)}
     * @param input the value to remove the ratio from
     * @return the input with the ratio conversion removed
     */
    public double removeFrom(double input) {
        return input/conversionFactor;
    }

    private double parse(String ratio) {
        //funky lil regex
        if (!ratio.matches("(?:\\d*+\\.\\d++|\\d++)\\s*+:\\s*+(?:\\d*+\\.\\d++|\\d++)")) throw new IllegalArgumentException("Input must be in the format \"#:#\"");
        String[] nums = ratio.split("\\s*+:\\s*+");
        return Double.parseDouble(nums[1])/Double.parseDouble(nums[0]);
    }
}