package org.ice.util.motor;

/**
 * Super simple util class for when im stupid and forget how gear ratios work :P
 */
public class GearRatio {

    private double conversionFactor;

    public GearRatio(double input, double output) {
        conversionFactor = output/input;
    }

    /**
     * @param ratio the gear ratio in terms of {@code "#:#"}
     */
    public GearRatio(String ratio) {
        //funky lil regex
        conversionFactor = parse(ratio);
    }

    public GearRatio append(GearRatio other) {
        conversionFactor *= other.conversionFactor;
        return this;
    }
    public GearRatio append(double input, double output) {
        conversionFactor *= output/input;
        return this;
    }
    public GearRatio append(String ratio) {
        conversionFactor *= parse(ratio);
        return this;
    }

    public double getConversionFactor() {
        return conversionFactor;
    }
    public GearRatio inDegrees() {
        conversionFactor /= 360.0;
        return this;
    }
    public GearRatio inRadians() {
        conversionFactor /= (Math.PI*2.0);
        return this;
    }

    public double applyTo(double input) {
        return input * conversionFactor;
    }

    public double removeFrom(double input) {
        return input/conversionFactor;
    }
    public GearRatio invert() {
        conversionFactor = 1.0/conversionFactor;
        return this;
    }
    private double parse(String ratio) {
        //funky lil regex
        if (!ratio.matches("(?:\\d*+\\.\\d++|\\d++)\\s*+:\\s*+(?:\\d*+\\.\\d++|\\d++)")) throw new IllegalArgumentException("Input must be in the format \"#:#\"");
        String[] nums = ratio.split("\\s*+:\\s*+");
        return Double.parseDouble(nums[1])/Double.parseDouble(nums[0]);
    }
}