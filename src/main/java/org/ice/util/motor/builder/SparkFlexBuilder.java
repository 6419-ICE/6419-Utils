package org.ice.util.motor.builder;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.ice.util.motor.GenericSpark;

public class SparkFlexBuilder extends SparkBuilder<SparkFlexBuilder>{
    private SparkFlex motor;
    SparkFlexBuilder(SparkFlex motor) {
        super(new SparkMaxConfig());
        this.motor = motor;
    }

    public SparkFlexBuilder withExternalEncoderInverted(boolean inverted) {
        ((SparkFlexConfig)config).externalEncoder.inverted(inverted);
        return this;
    }

    public SparkFlexBuilder withExternalEncoderVelecoityConversionFactor(double factor) {
        ((SparkFlexConfig)config).externalEncoder.velocityConversionFactor(factor);
        return this;
    }

    public SparkFlexBuilder withExternalEncoderAverageDepth(int depth) {
        ((SparkFlexConfig)config).externalEncoder.averageDepth(depth);
        return  this;
    }

    public SparkFlexBuilder withExternalEncoderPositionConversionFactor(double factor) {
        ((SparkFlexConfig)config).externalEncoder.positionConversionFactor(factor);
        return this;
    }

    public SparkFlexBuilder withExternalEncoderCountsPerRevolution(int cpr) {
        ((SparkFlexConfig)config).externalEncoder.countsPerRevolution(cpr);
        return this;
    }

    public SparkFlexBuilder withExternalMeasurementPeriod(int periodMs) {
        ((SparkFlexConfig)config).externalEncoder.measurementPeriod(periodMs);
        return this;
    }

    @Override
    protected SparkFlexBuilder self() {
        return this;
    }

    @Override
    protected GenericSpark build(SparkBaseConfig config) {
        return new GenericSpark(motor, config);
    }
}
