package org.ice.util.motor.builder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.ice.util.motor.GenericSpark;

public class SparkMaxBuilder extends SparkBuilder<SparkMaxBuilder>{
    private SparkMax motor;
    SparkMaxBuilder(SparkMax motor) {
        super(new SparkMaxConfig());
        this.motor = motor;
    }

    public SparkMaxBuilder usingAlternateEncoder() {
        ((SparkMaxConfig)config).alternateEncoder.setSparkMaxDataPortConfig();
        return this;
    }
    public SparkMaxBuilder withAlternateEncoderInverted(boolean inverted) {
        ((SparkMaxConfig)config).alternateEncoder.inverted(inverted);
        return this;
    }
    public SparkMaxBuilder withAlternateEncoderAverageDepth(int depth) {
        ((SparkMaxConfig)config).alternateEncoder.averageDepth(depth);
        return this;
    }
    public SparkMaxBuilder withAlternateEncoderMeasurementPeriod(int periodMs) {
        ((SparkMaxConfig)config).alternateEncoder.measurementPeriod(periodMs);
        return this;
    }
    public SparkMaxBuilder withAlternateEncoderCountsPerRevolution(int cpr) {
        ((SparkMaxConfig)config).alternateEncoder.countsPerRevolution(cpr);
        return this;
    }
    public SparkMaxBuilder withAlternateEncoderPositionConversionFactor(double factor ) {
        ((SparkMaxConfig)config).alternateEncoder.positionConversionFactor(factor);
        return this;
    }
    public SparkMaxBuilder withAlternateEncoderVelocityConversionFactor(double factor ) {
        ((SparkMaxConfig)config).alternateEncoder.velocityConversionFactor(factor);
        return this;
    }

    @Override
    protected SparkMaxBuilder self() {
        return this;
    }



    @Override
    protected GenericSpark build(SparkBaseConfig config) {
        return new GenericSpark(motor, config);
    }

}
