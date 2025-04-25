package org.ice.util.motor.builder;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import org.ice.util.motor.GenericSpark;
import org.ice.util.swerve.PIDValues;

abstract class SparkBuilder<T extends SparkBuilder<T>> {
    protected SparkBaseConfig config;
    protected SparkBuilder(SparkBaseConfig config) {
        this.config = config;
    }
    public T usingAbsoluteEncoder() {
        config.absoluteEncoder.setSparkMaxDataPortConfig();
        return self();
    }
    public T withAbsoluteEncoderAverageDepth(int depth) {
        config.absoluteEncoder.averageDepth(depth);
        return self();
    }
    public T withAbsoluteEncoderEndPulseUs(double endPulse ) {
        config.absoluteEncoder.endPulseUs(endPulse);
        return self();
    }
    public T withAbsoluteEncoderStartPulseUs(double startPulse) {
        config.absoluteEncoder.startPulseUs(startPulse);
        return self();
    }
    public T withAbsoluteEncoderPositionConversionFactor(double factor) {
        config.absoluteEncoder.positionConversionFactor(factor);
        return self();
    }
    public T withAbsoluteEncoderVelocityConversionFactor(double factor) {
        config.absoluteEncoder.velocityConversionFactor(factor);
        return  self();
    }
    public T withAbsoluteEncoderZeroCentered(boolean centered) {
        config.absoluteEncoder.zeroCentered(centered);
        return self();
    }
    public T withAbsoluteEncoderZeroOffset(double offset) {
        config.absoluteEncoder.zeroOffset(offset);
        return  self();
    }
    public T withAnalogSensorPositionConversionFactor(double factor) {
        config.analogSensor.positionConversionFactor(factor);
        return self();
    }
    public T withAnalogSensorVelocityConversionFactor(double factor) {
        config.analogSensor.velocityConversionFactor(factor);
        return  self();
    }
    public T withClosedLoopOutputRange(double min, double max) {
        config.closedLoop.outputRange(min,max);
        return self();
    }
    public T withClosedLoopFeedbackSensor(ClosedLoopConfig.FeedbackSensor feedbackSensor) {
        config.closedLoop.feedbackSensor(feedbackSensor);
        return self();
    }
    public T withClosedLoopIZone(double iZone, ClosedLoopSlot slot) {
        config.closedLoop.iZone(iZone,slot);
        return self();
    }
    public T withClosedLoopDFilter(double dFilter, ClosedLoopSlot slot) {
        config.closedLoop.dFilter(dFilter, slot);
        return self();
    }
    public T withClosedLoopPIDValues(PIDValues pid) {
        config.closedLoop.pidf(pid.getP(),pid.getI(),pid.getD(),pid.getFF());
        return self();
    }
    public T withClosedLoopPositionWrappingInputRange(double min, double max) {
        config.closedLoop.positionWrappingInputRange(min,max);
        return self();
    }
    public T withClosedLoopiMaxAccum(double maxAccum, ClosedLoopSlot slot) {
        config.closedLoop.iMaxAccum(maxAccum,slot);
        return self();
    }
    public T withEncoderPositionConversionFactor(double factor) {
        config.encoder.positionConversionFactor(factor);
        return self();
    }
    public T withEncoderQuadratureAverageDepth(int depth) {
        config.encoder.quadratureAverageDepth(depth);
        return self();
    }
    public T withEncoderQuadratureMeasurementPeriod(int depth) {
        config.encoder.quadratureMeasurementPeriod(depth);
        return self();
    }
    public T withEncoderUvwAverageDepth(int depth) {
        config.encoder.uvwAverageDepth(depth);
        return self();
    }
    public T withEncoderUvwMeasurementPeriod(int depth) {
        config.encoder.uvwMeasurementPeriod(depth);
        return self();
    }
    public T withEncoderVelocityConversionFactor(double factor) {
        config.encoder.velocityConversionFactor(factor);
        return self();
    }
    public T withEncoderCountsPerRevolution(int cpr) {
        config.encoder.countsPerRevolution(cpr);
        return self();
    }
    public T withLimitSwitchForwardLimitSwitchEnabled(boolean enabled) {
        config.limitSwitch.forwardLimitSwitchEnabled(enabled);
        return self();
    }
    public T withLimitSwitchForwardLimitSwitchType(LimitSwitchConfig.Type type) {
        config.limitSwitch.forwardLimitSwitchType(type);
        return self();
    }
    public T withLimitSwitchReverseLimitSwitchEnabled(boolean enabled) {
        config.limitSwitch.reverseLimitSwitchEnabled(enabled);
        return self();
    }
    public T withLimitSwitchReverseLimitSwitchType(LimitSwitchConfig.Type type) {
        config.limitSwitch.reverseLimitSwitchType(type);
        return self();
    }
    public T withSoftLimitForwardSoftLimit(double limit) {
        config.softLimit.forwardSoftLimit(limit);
        return self();
    }
    public T withSoftLimitReverseSoftLimit(double limit) {
        config.softLimit.reverseSoftLimit(limit);
        return self();
    }
    public T withSoftLimitForwardSoftLimitEnabled(boolean enabled) {
        config.softLimit.forwardSoftLimitEnabled(enabled);
        return self();
    }
    public T withSoftLimitReverseSoftLimitEnabled(boolean enabled) {
        config.softLimit.reverseSoftLimitEnabled(enabled);
        return self();
    }
    public T withVoltageCompensation(double nomVoltage) {
        config.voltageCompensation(nomVoltage);
        return self();
    }
    public T withClosedLoopRampRate(double rate){
        config.closedLoopRampRate(rate);
        return self();
    }
    public T withOpenLoopRampRate(double rate) {
        config.openLoopRampRate(rate);
        return self();
    }
    public T withSmartCurrentLimit(int stallLimit, int freeLimit, int limitRpm) {
        config.smartCurrentLimit(stallLimit,freeLimit,limitRpm);
        return self();
    }
    public T withSecondaryCurrentLimit(double limit, int chopCycles) {
        config.secondaryCurrentLimit(limit,chopCycles);
        return self();
    }
    public T withIdleMode(SparkBaseConfig.IdleMode idleMode) {
        config.idleMode(idleMode);
        return self();
    }
    public T withMotorInverted(boolean inverted){
        config.inverted(inverted);
        return self();
    }
    public T withSignalsConfig(SignalsConfig signalsConfig) {
        config.apply(signalsConfig);
        return self();
    }

    protected abstract T self();
    protected abstract GenericSpark build(SparkBaseConfig config);
    public GenericSpark build() {
        return build(config);
    }
}
