package org.example.motor;

import com.ctre.phoenix6.controls.*;
import com.revrobotics.spark.SparkBase;

public enum ControlType {
    POSITION,
    DUTY_CYCLE,
    VELOCITY,
    MM_VELOCITY,
    MM_POSITION,
    VOLTAGE;

    SparkBase.ControlType asSparkControl() {
        return switch(this) {
            case POSITION -> SparkBase.ControlType.kPosition;
            case DUTY_CYCLE -> SparkBase.ControlType.kDutyCycle;
            case VELOCITY -> SparkBase.ControlType.kVelocity;
            case MM_VELOCITY -> SparkBase.ControlType.kMAXMotionVelocityControl;
            case MM_POSITION -> SparkBase.ControlType.kMAXMotionPositionControl;
            case VOLTAGE -> SparkBase.ControlType.kVoltage;
        };
    }

    ControlRequest asTalonControl(double input) {
        return switch(this) {
            case POSITION -> new PositionDutyCycle(input);
            case DUTY_CYCLE -> new DutyCycleOut(input);
            case VELOCITY -> new VelocityDutyCycle(input);
            case MM_VELOCITY -> new MotionMagicVelocityDutyCycle(input);
            case MM_POSITION -> new MotionMagicDutyCycle(input);
            case VOLTAGE -> new VoltageOut(input);
        };
    }
}
