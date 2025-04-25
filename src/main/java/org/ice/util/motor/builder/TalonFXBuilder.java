package org.ice.util.motor.builder;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class TalonFXBuilder extends TalonBuilder<TalonFXBuilder>{
    private TalonFX motor;
    TalonFXBuilder(TalonFX motor) {
        super(new TalonFXConfiguration());
        this.motor = motor;
    }
}
