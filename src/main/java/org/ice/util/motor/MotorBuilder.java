package org.ice.util.motor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import org.ice.util.swerve.PIDValues;

public abstract sealed class MotorBuilder<T extends MotorBuilder<T,S>, S extends GenericMotorController<?>> {
    private MotorBuilder() {}
    public static SparkMaxBuilder buildSparkMax(int motorID, SparkLowLevel.MotorType motorType) {
        return new SparkMaxBuilder(new SparkMax(motorID,motorType));
    }
    public static SparkFlexBuilder buildSparkFlex(int motorID, SparkLowLevel.MotorType motorType) {
        return new SparkFlexBuilder(new SparkFlex(motorID,motorType));
    }
    public static TalonFXBuilder buildTalonFX(int motorID) {
        return new TalonFXBuilder(new TalonFX(motorID));
    }
    //---------------------------------Class Body---------------------------------
    public abstract T withPIDValues(PIDValues pidValues);
    public abstract T withInverted(boolean inverted);

    public abstract S build();
    public static final class TalonFXBuilder extends MotorBuilder<TalonFXBuilder,GenericTalon> {
        private TalonFX motor;
        private TalonFXConfiguration config;
        private TalonFXBuilder(TalonFX motor) {
            this.motor = motor;
            this.config = new TalonFXConfiguration();
        }

        @Override
        public TalonFXBuilder withPIDValues(PIDValues pidValues) {
            config.Slot0
                    .withKP(pidValues.getP())
                    .withKI(pidValues.getI())
                    .withKD(pidValues.getD())
                    .withKV(pidValues.getFF() == 0 ? 0 : 1 / pidValues.getFF());
            return this;
        }

        @Override
        public GenericTalon build() {
            return new GenericTalon(motor,config);
        }
    }
    public static final class TalonFXSBuilder extends MotorBuilder<TalonFXSBuilder,GenericTalon> {
        private TalonFXS motor;
        private TalonFXSConfiguration config;
        public TalonFXSBuilder(TalonFXS motor) {
            this.motor = motor;
            config = new TalonFXSConfiguration();

        }
        @Override
        public TalonFXSBuilder withPIDValues(PIDValues pidValues) {
            config.Slot0
                    .withKP(pidValues.getP())
                    .withKI(pidValues.getI())
                    .withKD(pidValues.getD())
                    .withKV(pidValues.getFF() == 0 ? 0 : 1 / pidValues.getFF());
            return this;
        }

        @Override
        public GenericTalon build() {
            return new GenericTalon(motor,config);
        }
    }
    private static abstract sealed class SparkBuilder<T extends SparkBuilder<T,S>, S extends SparkBaseConfig> extends MotorBuilder<T,GenericSpark> {
        private SparkBase motor;
        private S config;
        public SparkBuilder(SparkBase motor, S config) {
            this.motor = motor;
            this.config = config;
        }
        @Override
        public GenericSpark build() {
            return new GenericSpark(motor,config);
        }
        @Override
        public T withPIDValues(PIDValues pidValues) {
            config.closedLoop.pidf(pidValues.getP(), pidValues.getI(), pidValues.getD(),pidValues.getFF());
            return self();
        }
        protected abstract T self();
    }
    public static final class SparkMaxBuilder extends SparkBuilder<SparkMaxBuilder,SparkMaxConfig> {

        private SparkMaxBuilder(SparkMax motor) {
            super(motor, new SparkMaxConfig());
        }

        @Override
        protected SparkMaxBuilder self() {
            return this;
        }
    }
    public static final class SparkFlexBuilder extends SparkBuilder<SparkFlexBuilder, SparkFlexConfig> {

        private SparkFlexBuilder(SparkFlex motor) {
            super(motor, new SparkFlexConfig());
        }

        @Override
        protected SparkFlexBuilder self() {
            return this;
        }
    }
}
