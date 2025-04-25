package org.ice.util.motor.builder;

import com.ctre.phoenix6.configs.*;

abstract class TalonBuilder<T extends TalonBuilder<T>> {
    private ClosedLoopGeneralConfigs closedLoopGeneralConfigs;
    private ClosedLoopRampsConfigs closedLoopRampsConfigs;
    private CurrentLimitsConfigs currentLimitsConfigs;
    private CustomParamsConfigs customParamsConfigs;
    private DifferentialConstantsConfigs differentialConstantsConfigs;
    private DifferentialSensorsConfigs differentialSensorsConfigs;
    private FeedbackConfigs feedbackConfigs;
    private HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs;
    private MotorOutputConfigs motorOutputConfigs;
    private OpenLoopRampsConfigs openLoopRampsConfigs;
    private SlotConfigs slot0, slot1, slot2;
    private SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs;
    private TorqueCurrentConfigs torqueCurrentConfigs;
    private VoltageConfigs voltageConfigs;

    protected TalonBuilder(ClosedLoopGeneralConfigs closedLoopGeneralConfigs,
                           ClosedLoopRampsConfigs closedLoopRampsConfigs, CurrentLimitsConfigs currentLimitsConfigs,
                           CustomParamsConfigs customParamsConfigs, DifferentialConstantsConfigs differentialConstantsConfigs,
                           DifferentialSensorsConfigs differentialSensorsConfigs, FeedbackConfigs feedbackConfigs,
                           HardwareLimitSwitchConfigs hardwareLimitSwitchConfigs, MotorOutputConfigs motorOutputConfigs,
                           OpenLoopRampsConfigs openLoopRampsConfigs, SlotConfigs slot0, SlotConfigs slot1, SlotConfigs slot2,
                           SoftwareLimitSwitchConfigs softwareLimitSwitchConfigs, TorqueCurrentConfigs torqueCurrentConfigs,
                           VoltageConfigs voltageConfigs) {
        this.closedLoopGeneralConfigs = closedLoopGeneralConfigs;
        this.closedLoopRampsConfigs = closedLoopRampsConfigs;
        this.currentLimitsConfigs = currentLimitsConfigs;
        this.customParamsConfigs = customParamsConfigs;
        this.differentialConstantsConfigs = differentialConstantsConfigs;
        this.differentialSensorsConfigs = differentialSensorsConfigs;
        this.feedbackConfigs = feedbackConfigs;
        this.hardwareLimitSwitchConfigs = hardwareLimitSwitchConfigs;
        this.motorOutputConfigs = motorOutputConfigs;
        this.openLoopRampsConfigs = openLoopRampsConfigs;
        this.slot0 = slot0;
        this.slot1 = slot1;
        this.slot2 = slot2;
        this.softwareLimitSwitchConfigs = softwareLimitSwitchConfigs;
        this.torqueCurrentConfigs = torqueCurrentConfigs;
        this.voltageConfigs = voltageConfigs;
    }
}
