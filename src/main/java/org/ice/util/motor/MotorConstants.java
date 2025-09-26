package org.ice.util.motor;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.*;

public final class MotorConstants {

    public static final class KrakenX60 {
        public static final Mass WEIGHT = Units.Pounds.of(1.2);
        public static final AngularVelocity FREE_SPEED = Units.RotationsPerSecond.of(6000.0);
        public static final Torque STALL_TORQUE = Units.NewtonMeters.of(7.09);
        public static final Current STALL_CURRENT = Units.Amps.of(366.0);
        public static final Power PEAK_POWER_20A = Units.Watts.of(214.0);
        public static final Power PEAK_POWER_40A = Units.Watts.of(419.0);
        public static final Power PEAK_POWER_80A = Units.Watts.of(750.0);
        public static final double kT = 0.0194;
        public static final double kV = 502.1;
        public static final double kM = 0.107;
    }

    public static final class KrakenX60FOC {
        public static final Mass WEIGHT = Units.Pounds.of(1.2);
        public static final AngularVelocity FREE_SPEED = Units.RotationsPerSecond.of(5800.0);
        public static final Torque STALL_TORQUE = Units.NewtonMeters.of(9.37);
        public static final Current STALL_CURRENT = Units.Amps.of(483.0);
        public static final Power PEAK_POWER_20A = Units.Watts.of(210.0);
        public static final Power PEAK_POWER_40A = Units.Watts.of(417.0);
        public static final Power PEAK_POWER_80A = Units.Watts.of(774.0);
        public static final double kT = 0.0194;
        public static final double kV = 484.8;
        public static final double kM = 0.123;
    }

    public static final class NEOVortex {
        public static final Mass WEIGHT = Units.Pounds.of(1.28);
        public static final AngularVelocity FREE_SPEED = Units.RotationsPerSecond.of(6784.0);
        public static final Torque STALL_TORQUE = Units.NewtonMeters.of(3.6);
        public static final Current STALL_CURRENT = Units.Amps.of(211.0);
        public static final Power PEAK_POWER_20A = Units.Watts.of(183.0);
        public static final Power PEAK_POWER_40A = Units.Watts.of(364.0);
        public static final Power PEAK_POWER_80A = Units.Watts.of(585.0);
        public static final double kT = 0.0171;
        public static final double kV = 575.1;
        public static final double kM = 0.072;
    }

    public static final class NEOBrushless {
        public static final Mass WEIGHT = Units.Pounds.of(1.19);
        public static final AngularVelocity FREE_SPEED = Units.RotationsPerSecond.of(5880.0);
        public static final Torque STALL_TORQUE = Units.NewtonMeters.of(3.28);
        public static final Current STALL_CURRENT = Units.Amps.of(181.0);
        public static final Power PEAK_POWER_20A = Units.Watts.of(187.0);
        public static final Power PEAK_POWER_40A = Units.Watts.of(339.0);
        public static final Power PEAK_POWER_80A = Units.Watts.of(494.0);
        public static final double kT = 0.0181;
        public static final double kV = 493.5;
        public static final double kM = 0.07;
    }

    public static final class KrakenX44 {
        public static final Mass WEIGHT = Units.Pounds.of(0.75);
        public static final AngularVelocity FREE_SPEED = Units.RotationsPerSecond.of(7530.0);
        public static final Torque STALL_TORQUE = Units.NewtonMeters.of(4.05);
        public static final Current STALL_CURRENT = Units.Amps.of(275.0);
        public static final Power PEAK_POWER_20A = Units.Watts.of(201.0);
        public static final Power PEAK_POWER_40A = Units.Watts.of(385.0);
        public static final Power PEAK_POWER_80A = Units.Watts.of(651.0);
        public static final double kT = 0.0147;
        public static final double kV = 630.7;
        public static final double kM = 0.071;
    }

    public static final class Falcon500 {
        public static final Mass WEIGHT = Units.Pounds.of(1.1);
        public static final AngularVelocity FREE_SPEED = Units.RotationsPerSecond.of(6380.0);
        public static final Torque STALL_TORQUE = Units.NewtonMeters.of(4.69);
        public static final Current STALL_CURRENT = Units.Amps.of(257.0);
        public static final Power PEAK_POWER_20A = Units.Watts.of(209.0);
        public static final Power PEAK_POWER_40A = Units.Watts.of(399.0);
        public static final Power PEAK_POWER_80A = Units.Watts.of(663.0);
        public static final double kT = 0.0182;
        public static final double kV = 534.8;
        public static final double kM = 0.084;
    }

    public static final class Cu60 {
        public static final Mass WEIGHT = Units.Pounds.of(1.4);
        public static final AngularVelocity FREE_SPEED = Units.RotationsPerSecond.of(6780.0);
        public static final Torque STALL_TORQUE = Units.NewtonMeters.of(7.3);
        public static final Current STALL_CURRENT = Units.Amps.of(440.0);
        public static final Power PEAK_POWER_20A = Units.Watts.of(203.0);
        public static final Power PEAK_POWER_40A = Units.Watts.of(409.0);
        public static final Power PEAK_POWER_80A = Units.Watts.of(755.0);
        public static final double kT = 0.0166;
        public static final double kV = 567.6;
        public static final double kM = 0.1;
    }

    public static final class Falcon500FOC {
        public static final Mass WEIGHT = Units.Pounds.of(1.1);
        public static final AngularVelocity FREE_SPEED = Units.RotationsPerSecond.of(6080.0);
        public static final Torque STALL_TORQUE = Units.NewtonMeters.of(5.84);
        public static final Current STALL_CURRENT = Units.Amps.of(304.0);
        public static final Power PEAK_POWER_20A = Units.Watts.of(212.0);
        public static final Power PEAK_POWER_40A = Units.Watts.of(411.0);
        public static final Power PEAK_POWER_80A = Units.Watts.of(711.0);
        public static final double kT = 0.0192;
        public static final double kV = 509.2;
        public static final double kM = 0.097;
    }

    public static final class Minion {
        public static final Mass WEIGHT = Units.Pounds.of(0.97);
        public static final AngularVelocity FREE_SPEED = Units.RotationsPerSecond.of(7384.0);
        public static final Torque STALL_TORQUE = Units.NewtonMeters.of(3.1);
        public static final Current STALL_CURRENT = Units.Amps.of(200.0);
        public static final Power PEAK_POWER_20A = Units.Watts.of(176.0);
        public static final Power PEAK_POWER_40A = Units.Watts.of(352.0);
        public static final Power PEAK_POWER_80A = Units.Watts.of(558.0);
        public static final double kT = 0.0155;
        public static final double kV = 627.6;
        public static final double kM = 0.063;
    }

    public static final class NEO550 {
        public static final Mass WEIGHT = Units.Pounds.of(0.56);
        public static final AngularVelocity FREE_SPEED = Units.RotationsPerSecond.of(11710.0);
        public static final Torque STALL_TORQUE = Units.NewtonMeters.of(1.08);
        public static final Current STALL_CURRENT = Units.Amps.of(111.0);
        public static final Power PEAK_POWER_20A = Units.Watts.of(187.0);
        public static final Power PEAK_POWER_40A = Units.Watts.of(300.0);
        public static final Power PEAK_POWER_80A = Units.Watts.of(328.0);
        public static final double kT = 0.0097;
        public static final double kV = 985.6;
        public static final double kM = 0.03;
    }

    public static final class Pro775 {
        public static final Mass WEIGHT = Units.Pounds.of(1.05);
        public static final AngularVelocity FREE_SPEED = Units.RotationsPerSecond.of(18730.0);
        public static final Torque STALL_TORQUE = Units.NewtonMeters.of(0.71);
        public static final Current STALL_CURRENT = Units.Amps.of(134.0);
        public static final Power PEAK_POWER_20A = Units.Watts.of(172.0);
        public static final Power PEAK_POWER_40A = Units.Watts.of(288.0);
        public static final Power PEAK_POWER_80A = Units.Watts.of(346.0);
        public static final double kT = 0.0053;
        public static final double kV = 1569.0;
        public static final double kM = 0.018;
    }

    public static final class RedLine775 {
        public static final Mass WEIGHT = Units.Pounds.of(1.06);
        public static final AngularVelocity FREE_SPEED = Units.RotationsPerSecond.of(19500.0);
        public static final Torque STALL_TORQUE = Units.NewtonMeters.of(0.64);
        public static final Current STALL_CURRENT = Units.Amps.of(122.0);
        public static final Power PEAK_POWER_20A = Units.Watts.of(159.0);
        public static final Power PEAK_POWER_40A = Units.Watts.of(275.0);
        public static final Power PEAK_POWER_80A = Units.Watts.of(320.0);
        public static final double kT = 0.0052;
        public static final double kV = 1660.4;
        public static final double kM = 0.017;
    }

    public static final class CIM {
        public static final Mass WEIGHT = Units.Pounds.of(3.07);
        public static final AngularVelocity FREE_SPEED = Units.RotationsPerSecond.of(5330.0);
        public static final Torque STALL_TORQUE = Units.NewtonMeters.of(2.41);
        public static final Current STALL_CURRENT = Units.Amps.of(131.0);
        public static final Power PEAK_POWER_20A = Units.Watts.of(154.0);
        public static final Power PEAK_POWER_40A = Units.Watts.of(272.0);
        public static final Power PEAK_POWER_80A = Units.Watts.of(329.0);
        public static final double kT = 0.0184;
        public static final double kV = 453.5;
        public static final double kM = 0.061;
    }

    public static final class MiniCIM {
        public static final Mass WEIGHT = Units.Pounds.of(2.41);
        public static final AngularVelocity FREE_SPEED = Units.RotationsPerSecond.of(5840.0);
        public static final Torque STALL_TORQUE = Units.NewtonMeters.of(1.41);
        public static final Current STALL_CURRENT = Units.Amps.of(89.0);
        public static final Power PEAK_POWER_20A = Units.Watts.of(132.0);
        public static final Power PEAK_POWER_40A = Units.Watts.of(204.0);
        public static final Power PEAK_POWER_80A = Units.Watts.of(208.0);
        public static final double kT = 0.0158;
        public static final double kV = 503.6;
        public static final double kM = 0.043;
    }

    public static final class BAG {
        public static final Mass WEIGHT = Units.Pounds.of(0.96);
        public static final AngularVelocity FREE_SPEED = Units.RotationsPerSecond.of(13180.0);
        public static final Torque STALL_TORQUE = Units.NewtonMeters.of(0.43);
        public static final Current STALL_CURRENT = Units.Amps.of(53.0);
        public static final Power PEAK_POWER_20A = Units.Watts.of(131.0);
        public static final Power PEAK_POWER_40A = Units.Watts.of(143.0);
        public static final Power PEAK_POWER_80A = Units.Watts.of(143.0);
        public static final double kT = 0.0081;
        public static final double kV = 1136.9;
        public static final double kM = 0.017;
    }

    public static final class AM9015 {
        public static final Mass WEIGHT = Units.Pounds.of(0.75);
        public static final AngularVelocity FREE_SPEED = Units.RotationsPerSecond.of(14270.0);
        public static final Torque STALL_TORQUE = Units.NewtonMeters.of(0.36);
        public static final Current STALL_CURRENT = Units.Amps.of(71.0);
        public static final Power PEAK_POWER_20A = Units.Watts.of(94.0);
        public static final Power PEAK_POWER_40A = Units.Watts.of(127.0);
        public static final Power PEAK_POWER_80A = Units.Watts.of(127.0);
        public static final double kT = 0.0051;
        public static final double kV = 1254.5;
        public static final double kM = 0.012;
    }

    public static final class BaneBots550 {
        public static final Mass WEIGHT = Units.Pounds.of(0.73);
        public static final AngularVelocity FREE_SPEED = Units.RotationsPerSecond.of(19300.0);
        public static final Torque STALL_TORQUE = Units.NewtonMeters.of(0.49);
        public static final Current STALL_CURRENT = Units.Amps.of(85.0);
        public static final Power PEAK_POWER_20A = Units.Watts.of(168.0);
        public static final Power PEAK_POWER_40A = Units.Watts.of(241.0);
        public static final Power PEAK_POWER_80A = Units.Watts.of(243.0);
        public static final double kT = 0.0057;
        public static final double kV = 1635.3;
        public static final double kM = 0.015;
    }

    public static final class Snowblower {
        public static final Mass WEIGHT = Units.Pounds.of(1.35);
        public static final AngularVelocity FREE_SPEED = Units.RotationsPerSecond.of(100.0);
        public static final Torque STALL_TORQUE = Units.NewtonMeters.of(7.91);
        public static final Current STALL_CURRENT = Units.Amps.of(24.0);
        public static final Power PEAK_POWER_20A = Units.Watts.of(16.0);
        public static final Power PEAK_POWER_40A = Units.Watts.of(16.0);
        public static final Power PEAK_POWER_80A = Units.Watts.of(16.0);
        public static final double kT = 0.3295;
        public static final double kV = 10.5;
        public static final double kM = 0.466;
    }

}

