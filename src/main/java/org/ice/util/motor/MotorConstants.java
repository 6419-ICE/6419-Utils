package org.ice.util.motor;

/**
 * An (Incomplete) list of statistics and information about FRC-legal motors.
 */
public final class MotorConstants {
    /**
     * <pre>
     * Constants for the REV "NEO 550" Brushless Motor. Stats taken from: <a href="https://docs.revrobotics.com/brushless/neo/550">REV's 2025 docs</a>
     * NOTE: These docs do not include the motor's "typical output power" so it is not included.
     * </pre>
     */
     public static final class Neo550 {

        /**The motor's Velocity Constant*/
        public static final double kV = 917;

        /**The motor's feedforward constant. This should only be used for Velocity PIDFs as the "kFF" value. This is NOT the same as arbitrary feedforward, and may cause issues when used in non-velocity PIDFs*/
        public static final double kFF = 1.0/kV;

        /**The motor's free speed in RPM (Rotations Per Minute)*/
        public static final double freeSpeed = 11000;

        /**The motor's free running current in Amps*/
        public static final double freeCurrent = 1.7;

        /**The motor's stall current in Amps*/
        public static final double stallCurrent = 100;

        /**The motor's stall torque in Newton-meters*/
        public static final double stallTorque = 0.97;
        /**The motor's peak output power in Watts*/
        public static final double peakOutputPower = 279;

        /**The resolution of the motor's integrated hall-sensor encoder in counts per revolution*/
        public static final double encoderResolution = 42;
     }
    /**
     * Constants for the REV "NEO V1.1" Brushless Motor. Stats taken from: <a href="https://docs.revrobotics.com/brushless/neo/v1.1">REV's 2025 docs</a>
     */
     public static final class NeoBrushless {
         /**The motor's velocity constant*/
         public static final double kV = 473;

         /**The motor's feedforward constant. This should only be used for Velocity PIDFs as the "kFF" value. This is NOT the same as arbitrary feedforward, and may cause issues when used in non-velocity PIDFs*/
         public static final double kFF = 1.0/kV;

         /**The motor's free speed in RPM (Rotations Per Minute)*/
         public static final double freeSpeed = 5676;

         /**The motor's free running current in Amps*/
         public static final double freeCurrent = 1.8;

         /**The motor's stall current in Amps*/
         public static final double stallCurrent = 105;

         /**The motor's stall torque in Newton-meters*/
         public static final double stallTorque = 2.6;
         /**The motor's peak output power in Watts*/
         public static final double peakOutputPower = 406;

         /**The motor's typical output power at 40 Amps in Watts*/
         public static final double typicalOutputPower = 380;

         /**The resolution of the motor's integrated encoder in counts per revolution*/
         public static final double encoderResolution = 42;
     }
     /**
     * Constants for the REV "NEO Vortex" Brushless Motor. Stats taken from: <a href="https://docs.revrobotics.com/brushless/neo/vortex">REV's 2025 docs</a>
     */
     public static final class NeoVortex {
         /**The motor's velocity constant*/
         public static final double kV = 565;

         /**The motor's feedforward constant. This should only be used for Velocity PIDFs as the "kFF" value. This is NOT the same as arbitrary feedforward, and may cause issues when used in non-velocity PIDFs*/
         public static final double kFF = 1.0/kV;

         /**The motor's free speed in RPM (Rotations Per Minute)*/
         public static final double freeSpeed = 6784;

         /**The motor's free running current in Amps*/
         public static final double freeCurrent = 3.6;

         /**The motor's stall current in Amps*/
         public static final double stallCurrent = 211;

         /**The motor's stall torque in Newton-meters*/
         public static final double stallTorque = 3.6;
         /**The motor's peak output power in Watts*/
         public static final double peakOutputPower = 640;

         /**The motor's typical output power at 40 Amps in Watts*/
         public static final double typicalOutputPower = 375;

         /**The resolution of the motor's integrated encoder in counts per revolution*/
         public static final double encoderResolution = 42;

         /**The resolution of the SPARK Flex's integrated encoder in counts per revolution*/
         public static final double flexEncoderResolution = 7168;
     }
    /**
     * <pre>
     * Constants for the CTRE "Kraken X60" Brushless Motor. Stats taken from: <a href="https://docs.wcproducts.com/kraken-x60/kraken-x60-motor/overview-and-features/motor-performance">WCP's 2025 docs</a>
     * NOTE: These are statistics for the "Trapezoidal Commutation", not Field Oriented Control.
     * Also, the motor's docs do not state the motor's encoder resolution, so it is excluded.
     * ALSO, the motor's docs do note state the motor's velocity constant >:(, so no kV or kFF :(.
     * </pre>
     */
    public static final class KrakenX60 {

         /**The motor's free speed in RPM (Rotations Per Minute)*/
         public static final double freeSpeed = 6000;

         /**The motor's free running current in Amps*/
         public static final double freeCurrent = 2;

         /**The motor's stall current in Amps*/
         public static final double stallCurrent = 366;

         /**The motor's stall torque in Newton-meters*/
         public static final double stallTorque = 7.06;

         /**The motor's peak output power in Watts*/
         public static final double peakOutputPower = 1108;


        /**The motor's typical output power at 40 Amps in Watts*/
        public static final double typicalOutputPower = 413;

        /**The motor's typical output power at 30 Amps in Watts*/
        public static final double typicalOutputPower30 = 313;

        /**The motor's typical output power at 20 Amps in Watts*/
        public static final double typicalOutputPower20 = 207;

        /**The motor's typical output power at 10 Amps in Watts*/
        public static final double typicalOutputPower10 = 94;
     }
}
