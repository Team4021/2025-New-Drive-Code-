package frc.lib.util.swerveUtil;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.util.Units;

/* Contains values and required settings for common COTS swerve modules. */
public class COTSNeoSwerveConstants {
    public final double wheelDiameter;
    public final double wheelCircumference;
    public final double angleGearRatio;
    public final double driveGearRatio;
    public final double angleKP;
    public final double angleKI;
    public final double angleKD;
    public final double angleKF;
    public final boolean driveMotorInvert;
    public final boolean angleMotorInvert;
    public SensorDirectionValue cancoderInvert;

    public COTSNeoSwerveConstants(double wheelDiameter, double angleGearRatio, double driveGearRatio, double angleKP, double angleKI, double angleKD, double angleKF, boolean driveMotorInvert, boolean angleMotorInvert, SensorDirectionValue canCoderInvert){
        this.wheelDiameter = wheelDiameter;
        this.wheelCircumference = wheelDiameter * Math.PI;
        this.angleGearRatio = angleGearRatio;
        this.driveGearRatio = driveGearRatio;
        this.angleKP = angleKP;
        this.angleKI = angleKI;
        this.angleKD = angleKD;
        this.angleKF = angleKF;
        this.driveMotorInvert = driveMotorInvert;
        this.angleMotorInvert = angleMotorInvert;
        this.cancoderInvert = canCoderInvert;
    }
    
    /** Swerve Drive Specialties - MK3 Module*/
    public static COTSNeoSwerveConstants SDSMK3(double driveGearRatio){
        double wheelDiameter = Units.inchesToMeters(4.0);
 
        /** 12.8 : 1 */
        double angleGearRatio = (12.8 / 1.0);
 
        double angleKP = 0.2;
        double angleKI = 0.0;
        double angleKD = 0.0;
        double angleKF = 0.0;
 
        boolean driveMotorInvert = false;
        boolean angleMotorInvert = false;
        SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;
        return new COTSNeoSwerveConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, angleKF, driveMotorInvert, angleMotorInvert, cancoderInvert);
    }

    /** Swerve Drive Specialties - MK4 Module*/
    public static COTSNeoSwerveConstants SDSMK4(double driveGearRatio){
        double wheelDiameter = Units.inchesToMeters(4.0);
 
        /** 12.8 : 1 */
        double angleGearRatio = (12.8 / 1.0);
 
        double angleKP = 0.2;
        double angleKI = 0.0;
        double angleKD = 0.0;
        double angleKF = 0.0;
 
        boolean driveMotorInvert = false;
        boolean angleMotorInvert = false;
        SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;
        return new COTSNeoSwerveConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, angleKF, driveMotorInvert, angleMotorInvert, cancoderInvert);
    }

    /** Swerve Drive Specialties - MK4i Module*/
    public static COTSNeoSwerveConstants SDSMK4i(double driveGearRatio){
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (150 / 7) : 1 */
        double angleGearRatio = ((150.0 / 7.0) / 1.0);

        double angleKP = 0.3;
        double angleKI = 0.0;
        double angleKD = 0.0;
        double angleKF = 0.0;

        boolean driveMotorInvert = false;
        boolean angleMotorInvert = true;
        SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;
        return new COTSNeoSwerveConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, angleKF, driveMotorInvert, angleMotorInvert, cancoderInvert);
    }

    /** West Coast Products - Flipped Gear Module*/
    public static COTSNeoSwerveConstants WCPxFlippedGear(double driveGearRatio){
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (150 / 7) : 1 */
        double angleGearRatio = (13.71 / 1.0);

        double angleKP = 0.3; //this value does need to be tested :skull:
        double angleKI = 0.0;
        double angleKD = 0.0;
        double angleKF = 0.0;

        boolean driveMotorInvert = false;
        boolean angleMotorInvert = true;
        SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;
        return new COTSNeoSwerveConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, angleKF, driveMotorInvert, angleMotorInvert, cancoderInvert);
    }

    /** West Coast Products - Flipped Belt Module*/
    public static COTSNeoSwerveConstants WCPxFlippedBelt(double driveGearRatio){
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (150 / 7) : 1 */
        double angleGearRatio = (10.29 / 1.0);

        double angleKP = 0.3; //this value does need to be tested :skull:
        double angleKI = 0.0;
        double angleKD = 0.0;
        double angleKF = 0.0;

        boolean driveMotorInvert = false;
        boolean angleMotorInvert = false;
        SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;
        return new COTSNeoSwerveConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, angleKF, driveMotorInvert, angleMotorInvert, cancoderInvert);
    }

    /** West Coast Products - Non-Flipped Belt Module*/
    public static COTSNeoSwerveConstants WCPxNonFlippedBelt(double driveGearRatio){
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (150 / 7) : 1 */
        double angleGearRatio = (13.71 / 1.0);

        double angleKP = 0.3; //this value does need to be tested :skull:
        double angleKI = 0.0;
        double angleKD = 0.0;
        double angleKF = 0.0;

        boolean driveMotorInvert = false;
        boolean angleMotorInvert = true;
        SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;
        return new COTSNeoSwerveConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, angleKF, driveMotorInvert, angleMotorInvert, cancoderInvert);
    }

    /** REV Robotics - MAXSwerve Module */
    public static COTSNeoSwerveConstants REVMaxSwerve(double driveGearRatio){
        double wheelDiameter = Units.inchesToMeters(4.0);

        /** (150 / 7) : 1 */
        double angleGearRatio = ((9424.0 / 203.0) / 1.0);

        double angleKP = 0.3; //this value does need to be tested :skull:
        double angleKI = 0.0;
        double angleKD = 0.0;
        double angleKF = 0.0;

        boolean driveMotorInvert = false;
        boolean angleMotorInvert = true;
        SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;
        return new COTSNeoSwerveConstants(wheelDiameter, angleGearRatio, driveGearRatio, angleKP, angleKI, angleKD, angleKF, driveMotorInvert, angleMotorInvert, cancoderInvert);
    }

    /* Drive Gear Ratios for all supported modules */
    public class driveGearRatios{
        /* SDS MK3 */
        /** SDS MK3 - 8.16 : 1 */
        public static final double SDSMK3_Standard = (8.16 / 1.0);
        /** SDS MK3 - 6.86 : 1 */
        public static final double SDSMK3_Fast = (6.86 / 1.0);

        /* SDS MK4 */
        /** SDS MK4 - 8.14 : 1 */
        public static final double SDSMK4_L1 = (8.14 / 1.0);
        /** SDS MK4 - 6.75 : 1 */
        public static final double SDSMK4_L2 = (6.75 / 1.0);
        /** SDS MK4 - 6.12 : 1 */
        public static final double SDSMK4_L3 = (6.12 / 1.0);
        /** SDS MK4 - 5.14 : 1 */
        public static final double SDSMK4_L4 = (5.14 / 1.0);
        
        /* SDS MK4i */
        /** SDS MK4i - 8.14 : 1 */
        public static final double SDSMK4i_L1 = (8.14 / 1.0);
        /** SDS MK4i - 6.75 : 1 */
        public static final double SDSMK4i_L2 = (6.75 / 1.0);
        /** SDS MK4i - 6.12 : 1 */
        public static final double SDSMK4i_L3 = (6.12 / 1.0);
        /* WCP Swerve X */
        /* Flipped Gear */
        public static final double WCPXflippedGear_675 = (6.75 / 1.0);
        public static final double WCPXflippedGear_736 = (7.36 / 1.0);
        public static final double WCPXflippedGear_810 = (8.10 / 1.0);
        /* Flipped Belt */
        public static final double WCPXflippedBelt_550 = (5.50 / 1.0);
        public static final double WCPXflippedBelt_655 = (5.55 / 1.0);
        public static final double WCPXflippedBelt_780 = (7.80 / 1.0);
        /* Non-Flipped */
        public static final double WCPXnonFlipped_550 = (6.54 / 1.0);
        public static final double WCPXnonFlipped_655 = (7.13 / 1.0);
        public static final double WCPXnonFlipped_780 = (7.85 / 1.0);
        /* REV MaxSwerve */
        public static final double REVMax12T = (5.50 / 1.0);
        public static final double REVMax13T = (5.08 / 1.0);
        public static final double REVMax14T = (4.71 / 1.0);


    }
}
