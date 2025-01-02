package frc.robot;

import frc.lib.robotprovider.PneumaticsModuleType;
import frc.lib.robotprovider.PowerDistributionModuleType;

/**
 * All constants describing how the electronics are plugged together.
 * 
 * @author Will
 * 
 */
public class ElectronicsConstants
{
    public static final int PCM_A_MODULE = 0;
    public static final int PCM_B_MODULE = 1;

    // We expect the following to be true.  Change INVERT_*_AXIS to true if any of the following are not met:
    // 1. forwards/up on a joystick is positive, backwards/down is negative.
    // 2. right on a joystick is positive, left on a joystick is negative.
    // 3. pressed on a trigger is positive, released is negative/zero.
    public static final boolean INVERT_XBONE_LEFT_X_AXIS = false;
    public static final boolean INVERT_XBONE_RIGHT_X_AXIS = false;
    public static final boolean INVERT_XBONE_LEFT_Y_AXIS = true;
    public static final boolean INVERT_XBONE_RIGHT_Y_AXIS = true;
    public static final boolean INVERT_XBONE_LEFT_TRIGGER = false;
    public static final boolean INVERT_XBONE_RIGHT_TRIGGER = false;

    public static final boolean INVERT_PS4_LEFT_X_AXIS = false;
    public static final boolean INVERT_PS4_RIGHT_X_AXIS = false;
    public static final boolean INVERT_PS4_LEFT_Y_AXIS = true;
    public static final boolean INVERT_PS4_RIGHT_Y_AXIS = true;
    public static final boolean INVERT_PS4_LEFT_TRIGGER = false;
    public static final boolean INVERT_PS4_RIGHT_TRIGGER = false;

    public static final boolean INVERT_THROTTLE_AXIS = true;
    public static final boolean INVERT_TRIGGER_AXIS = false;

    public static final int POWER_DISTRIBUTION_CAN_ID = 1;
    public static final PowerDistributionModuleType POWER_DISTRIBUTION_TYPE = PowerDistributionModuleType.PowerDistributionHub;

    public static final double REV_THROUGHBORE_ENCODER_DUTY_CYCLE_MIN = 1.0 / 1024.0;
    public static final double REV_THROUGHBORE_ENCODER_DUTY_CYCLE_MAX = 1023.0 / 1024.0;

    public static final String CANIVORE_NAME = "CANIVORE1"; // Module A

    public static final int PNEUMATICS_MODULE_A = 1; // Module A
    public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE_A = PneumaticsModuleType.PneumaticsControlModule; // Module A

    public static final int PNEUMATICS_MODULE_B = 2; // Module B
    public static final PneumaticsModuleType PNEUMATICS_MODULE_TYPE_B = PneumaticsModuleType.PneumaticsControlModule; // Module B

    public static final boolean PNEUMATICS_USE_HYBRID = false;
    public static final boolean PNEUMATICS_USE_ANALOG = false;
    public static final double PNEUMATICS_MIN_PSI = 110.0;
    public static final double PNEUMATICS_MAX_PSI = 120.0;

    //================================================== IMU ==============================================================

    public static final int PIGEON_IMU_CAN_ID = 0;

    //================================================== Indicator Lights ==============================================================

    public static final int INDICATOR_LIGHT_CANDLE_CAN_ID = 55;

    //================================================== Tank DriveTrain ==============================================================

    public static final int TANK_DRIVETRAIN_LEFT_MASTER_CAN_ID = 1;
    public static final int TANK_DRIVETRAIN_LEFT_MASTER_PDP_SLOT = 1;
    public static final int TANK_DRIVETRAIN_LEFT_FOLLOWER_CAN_ID = 2;
    public static final int TANK_DRIVETRAIN_LEFT_FOLLOWER_PDP_SLOT = 0;
    public static final int TANK_DRIVETRAIN_RIGHT_MASTER_CAN_ID = 3;
    public static final int TANK_DRIVETRAIN_RIGHT_MASTER_PDP_SLOT = 14;
    public static final int TANK_DRIVETRAIN_RIGHT_FOLLOWER_CAN_ID = 4;
    public static final int TANK_DRIVETRAIN_RIGHT_FOLLOWER_PDP_SLOT = 15;

    public static final int SIMPLETANK_DRIVETRAIN_LEFT_TALON_CHANNEL = 5;
    public static final int SIMPLETANK_DRIVETRAIN_RIGHT_TALON_CHANNEL = 4;
    public static final int SIMPLETANK_DRIVETRAIN_LEFT_ENCODER_CHANNEL_A = 2;
    public static final int SIMPLETANK_DRIVETRAIN_LEFT_ENCODER_CHANNEL_B = 3;
    public static final int SIMPLETANK_DRIVETRAIN_RIGHT_ENCODER_CHANNEL_A = 0;
    public static final int SIMPLETANK_DRIVETRAIN_RIGHT_ENCODER_CHANNEL_B = 1;

    //=================================================== Shooter ===================================================================

    public static final int SHOOTER_TALON_CHANNEL = 1;
    public static final int SHOOTER_KICKER_CHANNEL_A = 3;
    public static final int SHOOTER_KICKER_CHANNEL_B = 2;
    public static final int SHOOTER_HOOD_CHANNEL_A = 5;
    public static final int SHOOTER_HOOD_CHANNEL_B = 2;
    public static final int SHOOTER_ENCODER_CHANNEL_A = 8;
    public static final int SHOOTER_ENCODER_CHANNEL_B = 9;
    public static final int SHOOTER_READY_LIGHT_PORT = 1;
    public static final int SHOOTER_TARGETING_LIGHT_PORT = 4;

    //=================================================== Intake =================================================================

    public static final int INTAKE_MOTOR_CHANNEL = 3;
    public static final int INTAKE_LIGHT_CHANNEL = 0;
    public static final int INTAKE_THROUGH_BEAM_SENSOR_CHANNEL = 0;

    public static final int INTAKE_SOLENOID_CHANNEL_A = 4;
    public static final int INTAKE_SOLENOID_CHANNEL_B = 3;

    //================================================== Stinger ==================================================

    public static final int STINGER_MOTOR_CHANNEL = 0;
}
