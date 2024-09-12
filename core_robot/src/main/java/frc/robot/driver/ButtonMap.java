package frc.robot.driver;

import java.util.EnumSet;

import javax.inject.Singleton;

import frc.lib.driver.*;
import frc.lib.driver.buttons.*;
import frc.lib.driver.descriptions.*;
import frc.lib.helpers.Helpers;
import frc.robot.*;
import frc.robot.driver.controltasks.*;
import frc.robot.driver.controltasks.FollowPathTask.Type;

@Singleton
public class ButtonMap implements IButtonMap
{
    private static ShiftDescription[] ShiftButtonSchema = new ShiftDescription[]
    {
        new ShiftDescription(
            Shift.DriverDebug,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_LEFT_BUTTON),

        new ShiftDescription(
            Shift.OperatorDebug,
            UserInputDevice.Operator,
            UserInputDeviceButton.PS4_LEFT_BUTTON),
    };

    public static AnalogOperationDescription[] AnalogOperationSchema = new AnalogOperationDescription[]
    {
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainMoveForward,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_LSY,
            ElectronicsConstants.INVERT_XBONE_LEFT_Y_AXIS,
            -TuningConstants.DRIVETRAIN_DEAD_ZONE_VELOCITY,
            TuningConstants.DRIVETRAIN_DEAD_ZONE_VELOCITY,
            TuningConstants.DRIVETRAIN_MAX_VELOCITY,
            1.0),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainMoveRight,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_LSX,
            ElectronicsConstants.INVERT_XBONE_LEFT_X_AXIS,
            -TuningConstants.DRIVETRAIN_DEAD_ZONE_VELOCITY,
            TuningConstants.DRIVETRAIN_DEAD_ZONE_VELOCITY,
            TuningConstants.DRIVETRAIN_MAX_VELOCITY,
            1.0),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainTurnAngleGoal,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_RSX,
            AnalogAxis.XBONE_RSY,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            !ElectronicsConstants.INVERT_XBONE_RIGHT_X_AXIS, // make left positive...
            ElectronicsConstants.INVERT_XBONE_RIGHT_Y_AXIS,
            0.0,
            TuningConstants.DRIVETRAIN_SKIP_OMEGA_ON_ZERO_DELTA,
            true,
            TuningConstants.MAGIC_NULL_VALUE,
            (x, y) -> Helpers.atan2d(x, y)),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainTurnSpeed,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_RSX,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            !ElectronicsConstants.INVERT_XBONE_RIGHT_X_AXIS, // make left positive, as counter-clockwise is positive
            -TuningConstants.DRIVETRAIN_DEAD_ZONE_TURN,
            TuningConstants.DRIVETRAIN_DEAD_ZONE_TURN),

/*
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainRotationA,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_LT,
            ElectronicsConstants.INVERT_TRIGGER_AXIS,
            -TuningConstants.DRIVETRAIN_DEAD_ZONE_TRIGGER_AB,
            TuningConstants.DRIVETRAIN_DEAD_ZONE_TRIGGER_AB,
            TuningConstants.DRIVETRAIN_ROTATION_A_MULTIPLIER),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainRotationB,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_RT,
            ElectronicsConstants.INVERT_TRIGGER_AXIS,
            -TuningConstants.DRIVETRAIN_DEAD_ZONE_TRIGGER_AB,
            TuningConstants.DRIVETRAIN_DEAD_ZONE_TRIGGER_AB,
            TuningConstants.DRIVETRAIN_ROTATION_B_MULTIPLIER),
*/
        new AnalogOperationDescription(
            AnalogOperation.PowerCellFlywheelVelocity,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_LT,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            false,
            -1.0,
            0.125,
            0.175,
            1.0),

        new AnalogOperationDescription(
            AnalogOperation.PowerCellFlywheelVelocity2,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_LT,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            false,
            -1.0,
            0.125,
            0.50,
            1.0),

/*        new AnalogOperationDescription(
            AnalogOperation.PowerCellFlywheelVelocity,
            UserInputDevice.Operator,
            AnalogAxis.PS4_LSX,
            AnalogAxis.PS4_LSY,
            EnumSet.of(Shift.OperatorDebug),
            EnumSet.of(Shift.OperatorDebug),
            ElectronicsConstants.INVERT_PS4_LEFT_X_AXIS,
            ElectronicsConstants.INVERT_PS4_LEFT_Y_AXIS,
            0.0,
            TuningConstants.DRIVETRAIN_SKIP_OMEGA_ON_ZERO_DELTA,
            true,
            1.0,
            0,
            (x, y) -> (Helpers.atan2d(x, y) + 180.0) / 360),*/
        // new AnalogOperationDescription(
        //     AnalogOperation.PowerCellCarousel,
        //     UserInputDevice.Driver,
        //     AnalogAxis.XBONE_LT,
        //    EnumSet.of(Shift.DriverDebug),
        //    EnumSet.of(Shift.DriverDebug),
        //     ElectronicsConstants.INVERT_TRIGGER_AXIS,
        //     -1.01,
        //     TuningConstants.DRIVETRAIN_DEAD_ZONE_TRIGGER_AB),
    };

    public static DigitalOperationDescription[] DigitalOperationSchema = new DigitalOperationDescription[]
    {
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainReset,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_A_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainEnableFieldOrientation,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_X_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Click),
        // new DigitalOperationDescription(
        //     DigitalOperation.DriveTrainDisableFieldOrientation,
        //     UserInputDevice.Driver,
        //     UserInputDeviceButton.XBONE_B_BUTTON,
        //     EnumSet.of(Shift.DriverDebug),
        //     EnumSet.of(Shift.DriverDebug),
        //     ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.PositionResetFieldOrientation,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainEnableMaintainDirectionMode,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_RIGHT_STICK_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.DriveTrainDisableMaintainDirectionMode,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_LEFT_STICK_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Click),

        new DigitalOperationDescription(
            DigitalOperation.PowerCellOuttake,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_RT,
            0.5,
            1.0,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.PowerCellIntake,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_RT,
            0.5,
            1.0,
            EnumSet.of(Shift.DriverDebug),
           EnumSet.noneOf(Shift.class),
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.PowerCellIntakeExtend,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_Y_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.PowerCellIntakeRetract,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_A_BUTTON,
            EnumSet.of(Shift.DriverDebug),
           EnumSet.noneOf(Shift.class),
            ButtonType.Click),
/*
        new DigitalOperationDescription(
            DigitalOperation.PowerCellIntakeExtend,
            UserInputDevice.Operator,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_9,
            EnumSet.of(Shift.OperatorDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.PowerCellIntakeRetract,
            UserInputDevice.Operator,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_10,
            EnumSet.of(Shift.OperatorDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.PowerCellOuttake,
            UserInputDevice.Operator,
            UserInputDeviceButton.PS4_RIGHT_BUTTON,
            EnumSet.of(Shift.OperatorDebug),
            EnumSet.of(Shift.OperatorDebug),
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.PowerCellIntake,
            UserInputDevice.Operator,
            UserInputDeviceButton.PS4_RIGHT_BUTTON,
            EnumSet.of(Shift.OperatorDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.PowerCellIntakeExtend,
            UserInputDevice.Operator,
            UserInputDeviceButton.PS4_TRIANGLE_BUTTON,
            EnumSet.of(Shift.OperatorDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.PowerCellIntakeRetract,
            UserInputDevice.Operator,
            UserInputDeviceButton.PS4_X_BUTTON,
            EnumSet.of(Shift.OperatorDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Click),
*/

        new DigitalOperationDescription(
            DigitalOperation.PowerCellHoodPointBlank,
            UserInputDevice.Driver,
            180,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.PowerCellHoodShort,
            UserInputDevice.Driver,
            90,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.PowerCellHoodMedium,
            UserInputDevice.Driver,
            270,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.PowerCellHoodLong,
            UserInputDevice.Driver,
            0,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.of(Shift.DriverDebug),
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.PowerCellFlywheelReverse,
            UserInputDevice.Operator,
            0,
            EnumSet.of(Shift.OperatorDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.PowerCellKickerSpinReverse,
            UserInputDevice.Operator,
            180,
            EnumSet.of(Shift.OperatorDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.PowerCellKick,
            UserInputDevice.Operator,
            UserInputDeviceButton.PS4_TRIANGLE_BUTTON,
            EnumSet.of(Shift.OperatorDebug),
            EnumSet.of(Shift.OperatorDebug),
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.PowerCellKickerSpin,
            UserInputDevice.Operator,
            UserInputDeviceButton.PS4_X_BUTTON,
            EnumSet.of(Shift.OperatorDebug),
            EnumSet.of(Shift.OperatorDebug),
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.VisionEnableRetroreflectiveProcessing,
            UserInputDevice.Operator,
            UserInputDeviceButton.PS4_PLAYSTATION_BUTTON,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.PositionBeginTemperatureCalibration,
            UserInputDevice.Operator,
            UserInputDeviceButton.PS4_OPTIONS_BUTTON,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            ButtonType.Click),
    };

    public static MacroOperationDescription[] MacroSchema = new MacroOperationDescription[]
    {
        new MacroOperationDescription(
            MacroOperation.PIDBrake,
            UserInputDevice.Driver,
            180, // DPad Down
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Simple,
            () -> new PIDBrakeTask(),
            new IOperation[]
            {
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainTurnSpeed,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
            }),
        new MacroOperationDescription(
            MacroOperation.VisionCenter,
            UserInputDevice.Driver,
            0, // DPad Up
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> SequentialTask.Sequence(
                new VisionCenteringTask(),
                new DriveTrainFieldOrientationModeTask(true)),
            new IOperation[]
            {
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainMoveRight,
                AnalogOperation.DriveTrainTurnAngleGoal,
                AnalogOperation.DriveTrainTurnSpeed,
                AnalogOperation.DriveTrainRotationA,
                AnalogOperation.DriveTrainRotationB,
                AnalogOperation.DriveTrainPathXGoal,
                AnalogOperation.DriveTrainPathYGoal,
                AnalogOperation.DriveTrainPathXVelocityGoal,
                AnalogOperation.DriveTrainPathYVelocityGoal,
                AnalogOperation.DriveTrainPathAngleVelocityGoal,
                AnalogOperation.DriveTrainPositionDrive1,
                AnalogOperation.DriveTrainPositionDrive2,
                AnalogOperation.DriveTrainPositionDrive3,
                AnalogOperation.DriveTrainPositionDrive4,
                AnalogOperation.DriveTrainPositionSteer1,
                AnalogOperation.DriveTrainPositionSteer2,
                AnalogOperation.DriveTrainPositionSteer3,
                AnalogOperation.DriveTrainPositionSteer4,
                DigitalOperation.DriveTrainPositionMode,
                DigitalOperation.DriveTrainPathMode,
                DigitalOperation.DriveTrainReset,
                DigitalOperation.DriveTrainEnableFieldOrientation,
                DigitalOperation.DriveTrainDisableFieldOrientation,
                DigitalOperation.VisionDisableStream,
                DigitalOperation.VisionEnablePowercellProcessing,
                DigitalOperation.VisionEnableRetroreflectiveProcessing,
                DigitalOperation.VisionForceDisable,
            }),
        new MacroOperationDescription(
            MacroOperation.ShootHopper,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_B_BUTTON,
            EnumSet.noneOf(Shift.class),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new FullHopperShotTask(),
            new IOperation[]
            {
                DigitalOperation.PowerCellKick,
                DigitalOperation.PowerCellKickerSpin,
                DigitalOperation.PowerCellRotateCarousel,
            }),
        new MacroOperationDescription(
            MacroOperation.SpinFlywheel,
            UserInputDevice.Driver,
            UserInputDeviceButton.XBONE_X_BUTTON,
            EnumSet.of(Shift.DriverDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> ConcurrentTask.AllTasks( //new FlywheelVisionSpinTask(),
                new FlywheelFixedSpinTask(TuningConstants.POWERCELL_FLYWHEEL_SHOOT_PERCENTILE, 10.0)),
            new IOperation[]
            {
                AnalogOperation.PowerCellFlywheelVelocity,
                AnalogOperation.PowerCellFlywheelVelocity2
            }),

/*
        new MacroOperationDescription(
            MacroOperation.ShootHopper,
            UserInputDevice.Operator,
            UserInputDeviceButton.PS4_CIRCLE_BUTTON,
            EnumSet.of(Shift.OperatorDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> new FullHopperShotTask(),
            new IOperation[]
            {
                DigitalOperation.PowerCellKick,
                DigitalOperation.PowerCellKickerSpin,
                DigitalOperation.PowerCellRotateCarousel,
            }),
        new MacroOperationDescription(
            MacroOperation.SpinFlywheel,
            UserInputDevice.Operator,
            UserInputDeviceButton.PS4_SQUARE_BUTTON,
            EnumSet.of(Shift.OperatorDebug),
            EnumSet.noneOf(Shift.class),
            ButtonType.Toggle,
            () -> ConcurrentTask.AllTasks( //new FlywheelVisionSpinTask(),
                new FlywheelFixedSpinTask(0.45, 10.0),
                new ShooterHoodPositionTask(DigitalOperation.PowerCellHoodShort)),
            new IOperation[]
            {
                AnalogOperation.PowerCellFlywheelVelocity,
                AnalogOperation.PowerCellFlywheelVelocity2,
                DigitalOperation.PowerCellHoodPointBlank,
                DigitalOperation.PowerCellHoodShort,
                DigitalOperation.PowerCellHoodMedium,
                DigitalOperation.PowerCellHoodLong,
            }),
*/
    };

    @Override
    public ShiftDescription[] getShiftSchema()
    {
        return ButtonMap.ShiftButtonSchema;
    }

    @Override
    public AnalogOperationDescription[] getAnalogOperationSchema()
    {
        return ButtonMap.AnalogOperationSchema;
    }

    @Override
    public DigitalOperationDescription[] getDigitalOperationSchema()
    {
        return ButtonMap.DigitalOperationSchema;
    }

    @Override
    public MacroOperationDescription[] getMacroOperationSchema()
    {
        return ButtonMap.MacroSchema;
    }
}
