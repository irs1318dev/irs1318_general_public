package frc.robot.driver;

import javax.inject.Singleton;

import frc.robot.*;
import frc.robot.driver.common.*;
import frc.robot.driver.common.buttons.*;
import frc.robot.driver.common.descriptions.*;
import frc.robot.driver.controltasks.*;

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
            Shift.CodriverDebug,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_16),
        new ShiftDescription(
            Shift.Test1Debug,
            UserInputDevice.Test1,
            UserInputDeviceButton.XBONE_LEFT_BUTTON),
        // new ShiftDescription(
        //     Shift.Test2Debug,
        //     UserInputDevice.Test2,
        //     UserInputDeviceButton.XBONE_LEFT_BUTTON),
    };

    public static AnalogOperationDescription[] AnalogOperationSchema = new AnalogOperationDescription[]
    {
        // DriveTrain operations
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainMoveForward,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_LSY,
            ElectronicsConstants.INVERT_XBONE_LEFT_Y_AXIS,
            TuningConstants.DRIVETRAIN_Y_DEAD_ZONE),
        new AnalogOperationDescription(
            AnalogOperation.DriveTrainTurn,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_LSX,
            ElectronicsConstants.INVERT_XBONE_RIGHT_X_AXIS,
            TuningConstants.DRIVETRAIN_X_DEAD_ZONE),

        // Operations for the OneMotor
        new AnalogOperationDescription(
            AnalogOperation.OneMotorPower,
            UserInputDevice.Driver,
            AnalogAxis.XBONE_RSY,
            ElectronicsConstants.INVERT_XBONE_LEFT_Y_AXIS,
            0.05),
    };

    public static DigitalOperationDescription[] DigitalOperationSchema = new DigitalOperationDescription[]
    {
        // driving operations
        new DigitalOperationDescription(
            DigitalOperation.PositionResetFieldOrientation,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_1,
            Shift.CodriverDebug,
            Shift.None,
            ButtonType.Click),
        new DigitalOperationDescription(
            DigitalOperation.VisionEnableGamePieceProcessing,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_2,
            Shift.None,
            Shift.None,
            ButtonType.Simple),
        new DigitalOperationDescription(
            DigitalOperation.VisionEnableRetroreflectiveProcessing,
            UserInputDevice.Codriver,
            UserInputDeviceButton.BUTTON_PAD_BUTTON_3,
            Shift.None,
            Shift.None,
            ButtonType.Simple),
    };

    public static MacroOperationDescription[] MacroSchema = new MacroOperationDescription[]
    {
        // driving macros
        new MacroOperationDescription(
            MacroOperation.PIDBrake,
            UserInputDevice.Driver,
            0, // DPAD-up
            Shift.DriverDebug,
            Shift.None,
            ButtonType.Simple,
            () -> new PIDBrakeTask(),
            new IOperation[]
            {
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainTurn,
                AnalogOperation.DriveTrainLeftPosition,
                AnalogOperation.DriveTrainRightPosition,
                AnalogOperation.DriveTrainLeftVelocity,
                AnalogOperation.DriveTrainRightVelocity,
                AnalogOperation.DriveTrainHeadingCorrection,
                DigitalOperation.DriveTrainEnablePID,
                DigitalOperation.DriveTrainDisablePID,
                DigitalOperation.DriveTrainSimpleMode,
                DigitalOperation.DriveTrainUseBrakeMode,
                DigitalOperation.DriveTrainUsePositionalMode,
                DigitalOperation.DriveTrainUsePathMode,
                DigitalOperation.DriveTrainSwapFrontOrientation,
            }),
        new MacroOperationDescription(
            MacroOperation.VisionCenterHub,
            UserInputDevice.Driver,
            0, // DPAD-up
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Toggle,
            () -> new VisionCenteringTask(false, true),
            new IOperation[]
            {
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainTurn,
                AnalogOperation.DriveTrainLeftPosition,
                AnalogOperation.DriveTrainRightPosition,
                AnalogOperation.DriveTrainLeftVelocity,
                AnalogOperation.DriveTrainRightVelocity,
                AnalogOperation.DriveTrainHeadingCorrection,
                DigitalOperation.DriveTrainEnablePID,
                DigitalOperation.DriveTrainDisablePID,
                DigitalOperation.DriveTrainSimpleMode,
                DigitalOperation.DriveTrainUseBrakeMode,
                DigitalOperation.DriveTrainUsePositionalMode,
                DigitalOperation.DriveTrainUsePathMode,
                DigitalOperation.DriveTrainSwapFrontOrientation,
                DigitalOperation.VisionDisableStream,
                DigitalOperation.VisionEnableGamePieceProcessing,
                DigitalOperation.VisionEnableRetroreflectiveProcessing,
                DigitalOperation.VisionForceDisable,
            }),
        new MacroOperationDescription(
            MacroOperation.VisionCenterCargo,
            UserInputDevice.Driver,
            90, // DPAD-right
            Shift.DriverDebug,
            Shift.DriverDebug,
            ButtonType.Toggle,
            () -> new VisionCenteringTask(true, true),
            new IOperation[]
            {
                AnalogOperation.DriveTrainMoveForward,
                AnalogOperation.DriveTrainTurn,
                AnalogOperation.DriveTrainLeftPosition,
                AnalogOperation.DriveTrainRightPosition,
                AnalogOperation.DriveTrainLeftVelocity,
                AnalogOperation.DriveTrainRightVelocity,
                AnalogOperation.DriveTrainHeadingCorrection,
                DigitalOperation.DriveTrainEnablePID,
                DigitalOperation.DriveTrainDisablePID,
                DigitalOperation.DriveTrainSimpleMode,
                DigitalOperation.DriveTrainUseBrakeMode,
                DigitalOperation.DriveTrainUsePositionalMode,
                DigitalOperation.DriveTrainUsePathMode,
                DigitalOperation.DriveTrainSwapFrontOrientation,
                DigitalOperation.VisionDisableStream,
                DigitalOperation.VisionEnableGamePieceProcessing,
                DigitalOperation.VisionEnableRetroreflectiveProcessing,
                DigitalOperation.VisionForceDisable,
            }),
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