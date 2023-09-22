package frc.robot.mechanisms;

import static org.mockito.Matchers.eq;
import static org.mockito.Mockito.doReturn;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.verifyNoMoreInteractions;

import org.junit.jupiter.api.Test;
import frc.lib.driver.Driver;
import frc.lib.mechanisms.LoggingManager;
import frc.lib.robotprovider.*;
import frc.robot.*;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;

public class TankDriveTrainMechanismTests
{
    @Test
    public void testSetPower_Zero()
    {
        LoggingManager logger = mock(LoggingManager.class);
        ITimer timer = mock(ITimer.class);
        TestProvider testProvider = new TestProvider();
        ITalonFX leftMotor = testProvider.getTalonFX(ElectronicsConstants.TANK_DRIVETRAIN_LEFT_MASTER_CAN_ID);
        ITalonFX rightMotor = testProvider.getTalonFX(ElectronicsConstants.TANK_DRIVETRAIN_RIGHT_MASTER_CAN_ID);
        ITalonFX leftFollowerMotor = testProvider.getTalonFX(ElectronicsConstants.TANK_DRIVETRAIN_LEFT_FOLLOWER_CAN_ID);
        ITalonFX rightFollowerMotor = testProvider.getTalonFX(ElectronicsConstants.TANK_DRIVETRAIN_RIGHT_FOLLOWER_CAN_ID);

        Driver driver = mock(Driver.class);

        doReturn(false).when(driver).getDigital(DigitalOperation.DriveTrainDisablePID);
        doReturn(false).when(driver).getDigital(DigitalOperation.DriveTrainEnablePID);
        doReturn(0.0).when(driver).getAnalog(AnalogOperation.DriveTrainLeftPosition);
        doReturn(0.0).when(driver).getAnalog(AnalogOperation.DriveTrainRightPosition);
        doReturn(false).when(driver).getDigital(DigitalOperation.DriveTrainUsePositionalMode);
        doReturn(false).when(driver).getDigital(DigitalOperation.DriveTrainSwapFrontOrientation);
        doReturn(false).when(driver).getDigital(DigitalOperation.DriveTrainSimpleMode);
        doReturn(0.0).when(driver).getAnalog(AnalogOperation.DriveTrainMoveForward);
        doReturn(0.0).when(driver).getAnalog(AnalogOperation.DriveTrainTurn);
        doReturn(0.0).when(leftMotor).getError();
        doReturn(0.0).when(leftMotor).getVelocity();
        doReturn(0.0).when(leftMotor).getPosition();
        doReturn(0.0).when(rightMotor).getError();
        doReturn(0.0).when(rightMotor).getVelocity();
        doReturn(0.0).when(rightMotor).getPosition();

        TankDriveTrainMechanism driveTrainMechanism = new TankDriveTrainMechanism(driver, logger, testProvider, timer);
        driveTrainMechanism.readSensors();
        driveTrainMechanism.update();

        // from constructor:
        verify(leftMotor).setNeutralMode(eq(MotorNeutralMode.Brake));
        verify(leftMotor).setInvertOutput(eq(HardwareConstants.TANK_DRIVETRAIN_LEFT_MASTER_INVERT_OUTPUT));
        verify(leftMotor).setInvertSensor(eq(HardwareConstants.TANK_DRIVETRAIN_LEFT_INVERT_SENSOR));
        verify(leftMotor).setSensorType(TalonXFeedbackDevice.IntegratedSensor);
        verify(leftMotor).setVoltageCompensation(TuningConstants.TANK_DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED, TuningConstants.TANK_DRIVETRAIN_VOLTAGE_COMPENSATION);
        verify(leftMotor).setSupplyCurrentLimit(TuningConstants.TANK_DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED, TuningConstants.TANK_DRIVETRAIN_SUPPLY_CURRENT_MAX, TuningConstants.TANK_DRIVETRAIN_SUPPLY_TRIGGER_CURRENT, TuningConstants.TANK_DRIVETRAIN_SUPPLY_TRIGGER_DURATION);
        verify(leftMotor).setFeedbackFramePeriod(5);
        verify(leftMotor).setPIDFFramePeriod(5);
        verify(leftMotor).configureVelocityMeasurements(eq(10), eq(32));
        verify(leftMotor).setSelectedSlot(eq(0));
        verify(leftFollowerMotor).setNeutralMode(eq(MotorNeutralMode.Brake));
        verify(leftFollowerMotor).setInvertOutput(eq(HardwareConstants.TANK_DRIVETRAIN_LEFT_FOLLOWER1_INVERT_OUTPUT));
        verify(leftFollowerMotor).follow(eq(leftMotor));
        verify(leftFollowerMotor).setVoltageCompensation(TuningConstants.TANK_DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED, TuningConstants.TANK_DRIVETRAIN_VOLTAGE_COMPENSATION);
        verify(leftFollowerMotor).setSupplyCurrentLimit(TuningConstants.TANK_DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED, TuningConstants.TANK_DRIVETRAIN_SUPPLY_CURRENT_MAX, TuningConstants.TANK_DRIVETRAIN_SUPPLY_TRIGGER_CURRENT, TuningConstants.TANK_DRIVETRAIN_SUPPLY_TRIGGER_DURATION);
        verify(rightMotor).setNeutralMode(eq(MotorNeutralMode.Brake));
        verify(rightMotor).setInvertOutput(eq(HardwareConstants.TANK_DRIVETRAIN_RIGHT_MASTER_INVERT_OUTPUT));
        verify(rightMotor).setInvertSensor(eq(HardwareConstants.TANK_DRIVETRAIN_RIGHT_INVERT_SENSOR));
        verify(rightMotor).setSensorType(TalonXFeedbackDevice.IntegratedSensor);
        verify(rightMotor).setVoltageCompensation(TuningConstants.TANK_DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED, TuningConstants.TANK_DRIVETRAIN_VOLTAGE_COMPENSATION);
        verify(rightMotor).setSupplyCurrentLimit(TuningConstants.TANK_DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED, TuningConstants.TANK_DRIVETRAIN_SUPPLY_CURRENT_MAX, TuningConstants.TANK_DRIVETRAIN_SUPPLY_TRIGGER_CURRENT, TuningConstants.TANK_DRIVETRAIN_SUPPLY_TRIGGER_DURATION);
        verify(rightMotor).setFeedbackFramePeriod(5);
        verify(rightMotor).setPIDFFramePeriod(5);
        verify(rightMotor).configureVelocityMeasurements(eq(10), eq(32));
        verify(rightMotor).setSelectedSlot(eq(0));
        verify(rightFollowerMotor).setNeutralMode(eq(MotorNeutralMode.Brake));
        verify(rightFollowerMotor).setInvertOutput(eq(HardwareConstants.TANK_DRIVETRAIN_RIGHT_FOLLOWER1_INVERT_OUTPUT));
        verify(rightFollowerMotor).follow(eq(rightMotor));
        verify(rightFollowerMotor).setVoltageCompensation(TuningConstants.TANK_DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED, TuningConstants.TANK_DRIVETRAIN_VOLTAGE_COMPENSATION);
        verify(rightFollowerMotor).setSupplyCurrentLimit(TuningConstants.TANK_DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED, TuningConstants.TANK_DRIVETRAIN_SUPPLY_CURRENT_MAX, TuningConstants.TANK_DRIVETRAIN_SUPPLY_TRIGGER_CURRENT, TuningConstants.TANK_DRIVETRAIN_SUPPLY_TRIGGER_DURATION);

        // from setDriver:
        verify(leftMotor).setPIDF(
            eq(TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_LEFT_KP),
            eq(TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_LEFT_KI),
            eq(TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_LEFT_KD),
            eq(TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_LEFT_KF),
            eq(0));
        verify(rightMotor).setPIDF(
            eq(TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_RIGHT_KP),
            eq(TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_RIGHT_KI),
            eq(TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_RIGHT_KD),
            eq(TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_RIGHT_KF),
            eq(0));
        verify(leftMotor).setControlMode(eq(TalonXControlMode.Velocity));
        verify(rightMotor).setControlMode(eq(TalonXControlMode.Velocity));

        // from readSensors:
        verify(leftMotor).getError();
        verify(leftMotor).getVelocity();
        verify(leftMotor).getPosition();
        verify(rightMotor).getError();
        verify(rightMotor).getVelocity();
        verify(rightMotor).getPosition();

        // from update:
        verify(leftMotor).set(eq(0.0));
        verify(rightMotor).set(eq(0.0));

        verifyNoMoreInteractions(leftMotor);
        verifyNoMoreInteractions(rightMotor);
        verifyNoMoreInteractions(leftFollowerMotor);
        verifyNoMoreInteractions(rightFollowerMotor);
    }

    @Test
    public void testStop()
    {
        LoggingManager logger = mock(LoggingManager.class);
        ITimer timer = mock(ITimer.class);
        TestProvider testProvider = new TestProvider();
        ITalonFX leftMotor = testProvider.getTalonFX(ElectronicsConstants.TANK_DRIVETRAIN_LEFT_MASTER_CAN_ID);
        ITalonFX rightMotor = testProvider.getTalonFX(ElectronicsConstants.TANK_DRIVETRAIN_RIGHT_MASTER_CAN_ID);
        ITalonFX leftFollowerMotor = testProvider.getTalonFX(ElectronicsConstants.TANK_DRIVETRAIN_LEFT_FOLLOWER_CAN_ID);
        ITalonFX rightFollowerMotor = testProvider.getTalonFX(ElectronicsConstants.TANK_DRIVETRAIN_RIGHT_FOLLOWER_CAN_ID);

        Driver driver = mock(Driver.class);

        doReturn(false).when(driver).getDigital(DigitalOperation.DriveTrainDisablePID);
        doReturn(false).when(driver).getDigital(DigitalOperation.DriveTrainEnablePID);
        doReturn(0.0).when(driver).getAnalog(AnalogOperation.DriveTrainLeftPosition);
        doReturn(0.0).when(driver).getAnalog(AnalogOperation.DriveTrainRightPosition);
        doReturn(false).when(driver).getDigital(DigitalOperation.DriveTrainUsePositionalMode);
        doReturn(false).when(driver).getDigital(DigitalOperation.DriveTrainSwapFrontOrientation);
        doReturn(false).when(driver).getDigital(DigitalOperation.DriveTrainSimpleMode);
        doReturn(0.0).when(driver).getAnalog(AnalogOperation.DriveTrainMoveForward);
        doReturn(0.0).when(driver).getAnalog(AnalogOperation.DriveTrainTurn);
        doReturn(0.0).when(leftMotor).getError();
        doReturn(0.0).when(leftMotor).getVelocity();
        doReturn(0.0).when(leftMotor).getPosition();
        doReturn(0.0).when(rightMotor).getError();
        doReturn(0.0).when(rightMotor).getVelocity();
        doReturn(0.0).when(rightMotor).getPosition();

        TankDriveTrainMechanism driveTrainMechanism = new TankDriveTrainMechanism(driver, logger, testProvider, timer);
        driveTrainMechanism.stop();

        // from constructor:
        verify(leftMotor).setNeutralMode(eq(MotorNeutralMode.Brake));
        verify(leftMotor).setInvertOutput(eq(HardwareConstants.TANK_DRIVETRAIN_LEFT_MASTER_INVERT_OUTPUT));
        verify(leftMotor).setInvertSensor(eq(HardwareConstants.TANK_DRIVETRAIN_LEFT_INVERT_SENSOR));
        verify(leftMotor).setSensorType(TalonXFeedbackDevice.IntegratedSensor);
        verify(leftMotor).setVoltageCompensation(TuningConstants.TANK_DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED, TuningConstants.TANK_DRIVETRAIN_VOLTAGE_COMPENSATION);
        verify(leftMotor).setSupplyCurrentLimit(TuningConstants.TANK_DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED, TuningConstants.TANK_DRIVETRAIN_SUPPLY_CURRENT_MAX, TuningConstants.TANK_DRIVETRAIN_SUPPLY_TRIGGER_CURRENT, TuningConstants.TANK_DRIVETRAIN_SUPPLY_TRIGGER_DURATION);
        verify(leftMotor).setFeedbackFramePeriod(5);
        verify(leftMotor).setPIDFFramePeriod(5);
        verify(leftMotor).configureVelocityMeasurements(eq(10), eq(32));
        verify(leftMotor).setSelectedSlot(eq(0));
        verify(leftFollowerMotor).setNeutralMode(eq(MotorNeutralMode.Brake));
        verify(leftFollowerMotor).setInvertOutput(eq(HardwareConstants.TANK_DRIVETRAIN_LEFT_FOLLOWER1_INVERT_OUTPUT));
        verify(leftFollowerMotor).follow(eq(leftMotor));
        verify(leftFollowerMotor).setVoltageCompensation(TuningConstants.TANK_DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED, TuningConstants.TANK_DRIVETRAIN_VOLTAGE_COMPENSATION);
        verify(leftFollowerMotor).setSupplyCurrentLimit(TuningConstants.TANK_DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED, TuningConstants.TANK_DRIVETRAIN_SUPPLY_CURRENT_MAX, TuningConstants.TANK_DRIVETRAIN_SUPPLY_TRIGGER_CURRENT, TuningConstants.TANK_DRIVETRAIN_SUPPLY_TRIGGER_DURATION);
        verify(rightMotor).setNeutralMode(eq(MotorNeutralMode.Brake));
        verify(rightMotor).setInvertOutput(eq(HardwareConstants.TANK_DRIVETRAIN_RIGHT_MASTER_INVERT_OUTPUT));
        verify(rightMotor).setInvertSensor(eq(HardwareConstants.TANK_DRIVETRAIN_RIGHT_INVERT_SENSOR));
        verify(rightMotor).setSensorType(TalonXFeedbackDevice.IntegratedSensor);
        verify(rightMotor).setVoltageCompensation(TuningConstants.TANK_DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED, TuningConstants.TANK_DRIVETRAIN_VOLTAGE_COMPENSATION);
        verify(rightMotor).setSupplyCurrentLimit(TuningConstants.TANK_DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED, TuningConstants.TANK_DRIVETRAIN_SUPPLY_CURRENT_MAX, TuningConstants.TANK_DRIVETRAIN_SUPPLY_TRIGGER_CURRENT, TuningConstants.TANK_DRIVETRAIN_SUPPLY_TRIGGER_DURATION);
        verify(rightMotor).setFeedbackFramePeriod(5);
        verify(rightMotor).setPIDFFramePeriod(5);
        verify(rightMotor).configureVelocityMeasurements(eq(10), eq(32));
        verify(rightMotor).setSelectedSlot(eq(0));
        verify(rightFollowerMotor).setNeutralMode(eq(MotorNeutralMode.Brake));
        verify(rightFollowerMotor).setInvertOutput(eq(HardwareConstants.TANK_DRIVETRAIN_RIGHT_FOLLOWER1_INVERT_OUTPUT));
        verify(rightFollowerMotor).follow(eq(rightMotor));
        verify(rightFollowerMotor).setVoltageCompensation(TuningConstants.TANK_DRIVETRAIN_VOLTAGE_COMPENSATION_ENABLED, TuningConstants.TANK_DRIVETRAIN_VOLTAGE_COMPENSATION);
        verify(rightFollowerMotor).setSupplyCurrentLimit(TuningConstants.TANK_DRIVETRAIN_SUPPLY_CURRENT_LIMITING_ENABLED, TuningConstants.TANK_DRIVETRAIN_SUPPLY_CURRENT_MAX, TuningConstants.TANK_DRIVETRAIN_SUPPLY_TRIGGER_CURRENT, TuningConstants.TANK_DRIVETRAIN_SUPPLY_TRIGGER_DURATION);

        // from setDriver:
        verify(leftMotor).setPIDF(
            eq(TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_LEFT_KP),
            eq(TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_LEFT_KI),
            eq(TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_LEFT_KD),
            eq(TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_LEFT_KF),
            eq(0));
        verify(rightMotor).setPIDF(
            eq(TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_RIGHT_KP),
            eq(TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_RIGHT_KI),
            eq(TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_RIGHT_KD),
            eq(TuningConstants.TANK_DRIVETRAIN_VELOCITY_PID_RIGHT_KF),
            eq(0));
        verify(leftMotor).setControlMode(eq(TalonXControlMode.Velocity));
        verify(rightMotor).setControlMode(eq(TalonXControlMode.Velocity));

        // from stop:
        verify(leftMotor).stop();
        verify(rightMotor).stop();
        verify(leftMotor).reset();
        verify(rightMotor).reset();

        verifyNoMoreInteractions(leftMotor);
        verifyNoMoreInteractions(rightMotor);
        verifyNoMoreInteractions(leftFollowerMotor);
        verifyNoMoreInteractions(rightFollowerMotor);
    }
}
