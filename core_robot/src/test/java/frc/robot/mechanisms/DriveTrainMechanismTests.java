package frc.robot.mechanisms;

import frc.robot.HardwareConstants;
import frc.robot.TestProvider;
import frc.robot.TuningConstants;
import frc.robot.common.Helpers;
import frc.robot.common.LoggingManager;
import frc.robot.common.robotprovider.INavx;
import frc.robot.common.robotprovider.ITalonFX;
import frc.robot.common.robotprovider.ITalonSRX;
import frc.robot.common.robotprovider.ITimer;
import frc.robot.common.robotprovider.IVictorSPX;
import frc.robot.common.robotprovider.MotorNeutralMode;
import frc.robot.common.robotprovider.NullLogger;
import frc.robot.common.robotprovider.Pose2d;
import frc.robot.common.robotprovider.RobotMode;
import frc.robot.common.robotprovider.TalonSRXControlMode;
import frc.robot.common.robotprovider.TalonXFeedbackDevice;
import frc.robot.common.robotprovider.TalonXLimitSwitchStatus;
import frc.robot.driver.AnalogOperation;
import frc.robot.driver.DigitalOperation;
import frc.robot.driver.common.Driver;
import frc.robot.driver.common.IDriver;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class DriveTrainMechanismTests
{
    @Test
    public void testStill()
    {
        TestProvider provider = new TestProvider();

        MockTimer timer = new MockTimer();
        MockNavx navx = new MockNavx();
        provider.setNavx(navx);

        MockTalonFX[] steer = new MockTalonFX[4];
        MockTalonFX[] drive = new MockTalonFX[4];
        for (int i = 0; i < 4; i++)
        {
            steer[i] = new MockTalonFX(2 * i + 1);
            drive[i] = new MockTalonFX(2 * i + 2);

            provider.setTalonFX(2 * i + 1, steer[i]);
            provider.setTalonFX(2 * i + 2, drive[i]);
        }

        LoggingManager logger = new LoggingManager(new NullLogger());
        NavxManager navxManager = new NavxManager(new MockDriver(), logger, provider);
        DriveTrainMechanism driveTrain = new DriveTrainMechanism(
            new MockDriver(), 
            logger,
            provider,
            navxManager,
            timer);

        for (int timestep = 0; timestep < 10; timestep++)
        {
            navxManager.readSensors();
            driveTrain.readSensors();
            timer.increment(0.02);
        }

        Pose2d pose = driveTrain.getPose();
        assertEquals(0.0, pose.angle, 0.0001);
        assertEquals(0.0, pose.x, 0.0001);
        assertEquals(0.0, pose.y, 0.0001);
    }

    @Test
    public void testForward1()
    {
        TestProvider provider = new TestProvider();

        MockTimer timer = new MockTimer();
        MockNavx navx = new MockNavx();
        provider.setNavx(navx);

        MockTalonFX[] steer = new MockTalonFX[4];
        MockTalonFX[] drive = new MockTalonFX[4];
        for (int i = 0; i < 4; i++)
        {
            steer[i] = new MockTalonFX(2 * i + 1);
            drive[i] = new MockTalonFX(2 * i + 2);

            provider.setTalonFX(2 * i + 1, steer[i]);
            provider.setTalonFX(2 * i + 2, drive[i]);
        }

        LoggingManager logger = new LoggingManager(new NullLogger());
        NavxManager navxManager = new NavxManager(new MockDriver(), logger, provider);
        DriveTrainMechanism driveTrain = new DriveTrainMechanism(
            new MockDriver(), 
            logger,
            provider,
            navxManager,
            timer);

        for (int timestep = 0; timestep < 50; timestep++)
        {
            for (int i = 0; i < 4; i++)
            {
                // 10% max-speed
                drive[i].set(0.1 * TuningConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS);;
            }

            navxManager.readSensors();
            driveTrain.readSensors();
            timer.increment(0.02);
        }

        Pose2d pose = driveTrain.getPose();
        assertEquals(0.0, pose.angle, 0.5);
        assertEquals(0.0, pose.x, 0.5);
        assertEquals(0.1 * TuningConstants.DRIVETRAIN_MAX_VELOCITY, pose.y, 0.5);
    }

    @Test
    public void testForward2()
    {
        TestProvider provider = new TestProvider();

        MockTimer timer = new MockTimer();
        MockNavx navx = new MockNavx();
        provider.setNavx(navx);

        MockTalonFX[] steer = new MockTalonFX[4];
        MockTalonFX[] drive = new MockTalonFX[4];
        for (int i = 0; i < 4; i++)
        {
            steer[i] = new MockTalonFX(2 * i + 1);
            drive[i] = new MockTalonFX(2 * i + 2);

            provider.setTalonFX(2 * i + 1, steer[i]);
            provider.setTalonFX(2 * i + 2, drive[i]);
        }

        LoggingManager logger = new LoggingManager(new NullLogger());
        NavxManager navxManager = new NavxManager(new MockDriver(), logger, provider);
        DriveTrainMechanism driveTrain = new DriveTrainMechanism(
            new MockDriver(), 
            logger,
            provider,
            navxManager,
            timer);

        for (int timestep = 0; timestep < 50; timestep++)
        {
            for (int i = 0; i < 4; i++)
            {
                // 10% max-speed
                drive[i].set(-0.1 * TuningConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS);;
            }

            navx.set(-180.0);
            navxManager.readSensors();
            driveTrain.readSensors();
            timer.increment(0.02);
        }

        Pose2d pose = driveTrain.getPose();
        assertEquals(180.0, pose.angle, 0.5);
        assertEquals(0.0, pose.x, 0.5);
        assertEquals(0.1 * TuningConstants.DRIVETRAIN_MAX_VELOCITY, pose.y, 0.5);
    }

    @Test
    public void testLeft1()
    {
        TestProvider provider = new TestProvider();

        MockTimer timer = new MockTimer();
        MockNavx navx = new MockNavx();
        provider.setNavx(navx);

        MockTalonFX[] steer = new MockTalonFX[4];
        MockTalonFX[] drive = new MockTalonFX[4];
        for (int i = 0; i < 4; i++)
        {
            steer[i] = new MockTalonFX(2 * i + 1);
            drive[i] = new MockTalonFX(2 * i + 2);

            provider.setTalonFX(2 * i + 1, steer[i]);
            provider.setTalonFX(2 * i + 2, drive[i]);
        }

        LoggingManager logger = new LoggingManager(new NullLogger());
        NavxManager navxManager = new NavxManager(new MockDriver(), logger, provider);
        DriveTrainMechanism driveTrain = new DriveTrainMechanism(
            new MockDriver(), 
            logger,
            provider,
            navxManager,
            timer);

        for (int timestep = 0; timestep < 50; timestep++)
        {
            for (int i = 0; i < 4; i++)
            {
                // 10% max-speed
                drive[i].set(0.1 * TuningConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS);;
                steer[i].set(90.0 * HardwareConstants.DRIVETRAIN_STEER_TICKS_PER_DEGREE);
            }

            navx.set(0.0);
            navxManager.readSensors();
            driveTrain.readSensors();
            timer.increment(0.02);
        }

        Pose2d pose = driveTrain.getPose();
        assertEquals(0.0, pose.angle, 0.5);
        assertEquals(-0.1 * TuningConstants.DRIVETRAIN_MAX_VELOCITY, pose.x, 0.5);
        assertEquals(0.0, pose.y, 0.5);
    }

    @Test
    public void testLeft2()
    {
        TestProvider provider = new TestProvider();

        MockTimer timer = new MockTimer();
        MockNavx navx = new MockNavx();
        provider.setNavx(navx);

        MockTalonFX[] steer = new MockTalonFX[4];
        MockTalonFX[] drive = new MockTalonFX[4];
        for (int i = 0; i < 4; i++)
        {
            steer[i] = new MockTalonFX(2 * i + 1);
            drive[i] = new MockTalonFX(2 * i + 2);

            provider.setTalonFX(2 * i + 1, steer[i]);
            provider.setTalonFX(2 * i + 2, drive[i]);
        }

        LoggingManager logger = new LoggingManager(new NullLogger());
        NavxManager navxManager = new NavxManager(new MockDriver(), logger, provider);
        DriveTrainMechanism driveTrain = new DriveTrainMechanism(
            new MockDriver(), 
            logger,
            provider,
            navxManager,
            timer);

        for (int timestep = 0; timestep < 50; timestep++)
        {
            for (int i = 0; i < 4; i++)
            {
                // 10% max-speed
                drive[i].set(0.1 * TuningConstants.DRIVETRAIN_DRIVE_MOTOR_VELOCITY_PID_KS);
            }

            navx.set(-90.0);
            navxManager.readSensors();
            driveTrain.readSensors();
            timer.increment(0.02);
        }

        Pose2d pose = driveTrain.getPose();
        assertEquals(90.0, pose.angle, 0.5);
        assertEquals(-0.1 * TuningConstants.DRIVETRAIN_MAX_VELOCITY, pose.x, 0.5);
        assertEquals(0.0, pose.y, 0.5);
    }

    @Test
    public void testTwist1Rad()
    {
        TestProvider provider = new TestProvider();

        MockTimer timer = new MockTimer();
        MockNavx navx = new MockNavx();
        provider.setNavx(navx);

        MockTalonFX[] steer = new MockTalonFX[4];
        MockTalonFX[] drive = new MockTalonFX[4];
        for (int i = 0; i < 4; i++)
        {
            steer[i] = new MockTalonFX(2 * i + 1);
            drive[i] = new MockTalonFX(2 * i + 2);

            provider.setTalonFX(2 * i + 1, steer[i]);
            provider.setTalonFX(2 * i + 2, drive[i]);
        }

        LoggingManager logger = new LoggingManager(new NullLogger());
        NavxManager navxManager = new NavxManager(new MockDriver(), logger, provider);
        DriveTrainMechanism driveTrain = new DriveTrainMechanism(
            new MockDriver(), 
            logger,
            provider,
            navxManager,
            timer);

        double robotVelocityRight = 0.0;
        double robotVelocityForward = 0.0;
        double omega = 1.0;
        for (double timestep = 0; timestep <= 50.0; timestep += 1.0)
        {
            for (int i = 0; i < 4; i++)
            {
                double moduleVelocityRight = robotVelocityRight + omega * DriveTrainMechanism.MODULE_OFFSET_Y[i];
                double moduleVelocityForward = robotVelocityForward - omega * DriveTrainMechanism.MODULE_OFFSET_X[i];

                double moduleSteerPositionGoal = Helpers.atan2d(-moduleVelocityRight, moduleVelocityForward);
                moduleSteerPositionGoal *= TuningConstants.DRIVETRAIN_STEER_MOTOR_POSITION_PID_KS;

                double moduleDriveVelocityGoal = Math.sqrt(moduleVelocityRight * moduleVelocityRight + moduleVelocityForward * moduleVelocityForward);
                moduleDriveVelocityGoal *= HardwareConstants.DRIVETRAIN_DRIVE_INCHES_PER_SECOND_TO_MOTOR_VELOCITY;

                drive[i].set(moduleDriveVelocityGoal);
                steer[i].set(moduleSteerPositionGoal);
            }

            navx.set(-timestep * 0.02 * 180.0 / Math.PI);
            navxManager.readSensors();
            driveTrain.readSensors();
            timer.increment(0.02);
        }

        Pose2d pose = driveTrain.getPose();
        assertEquals(180.0 / Math.PI, pose.angle, 1.0);
        assertEquals(0.0, pose.x, 0.5);
        assertEquals(0.0, pose.y, 0.5);
    }

    private class MockTimer implements ITimer
    {
        private double currentTime;

        MockTimer()
        {
            this.currentTime = 0.0;
        }

        @Override
        public void start()
        {
        }

        @Override
        public void stop()
        {
        }

        @Override
        public double get()
        {
            return this.currentTime;
        }

        @Override
        public void reset()
        {
            this.currentTime = 0.0;
        }

        public void set(double value)
        {
            this.currentTime = value;
        }

        public void increment(double value)
        {
            this.currentTime += value;
        }
    }

    private class MockNavx implements INavx
    {
        private double currentAngle;

        MockNavx()
        {
        }

        @Override
        public boolean isConnected()
        {
            return true;
        }

        @Override
        public double getAngle()
        {
            return this.currentAngle;
        }

        @Override
        public double getPitch()
        {
            return this.currentAngle;
        }

        @Override
        public double getRoll()
        {
            return this.currentAngle;
        }

        @Override
        public double getYaw()
        {
            return this.currentAngle;
        }

        @Override
        public double getDisplacementX()
        {
            return 0;
        }

        @Override
        public double getDisplacementY()
        {
            return 0;
        }

        @Override
        public double getDisplacementZ()
        {
            return 0;
        }

        @Override
        public void reset()
        {
            this.currentAngle = 0.0;
        }

        @Override
        public void resetDisplacement()
        {
        }

        public void set(double angle)
        {
            this.currentAngle = angle;
        }
    }

    private class MockTalonFX implements ITalonFX
    {
        private final int deviceId;
        private double currentValue;

        MockTalonFX(int deviceId)
        {
            this.deviceId = deviceId;
        }

        @Override
        public void follow(ITalonSRX talonSRX)
        {
        }

        @Override
        public void follow(ITalonFX talonFX)
        {
        }

        @Override
        public void follow(IVictorSPX victorSPX)
        {
        }

        @Override
        public void setControlMode(TalonSRXControlMode mode)
        {
        }

        @Override
        public void setSensorType(TalonXFeedbackDevice feedbackDevice)
        {
        }

        @Override
        public void setFeedbackFramePeriod(int periodMS)
        {
        }

        @Override
        public void setPIDFFramePeriod(int periodMS)
        {
        }

        @Override
        public void configureVelocityMeasurements(int periodMS, int windowSize)
        {
        }

        @Override
        public void configureAllowableClosedloopError(int slotId, int error)
        {
        }

        @Override
        public void setSelectedSlot(int slotId)
        {
        }

        @Override
        public void setPIDF(double p, double i, double d, double f, int slotId)
        {
        }

        @Override
        public void setMotionMagicPIDF(double p, double i, double d, double f, int velocity, int acceleration, int slotId)
        {
        }

        @Override
        public void setPIDF(double p, double i, double d, double f, int izone, double closeLoopRampRate, int slotId)
        {
        }

        @Override
        public void setForwardLimitSwitch(boolean enabled, boolean normallyOpen)
        {
        }

        @Override
        public void setReverseLimitSwitch(boolean enabled, boolean normallyOpen)
        {
        }

        @Override
        public void setInvertOutput(boolean flip)
        {
        }

        @Override
        public void setInvertSensor(boolean flip)
        {
        }

        @Override
        public void setNeutralMode(MotorNeutralMode neutralMode)
        {
        }

        @Override
        public void setVoltageCompensation(boolean enabled, double maxVoltage)
        {
        }

        @Override
        public void stop()
        {
        }

        @Override
        public void setPosition(double position)
        {
        }

        @Override
        public void reset()
        {
        }

        @Override
        public double getPosition() 
        {
            return this.currentValue;
        }

        @Override
        public double getVelocity()
        {
            return this.currentValue;
        }

        @Override
        public double getError()
        {
            return 0;
        }

        @Override
        public TalonXLimitSwitchStatus getLimitSwitchStatus()
        {
            return null;
        }

        @Override
        public void set(double power)
        {
            this.currentValue = power;
        }

        @Override
        public void setSupplyCurrentLimit(boolean enabled, double currentLimit, double triggerThresholdCurrent, double triggerThresholdTime)
        {
        }
    }

    private class MockDriver implements IDriver
    {
        private RobotMode currentMode = RobotMode.Teleop;

        @Override
        public RobotMode getMode()
        {
            return this.currentMode;
        }

        @Override
        public void update()
        {
        }

        @Override
        public void stop()
        {
        }

        @Override
        public void startMode(RobotMode mode)
        {
            this.currentMode = mode;
        }

        @Override
        public boolean getDigital(DigitalOperation digitalOperation)
        {
            return false;
        }

        @Override
        public double getAnalog(AnalogOperation analogOperation)
        {
            return 0.0;
        }
    }
}