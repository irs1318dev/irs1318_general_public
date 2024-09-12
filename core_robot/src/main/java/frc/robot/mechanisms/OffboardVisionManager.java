package frc.robot.mechanisms;

import frc.lib.driver.IDriver;
import frc.lib.helpers.Helpers;
import frc.lib.mechanisms.IMechanism;
import frc.lib.mechanisms.LoggingManager;
import frc.lib.robotprovider.IDigitalOutput;
import frc.lib.robotprovider.IDoubleSubscriber;
import frc.lib.robotprovider.ILogger;
import frc.lib.robotprovider.INetworkTableProvider;
import frc.lib.robotprovider.IRobotProvider;
import frc.lib.robotprovider.RobotMode;
import frc.robot.*;
import frc.robot.driver.*;

import com.google.inject.Inject;
import com.google.inject.Singleton;

/**
 * Offboard Vision manager.
 * 
 * @author Will
 *
 */
@Singleton
public class OffboardVisionManager implements IMechanism
{
    private final IDriver driver;
    private final INetworkTableProvider networkTable;
    private final ILogger logger;

    private final IDigitalOutput ringLight;

    private IDoubleSubscriber centerXSubscriber;
    private IDoubleSubscriber centerYSubscriber;
    private IDoubleSubscriber widthSubscriber;
    private IDoubleSubscriber heightSubscriber;
    private IDoubleSubscriber angleSubscriber;

    private IDoubleSubscriber heartbeatSubscriber;

    private double centerX;
    private double centerY;
    private double width;
    private double height;
    private double angle;

    private int missedHeartbeats;
    private double prevHeartbeat;

    private Double distance;
    private Double horizontalAngle;

    /**
     * Initializes a new OffboardVisionManager
     * @param driver for obtaining operations
     * @param logger for logging to smart dashboard
     * @param provider for obtaining electronics objects
     */
    @Inject
    public OffboardVisionManager(IDriver driver, LoggingManager logger, IRobotProvider provider)
    {
        this.driver = driver;
        this.logger = logger;

        this.networkTable = provider.getNetworkTableProvider();
        this.ringLight = provider.getDigitalOutput(ElectronicsConstants.VISION_RING_LIGHT_DIO);

        this.centerXSubscriber = this.networkTable.getDoubleSubscriber("v.pointX", TuningConstants.MAGIC_NULL_VALUE);
        this.centerYSubscriber = this.networkTable.getDoubleSubscriber("v.pointY", TuningConstants.MAGIC_NULL_VALUE);
        this.widthSubscriber = this.networkTable.getDoubleSubscriber("v.width", TuningConstants.MAGIC_NULL_VALUE);
        this.heightSubscriber = this.networkTable.getDoubleSubscriber("v.height", TuningConstants.MAGIC_NULL_VALUE);
        this.angleSubscriber = this.networkTable.getDoubleSubscriber("v.angle", TuningConstants.MAGIC_NULL_VALUE);

        this.heartbeatSubscriber = this.networkTable.getDoubleSubscriber("v.heartbeat", 0);

        this.centerX = 0.0;
        this.centerY = 0.0;

        this.width = 0.0;
        this.height = 0.0;
        this.angle = 0.0;

        this.missedHeartbeats = 0;
        this.prevHeartbeat = 0.0;
    }

    /**
     * read all of the sensors for the mechanism that we will use in macros/autonomous mode and record their values
     */
    @Override
    public void readSensors()
    {
        this.centerX = this.centerXSubscriber.get();
        this.centerY = this.centerYSubscriber.get();
        this.width = this.widthSubscriber.get();
        this.height = this.heightSubscriber.get();
        this.angle = this.angleSubscriber.get();

        this.logger.logNumber(LoggingKey.OffboardVisionX, this.centerX);
        this.logger.logNumber(LoggingKey.OffboardVisionY, this.centerY);
        this.logger.logNumber(LoggingKey.OffboardVisionWidth, this.width);
        this.logger.logNumber(LoggingKey.OffboardVisionHeight, this.height);
        this.logger.logNumber(LoggingKey.OffboardVisionAngle, this.angle);

        double newHeartbeat = this.heartbeatSubscriber.get();
        if (!Helpers.RoughEquals(this.prevHeartbeat, newHeartbeat, 0.5))
        {
            this.missedHeartbeats = 0;
        }
        else
        {
            this.missedHeartbeats++;
        }

        this.prevHeartbeat = newHeartbeat;
        this.logger.logNumber(LoggingKey.OffboardVisionMissedHeartbeats, this.missedHeartbeats);

        // return if we couldn't find a vision target
        if (this.centerX < 0.0 || this.centerY < 0.0 || this.missedHeartbeats > TuningConstants.VISION_MISSED_HEARTBEAT_THRESHOLD)
        {
            this.distance = null;
            this.horizontalAngle = null;

            return;
        }

        double yOffset = VisionConstants.LIFECAM_CAMERA_CENTER_WIDTH - this.centerY;
        double verticalAngle = Helpers.atand(yOffset / VisionConstants.LIFECAM_CAMERA_FOCAL_LENGTH_Y);

        this.distance = (HardwareConstants.CAMERA_TO_TARGET_Z_OFFSET / Helpers.tand(verticalAngle + HardwareConstants.CAMERA_PITCH)) - HardwareConstants.CAMERA_X_OFFSET;

        double xOffset = this.centerX - VisionConstants.LIFECAM_CAMERA_CENTER_WIDTH;
        this.horizontalAngle = Helpers.atand(xOffset / VisionConstants.LIFECAM_CAMERA_FOCAL_LENGTH_X) + HardwareConstants.CAMERA_YAW;

        this.logger.logNumber(LoggingKey.OffboardVisionDistance, this.distance);
        this.logger.logNumber(LoggingKey.OffboardVisionHorizontalAngle, this.horizontalAngle);
    }

    @Override
    public void update(RobotMode mode)
    {
        boolean enableVision = !this.driver.getDigital(DigitalOperation.VisionForceDisable);
        boolean enableVideoStream = !this.driver.getDigital(DigitalOperation.VisionDisableStream);
        boolean enablePowercellProcessing = this.driver.getDigital(DigitalOperation.VisionEnablePowercellProcessing);
        boolean enableRetroreflectiveProcessing = this.driver.getDigital(DigitalOperation.VisionEnableRetroreflectiveProcessing);

        double visionProcessingMode = 0.0;
        if (enableVision)
        {
            if (enableRetroreflectiveProcessing)
            {
                visionProcessingMode = 1.0;
            }
            else if (enablePowercellProcessing)
            {
                visionProcessingMode = 2.0;
            }
        }

        this.logger.logBoolean(LoggingKey.OffboardVisionEnableVision, enableVision);
        this.logger.logBoolean(LoggingKey.OffboardVisionEnableStream, enableVideoStream);
        this.logger.logNumber(LoggingKey.OffboardVisionEnableProcessing, visionProcessingMode);

        this.ringLight.set(enableVision && enableRetroreflectiveProcessing);
    }

    @Override
    public void stop()
    {
        this.ringLight.set(false);

        this.logger.logBoolean(LoggingKey.OffboardVisionEnableVision, false);
        this.logger.logBoolean(LoggingKey.OffboardVisionEnableStream, false);
        this.logger.logNumber(LoggingKey.OffboardVisionEnableProcessing, 0.0);
    }

    public Double getHorizontalAngle()
    {
        return this.horizontalAngle;
    }

    public Double getDistance()
    {
        return this.distance;
    }

    public double getPowercellX() 
    {
        return this.centerX;
    }

    public double getPowercellY() 
    {
        return this.centerY;
    }
}
