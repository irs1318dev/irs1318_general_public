package frc.lib.robotprovider;

import com.ctre.phoenix6.StatusCode;

public class CTREStatusCodeHelper
{
    public static void printError(StatusCode sc, String operation)
    {
        if (sc != StatusCode.OK)
        {
            System.err.println(operation + " failed with " + sc.toString());
        }
    }
}
