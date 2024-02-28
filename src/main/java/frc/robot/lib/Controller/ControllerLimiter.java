package frc.robot.lib.Controller;

import java.util.function.DoubleSupplier;

public class ControllerLimiter {
    
    public static double quadratic(double x){

        double magnitude = Math.pow(x, 2);

        return (x>0) ? magnitude : -magnitude;

       

    }
}
