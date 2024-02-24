package frc.robot.lib.Controller;

import java.util.function.DoubleSupplier;

public class ControllerLimiter {
    
    public static double cubic(double x){
        if(x > 1){
        return Math.min(Math.pow(x, 3) + 0.2*x, 1);
        }
        else{
        return Math.max(Math.pow(x, 3) + 0.2*x, -1);
        }

    }
}
