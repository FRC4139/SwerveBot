package frc.robot;

public class Parallax {
    public static double LIMELIGHT_HEIGHT = 2.8; //feet
    public static double TAPE_HEIGHT = 8.0; //feet
    public static double HEIGHT_DIFF = TAPE_HEIGHT - LIMELIGHT_HEIGHT; //feet
    /**
     * Returns horizontal distance from limelight to basket using parallax. Requires tx and ty from two viewpoints.
     * @param dx Change in translational distance
     * @param tx1 Horizontal offset from crosshair to target at viewpoint 1
     * @param ty1 Vertical offset from crosshair to target at viewpoint 1
     * @param tx2 Horizontal offset from crosshair to target at viewpoint 2
     * @param ty2 Vertical offset from crosshair to target at viewpoint 2
     * @return
     */
    public static double getDistanceToTarget(double tx1, double ty1, double tx2, double ty2, double dx, double dz){
        //Returns horizontal distance from lightlight to basket using parallax
        double distance;


        double k = dz / (Math.tan(ty2) / Math.tan(ty1) - 1);
        distance = k / Math.cos(tx2);
        return distance;
    }
    public static double getDistanceToTarget(double tx1, double ty1, double tx2, double ty2, double dx, double dz, boolean getDirectDistance){
        //if getDirectDistance is true, returns Euclidean distance using parallax
        double distance;
        if(getDirectDistance){
            double k = dz / (Math.tan(ty2) / Math.tan(ty1) - 1);
            distance = k * Math.sqrt(Math.pow(Math.tan(tx2),2) + Math.pow(Math.tan(ty2),2) + 1);
            return distance;
        }
        else{
            return getDistanceToTarget(tx1, ty1, tx2, ty2, dx, dz);
        }
    }
    /**
     * Returns the horizontal distance from limelight to basket using known heights of limelight and basket.
     * Formula: HEIGHT_DIFF / (tan(ty) * cos(tx))
     * @param tx Horizontal offset from crosshair to target
     * @param ty Vertical offset from crosshair to target
     * @return The horizontal distance from limelight to basket (no vertical)
     */
    public static double getDistanceToTarget(double tx, double ty){
        //Returns horizontal distance from limelight to basket using known height of basket
        double distance;
        distance = HEIGHT_DIFF / Math.tan(ty) / Math.cos(tx);
        return distance;
    }
    /**
     * Returns the distance from limelight to basket, either horizontal or straight-line, depending on getDirectDistance.
     * @param tx Horizontal offset from crosshair to target
     * @param ty Vertical offset from crosshair to target
     * @param getDirectDistance If true, calculate the Euclidean distance (Straight-line distance, vertical included). If false, calculate horizontal distance (no vertical).
     * Formula for Euclidean distance: (HEIGHT_DIFF / tan(ty)) * norm([tan(tx),tan(ty),1])
     * @return Desired form of distance
     */
    public static double getDistanceToTarget(double tx, double ty, boolean getDirectDistance){
        //if getDirectDistance is true, return the Euclidean distance from limelight to basket (straight line distance, vertical distance is included)
        double distance;
        if(getDirectDistance){
            distance = HEIGHT_DIFF / Math.tan(ty) * Math.sqrt(Math.pow(Math.tan(tx),2) + Math.pow(Math.tan(ty),2) + 1);
        }
        else{
            distance = getDistanceToTarget(tx,ty);
        }
        return distance;
    }
    public static double getDistanceToTarget(double tx, double ty, double cameraInclination){
        double tyCorrected = ty + cameraInclination;
        return getDistanceToTarget(tx, tyCorrected);
    }
    public static double getDistanceToTarget(double tx, double ty, double cameraInclination, boolean getDirectDistance){
        double tyCorrected = ty + cameraInclination;
        return getDistanceToTarget(tx, tyCorrected, getDirectDistance);
    }
    public static boolean checkValidVectors(double tx1, double ty1, double tx2, double ty2, double dx, double dz, double threshold){
        if (threshold <= 0){
            threshold = 1.0;
        }
        double dxCalculated = dz * (Math.tan(ty2)*Math.tan(tx1) - Math.tan(ty1)*Math.tan(tx2))/(Math.tan(ty2) - Math.tan(ty1));
        
        if (Math.abs(dx - dxCalculated) <= threshold){
            return true;
        }
        else{
            return false;
        }
    }
    
}
