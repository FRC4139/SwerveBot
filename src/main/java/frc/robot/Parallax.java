package frc.robot;

public class Parallax {
    public static double LIMELIGHT_HEIGHT = 2.8; //feet
    public static double TAPE_HEIGHT = 8.0; //feet
    /**
     * Returns horizontal distance from limelight to basket using parallax. Requires tx and ty from two viewpoints.
     * @param dx Change in translational distance
     * @param tx1 Horizontal offset from crosshair to target at viewpoint 1
     * @param ty1 Vertical offset from crosshair to target at viewpoint 1
     * @param tx2 Horizontal offset from crosshair to target at viewpoint 2
     * @param ty2 Vertical offset from crosshair to target at viewpoint 2
     * @return
     */
    public static double getDistanceToTarget(double dx, double tx1, double ty1, double tx2, double ty2){
        //Returns horizontal distance from lightlight to basket using parallax
        double distance;

        double altitude = Math.abs(dx) / (Math.abs(Math.tan(tx1)) + Math.abs(Math.tan(tx2)));
        distance = altitude / Math.cos(tx2);
        return distance;
    }
    public static double getDistanceToTarget(double dx, double tx1, double ty1, double tx2, double ty2, boolean getDirectDistance){
        //if getDirectDistance is true, returns Euclidean distance using parallax
        double distance;
        if(getDirectDistance){
            return 1.0;
        }
        return 1.0;
    }
    /**
     * Returns the horizontal distance from limelight to basket using known heights of limelight and basket.
     * Formula: (TAPE_HEIGHT - LIMELIGHT_HEIGHT) * cotan(ty)
     * @param tx Horizontal offset from crosshair to target
     * @param ty Vertical offset from crosshair to target
     * @return The horizontal distance from limelight to basket (no vertical)
     */
    public static double getDistanceToTarget(double tx, double ty){
        //Returns horizontal distance from limelight to basket using known height of basket
        double distance;
        double distanceLeg = (TAPE_HEIGHT - LIMELIGHT_HEIGHT) / Math.tan(ty);
        distance = distanceLeg / Math.cos(tx);
        return distance;
    }
    /**
     * Returns the distance from limelight to basket, either horizontal or straight-line, depending on getDirectDistance.
     * @param tx Horizontal offset from crosshair to target
     * @param ty Vertical offset from crosshair to target
     * @param getDirectDistance If true, calculate the Euclidean distance (Straight-line distance, vertical included). If false, calculate horizontal distance (no vertical).
     * @return Desired form of distance
     */
    public static double getDistanceToTarget(double tx, double ty, boolean getDirectDistance){
        //if getDirectDistance is true, return the Euclidean distance from limelight to basket (straight line distance, vertical distance is included)
        double distance;
        if(getDirectDistance){
            double horizontalDistance = getDistanceToTarget(tx,ty);
            distance = horizontalDistance / Math.sin(ty);
        }
        else{
            distance = getDistanceToTarget(tx,ty);
        }
        return distance;
    }


    
}
