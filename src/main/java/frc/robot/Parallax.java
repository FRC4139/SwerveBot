package frc.robot;

public class Parallax {
    public static double LIMELIGHT_HEIGHT = 2.6667; //feet
    public static double TAPE_HEIGHT = 8.0; //feet
    public static double HEIGHT_DIFF = TAPE_HEIGHT - LIMELIGHT_HEIGHT; //feet
    public static double DEFAULT_THRESHOLD = 0.1; //feet
    public static double CAMERA_PITCH = 0.436332; //radians
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
        double correctedTy1, correctedTy2;
        
        correctedTy1 = ty1 + CAMERA_PITCH;
        correctedTy2 = ty2 + CAMERA_PITCH;

        double k = dz / (Math.tan(correctedTy2) / Math.tan(correctedTy1) - 1);
        distance = k / Math.cos(tx2);
        return distance;
    }

    public static double getDistanceToTarget(double tx1, double ty1, double tx2, double ty2, double dx, double dz, double yaw){
        double correctedTx = tx2 + yaw;
        return getDistanceToTarget(tx1, ty1, correctedTx, ty2, dx, dz);
    }
    


    public static double getDistanceToTarget(double tx, double ty, double yaw, double pitch){
        double[] calculatedVector = getVectorToTarget(tx, ty, yaw, pitch);
        double x = calculatedVector[0];
        double z = calculatedVector[2];
        return Math.sqrt(Math.pow(x, 2) + Math.pow(z, 2));
    }
    
    public static double getDistanceToTarget(double tx, double ty, double yaw){
        return getDistanceToTarget(tx, ty, yaw, CAMERA_PITCH);
    }

    public static double getDistanceToTarget(double tx, double ty){
        return getDistanceToTarget(tx, ty, 0, CAMERA_PITCH);
    }
    /**
     * Returns vector from limelight to target, given tx, ty, yaw, and pitch of the limelight. 0 yaw is defined as straight forward, positive yaw is clockwise (to the right), negative yaw is ccw (to the left).
     * @param tx
     * @param ty
     * @param yaw of the camera. 0 yaw is defined as straightforward, positive is to the RIGHT, negative is to the LEFT.
     * @param pitch of the camera. 0 pitch is defined as straightforward, positive is upward.
     * @return the vector [x, y, z] of target assuming limelight is at [0, 0, 0]
     */
    public static double[] getVectorToTarget(double tx, double ty, double yaw, double pitch){
        /* System of equations:
         * a1x + b1z = c1y
         * a2x + b2z = c2y
        */
        double a1 = Math.cos(yaw) - Math.tan(tx) * Math.sin(yaw) * Math.cos(pitch);
        double b1 = -Math.sin(yaw) - Math.tan(tx) * Math.cos(yaw) * Math.cos(pitch);
        double c1 = Math.tan(tx) * Math.sin(pitch);
        double a2 = -Math.sin(pitch) * Math.sin(yaw) - Math.tan(ty) * Math.sin(yaw) * Math.cos(pitch);
        double b2 = -Math.sin(pitch) * Math.cos(yaw) - Math.tan(ty) * Math.cos(yaw) * Math.cos(pitch);
        double c2 = -Math.cos(pitch) + Math.tan(ty) * Math.sin(pitch);

        double calculatedZ = HEIGHT_DIFF * (c1 * a2 - c2 * a1) / (b1 * a2 - a1 * b2);
        double calculatedX = (c1 * HEIGHT_DIFF - b1 * calculatedZ) / a1;

        double[] returnedVec = {calculatedX, HEIGHT_DIFF, calculatedZ};

        return returnedVec;
    }

    public static double[] getVectorToTarget(double tx, double ty, double yaw){
        return getVectorToTarget(tx, ty, yaw, CAMERA_PITCH);
    }

    public static boolean checkValidVectors(double tx1, double ty1, double tx2, double ty2, double dx, double dz, double yaw, double threshold){
        double correctedTx,correctedTy1,correctedTy2;

        correctedTx = tx2 + yaw;
        correctedTy1 = ty1 + CAMERA_PITCH;
        correctedTy2 = ty2 + CAMERA_PITCH;
        
        double dxCalculated = dz * (Math.tan(correctedTy2)*Math.tan(tx1) - Math.tan(correctedTy1)*Math.tan(correctedTx))/(Math.tan(correctedTy2) - Math.tan(correctedTy1));
        
        if (Math.abs(dx - dxCalculated) <= threshold){
            return true;
        }
        else{
            return false;
        }
    }
    public static boolean checkValidVectors(double tx1, double ty1, double tx2, double ty2, double dx, double dz, double yaw){
        return checkValidVectors(tx1,ty1,tx2,ty2,dx,dz,yaw,DEFAULT_THRESHOLD);
    }
    /**
     * Returns the distance if monocular and binocular distance measures agree (within threshold), and returns -1.0 if measures don't agree.
     * @param tx1
     * @param ty1
     * @param tx2
     * @param ty2
     * @param dx
     * @param dz
     * @param threshold
     * @return
     */
    public static double getAgreedDistance(double tx1, double ty1, double tx2, double ty2, double dx, double dz, double yaw, double threshold){
        double binocularDistance = getDistanceToTarget(tx1,ty1,tx2,ty2,dx,dz,yaw);
        double monocularDistance = getDistanceToTarget(tx2,ty2);
        if (!checkValidVectors(tx1,ty1,tx2,ty2,dx,dz,yaw,threshold)){
            return -1.0;
        }
        if(Math.abs(binocularDistance - monocularDistance) <= threshold){
            return (binocularDistance + monocularDistance) / 2;
        }
        else{
            return -1.0;
        }
    }
    public static double getAgreedDistance(double tx1, double ty1, double tx2, double ty2, double dx, double dz, double yaw){
        return getAgreedDistance(tx1,ty1,tx2,ty2,dx,dz,yaw,DEFAULT_THRESHOLD);
    }
    
}
