package frc.robot;

public class Parallax {
    public static double LIMELIGHT_HEIGHT = 34/12; //feet
    public static double TAPE_HEIGHT = 104/12; //feet
    public static double HEIGHT_DIFF = TAPE_HEIGHT - LIMELIGHT_HEIGHT; //feet
    public static double DEFAULT_THRESHOLD = 0.1; //feet
    public static double CAMERA_PITCH = Math.toRadians(25); //radians
    /**
     * Returns horizontal distance using parallax, requires data from two viewpoints, as well as change in position between the two viewpoints
     * @param tx1 tx of viewpoint 1
     * @param ty1 ty of viewpoint 1
     * @param tx2 tx of viewpoint 2
     * @param ty2 ty of viewpoint 2
     * @param dx difference in x position between viewpoints (viewpoint 2 - viewpoint 1)
     * @param dz difference in z position between viewpoints (viewpoint 2 - viewpoint 1)
     * @param yaw difference between viewpoints. POSITIVE if viewpoint 2 points to the RIGHT of viewpoint 1, NEGATIVE if viewpoint 2 points to the LEFT of viewpoint 1.
     * @param pitch of the camera (assumes pitch does not change)
     * @return horizontal distance using parallax
     */
    public static double getDistanceToTarget(double tx1, double ty1, double tx2, double ty2, double dx, double dz, double yaw, double pitch){
        //Returns horizontal distance from lightlight to basket using parallax
        
        double[] v1 = getVectorToTarget(tx1, ty1, 0, pitch);
        double[] v2 = getVectorToTarget(tx2, ty2, yaw, pitch);

        double k = dz/((v1[2] * v2[1])/(v1[1]) - v2[2]);

        //check: if k == 1, then binocular agrees with monocular
        //check: valid vectors

        double distance = k * getDistanceToTarget(tx2, ty2, yaw, pitch);
        
        return distance;
    }

    public static double getDistanceToTarget(double tx1, double ty1, double tx2, double ty2, double dx, double dz, double yaw){
        return getDistanceToTarget(tx1, ty1, tx2, ty2, dx, dz, yaw, CAMERA_PITCH);
    }
    
    public static double getDistanceToTarget(double tx1, double ty1, double tx2, double ty2, double ds, double yaw){
        double[] v1 = getVectorToTarget(tx1, ty1, 0, CAMERA_PITCH);
        double[] v2 = getVectorToTarget(tx2, ty2, yaw, CAMERA_PITCH);
        double a = Math.pow(v2[1]*v1[0]-v2[0]*v1[1],2) + Math.pow(v1[2]*v2[1]-v2[2]*v1[1],2);
        double k = ds * v1[1] / Math.sqrt(a);

        double distance = k * getDistanceToTarget(tx2, ty2, yaw, CAMERA_PITCH);

        return distance;
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

    public static boolean checkValidVectors(double tx1, double ty1, double tx2, double ty2, double dx, double dz, double yaw, double pitch, double threshold){
        double[] v1 = getVectorToTarget(tx1, ty1, 0, pitch);
        double[] v2 = getVectorToTarget(tx2, ty2, yaw, pitch);
        double k = dz/((v1[2] * v2[1])/(v1[1]) - v2[2]);
        double j = k * v2[1] / v1[1];

        double dxCalculated = j * v1[0] - k*v2[0];

        return (Math.abs(dxCalculated - dx) <= threshold);
    }
    public static boolean checkValidVectors(double tx1, double ty1, double tx2, double ty2, double dx, double dz, double yaw, double threshold){
        return checkValidVectors(tx1,ty1,tx2,ty2,dx,dz,yaw,CAMERA_PITCH,threshold);
    }
    public static boolean checkValidVectors(double tx1, double ty1, double tx2, double ty2, double dx, double dz, double yaw){
        return checkValidVectors(tx1,ty1,tx2,ty2,dx,dz,yaw,CAMERA_PITCH,DEFAULT_THRESHOLD);
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
    public static double getAgreedDistance(double tx1, double ty1, double tx2, double ty2, double dx, double dz, double yaw, double pitch, double threshold){
        double binocularDistance = getDistanceToTarget(tx1,ty1,tx2,ty2,dx,dz,yaw,pitch);
        double monocularDistance = getDistanceToTarget(tx2,ty2,yaw,pitch);
        if (!checkValidVectors(tx1,ty1,tx2,ty2,dx,dz,yaw,pitch,threshold)){
            return -1.0;
        }
        if(Math.abs(binocularDistance - monocularDistance) <= threshold){
            return (binocularDistance + monocularDistance) / 2;
        }
        else{
            return -1.0;
        }
    }
    public static double getAgreedDistance(double tx1, double ty1, double tx2, double ty2, double dx, double dz, double yaw, double threshold){
        return getAgreedDistance(tx1,ty1,tx2,ty2,dx,dz,yaw, CAMERA_PITCH, threshold);
    }
    public static double getAgreedDistance(double tx1, double ty1, double tx2, double ty2, double dx, double dz, double yaw){
        return getAgreedDistance(tx1,ty1,tx2,ty2,dx,dz,yaw, CAMERA_PITCH, DEFAULT_THRESHOLD);
    }
    
}
