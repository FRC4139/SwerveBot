package frc.robot;

public class SwerveMath{

    public static double ROBOT_WIDTH = 22.5; //wheel distance between FR, FL, in inches
    public static double ROBOT_LENGTH = 22.5; //wheel distance between FR, BR, in inches
    public static double ROBOT_RADIUS = Math.sqrt(Math.pow(ROBOT_LENGTH / 2, 2) + Math.pow(ROBOT_WIDTH / 2, 2));
    /**
     * Returns 4 vectors (ordered FR, FL, BL, BR), with (angle(radians), speed).
     * @param rotationSpeed positive is counterclockwise
     * @param dx desired translation x
     * @param dz desired translation z
     * @param yaw 0 is to the right, positive is counterclockwise
     * @return double[4][2] with pairs of (angle,speed)
     */
    public static double[][] getDesiredAngleAndSpeed(double rotationSpeed, double dx, double dz, double yaw){
        double tangentialV = rotationSpeed * ROBOT_RADIUS;
        double[] translationComponent = {Math.atan2(dz,dx) - yaw, Math.sqrt(Math.pow(dx,2) + Math.pow(dz,2))};
        double[][] retArr = new double[4][2];

        for(int i = 0; i < 4; i++){
            double[] rotationComponent = {Math.atan2(ROBOT_LENGTH, ROBOT_WIDTH) + (i+1) * Math.PI/2, tangentialV};
            retArr[i] = getAddedPolarVectors(rotationComponent, translationComponent);
        }

        return retArr;

    }
    /**
     * Adds two vectors given in polar form (angle, magnitude). Legit brain damage.
     * @param vector1 (PUT IT IN POLAR FORM OR DIE)
     * @param vector2 (PUT IT IN POLAR FORM OR DIE)
     * @return AHHHHHH
     */
    private static double[] getAddedPolarVectors(double[] vector1, double[] vector2){
        double theta1 = vector1[0];
        double r1 = vector1[1];
        double theta2 = vector2[0];
        double r2 = vector2[1];

        double addedX = r1 * Math.cos(theta1) + r2 * Math.cos(theta2);
        double addedY = r1 * Math.sin(theta1) + r2 * Math.sin(theta2);

        double addedTheta = Math.atan2(addedY, addedX);
        double addedR = Math.sqrt(Math.pow(addedX,2) + Math.pow(addedY,2));

        double[] retArr = {addedTheta, addedR};

        return retArr;
    }
}