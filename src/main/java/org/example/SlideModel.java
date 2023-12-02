package org.example;

public class SlideModel {
    //Distances all in mm
    public static double ARMLENGTH = 275; //distance from pivot to pivot
    public static double SLIDEANGLE = Math.toRadians(60); //degrees
    public static double WRISTLENGTH = 100; //mm
    public static double MAXWRISTANGLE = Math.toRadians(270); //degrees
    public static double MINWRISTANGLE = Math.toRadians(0); //degrees
    public static double WRISTANGLE = Math.toRadians(-30); //degrees

    //Information About motor
    private final double WRISTGEARRATIO = (26.0/28.0);


    //minimum distance from the slide to the base of the robot
    private final double MINSLIDEDISTANCE = 275;



    public double radiansToServo(double angle){
        double servoRange = MAXWRISTANGLE - MINWRISTANGLE;
        // Calculate the normalized position of the angle in the servo range
        double normalizedPosition = (angle - MINWRISTANGLE) / servoRange;
        // Convert the normalized position into a servo position between 1 and 0
        double servoPosition = 1 - normalizedPosition;
        // Return the servo position
        return servoPosition;
    }

    /**
     * @param targetX
     * @param targetY
     * @return
     */
    public double[] inverseKinematics(int targetX, int targetY) {
        int tx = targetX - (int)(Math.cos(WRISTANGLE)*WRISTLENGTH);
        double ty = targetY - (int)(Math.sin(WRISTANGLE)*WRISTLENGTH);
        if(Math.atan2(ty, tx) > SLIDEANGLE) return null;
        double minDistance = minDistance(-Math.tan(SLIDEANGLE), 1, 0, tx,ty);
        if(minDistance > ARMLENGTH) return null;

        double[] point = closestPoint(-Math.tan(SLIDEANGLE), 1, 0, tx,ty);

        double parallelDist = Math.sqrt((ARMLENGTH*ARMLENGTH) - (minDistance*minDistance));
        double armAngle;
        double tempY1 = point[1] - (Math.sin(SLIDEANGLE)*parallelDist);
        double tempY2 = point[1] - (Math.sin(SLIDEANGLE+Math.toRadians(180))*parallelDist);
        double slideDistanceSolution1 = Math.hypot(point[0] - (Math.cos(SLIDEANGLE)*parallelDist), tempY1);
        double slideDistanceSolution2 = Math.hypot(point[0] - (Math.cos(SLIDEANGLE+Math.toRadians(180))*parallelDist), tempY2);

        if(slideDistanceSolution2 < 400 && tempY2 > 0){
            armAngle = Math.atan2(minDistance, parallelDist) + Math.toRadians(180) + SLIDEANGLE;
            return new double[]{slideDistanceSolution2, armAngle};
        }else if(slideDistanceSolution1 < 400 && tempY1 > 0){
            armAngle = Math.atan2(parallelDist, minDistance)- (Math.toRadians(90)-SLIDEANGLE);
            return new double[]{slideDistanceSolution1, armAngle};
        }   else return null;

    }

    public double minDistance(double a, double b, double c, double x0, double y0) {
        // Use the formula d = |ax0 + by0 + c| / sqrt(a^2 + b^2)
        double numerator = Math.abs(a * x0 + b * y0 + c);
        double denominator = Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
        return numerator / denominator;
    }

    public double[] closestPoint(double a, double b, double c, double x0, double y0) {
        // Use the formula x = (b(bx0 - ay0) - ac) / (a^2 + b^2) and y = (a(-bx0 + ay0) - bc) / (a^2 + b^2)
        double x = (b * (b * x0 - a * y0) - a * c) / (Math.pow(a, 2) + Math.pow(b, 2));
        double y = (a * (-b * x0 + a * y0) - b * c) / (Math.pow(a, 2) + Math.pow(b, 2));
        return new double[] {x, y};
    }


    public double map(double x, double in_min, double in_max, double out_min, double out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
    public int[] anglesToPosition(float shoulderRot, float elbowRot){
//        //Using forward-kinematics, the position can be calculated
//        int elbowX = (int) (LOWERARMLENGTH * Math.cos(Math.toRadians(shoulderRot)));
//        int elbowY = (int) (LOWERARMLENGTH * Math.sin(Math.toRadians(shoulderRot)));
//
//        int wristX = (int) (elbowX + UPPERARMLENGTH * Math.cos(Math.toRadians(elbowRot)));
//        int wristY = (int) (elbowY + UPPERARMLENGTH * Math.sin(Math.toRadians(elbowRot)));
//        return new int[]{wristX, wristY};
        return null;
    }
}
