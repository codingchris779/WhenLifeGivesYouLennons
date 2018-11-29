package org.firstinspires.ftc.teamcode;
//EXIST
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

/**
 * A custom class to store the robot's position data. Stores X, Y, Z, and Azimuth (degrees).
 */
public class Location {
    private float[] pos;

    public Location() {
        pos = new float[4];
        setLocation(0,0,0,0);
    }
    public Location(float[] location) {
        pos = new float[4];
        setLocation(location);
    }
    public Location(float x, float y, float z, float rot) {
        pos = new float[4];
        setLocation(x,y,z,rot);
    }

    /** Returns float array of current position. [x,y,z,rotation]
     * @return float[]
     */
    public float[] getLocation() {
        return pos;
    }

    /** Returns float of desired index of position. [x,y,z,rotation]
     * @param index index of the desired return value.
     * @return float
     */
    public float getLocation(int index) {
        if(index < 0 || index > 4) throw new IllegalArgumentException("getLocation requires range of 1-4.");
        return pos[index];
    }

    /** Sets location to input.
     * @param location Float array of length 4. [x,y,z,rotation in degrees].
     */
    public void setLocation(float[] location) {
        if(location.length == 4) {
            pos = location;
            pos[3] %= 360;
        }
        else throw new IllegalArgumentException("Invalid location array: x,y,z,rot required.");
    }

    /** Sets location to input.
     * @param x X coordinate.
     * @param y Y coordinate.
     * @param z Z coordinate.
     * @param rot Rotation in degrees.
     */
    public void setLocation(float x, float y, float z, float rot) {
        pos[0] = x;
        pos[1] = y;
        pos[2] = z;
        pos[3] = rot%360;
    }

    /** Sets location to input.
     * @param o OpenGLMatrix for position.
     */
    public void setLocation(OpenGLMatrix o) {
        pos[0] = o.getTranslation().get(0);
        pos[1] = o.getTranslation().get(1);
        pos[2] = o.getTranslation().get(2);
        pos[3] = o.getTranslation().get(3); //<- test plz
    }

    /** Sets stored rotation.
     * @param rot Rotation in degrees.
     */
    public void setRotation(float rot) {
        pos[3] = rot%360;
    }

    /** Translates stored location forward given units based on object rotation.
     * Use negative values for opposite direction.
     * @param forward Translates position given units forward.
     */
    public void translateLocal(float forward) {
        translateLocal(forward, 0f);
    }

    /** Translates stored location forward and right given units based on object rotation.
     * Use negative values for opposite direction.
     * @param forward Translates position given units forward.
     * @param right Translates position given units right.
     */
    public void translateLocal(float forward, float right) {
        translateLocal(forward, 0f, right);
    }

    /** Translates stored location forward, up, and right given units based on object rotation.
     * Use negative values for opposite direction.
     * @param forward Translates position given units forward.
     * @param right Translates position given units right.
     * @param up Translates position given units up.
     */
    public void translateLocal(float forward, float up, float right) {
        pos[0] += (float)(forward*Math.cos(Math.toRadians(pos[3])) + right*Math.cos(Math.toRadians(pos[3]-90)));
        pos[1] += up;
        pos[2] += (float)(forward*Math.sin(Math.toRadians(pos[3])) + right*Math.sin(Math.toRadians(pos[3]-90)));
    }

    /** Translates object x and z in world coordinates.
     * Use negative values for opposite direction.
     * @param x Translates position given distance on x axis.
     * @param z Translates position given distance on z axis.
     */
    public void translateWorld(float x, float z) {
        pos[0] += x;
        pos[1] += z;
    }

    /** Returns string representation of location.
     * @return String
     */
    public String toString() {
        return "["+pos[0]+","+pos[1]+","+pos[2]+","+pos[3]+"]";
    }

}
