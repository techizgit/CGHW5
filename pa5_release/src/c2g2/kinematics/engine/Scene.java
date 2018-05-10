package c2g2.kinematics.engine;

import c2g2.kinematics.*;
import java.util.ArrayList;
import java.util.Arrays;
import org.joml.Vector2d;

public class Scene {

    public Skeleton2D skeleton;

    private int xmlFileCount = 0;

    public Scene() {}

    public void loadfromXML(String filename) {
        skeleton = new Skeleton2D(filename);
    }
    
    // Return a list of joint positions.
    public ArrayList<Vector2d> getJointPos() {
        ArrayList<Vector2d> posList = new ArrayList<Vector2d>();
        visitNodePos(skeleton.getRoot(), posList);
        return posList;
    }
    
    // Add parent-child pairs as needed by the renderer. 
    private void visitNodePos(Joint2D currentJoint, ArrayList<Vector2d> posList) {
        ArrayList<Joint2D> cJoints = currentJoint.getChildJoints();
        for (Joint2D cJoint: cJoints) {
            posList.add(currentJoint.getPos());
            posList.add(cJoint.getPos());
            if (cJoint.isLeaf()) { continue; }
            visitNodePos(cJoint, posList);
        }
    }

    // When you press 'P', this method is called. 
    public void printAnglesToScreen() {
        System.out.println(Arrays.toString(skeleton.getAngles()));
    }

    // When you press 'S', this method is called.
    public void savePoseToXML() {
        try {
            ++xmlFileCount;
            skeleton.printPoseToXML(xmlFileCount);
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
    
}
