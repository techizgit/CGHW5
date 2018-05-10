package c2g2.kinematics;

import java.util.ArrayList;

import org.joml.Matrix3d;
import org.joml.Vector2d;
import org.joml.Vector3d;

/*
 * Class that implements the forward Kinematics algorithm.
 */
public class ForwardKinematics {
    
    private Skeleton2D skeleton;
    // TODO: Feel free to add more variables as needed.
    
    public ForwardKinematics(Skeleton2D ske) {
        if (ske == null) throw new NullPointerException("The provided skeleton is NULL!");
        skeleton = ske;
   }


    public void updateState(double[] params) {
        // TODO: Implement the forward kinematics azlgorithm here
    	
        Joint2D root = skeleton.getRoot();
        ArrayList<Joint2D> nodeList = new ArrayList<Joint2D>();
        skeleton.findChildNode(root, nodeList);//Toposort the joints to implement the forward kinectmatics
        int i=0;
        for (Joint2D node: nodeList) {
        	double selfAngle = params[i];i=i+1;
            Vector2d p = node.getParentJoint().getPos();//Parent joint location
           	double rotateAngle = selfAngle + node.getParentJoint().getRotateAngle();
           	node.setRotateAngle(rotateAngle);//This a actually a key point, changing the rotation angle for all parents 
           	double cos = Math.cos(rotateAngle);
            double sin = Math.sin(rotateAngle);
            Matrix3d transform = new Matrix3d(cos, sin, 0, -sin, cos, 0, p.x, p.y, 1);//Forward kinectmatics
            double length = node.getLength();
            Vector3d res = (new Vector3d(length, 0, 1)).mul(transform);
            Vector2d position = new Vector2d(res.x, res.y);
            node.setPos(position);
       }
    }
}

