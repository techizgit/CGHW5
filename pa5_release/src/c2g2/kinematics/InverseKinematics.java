package c2g2.kinematics;

import java.util.ArrayList;
import org.ejml.simple.SimpleMatrix;
import org.joml.Matrix3d;
import org.joml.Vector2d;
import org.joml.Vector3d;

/*
 * This is the class to implement your inverse kinematics algorithm.
 * 
 * TODO:
 * Please include your data structure and methods for the inverse kinematics here.
 */
public class InverseKinematics {
    
    private Skeleton2D skeleton;
    // Feel free to add more variables as needed.
    
    public InverseKinematics(Skeleton2D ske) {
        if (ske == null) throw new NullPointerException("The provided skeleton is NULL!");
        skeleton = ske;
    }

    // Add methods for updating the skeleton.
    public void updateInverse(double x, double y,Joint2D endNode,ArrayList<Joint2D> nodeList)
    {
    	Joint2D root = skeleton.getRoot();	
    	ArrayList<Joint2D> mainNodeList = new ArrayList<Joint2D>();
    	Joint2D tempNode = endNode.getParentJoint();
    	while(tempNode != root) {
    		mainNodeList.add(tempNode);
    		tempNode = tempNode.getParentJoint();
    	}
    	mainNodeList.add(root);//All joints whose relative angles could be affected in the inverse kinectmatics process
    	double[][] jmat = new double[3][mainNodeList.size()];//Forming Jacobian
    	double[][] emat = new double[3][1];
    	for (int i=0;i<mainNodeList.size();i++) {
    		int ii = mainNodeList.size()-i-1;
    		Vector2d p = mainNodeList.get(i).getPos();
    		Vector3d m = new Vector3d(endNode.getPos().x-p.x,endNode.getPos().y-p.y,0);
    		m.cross(0, 0, 1).mul(-1);
    		jmat[0][ii]=m.x;jmat[1][ii]=m.y;jmat[2][ii]=m.z;//Please see my report for details about how I compute Jacobian

    	}
    	SimpleMatrix J = new SimpleMatrix(jmat);
		emat[0][0]=x-endNode.getPos().x;
		emat[1][0]=y-endNode.getPos().y;
		emat[2][0]=0;
    	J=J.pseudoInverse();   	//Use pseudo inverse operation to solve linear equations
    	SimpleMatrix de = new SimpleMatrix(emat);
    	SimpleMatrix dAngle = J.mult(de);
    	dAngle=dAngle.scale(0.2);//Now we have the angle change for all joints in the main node list
    	
    	
    	
    	//This part is actually a forward kinectmatics implementation, but with dAngle list
    	int ii=mainNodeList.size()-1;
    	double[] angles = skeleton.getAngles();
    	for (int i=0;i<nodeList.size();i++) {
    		Joint2D tNode = nodeList.get(i);
    		Vector2d p = tNode.getParentJoint().getPos();
    			if ((ii>=0)&&(tNode.getParentJoint()==mainNodeList.get(ii)) && (mainNodeList.contains(tNode) || tNode==endNode)) {
    				double rotateAngle = tNode.getRotateAngle()+dAngle.get(mainNodeList.size()-ii-1);
    				ii=ii-1;
    				tNode.setRotateAngle(rotateAngle);
    				double cos = Math.cos(rotateAngle);
    				double sin = Math.sin(rotateAngle);
    				Matrix3d transform = new Matrix3d(cos, sin, 0, -sin, cos, 0, p.x, p.y, 1);
    				double length = tNode.getLength();
    				Vector3d res = (new Vector3d(length, 0, 1)).mul(transform);
    				Vector2d position = new Vector2d(res.x, res.y);
    				tNode.setPos(position);
    			} else {
    				double rotateAngle = angles[i] + tNode.getParentJoint().getRotateAngle();
    				tNode.setRotateAngle(rotateAngle);
    				double cos = Math.cos(rotateAngle);
    				double sin = Math.sin(rotateAngle);
    				Matrix3d transform = new Matrix3d(cos, sin, 0, -sin, cos, 0, p.x, p.y, 1);
    				double length = tNode.getLength();
    				Vector3d res = (new Vector3d(length, 0, 1)).mul(transform);
    				Vector2d position = new Vector2d(res.x, res.y);
    				tNode.setPos(position);
    			}
    			

    	}
    	
    }
}
