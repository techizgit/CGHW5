package c2g2.kinematics.animation;

import java.util.ArrayList;

/* Cubic Bezier interpolation of angles. 
 * You want to take into account of all frames and use two consecutive frames 
 * to specify the tangent direction of the control points. */
public class CubicBezier extends Interpolator {

	// Add more variables if needed.
	private int duration; // the length of the animation
	private int N; // the number of total frames in the animation
	
	public CubicBezier(ArrayList<double[]> frames, ArrayList<Integer> times, int fps) {
		super(frames, times, fps);
		duration = kfTimestamps[kfTimestamps.length-1] - kfTimestamps[0];
		N = duration * fps;
	}
	
	private double[] pminus(double[] pa,double[] pb) {
		int l = pa.length;
		double[] p = new double[l];
		for (int i=0;i<l;i++) {
			p[i]=pa[i]-pb[i];
		}	
		return p;	
	}
	
	private double[] pplus(double[] pa,double[] pb) {
		int l = pa.length;
		double[] p = new double[l];
		for (int i=0;i<l;i++) {
			p[i]=pa[i]+pb[i];
		}	
		return p;	
	}
	
	private double[] pscale(double[] pa,double s) {
		int l = pa.length;
		double[] p = new double[l];
		for (int i=0;i<l;i++) {
			p[i]=pa[i]*s;
		}	
		return p;	
	}

	@Override
	public double[] interp(double t) {
		// TODO: implement cubic bezier interpolation. 
		int whichFrame = (int) (t * N); 
		for (int i = 0; i < kfTimestamps.length; i++) {
			if ((kfTimestamps[i]-kfTimestamps[0]) * fps > whichFrame) {
				whichFrame = i;
				break;
			}
		}
		if (whichFrame<=kfTimestamps.length-4) {//If this interval is not the last interval, we only have 3 points
			double[] p0 = kfAngles.get(((whichFrame-1)/2)*2);
			double[] p1 = kfAngles.get(((whichFrame-1)/2)*2+1);
			double[] p3 = kfAngles.get(((whichFrame-1)/2)*2+2);
			double[] p4 = kfAngles.get(((whichFrame-1)/2)*2+3);
			double[] dp = pminus(p4,p3);
			double[] p2 = pminus(p3,dp);//Calculating p2 for cubic bezier, for details please see my report
			double startTime = (double) kfTimestamps[((whichFrame-1)/2)*2];
			double endTime = (double) kfTimestamps[((whichFrame-1)/2)*2+2];
			double tL = (t-startTime/(double)duration)/((endTime-startTime)/(double)duration);//Calculating t for this interval
			double c0 = (1-3*tL+3*tL*tL-tL*tL*tL);	//Cubic Bezier interpolation
			double c1 = (3*tL-6*tL*tL+3*tL*tL*tL);
			double c2 = (3*tL*tL-3*tL*tL*tL);
			double c3 = tL*tL*tL;
			double p[] = pplus(pscale(p0,c0),pscale(p1,c1));
			p = pplus(p,pscale(p2,c2));
			p = pplus(p,pscale(p3,c3));
			return p;
		}else {//If this interval is the last interval, implement cubic bezier interpolation directly
			int head = kfTimestamps.length-4;
			double[] p0 = kfAngles.get(head);
			double[] p1 = kfAngles.get(head+1);
			double[] p2 = kfAngles.get(head+2);
			double[] p3 = kfAngles.get(head+3);
			double startTime = (double) kfTimestamps[head];
			double endTime = (double) kfTimestamps[head+3];
			double tL = (t-startTime/(double)duration)/((endTime-startTime)/(double)duration);//Calculating t for this interval
			double c0 = (1-3*tL+3*tL*tL-tL*tL*tL);	//Cubic Bezier interpolation
			double c1 = (3*tL-6*tL*tL+3*tL*tL*tL);
			double c2 = (3*tL*tL-3*tL*tL*tL);
			double c3 = tL*tL*tL;
			double p[] = pplus(pscale(p0,c0),pscale(p1,c1));
			p = pplus(p,pscale(p2,c2));
			p = pplus(p,pscale(p3,c3));
			return p;
		}
	}
}