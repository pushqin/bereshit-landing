
import java.text.DecimalFormat;

/**
 * This class represents the basic flight controller of the Bereshit space craft.
 * @author ben-moshe
 * The landing should take 17 minutes
 *  Z axis is front of space craft, plus is slowing down 1m/s^2
 */
public class working_pid_vertical_acc {
	public static final double WEIGHT_EMP = 164; // kg
	public static final double WEIGHT_FULL = 585; // kg// 585

	public static final double MAIN_ENG_FORCE = 430; // N
	public static final double SECOND_ENG_FORCE = 25; // N
	public static final double MAIN_BURN = 0.15; //liter per sec ----- 12 liter per m'
	public static final double SECOND_BURN = 0.009; //liter per sec ----- 0.6 liter per m'
	public static final double ALL_BURN = MAIN_BURN + 8*SECOND_BURN;

	public static double accMax(double weight) {
		return acceleration(weight, true,8);
	}

	public static double acceleration(double weight, boolean main, int seconds) {
		double t = 0;
		if(main) {t += MAIN_ENG_FORCE;}
		t += seconds* SECOND_ENG_FORCE;
		double ans = t/weight;
		return ans;
	}

	// 14095, 955.5, 24.8, 2.0
	public static void main(String[] args) {
		// 23:16(as in the simulation) // https://www.youtube.com/watch?time_continue=1396&v=HMdUcchBYRA&feature=emb_title
		System.out.println("Simulating Bereshit's Landing:");
		// starting point:

		double vs = 42.9;//42.9
		double hs = 1699;
		//double dist = 181*1000; ????
		double ang = 80; // zero is vertical (as in landing)
		double alt = 25000;//25137
		double time = 0;

		double dt = 1; // sec
		double acc = 1.663; // Acceleration rate (m/s^2)
		double fuel = 215.06; //
		double weight = WEIGHT_EMP + fuel;
		System.out.println("time, vs, hs, alt, ang,weight,acc");
		double NN = 1; // rate[0,1] // percentage of engine streights
		DecimalFormat df2 = new DecimalFormat("#.###");
		double prevVacc = 0;

		double limit =45;
		// ** main simulation loop ***
		while(alt>0) {

			// main computations
			double ang_rad = Math.toRadians(ang);
			double h_acc = Math.sin(ang_rad)*acc;
			double v_acc = Math.cos(ang_rad)*acc;
			double vacc = Moon.getAcc(hs);
			time+=dt;
			double dw = dt*ALL_BURN*NN;

			fuel -= dw;
			weight = WEIGHT_EMP + fuel;
			acc = NN* accMax(weight);
			v_acc -= vacc;
			hs -= h_acc*dt;
			vs -= v_acc*dt;
			alt -= dt*vs;

			ang += anglePid(prevVacc,v_acc);
			prevVacc = v_acc;

			// start landing pid move hs and vs to 0
//			if(hs < 50 ){
//				ang-=1.5;
//				NN=0.7;
//			}
			if(hs < 30){
				ang -= 2.45;
				NN = 0.71;
			}

//			if(hs < 2.5){
//				ang -= 2;
//			}

			if(alt < 200){
				NN = 0.745;
			}

			ang = setAngle(ang);
			NN = setEnginePower(NN);

			if(fuel < 0){
				alt=0;
				System.out.println("No Fuels");
			}

			System.out.println("time: "+df2.format(time)+", vs: "+df2.format(vs)+", " +
					"hs: "+df2.format(hs)+", alt: "+df2.format(alt)+",ang: "+df2.format(ang)+", " +
					"weight: "+df2.format(weight)+", acc: "+df2.format(acc)+" v_acc: "+df2.format(v_acc)+" h_acc: "+df2.format(h_acc)+ " fuel: "+df2.format(fuel) +
					",moonvacc: " + df2.format(vacc) + " ,NN: " +df2.format(NN));
		}


	}

	public static double anglePid(double prevVerticalAcc,double currentVerticalAcc){
		// TODO: add prevVerticalAcc to accuation
		double angleChange = 0.1;
		double desiredVerticalAcc = 0.01;
		double  deltaDesiredCurrent = desiredVerticalAcc/currentVerticalAcc;
		return angleChange * (1-deltaDesiredCurrent) ;
	}

	public static double setAngle(double angle){
		if(angle > 90){
			return 90;
		}
		if(angle < 0){
			return 0;
		}
		return angle;
	}

	public static double setEnginePower(double power){
		if(power > 1){
			return 1;
		}
		if(power < 0){
			return 0;
		}
		return power;
	}
}