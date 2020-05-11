
import java.text.DecimalFormat;
import java.time.Duration;

/**
 * This class represents the basic flight controller of the Bereshit space craft.
 * @author ben-moshe
 * The landing should take 17 minutes
 *  Z axis is front of space craft, plus is slowing down 1m/s^2
 */
public class working_real_p {
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
		double alt = 30000;//25137
		double time = 0;

		double dt = 1; // sec
		double acc = 1.663; // Acceleration rate (m/s^2)
		double fuel = 215.06; //
		double weight = WEIGHT_EMP + fuel;
		System.out.println("time, vs, hs, alt, ang,weight,acc");
		double NN = 1; // rate[0,1] // percentage of engine streights
		DecimalFormat df2 = new DecimalFormat("#.###");
		double prevVacc = 0;
		double prevVsSpeed = vs;
		double prevHsSpeed = hs;

		double limit =45;

		PidController vVelocityPid = new PidController(4.31715,0.01,0.1,400,-400);
		vVelocityPid.SetPoint = 0;
		vVelocityPid.setProcessVariable(vs);

		PidController hVelocityPid = new PidController(1,0.01,0.1,1699,-1699);
		hVelocityPid.SetPoint = 0;
		hVelocityPid.setProcessVariable(hs);


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

			prevVacc = v_acc;

			// start landing pid move hs and vs to 0
//			if(hs < 30) {
//				ang -= 2.45;
//				NN = 0.71;
//			}
//			if(alt < 200){
//				NN = 0.745;
//			}

			// start landing pid, move hs to 0
//			if(hs  > 50){
//				ang += anglePid(prevVacc,v_acc);
//				prevVacc = v_acc;
//
//			}
//			if(hs < 50 ) {
//				ang -= 2;
//				NN -= 0.01;
////				if(hs < 2.5){  // start vs pid move it to ms
////					//ang += speedPid(prevVacc,vs);
////					//NN += powerPid(prevVsSpeed,vs,1);
////					ang = 0;
////					NN = 0.463;
////
////				}
////				else{ // activate hs pid until it reaches 2.5ms
////					ang += speedPid(prevHsSpeed,hs);
////					NN += powerPid(prevHsSpeed,hs,1);
////
////				}
//			}
			prevVacc = v_acc;
			prevVsSpeed = vs;
			prevHsSpeed = hs;


			vVelocityPid.setProcessVariable(vs);
			double vVelocityResult  = vVelocityPid.ControlVariable(Duration.ofSeconds(1));

			hVelocityPid.setProcessVariable(hs);
			double hVelocityResult  = hVelocityPid.ControlVariable(Duration.ofSeconds(1));

			double tmpVResult = vVelocityResult/hVelocityResult;
			if(tmpVResult < 1 ){
				ang = setAngle(90/(vVelocityResult + hVelocityResult) * hVelocityResult);
			}
			else{
				ang = setAngle(90/(vVelocityResult + hVelocityResult) * hVelocityResult);
			}

			NN = 1;//setEnginePower(NN);

			if(fuel < 0){
				alt=0;
				System.out.println("No Fuels");
			}

//			System.out.println("time: "+df2.format(time)+", vs: "+df2.format(vs)+", " +
//					"hs: "+df2.format(hs)+", alt: "+df2.format(alt)+",ang: "+df2.format(ang)+", " +
//					"weight: "+df2.format(weight)+", acc: "+df2.format(acc)+" v_acc: "+df2.format(v_acc)+" h_acc: "+df2.format(h_acc)+ " fuel: "+df2.format(fuel) +
//					",moonvacc: " + df2.format(vacc) + " ,NN: " +df2.format(NN));

			System.out.println("ang: "+df2.format(ang)+",   PidV: " + df2.format(vVelocityResult) + ",    PidH: " +df2.format(hVelocityResult)+ ",  " +
					"  vs: " +df2.format(vs)+ ",    " + "hs: " +df2.format(hs)+ ",    alt: " +df2.format(alt)+ ",  " +
					"  v_acc: " +df2.format(v_acc)+",    h_acc: " +df2.format(h_acc)+ ",   Fuel: "+df2.format(fuel)+",    NN: " +df2.format(NN) );

		}


	}


	public static double anglePid(double prevVerticalAcc,double currentVerticalAcc){
		// TODO: add prevVerticalAcc to accuation
		double angleChange = 0.1;
		double desiredVerticalAcc = 0.01;
		double  deltaDesiredCurrent = desiredVerticalAcc/currentVerticalAcc;
		return angleChange * (1-deltaDesiredCurrent) ;
	}

	public static double speedPid(double prevSpeed,double currentSpeed){
		// TODO: add prevSpeed to accuation

		double angleChange = 1.88;
		double desiredSpeed = 1;
		double  deltaDesiredCurrent = desiredSpeed/currentSpeed;
		return angleChange * (deltaDesiredCurrent - 1) ;
	}

	public static double powerPid(double prevSpeed,double currentSpeed,double factor){
		// TODO: add prevSpeed to accuation
		double powerChange = -0.01;
		if(prevSpeed < currentSpeed){
			powerChange = 0.01;
		}
		//double powerChange = 0.01;
		double desiredSpeed = 2.5;
		double  deltaDesiredCurrent = desiredSpeed/currentSpeed;
		return powerChange * (deltaDesiredCurrent - 1) * factor;
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