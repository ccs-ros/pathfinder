/*	Navigation Filter
 *  nav_filter_node.cpp
 *
 *	Author: Jared Strader and Kyle Lassack
 *  Created On: 4/9/14
 *	Updated On: 4/10/14
 *
 */

#include <ros/ros.h>
#include <navigation/FilterInput.h>
#include <navigation/FilterOutput.h>
#include <armadillo>

using namespace std;

const float PI = 3.14159265;

class Sensors //Class containing callback function for data coming into filter
{
public:
  ros::Subscriber sub;
  float encFL, encFR, encBL, encBR, stopFlag, turnFlag;
  float p, q, r, ax, ay, az;

  Sensors() : encFL(0), encFR(0), encBL(0), encBR(0), stopFlag(0), 
  				turnFlag(0), p(0), q(0), r(0), ax(0), ay(0), az(0)
  {
    ros::NodeHandle node;
    sub = node.subscribe("nav/temp_data", 1, &Sensors::getDataCallback, this);
  }

  void getDataCallback(const navigation::FilterInput::ConstPtr &msg) {
    this->encFL = msg->encFL;
    this->encFR = msg->encFR;
    this->encBL = msg->encBL;
    this->encBR = msg->encBR;
    this->p = msg->p;
    this->q = msg->q;
    this->r = msg->r;
    this->ax = msg->ax;
    this->ay = msg->ay;
    this->az = msg->az;
    this->stopFlag = msg->stopFlag;
    this->turnFlag = msg->turnFlag;
  }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nav_filter_node");
    ros::NodeHandle nh;
    ros:: Publisher pub = nh.advertise<navigation::FilterOutput>("nav/filter_output",1);
    
    ROS_INFO("nav_filter_node started...");
    
    navigation::FilterOutput outData;
    Sensors inData;

	// Initial Conditions
	arma::mat Q_eq_IMU = 0.01*arma::eye<arma::mat>(6,6);
	arma::mat P = arma::eye<arma::mat>(5,5);
	float V_prev = 1.0;
	float P_v_prev = 0.0;
	float phi = 0.00, theta = 0.0, psi = 0;
	float d = 10.0, gamma = 0.0;
	
	// Initializations
	float Q_WE = 0.01;
	float Ts=0.02;
	float dist_W = 2;
	
	//Must have initialization
	float Q_deltaD, deltaD;
	
    while(ros::ok())
    {
		float V = V_prev+Ts*inData.ax;
		float P_v = P_v_prev+Ts*Ts*Q_eq_IMU(0,0);
    	
    	float dw1 = -1;
		float dw2 =  1;
		float dw3 = -1;
		float dw4 =  1;
		
		//rename encoder variables 
		float WE1 = inData.encFL, WE2 = inData.encFR, WE3 = inData.encBL, WE4 = inData.encBR;
		
		float WE1_pred_V = (WE1/Ts+inData.r*dw1);
		float WE2_pred_V = (WE2/Ts+inData.r*dw2);
		float WE3_pred_V = (WE3/Ts+inData.r*dw3);
		float WE4_pred_V = (WE4/Ts+inData.r*dw4);
		
		float P_v_thresh = 2*(P_v+Q_WE+Q_eq_IMU(3,3)*dist_W/4);
		float Q_r_thresh   = 4*(Q_WE+Q_eq_IMU(3,3)*dist_W/4);
		
		float Slip1 = 1, Slip2 = 1, Slip3 = 1, Slip4 = 1, All_Slip_Flag = 0;
		
		if(abs(V-WE1_pred_V)>P_v_thresh || min(abs(WE1_pred_V-WE2_pred_V), abs(WE1_pred_V-WE4_pred_V))>Q_r_thresh) 
		{

			Slip1=0;//outData.slipFL=0;
		}
								ROS_INFO("abs(V-WE1_pred_V) = %f",abs(V-WE1_pred_V));
		        ROS_INFO("P_v_thresh = %f",P_v_thresh);
		if(abs(V-WE2_pred_V)>P_v_thresh || min(abs(WE2_pred_V-WE1_pred_V), abs(WE2_pred_V-WE3_pred_V))>Q_r_thresh) 
		{
			Slip2=0;//outData.slipFR=0;
		}
		
		if(abs(V-WE3_pred_V)>P_v_thresh || min(abs(WE3_pred_V-WE2_pred_V), abs(WE3_pred_V-WE4_pred_V))>Q_r_thresh) 
		{
			Slip3=0;//outData.slipBL=0;
		}
		
		if(abs(V-WE4_pred_V)>P_v_thresh || min(abs(WE4_pred_V-WE1_pred_V), abs(WE4_pred_V-WE3_pred_V))>Q_r_thresh) 
		{

			Slip4=0;//outData.slipBR=0;
		}

		
		if (Slip1 + Slip3 != 0 && Slip2 + Slip4 != 0) //(outData.slipFL + outData.slipBL != 0 && outData.slipFR + outData.slipBR != 0)
		{
			deltaD  = ((Slip1*WE1+Slip3*WE3)/(Slip1+Slip3)+(Slip2*WE2+Slip4*WE4)/(Slip2+Slip4))/2;
			V = deltaD/Ts;
			Q_deltaD = (1/(Slip1/Q_WE+Slip3/Q_WE)+1/(Slip2/Q_WE+Slip4/Q_WE))/4; //check this
			P_v = Q_deltaD/(Ts*Ts);
		}
		else
		{
			All_Slip_Flag = 1;
			deltaD = Ts*V;
			Q_deltaD = P_v*Ts*Ts;
		}
		
		
		if (inData.stopFlag == 1)
		{
    		deltaD = 0;
   			V = 0;
    		Q_deltaD = Q_WE;
    		P_v = Q_deltaD/(Ts*Ts);
    	}
    	
    	arma::mat Q = arma::zeros<arma::mat>(4,4); //origionally written as (4,4)
    	Q(0,0)= Q_eq_IMU(3,3);
    	Q(1,1)= Q_eq_IMU(4,4);
    	Q(2,2)= Q_eq_IMU(5,5);
    	Q(3,3)= Q_deltaD;
    	
		arma::mat P_km = P;
		arma::vec x_km = 10*arma::ones<arma::vec>(5);

if (inData.stopFlag == 0 && inData.turnFlag == 0)
{
	x_km(0) = phi+Ts*(inData.p+inData.q*sin(phi)*tan(theta)+inData.r*cos(phi)*tan(theta));
    x_km(1) = theta+Ts*(inData.q*cos(phi)-inData.r*sin(phi));
    x_km(2) = psi+Ts*(inData.q*sin(phi)+inData.r*cos(phi))*(1/cos(theta));
    x_km(3) = d+cos(gamma)*cos(theta)*cos(psi)*deltaD+sin(gamma)*cos(theta)*sin(psi)*deltaD;
    x_km(4) = gamma+cos(gamma)*cos(theta)*sin(psi)*deltaD/d-sin(gamma)*cos(theta)*cos(psi)*deltaD/d;
   
    arma::mat A(5,5);
    A(0,0)=Ts*(inData.q*cos(phi)*tan(theta) - inData.r*sin(phi)*tan(theta))+1;
    A(0,1)=Ts*(inData.r*cos(phi)*((tan(theta)*tan(theta)) + 1) + inData.q*sin(phi)*((tan(theta)*tan(theta)) + 1));
    A(0,2)=0;
    A(0,3)=0;
    A(0,4)=0;
    A(1,0)=-Ts*(inData.r*cos(phi) + inData.q*sin(phi));
    A(1,1)=1;
    A(1,2)=0;
    A(1,3)=0;
    A(1,4)=0;
    A(2,0)=(Ts*(inData.q*cos(phi) - inData.r*sin(phi)))/cos(theta);
    A(2,1)=(Ts*sin(theta)*(inData.r*cos(phi) + inData.q*sin(phi)))/(cos(theta)*cos(theta));
    A(2,2)=1;
    A(2,3)=0;
    A(2,4)=0;
    A(3,0)=0;
    A(3,1)=-deltaD*cos(gamma)*cos(psi)*sin(theta) - deltaD*sin(gamma)*sin(psi)*sin(theta);;
    A(3,2)=deltaD*cos(psi)*cos(theta)*sin(gamma) - deltaD*cos(gamma)*cos(theta)*sin(psi);
    A(3,3)=1;
    A(3,4)=deltaD*cos(gamma)*cos(theta)*sin(psi) - deltaD*cos(psi)*cos(theta)*sin(gamma);
    A(4,0)=0;
    A(4,1)=(deltaD*cos(psi)*sin(gamma)*sin(theta))/d - (deltaD*cos(gamma)*sin(psi)*sin(theta))/d;
    A(4,2)=(deltaD*cos(theta)*sin(gamma)*sin(psi))/d + (deltaD*cos(gamma)*cos(psi)*cos(theta))/d;
    A(4,3)=(deltaD*cos(psi)*cos(theta)*sin(gamma))/(d*d) - (deltaD*cos(gamma)*cos(theta)*sin(psi))/(d*d);
    A(4,4)=1 - (deltaD*cos(gamma)*cos(psi)*cos(theta))/d - (deltaD*cos(theta)*sin(gamma)*sin(psi))/d;
   
    arma::mat W(5,4); // COV([p,q,r,deltaD])
    W(0,0)=Ts;
    W(0,1)=Ts*sin(phi)*tan(theta);
    W(0,2)=Ts*cos(phi)*tan(theta);
    W(0,3)=0;
    W(1,0)=0;
    W(1,1)=Ts*cos(phi);
    W(1,2)=-Ts*sin(phi);
    W(1,3)=0;
    W(2,0)=0;
    W(2,1)=(Ts*sin(phi))/cos(theta);
    W(2,2)=(Ts*cos(phi))/cos(theta);
    W(2,3)=0;
    W(3,0)=0;
    W(3,1)=0;
    W(3,2)=0;
    W(3,3)=cos(gamma)*cos(psi)*cos(theta) + cos(theta)*sin(gamma)*sin(psi);
    W(4,0)=0;
    W(4,1)=0;
    W(4,2)=0;
    W(4,3)=(cos(gamma)*cos(theta)*sin(psi))/d - (cos(psi)*cos(theta)*sin(gamma))/d;
    
    P_km = A*P*A.st()+W*Q*W.st();
    
    float phi_km   = x_km(0);
    float theta_km = x_km(1);
    float psi_km   = x_km(2);
    float d_km     = x_km(3);
    float gamma_km = x_km(4);
}

else if (inData.stopFlag == 1)
{
    x_km(0) = phi;
    x_km(0) = theta;
    x_km(0) = psi;
    x_km(0) = d;
    x_km(0) = gamma;
    
    float phi_km   = x_km(0);
    float theta_km = x_km(1);
    float psi_km   = x_km(2);
    float d_km     = x_km(3);
    float gamma_km = x_km(4);
    
    arma::mat Y_tilt;
    Y_tilt(0,0)=cos(theta_km)*inData.ax+sin(phi_km)*sin(theta_km)*inData.ay+cos(phi_km)*sin(theta_km)*inData.az;
    Y_tilt(1,0)=cos(phi_km)*inData.ay-sin(phi_km)*inData.az;
    Y_tilt(2,0)=-sin(theta_km)*inData.ax+cos(theta_km)*sin(phi_km)*inData.ay+cos(phi_km)*cos(theta_km)*inData.az;
    
    arma::mat H_tilt;
    H_tilt(0,0)=inData.ay*cos(phi_km)*sin(theta_km)-inData.az*sin(phi_km)*sin(theta_km);
    H_tilt(0,1)=inData.az*cos(phi_km)*cos(theta_km)-inData.ax*sin(theta_km)+inData.ay*cos(theta_km)*sin(phi_km);
    H_tilt(0,2)=0;
    H_tilt(0,3)=0;
    H_tilt(0,4)=0;
    H_tilt(1,0)=-inData.az*cos(phi_km)-inData.ay*sin(phi_km);
    H_tilt(1,1)=0;
    H_tilt(1,2)=0;
    H_tilt(1,3)=0;
    H_tilt(1,4)=0;
    H_tilt(2,0)=inData.ay*cos(phi_km)*cos(theta_km)-inData.az*cos(theta_km)*sin(phi_km);
    H_tilt(2,1)=-inData.ax*cos(theta_km)-inData.az*cos(phi_km)*sin(theta_km)-inData.ay*sin(phi_km)*sin(theta_km);
    H_tilt(2,2)=0;
    H_tilt(2,3)=0;
    H_tilt(2,4)=0;
    
    arma::mat Z_tilt;
    Z_tilt(0,0)=0;
    Z_tilt(1,0)=0;
    Z_tilt(2,0)=9.81;
    
    arma::mat y_tilt = Z_tilt-Y_tilt;
    arma::mat R_tilt = Q_eq_IMU.submat(1,1,3,3);  //Q_eq_IMU(1:3,1:3); 
    arma::mat S_tilt = H_tilt*P_km*H_tilt.st()+R_tilt; //Assuming the noise is about the same on each axis, the additive assumption is safe
    arma::mat K_tilt = P_km*H_tilt.st()*arma::inv(S_tilt);
    x_km = x_km+K_tilt*y_tilt;
    P_km = (arma::eye<arma::mat>(5,5)-K_tilt*H_tilt)*P_km;
}

else if (inData.turnFlag == 1)
{
    x_km(0) = phi+Ts*(inData.p+inData.q*sin(phi)*tan(theta)+inData.r*cos(phi)*tan(theta));
    x_km(1) = theta+Ts*(inData.q*cos(phi)-inData.r*sin(phi));
    x_km(2) = psi+Ts*(inData.q*sin(phi)+inData.r*cos(phi))*(1/cos(theta));
    x_km(3) = d;
    x_km(4) = gamma;
    
        
    arma::mat A(5,5);
    A(0,0)=Ts*(inData.q*cos(phi)*tan(theta) - inData.r*sin(phi)*tan(theta))+1;
    A(0,1)=Ts*(inData.r*cos(phi)*((tan(theta)*tan(theta)) + 1) + inData.q*sin(phi)*((tan(theta)*tan(theta)) + 1));
    A(0,2)=0;
    A(0,3)=0;
    A(0,4)=0;
    A(1,0)=-Ts*(inData.r*cos(phi) + inData.q*sin(phi));
    A(1,1)=1;
    A(1,2)=0;
    A(1,3)=0;
    A(1,4)=0;
    A(2,0)=(Ts*(inData.q*cos(phi) - inData.r*sin(phi)))/cos(theta);
    A(2,1)=(Ts*sin(theta)*(inData.r*cos(phi) + inData.q*sin(phi)))/(cos(theta)*cos(theta));
    A(2,2)=1;
    A(2,3)=0;
    A(2,4)=0;
    A(3,0)=0;
    A(3,1)=1;
    A(3,2)=0;
    A(3,3)=1;
    A(3,4)=0;
    A(4,0)=0;
    A(4,1)=0;
    A(4,2)=0;
    A(4,3)=0;
    A(4,4)=1;
   
    
    arma::mat W(5,4);
    W << Ts << Ts*sin(phi)*tan(theta) 	<<  Ts*cos(phi)*tan(theta) << 0 << arma::endr
      << 0  << Ts*cos(phi)				<<  -Ts*sin(phi)		   << 0 << arma::endr
      << 0  << (Ts*sin(phi))/cos(theta) << (Ts*cos(phi))/cos(theta)<< 0 << arma::endr
      << 0  << 0                        << 0					   << 0 << arma::endr
      << 0  << 0						<< 0					   << 0 << arma::endr; // COV([p,q,r,deltaD])
    
    P_km = A*P*A.st()+W*Q*W.st();
    
    float phi_km   = x_km(0);
    float theta_km = x_km(1);
    float psi_km   = x_km(2);
    float d_km     = x_km(3);
    float gamma_km = x_km(4);
}

		arma::mat P_k = P_km;
		float phi_k   = x_km(0);
		float theta_k = x_km(1);
		float psi_k   = x_km(2);
		float d_k     = x_km(3);
		float gamma_k = x_km(4);
		
		phi=phi_k;
		theta=theta_k;
		psi=psi_k;
		d=d_k;
		gamma=gamma_k;
		P=P_k;
		
		//OUTPUT
    	outData.slipFL=Slip1;
    	outData.slipFR=Slip2;
    	outData.slipBL=Slip3;
    	outData.slipBR=Slip4;
    	outData.slipFlag=All_Slip_Flag;
    	outData.deltaDistance=deltaD;
    	outData.roll=phi_k;
    	outData.pitch=theta_k;
    	outData.yaw=psi_k;
    	outData.distance=d_k;
    	outData.bearing=gamma_k;
    	
    	pub.publish(outData);
    	
		ros::spinOnce();
		ros::Duration(0.02).sleep();
	}

    return 0;
}
