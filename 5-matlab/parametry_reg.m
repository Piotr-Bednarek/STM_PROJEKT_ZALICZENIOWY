%%
% Skok
% K_109= 11.9254;
% K_112= 11.2886;
% K_115= 12.2396;
% K_120= 12.2772;
% K_125= 11.9280;
% K_130= 16.8417;
% K_140= 11.2035;
% 
% K_obl=[K_109, K_112, K_115, K_120, K_125];
% 
% K1= mean(K_obl)
% 
% %Mz(s) = s^(2)+ 2*K*Kd+ K*Kp
% %M(s) = s^(2)+ 2*wn*ksi+ wn^(2)
% 
% wn1= 4.5;
% ksi= 0.707;
% 
% Kp= wn1^(2)/K1;
% 
% Td= (2*ksi*wn1)/K1;
% 
% 
% Ti=3;

Ts= 0.037;

T_serw= 0.095;
c_ball= 5.886;  % mm/s^2
k_mech= 0.2;
K_rad= c_ball*1000*k_mech;  % [s^2*rad]
K_t= K_rad*pi/180 ;

K_d= 11.9318;  


Kp_STM= 0.26;
Ki_STM=0.0064;
Kd_STM= 4.4;

Kp= Kp_STM;
Ki= Ki_STM/Ts;
Kd= Kd_STM*Ts;











