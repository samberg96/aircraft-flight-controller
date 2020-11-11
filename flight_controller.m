%% Boeing 747 Flight Controller Model
% Sam Weinberg
% Dec. 14/2018

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% DESCRIPTION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% This file contains all of the Boeing 747 paramters as referenced
% from Etkin's "Dynamics of Flight: Stability and Control". The control
% model uses linearized longitudinal and lateral flight dynamics for
% tuning. In Simulink, both linear and nonlinear flight simulations were
% developed to analyze the controller's performance.

clc
close all
clear all

% Boeing 747 Parameters

W = 2.83176e+6; % weight (N)
S = 511; % wing plan area (m^2)
c_bar = 8.324; % mean chord (m)
b = 59.64; % wing span (m)
Ixx = 0.247e+8; % moments of inertia (kg m^2)
Iyy = 0.449e+8;
Izz = 0.673e+8;
Ixz = -0.212e+7;
Uo = 235.9; % steady flight velocity (m/s)
rho = 0.3045; % density (kg/m^3)
g = 9.81; % gravity (m/s^2)
m = W / g; % mass (kg)
theta_o = 0.0;
Cwo = W / (0.5 * rho * S * Uo^2); % non-dimensional term of weight \hat{mg}
CLo = Cwo * cos(theta_o); % initial C_L lift coefficients
CDo = 0.043; % initial C_D lift coefficients (given)

Czo = - CLo;
Cxo = Cwo * sin(theta_o);
CTo = CDo + Cxo;
Cmo = 0.0;

% (2.4) longitudinal terms
Cxu = -0.1080;
Cxa = 0.2193; % C_{x_\alpha}
Cxq = 0.0;
Cxdu = 0.0; % C_{x_{\dot u}}
Cxda = 0.0; % C_{x_{\dot \alpha}}
Cxdq = 0.0;
Czu = -0.1060;
Cza = -4.9200;
Czq = -5.9210;
Czdu = 0.0;
Czda = 5.8960;
Czdq = 0.0;
Cmu = 0.1043;
Cma = -1.0230;
Cmq = -23.9200;
Cmdu = 0.0;
Cmda = -6.3140;
Cmdq = 0.0;
Cxdelta_e = -3.818e-6; %C_{x_{\delta_e}}
Czdelta_e = -0.3648;
Cmdelta_e = -1.444;
Cxdelta_p = 0.3 * Cwo; % this and the following delta_p are assumptions
Czdelta_p = 0;
Cmdelta_p = 0;

% (2.5) lateral terms
Cyb = -0.8771; % C_{y_\beta}
Cyp = 0;
Cyr = 0;
Cydb = 0; % C_{y_{\dot \beta}}
Cydp = 0;
Cydr = 0;
Clb = -0.2797;
Clp = -0.3295;

Clr = 0.3040;
Cldb = 0; % C_{l_{\dot \beta}}
Cldp = 0;
Cldr = 0;
Cnb = 0.1946;
Cnp = -0.04073;
Cnr = -0.2737;
Cndb = 0; % C_{n_{\dot \beta}}
Cndp = 0;
Cndr = 0;
Cydelta_a = 0; %C_{y_{\delta_a}}
Cydelta_r = 0.1146;
Cldelta_a = -1.368e-2;
Cldelta_r = 6.976e-3;
Cndelta_a = -1.973e-4;
Cndelta_r = -0.1257;

%% Dimensional Terms

% Longitudinal
% Forward force
Xu = rho*Uo*S*Cwo*sin(theta_o) + 1/2*rho*Uo*S*Cxu;
Xw = 1/2*rho*Uo*S*Cxa;

% Vertical force
Zu = -rho*Uo*S*Cwo*cos(theta_o) + 1/2*rho*Uo*S*Czu;
Zw = 1/2*rho*Uo*S*Cza;
Zq = 1/4*rho*Uo*c_bar*S*Czq;

% Pitch Moment
Mu = 1/2*rho*Uo*c_bar*S*Cmu;
Mw = 1/2*rho*Uo*c_bar*S*Cma;
Mq = 1/4*rho*Uo*c_bar^2*S*Cmq;
Zw_dot = 1/4*rho*c_bar*S*Czda;
Mw_dot = 1/4*rho*c_bar^2*S*Cmda;

% Control Derivatives
Xdelta_e = Cxdelta_e*1/2*rho*Uo^2*S;
Xdelta_p = Cxdelta_p*1/2*rho*Uo^2*S;
Zdelta_e = Czdelta_e*1/2*rho*Uo^2*S;
Zdelta_p = Czdelta_p*1/2*rho*Uo^2*S;
Mdelta_e = Cmdelta_e*1/2*rho*Uo^2*S*c_bar;
Mdelta_p = Cmdelta_p*1/2*rho*Uo^2*S*c_bar;

% Lateral
% Side force
Yv = Cyb*1/2*rho*Uo*S;
Yp = Cyp*1/4*rho*Uo*S*b;
Yr = Cyr*1/4*rho*Uo*S*b;

% Roll Moment
Lv = Clb*1/2*rho*Uo*S*b;
Lp = Clp*1/4*rho*Uo*S*b^2;
Lr = Clr*1/4*rho*Uo*S*b^2;

% Yaw Moment
Nv = Cnb*1/2*rho*Uo*S*b;
Np = Cnp*1/4*rho*Uo*S*b^2;
Nr = Cnr*1/4*rho*Uo*S*b^2;

% Control Derviatives
Ydelta_r = Cydelta_r*1/2*rho*Uo^2*S;
Ldelta_a = Cldelta_a*1/2*rho*Uo^2*S*b;
Ndelta_a = Cndelta_a*1/2*rho*Uo^2*S*b;
Ldelta_r = Cldelta_r*1/2*rho*Uo^2*S*b;
Ndelta_r = Cndelta_r*1/2*rho*Uo^2*S*b;

% Simplified Products of Inertia
Ix2 = (Ixx*Izz-Ixz^2)/Izz;
Iz2 = (Ixx*Izz-Ixz^2)/Ixx;
Ixz2 = Ixz/(Ixx*Izz-Ixz^2);


%% State Space Representation

% Add rho to account for change in density


% Longitudinal
% System Matrix
a11_long = Xu/m;
a12_long = Xw/m;
a13_long = 0;
a14_long = -g*cos(theta_o);
a15_long = 0;

a21_long = Zu/(m-Zw_dot);
a22_long = Zw/(m-Zw_dot);
a23_long = (Zq + m*Uo)/(m - Zw_dot);
a24_long = -m*g*sin(theta_o)/(m - Zw_dot);
a25_long = 0;

a31_long = 1/Iyy*(Mu + Mw_dot*Zu/(m - Zw_dot));
a32_long = 1/Iyy*(Mw + Mw_dot*Zw/(m - Zw_dot));
a33_long = 1/Iyy*(Mq + Mw_dot*(Zq + m*Uo)/(m - Zw_dot));
a34_long = -Mw_dot*m*g*sin(theta_o)/(Iyy*(m - Zw_dot));
a35_long = 0;

a41_long = 0;
a42_long = 0;
a43_long = 1;
a44_long = 0;
a45_long = 0;

a51_long = -sin(theta_o);
a52_long = cos(theta_o);
a53_long = 0;
a54_long = -Uo*cos(theta_o);
a55_long = 0;

A_long = [a11_long, a12_long, a13_long, a14_long, a15_long; 
    a21_long, a22_long, a23_long, a24_long, a25_long;
    a31_long, a32_long, a33_long, a34_long, a35_long; 
    a41_long, a42_long, a43_long, a44_long, a45_long; 
    a51_long, a52_long, a53_long, a54_long, a55_long];


% Control Matrix
b11_long = Xdelta_e/m;
b12_long = Xdelta_p/m;
b21_long = Zdelta_e/m;
b22_long = Zdelta_p/m;
b31_long = Mdelta_e/Iyy;
b32_long = Mdelta_p/Iyy;
b41_long = 0;
b42_long = 0;
b51_long = 0;
b52_long = 0;

B_long = [b11_long, b12_long;
    b21_long, b22_long;
    b31_long, b32_long;
    b41_long, b42_long;
    b51_long, b52_long];

% C and D matrix from ch. 6 notes
C_long = eye(5);
D_long = [0, 0;
    0, 0;
    0, 0;
    0, 0;
    0, 0];

% Lateral
% System Matrix
a11_lat = Yv/m;
a12_lat = Yp/m;
a13_lat = Yr/m - Uo;
a14_lat = g*cos(theta_o);
a15_lat = 0;
a21_lat = Lv/Ix2 + Nv*Ixz2;
a22_lat = Lp/Ix2 + Np*Ixz2;
a23_lat = Lr/Ix2 + Nr*Ixz2;
a24_lat = 0;
a25_lat = 0;

a31_lat = Nv/Iz2 + Lv*Ixz2;
a32_lat = Np/Iz2 + Lp*Ixz2;
a33_lat = Nr/Iz2 + Lr*Ixz2;
a34_lat = 0;
a35_lat = 0;

a41_lat = 0;
a42_lat = 1;
a43_lat = tan(theta_o);
a44_lat = 0;
a45_lat = 0;

a51_lat = 0;
a52_lat = 0;
a53_lat = sec(theta_o);
a54_lat = 0;
a55_lat = 0;

A_lat = [a11_lat, a12_lat, a13_lat, a14_lat, a15_lat;
        a21_lat, a22_lat, a23_lat, a24_lat, a25_lat; 
        a31_lat, a32_lat, a33_lat, a34_lat, a35_lat;
        a41_lat, a42_lat, a43_lat, a44_lat, a45_lat; 
        a51_lat, a52_lat, a53_lat, a54_lat, a55_lat];

% Control Matrix
b11_lat = 0;
b12_lat = Ydelta_r/m;
b21_lat = Ldelta_a/Ix2+Ndelta_a*Ixz2;
b22_lat = Ldelta_r/Ix2+Ndelta_r*Ixz2;
b31_lat = Ndelta_a/Iz2+Ldelta_a*Ixz2;
b32_lat = Ndelta_r/Iz2+Ldelta_r*Ixz2;
b41_lat = 0;
b42_lat = 0;
b51_lat = 0;
b52_lat = 0;

B_lat = [b11_lat, b12_lat;
    b21_lat, b22_lat;
    b31_lat, b32_lat;
    b41_lat, b42_lat;
    b51_lat, b52_lat];

% C and D matrix from ch. 6 notes
C_lat = eye(5);
D_lat = zeros(5,2);


% Constructing State Space Representations

sys_long = ss(A_long, B_long, C_long, D_long,'OutputName',{'u','w','q','\theta','Ze'},...
'InputName',{'\delta_e','\delta_p'});

G_long = tf(sys_long(4,1)); % Extracting transfer function

sys_lat = ss(A_lat, B_lat, C_lat, D_lat,'OutputName',{'v','p','r','\phi','\psi'},...
'InputName',{'\delta_a','\delta_r'});

G_lat = tf(sys_long(3,1)); % Extracting transfer function

Gup = tf(sys_long(1,2));


% Visualizations of Longitudinal Results (Leave Commented)

% % Closed Loop
% cl = feedback(sys_lat(4,1), -1);
% 
% % Open Loop vs Closed Loop Response 
% figure(1)
% step(sys_lat(4,1),'-r',cl,'--k')
% hold off
%  
% figure(2)
% impulse(sys_lat(4,1),'-r',cl,'--k')
% hold off
%  
% figure(3)
% rlocus(-sys_lat(4,1))
% sgrid
% hold off
% 
% % sim('Linear_Model',200)
% % [l1,l2] = size(Z);
% % sim('Linear_Model',500)
% % comet3(Uo*cos(psi)*200, Uo*sin(psi)*200, Z)
% % comet3(x,y,z)