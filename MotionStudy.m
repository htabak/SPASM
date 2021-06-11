% John Tabakian
% UID: 804965211
% UCLA MAE 162E
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all; close all; clc;

%% Dimensions & Definitions
analysis = 'gate';
%analysis = 'chemical';
switch (analysis)
    case {'gate'} 
        r1_x = 2.54; %mm
        r1_y = 15.24; %mm
        r2 = 38.862; %mm
        r3 = 15.875; %mm
        r4 = 37.084; %mm
        rg3 = 54.014; %mm
        r = [r1_x, r2, r3, r4, r1_y]/1000; %m
        theta2_i = -29.76; %deg
        theta2_f = theta2_i + 65; %deg
        theta3_i = 90; %deg
        theta4_i = -31.68; %deg
        theta = [0, theta2_i, theta3_i, theta4_i, -90]; %deg

        om2 = 2*pi/3; %rad/s

        g = 9.81; 
        m2 = 0.00088; %kg
        m3 = 0.06; %kg
        m4 = .00085; %kg
   case {'chemical'} 
end

%% Main Loop
delta = 1; %deg
% Update Console
currentRuntime = 1;
totalRuntime = floor((floor(theta2_f) - ceil(theta2_i))/delta + 1) + 1;
t = [0:totalRuntime - 1]*delta*(pi/180)/om2;
disp("Solving " + currentRuntime + " of " + totalRuntime);
% Solve Angles
theta = solveAngles(r, theta*(pi/180));
error = theta(6:7);
theta = theta(1:5)*178/pi;
% Solve Velocities
oms = om2*[1, transpose(findFirstOrder(r, theta*pi/180))];
% Solve For Torque
R2 = r(2)*[cos(theta(2)*pi/180),sin(theta(2)*pi/180),0];
R4_c = 0.5*r(4)*[cos(theta(4)*pi/180),sin(theta(4)*pi/180),0];
R_p = R4_c + [r(1), -r(5),0];
T_in = 0.5*r(2)*m2*g + r(2)*m3*g + sqrt((R_p(1))^2+(R_p(2))^2);
%Update History
HistoryOutput = [theta(2:4), error, oms, T_in];
theta2 = ceil(theta2_i);
for currentRuntime = currentRuntime + 1:totalRuntime - 1
    theta2 = theta2 + delta;
    % Update Console
    clc;
    disp("Solving " + currentRuntime + " of " + totalRuntime);
    %Solve Angles
    theta = solveAngles(r, [theta(1), theta2, theta(3:5)]*(pi/180));
    error = theta(6:7);
    theta = theta(1:5)*180/pi;
    % Solve Velocities
    oms = om2*[1, transpose(findFirstOrder(r, theta*pi/180))];
    % Solve For Torque
    R2 = r(2)*[cos(theta(2)*pi/180),sin(theta(2)*pi/180),0];
    R4_c = 0.5*r(4)*[cos(theta(4)*pi/180),sin(theta(4)*pi/180),0];
    R_p = R4_c + [r(1), -r(5),0];
    T_in = 0.5*r(2)*m2*g + r(2)*m3*g + sqrt((R_p(1))^2+(R_p(2))^2);
    %Update History
    HistoryOutput = [HistoryOutput; theta(2:4), error, oms, T_in];
end
% Update Console
clc;
currentRuntime = currentRuntime + 1;
disp("Solving " + currentRuntime + " of " + totalRuntime);
% Solve Angles
theta = solveAngles(r, [theta(1), theta2_f, theta(3:5)]*(pi/180));
error = theta(6:7);
theta = theta(1:5)*180/pi;
% Solve Velocities
oms = om2*[1, transpose(findFirstOrder(r, theta*pi/180))];
% Solve For Torque
R2 = r(2)*[cos(theta(2)*pi/180),sin(theta(2)*pi/180),0];
R4_c = 0.5*r(4)*[cos(theta(4)*pi/180),sin(theta(4)*pi/180),0];
R_p = R4_c + [r(1), -r(5),0];
T_in = 0.5*r(2)*m2*g + r(2)*m3*g + sqrt((R_p(1))^2+(R_p(2))^2);
%Update History
HistoryOutput = [HistoryOutput; theta(2:4), error, oms, T_in];
disp("Done")

%% Results
figure(1)   
plot(t, HistoryOutput(:,1),t, HistoryOutput(:,2),t, HistoryOutput(:,3))
legend("\theta_2","\theta_3","\theta_4")
xlabel("Time [s]"); ylabel("Angle [deg]");
figure(2)
plot(t, ones(1,totalRuntime)*om2,t, HistoryOutput(:,7),t, HistoryOutput(:,8));
legend("\omega_2","\omega_3","\omega_4")
xlabel("Time [s]"); ylabel("Angular Velocity [rad/s]");
figure(3)
plot(t, HistoryOutput(:,9));
xlabel("Time [s]"); ylabel("T_{in} [N]");

%% Function Definitions
function val = f1(r, theta)
    val = r(2)*cos(theta(2)) - r(3)*cos(theta(3)) - r(4)*cos(theta(4)) - r(1);
end
function val = f2(r, theta)
    val = r(2)*sin(theta(2)) - r(3)*sin(theta(3)) - r(4)*sin(theta(4)) + r(5);
end
function theta = solveAngles(r, theta)
    % Settings
    N = 1;
    N_max = 1E3;
    Precision = 5E-5;

    % Iteration
    J = [r(3)*sin(theta(3)), r(4)*sin(theta(4));...
         -r(3)*cos(theta(3)), -r(4)*cos(theta(4))];
    theta(3:4) = -(J^(-1))*[f1(r, theta); f2(r, theta)]...
                    + [theta(3); theta(4)];
    error3 = abs(f1(r, theta));
    error4 = abs(f2(r, theta));
    while (N < N_max)&&((abs(error3) > Precision)||(abs(error4) > Precision))
        N = N + 1;
        J = [r(3)*sin(theta(3)), r(4)*sin(theta(4));...
             -r(3)*cos(theta(3)), -r(4)*cos(theta(4))];
        theta(3:4) = -(J^(-1))*[f1(r, theta); f2(r, theta)]...
                    + [theta(3); theta(4)];
        error3 = abs(f1(r, theta));
        error4 = abs(f2(r, theta));
        % Clean up Angle Values
        
        for i = 1:1:5    
            if (abs(theta(i)) > 2*pi)
                theta(i) = mod(theta(i), 2*pi);
            end
        end
        if N == N_max - 1
           disp("Uhh Ohh") 
        end
    end
    theta = [theta, error3, error4];
end
function coefficients = findFirstOrder(r, theta)
    A = [r(3)*sin(theta(3)), r(4)*sin(theta(4));...
         r(3)*cos(theta(3)), r(4)*cos(theta(4))];
    B = [r(2)*sin(theta(2)); r(2)*cos(theta(2))];
    coefficients = A^(-1)*B;
end
% function coefficients = findSecondOrder(r, theta)
%     c = findFirstOrder(r, theta);
%     A = [r(3)*sin(theta(3)), -r(4)*sin(theta(4));...
%          -r(3)*cos(theta(3)), r(4)*cos(theta(4))];
%     B = [-r(2)*cos(theta(2))+r(4)*c(2)^2*cos(theta(4))-r(3)*c(1)^2*cos(theta(3));... 
%         -r(2)*sin(theta(2))+r(4)*c(2)^2*sin(theta(4))-r(3)*c(1)^2*sin(theta(3))];
%     coefficients = A^(-1)*B;
% end
function vect = crossProduct(x, y)
    vect = [x(2)*y(3)-x(3)*y(2);...
            -x(1)*y(3)+x(3)*y(1);...
            x(1)*y(2)-x(2)*y(1)];
end