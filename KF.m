%   _  __     _                               _____ _ _ _
%  | |/ /__ _| |_ __ ___   __ _ _ __         |  ___(_) | |_  ___  _ __
%  | ' // _` | | '_ ` _ \ / _` | '_ \ _____  | |_  | | | __|/ _ \| '__|
%  | . \ (_| | | | | | | | (_| | | | |_____| |  _| | | | |_|  __/| |
%  |_|\_\__,_|_|_| |_| |_|\__,_|_| |_|       |_|   |_|_|\__|\___||_|

%% Title: Kalman filter apparoch for active noise control system to attenuate the dynamic noise.
% Author: DONGYAN SHI(DSHI003@ntu.edu.sg)
% Date  : 2019-9-1
%% Introduction
% This code implements the Kalman filter method for a single-channel active noise control (ANC) application.
% Furthermore, the FxLMS algorithm is conducted as a comparative analysis. The Kalman filter technique employs 
% the modified feed-forward active noise control (ANC) structure, whereas the FxLMS algorithm uses 
% the conventional feed-forward ANC structure.

%% Clear Memory
close all ;
clear     ;
clc       ;

%% Load the coefficient of filter 
load('PriPath_3200.mat');
load('SecPath_200_6000.mat') ;
figure ;
subplot(2,1,1)
plot(PriPath);
title('Primary Path');
grid on      ;
subplot(2,1,2);
plot(SecPath);
title('Secondary Path');
xlabel('Taps');
grid on      ;

%% System configuration 
fs = 16000    ; % sampling rate 16 kHz.
T  = 0.25      ; % Simulation duration (seconds).
t  = 0:1/fs:T ;% Time variable.
N  = length(t);
fw = 500     ;
fe = 300     ;
y = chirp(t,20,T,1600);
figure   ;
plot(t,y);
title('Reference signal x(n)');
xlabel('Time (seconds)') ;
ylabel('Magnitude')      ;
axis([-inf inf -1.05 1.05]);
grid on ;

%% Reference signal
%X  = 0.4*sin(2*pi*fw*t)+0.3*sin(2*pi*fe*t);
X = y;
%plot(X(end-100:end))
D  = filter(PriPath,1,X);
Rf = filter(SecPath,1,X);
q  = 0.00001;
Noise = randn(N,1)*sqrt(q);
D  = D +Noise';
%plot(D(end-100:end))

%% Construct FxLMS controller
L   = 80    ;
muW = 0.001;
noiseController = dsp.FilteredXLMSFilter('Length',L,'StepSize',muW, ...
    'SecondaryPathCoefficients',SecPath);
[y,e] = noiseController(X,D);
figure;
plot(t,e) ;
title('FxLMS algorithm')   ;
ylabel('Error signal e(n)');
xlabel('Time (seconds)')   ;
grid on ;

%% KF filter 
% q  = 0.00005;
P  = eye(L);
W  = zeros(L,1);
Xd = zeros(L,1);
ek = zeros(N,1);
w5 = zeros(N,1);
w60 = zeros(N,1);

for ii =1:N
    Xd     =[Rf(ii);Xd(1:end-1)];
    yt     = Xd'*W ;
    ek(ii) = D(ii)-yt ;
    K      = P*Xd/(Xd'*P*Xd + q);
    W      = W +K*ek(ii)        ;
    P      =(eye(L)-K*Xd')*P    ;
    %---------------------------
    w5(ii)  = W(5);
    w60(ii) = W(60);
    %---------------------------
end
figure;
plot(t,ek);
title('Kalman algorithm')   ;
ylabel('Error signal e(n)');
xlabel('Time (seconds)')
grid on ;
figure 
plot(t,w5,t,w60);
title('Control Filter Weights');
xlabel('Time (seconds)');
legend('w_5','w_{60}');
grid on ;
figure;
plot(t,e,t,ek);
title('FxLMS vs Kalman')   ;
ylabel('Error signal e(n)');
xlabel('Time (seconds)')   ;
legend('FxLMS algorithm','KF algorithm');
grid on ;
