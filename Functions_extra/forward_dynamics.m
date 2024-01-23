function [sys,x0,str,ts] = c(t,x,u,flag,robot, qi, dqi)
switch flag,

  %%%%%%%%%%%%%%%%%%
  % Initialization %
  %%%%%%%%%%%%%%%%%%
  case 0,
    [sys,x0,str,ts]=mdlInitializeSizes(3, qi, dqi); %initialize starting points and the dof

  %%%%%%%%%%%%%%%
  % Derivatives %
  %%%%%%%%%%%%%%%
  case 1,
    sys=mdlDerivatives(t,x,u,robot); %SEE DEFINITION BELOW

  %%%%%%%%%%%
  % Outputs %S
  %%%%%%%%%%%
  case 3,
    sys=mdlOutputs(t,x,u); %u is tau in output

  %%%%%%%%%%%%%%%%%%%
  % Unhandled flags %
  %%%%%%%%%%%%%%%%%%%
  case { 2, 4, 9 },
    sys = [];           %boh

      %%%%%%%%%%%%%%%%%%%%
  % Unexpected flags %
  %%%%%%%%%%%%%%%%%%%%
  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

% end csfunc
%
%=============================================================================
% mdlInitializeSizes
% Return the sizes, initial conditions, and sample times for the S-function.
%=============================================================================
%
function [sys,x0,str,ts]=mdlInitializeSizes(n, qi, dqi)

sizes = simsizes;
sizes.NumContStates  = 2*n;%q,qdot for each joint
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2*n; 
sizes.NumInputs      = n;%n dof
sizes.DirFeedthrough = 0;%no direct link to input and output 
sizes.NumSampleTimes = 1; %should be 1 to avoid errors

sys = simsizes(sizes);
x0  = [qi; dqi];  %starting positions and velocities
%x0  = [qi + [0.5,0.1,-0.2]'; dqi];
str = [];
ts  = [0 0];

% end mdlInitializeSizes
%
%=============================================================================
% mdlDerivatives
% Return the derivatives for the continuous states.
%=============================================================================
%
function sys=mdlDerivatives(t,x,u,robot)
    N = 3;
    q = x(1:N);
    dq = x(N+1:2*N);
    ddq = forw_dyn_val(q, dq, u); %extra function to get the acceleration to give in output the acc, inp = pos vel tau
    sys = [dq; ddq]; %sys vars
    

% end mdlDerivatives
%
%=============================================================================
% mdlOutputs
% Return the block outputs.
%=============================================================================
%
function sys=mdlOutputs(t,x,u)

sys = x;

% end mdlOutputs
