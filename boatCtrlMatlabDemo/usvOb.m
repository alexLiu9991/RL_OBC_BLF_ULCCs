function [sys, x0, str, ts, simStateCompliance] = usvOb(t, x, u, flag)
    %
    % The following outlines the general structure of an S-function.
    %
    
    switch flag,
    
      %%%%%%%%%%%%%%%%%%
      % Initialization %
      %%%%%%%%%%%%%%%%%%
      case 0,
        clear all;
        [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;
    
      %%%%%%%%%%%%%%%
      % Derivatives %
      %%%%%%%%%%%%%%%
      case 1,
        sys=mdlDerivatives(t,x,u);
    
      %%%%%%%%%%
      % Update %
      %%%%%%%%%%
      case 2,
        sys=[];
    
      %%%%%%%%%%%
      % Outputs %
      %%%%%%%%%%%
      case 3,
        sys=mdlOutputs(t,x,u);
    
      %%%%%%%%%%%%%%%%%%%%%%%
      % GetTimeOfNextVarHit %
      %%%%%%%%%%%%%%%%%%%%%%%
      case 4,
        sys=[];
    
      %%%%%%%%%%%%%
      % Terminate %
      %%%%%%%%%%%%%
      case 9,
        sys=[];
    
      %%%%%%%%%%%%%%%%%%%%
      % Unexpected flags %
      %%%%%%%%%%%%%%%%%%%%
      otherwise
        DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));
    
    end
    
    % end sfuntmpl
    
    %
    %=============================================================================
    % mdlInitializeSizes
    % Return the sizes, initial conditions, and sample times for the S-function.
    %=============================================================================
    %
    function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes
    
    %
    % call simsizes for a sizes structure, fill it in and convert it to a
    % sizes array.
    %
    % Note that in this example, the values are hard coded.  This is not a
    % recommended practice as the characteristics of the block are typically
    % defined by the S-function parameters.
    %
    sizes = simsizes;
    
    sizes.NumContStates  = 206;
    sizes.NumDiscStates  = 0;
    sizes.NumOutputs     = 18;
    sizes.NumInputs      = 0;
    sizes.DirFeedthrough = 1;
    sizes.NumSampleTimes = 1;   % at least one sample time is needed
    
    sys = simsizes(sizes);
    
    %
    % initialize the initial conditions
    %
    x0 = [2, 2, 1.57, 0, 0, 0, ...  % 系统状态
         0.1, 0.1 , 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, ... % Wc1
         0.1, 0.1 , 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, ...
         0.2, 0.2 , 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, ... % Wa1
         0.2, 0.2 , 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, ...
         0.3, 0.3 , 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, ... % Wf
         0.3, 0.3 , 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3, ...
         0.1, 0.1 , 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, ... % Wc2
         0.1, 0.1 , 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, ...
         0.2, 0.2 , 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, ... % Wa2
         0.2, 0.2 , 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2];
    
    
    %
    % str is always an empty matrix
    %
    str = [];
    
    %
    % initialize the array of sample times
    %
    ts  = [0 0];
    
    % Specify the block simStateCompliance. The allowed values are:
    %    'UnknownSimState', < The default setting; warn and assume DefaultSimState
    %    'DefaultSimState', < Same sim state as a built-in block
    %    'HasNoSimState',   < No sim state
    %    'DisallowSimState' < Error out when saving or restoring the model sim state
    simStateCompliance = 'UnknownSimState';
    
    % end mdlInitializeSizes
    
    %
    %=============================================================================
    % mdlDerivatives
    % Return the derivatives for the continuous states.
    %=============================================================================
    %
    function sys=mdlDerivatives(t, x, u)
    
    % 跟踪目标
    global fai_d;
    
    % 控制参数
    global nd_j1 nd_j2;
    global mu k1 k2;
    
    global start_point;

    %% 子系统控制器设计
    Phi = x(3);
    
    z1 = Phi - fai_d;
    S_j1 = zeros(nd_j1, 1);  
    for i = 1:1:nd_j1
        S_j1(i) = exp(-(z1-10 + (1/2)*i)^2/mu^2);
    end 
    S_j1_2 = S_j1*S_j1';
    W_c1 = x(start_point+1:start_point+nd_j1);
    W_a1 = x(start_point+nd_j1+1:start_point+nd_j1*2);
    
    global pb1; 
    alpha = -k1*z1 - (3/4)*(z1/(pb1^2 - z1^2)) - 0.5*W_a1'*S_j1;
    
    Vr = x(6);
    z2 = Vr - alpha;
    S_f = zeros(nd_j2, 1);  
    S_j2 = zeros(nd_j2, 1);
    for i=1:1:nd_j2
        S_f(i) = exp(-(z2-10 + (1/2)*i)^2/mu^2);
        S_j2(i) = exp(-norm([x(2),z2]'-[10, 10]'+(1/2)*[i,i]')^2/mu^2);
    end
    S_j2_2 = S_j2*S_j2';
    W_f = x(start_point+nd_j1*2+1 : start_point+nd_j1*2+nd_j2);
    W_c2 = x(start_point+nd_j1*2+nd_j2+1 : start_point+nd_j1*2+nd_j2*2);
    W_a2 = x(start_point+nd_j1*2+nd_j2*2+1 : start_point+nd_j1*2+nd_j2*3);
    
    global pb2; 
    b = 1/2.76;  % 1/m(3,3)
    tao_r = b (-k2*z2 - (5/4)*(z2/(pb2^2 - z2^2)) - W_f'*S_f - 0.5*W_a2'*S_j2);

    tao_r = max(min(tao_r, 50), -50);
    
    % 更新率
    scal=2;
    
    r_c1 = 4*scal;
    r_a1 = 6*scal;
    for i=1:1:nd_j1
        sys(start_point+i) = -r_c1*S_j1_2(i,:)*W_c1;  
    end
    for i=1:1:nd_j1
        sys(start_point+nd_j1+i) = -S_j1_2(i,:)*(r_a1*(W_a1-W_c1)+r_c1*W_c1); 
    end
    
    r_f = 2*scal;
    r_c2 = 4*scal;
    r_a2 = 6*scal;
    G_f =  1.8 * eye(nd_j2);
    for i=1:1:nd_j2
        if debugMode == 0
            sys(start_point+nd_j1*2+i) = G_f(i,:)*(S_f*z2 - r_f*W_f);
        else
            sys(start_point+nd_j1*2+i) = G_f(i,:)*(S_f*z2/(pb2^2 - z2^2) - r_f*W_f);
        end
    end
    for i=1:1:nd_j2
        sys(start_point+nd_j1*2+nd_j2+i) = -r_c2*S_j2_2(i,:)*W_c2;
    end
    for i=1:1:nd_j2
        sys(start_point+nd_j1*2+nd_j2*2+i) = -S_j2_2(i,:)*(r_a2*(W_a2-W_c2)+r_c2*W_c2);
    end
    % end mdlDerivatives
    
    %
    %=============================================================================
    % mdlOutputs
    % Return the block outputs.
    %=============================================================================
    %
    function sys=mdlOutputs(t,x,u)
    
    global Vu;
    Vu = 0.16;
    L_pp  = 0.2;
    Delta = 2 * L_pp;
    R     = L_pp;
    WP = [2  2  6  6 10 10 14 14 18 18;
          2 18  18 2  2 18 18 2  2  18];
    global counter;
    global wpIndex;
    global p_k;
    global p_kp1;
    global touchFlag;
    p = [x(1), x(2)]';
    if isempty(counter)                    
        counter = 1;
        wpIndex = 2;
        p_k = p;
        p_kp1 = WP(:, wpIndex);
    elseif counter == 2
        p_k = p;
        p_kp1 = WP(:, wpIndex);
    elseif norm(p - p_kp1) < R
        if wpIndex == size(WP, 2)
            ssSetErrorStatus(gcb, "Simulation complete");
            return;
        end
        if touchFlag == 0
            touchFlag = 1;
            wpIndex = wpIndex + 1;
        end
        p_k = WP(:, wpIndex-1);
        p_kp1 = WP(:, wpIndex);
    elseif norm(p - p_kp1) > R 
        touchFlag = 0;
    end
    
    alpha_k = atan2(p_kp1(2) - p_k(2), p_kp1(1) - p_k(1));
    
    R_mat = [cos(alpha_k), -sin(alpha_k); 
             sin(alpha_k), cos(alpha_k)];
    epsilon = R_mat' * (p - p_k); 
    e = epsilon(2); 
    
    fai_p = alpha_k;
    fai_r = atan2(-e, Delta);
    
    global fai_d;
    fai_d = fai_p + fai_r;
    
    global nd_j1 nd_j2;
    nd_j1 = 40;
    nd_j2 = 40;
    
    global mu k1 k2;
    mu = 3;
    if debugMode == 0
        k1 = 10;
        k2 = 10;
    else 
        k1 = 10;
        k2 = 10;
    end
    
    global start_point;
    start_point = 6;
    %% 子系统控制器设计
    Phi = x(3);
    z1 = Phi - fai_d;
    S_j1 = zeros(nd_j1, 1);  
    for i = 1:1:nd_j1
        S_j1(i)=exp(-(z1-10 + (1/2)*i)^2/mu^2);
    end 
    W_c1 = x(start_point+1:start_point+nd_j1);
    W_a1 = x(start_point+nd_j1+1:start_point+nd_j1*2);
    
    global pb1; 
    pb1 = 0.4;
    alpha = -k1*z1 - (3/4)*(z1/(pb1^2 - z1^2)) - 0.5*W_a1'*S_j1;
    
    Vr = x(6);
    z2 = Vr - alpha;
    S_f = zeros(nd_j2, 1);  
    S_j2 = zeros(nd_j2, 1);
    for i=1:1:nd_j2
        S_f(i) = exp(-(z2-10 + (1/2)*i)^2/mu^2);
        S_j2(i) = exp(-norm([x(2),z2]'-[10,10]'+(1/2)*[i, i]')^2/mu^2);
    end
    W_f = x(start_point+nd_j1*2+1 : start_point+nd_j1*2+nd_j2);
    W_c2 = x(start_point+nd_j1*2+nd_j2+1 : start_point+nd_j1*2+nd_j2*2);
    W_a2 = x(start_point+nd_j1*2+nd_j2*2+1 : start_point+nd_j1*2+nd_j2*3);
    
    global pb2; 
    pb2 = 0.4;
    b = 1/2.76;
    tao_r =  b (-k2*z2 - (5/4)*(z2/(pb2^2 - z2^2)) - W_f'*S_f - 0.5*W_a2'*S_j2);

    tao_r = max(min(tao_r, 25), -25);
    h1 = z1^2 + alpha^2;
    h2 = z2^2 + tao_r^2;
    
    sys(1) = Phi;
    sys(2) = fai_d;
    sys(3) = Vr;
    sys(4) = alpha;
    
    sys(5) = z1;
    sys(6) = z2;
    
    sys(7) = norm(W_c1);
    sys(8) = norm(W_a1);
    
    sys(9) = norm(W_f);
    sys(10) = norm(W_c2);
    sys(11) = norm(W_a2);
    
    sys(12) = h1;
    sys(13) = h2;
    
    sys(14) = tao_r;
    
    sys(15) = p(1);
    sys(16) = p(2);
    sys(17) = p_kp1(1);
    sys(18) = p_kp1(2);
    
    % end mdlOutputs