function forwardKF(app)
app.KF.globalT0 = app.KF.globalT1;
app.KF.globalT1 = toc(app.tStart)-app.KF.time_0;
app.KF.delt = app.KF.globalT1-app.KF.globalT0;

app.grfVec = app.vec_treadmillForce/app.mass;
app.KF.leftCop = app.xy_copLeft(1);
app.KF.rightCop = app.xy_copRight(1);
app.KF.fyl = -app.grfVec(1); 
app.KF.fyr = -app.grfVec(6+1);
app.KF.fzl = -app.grfVec(3);
app.KF.fzr = -app.grfVec(6+3);

%             if app.KF.fzl+app.KF.fzr>app.thresholdFz %flag may change
    app.existFlag = 1;
%             else
%                 app.existFlag = 0;
%             end

if app.existFlag 
   % predict state
   u = [app.KF.fyl+app.KF.fyr];
               
   app.KF.x = app.KF.A*app.KF.x + app.KF.B*u;
   app.KF.P = app.KF.A*app.KF.P*app.KF.A' + app.KF.Q;
    
   % update mean values of com, dcom, v_tm
   app.KF.com_step = (app.KF.com_step*app.KF.n_ts + app.KF.x(1))/(app.KF.n_ts + 1);
   app.KF.dcom_step = (app.KF.dcom_step*app.KF.n_ts + app.KF.x(2))/(app.KF.n_ts + 1);
   app.KF.v_tm_mean = (app.KF.v_tm_mean*app.KF.n_ts + app.vTm)/(app.KF.n_ts + 1);
   app.KF.n_ts = app.KF.n_ts + 1;
    
   % detect new step
   app.KF.flag_step = 0;
   if app.KF.stance_L_0 == 0 && app.KF.fzl > app.thresholdFz % left step?
       app.KF.flag_step = 1;
       app.KF.p_step_0 = app.KF.p_step_1;
       app.KF.p_step_1 = app.KF.leftCop;
       app.KF.leg_step_1 = 0; % 0: L
   end
   if app.KF.stance_R_0 == 0 && app.KF.fzr > app.thresholdFz % right step?
       app.KF.flag_step = 1;
       app.KF.p_step_0 = app.KF.p_step_1;
       app.KF.p_step_1 = app.KF.rightCop;
       app.KF.leg_step_1 = 1; % 1: R
   end
    
   % if new step made
   if app.KF.flag_step
       app.KF.t_step_0 = app.KF.t_step_1;
       time = toc(app.tStart) - app.KF.time_0;
       app.KF.t_step_1 = time;
       app.KF.len_step = app.KF.p_step_1 - app.KF.p_step_0;
       app.KF.leg_step_0 = app.KF.leg_step_1;
       app.KF.n_step = app.KF.n_step + 1;
       app.KF.n_ts = 0;
   end
                
   % update stance data
   app.KF.stance_L_0 = app.KF.fzl > app.thresholdFz;
   app.KF.stance_R_0 = app.KF.fzr > app.thresholdFz;
   app.KF.p_step_0 = app.KF.p_step_0 - app.vTm*app.KF.delt;
   app.KF.p_step_1 = app.KF.p_step_1 - app.vTm*app.KF.delt;
    
   if app.KF.flag_step && ((app.KF.t_step_1 - app.KF.t_step_0) < 1.2)
       flag_step_ok = 1;
   else
       flag_step_ok = 0;
   end
    
   if flag_step_ok% measure output (y)
       com_mes = (app.KF.p_step_1 + app.KF.p_step_0)/2;
       dcom_mes = app.KF.len_step/(app.KF.t_step_1 - app.KF.t_step_0) - app.KF.v_tm_mean;
       y_mes = [com_mes; dcom_mes];
                    
       % update state(C = [1 0; 0 1]; D = [0; 0])
       noise_v = [.025 0; 0 .085]; % from experiment (mocap as reference data)
       R = noise_v*noise_v';
          
       y_est = [app.KF.com_step; app.KF.dcom_step]; % y_est = C*[x(1); dcom_step] + D*u; % estimate output
       K = app.KF.P/(app.KF.P + R); % K = P*C'/(C*P*C' + R);
                   
       app.KF.x = app.KF.x + K*(y_mes - y_est);
       app.KF.P = (eye(2) - K)*app.KF.P; % P = (eye(2) - K*C)*P;
       app.com(2) = app.KF.x(1);
       app.dcom(1) = app.KF.x(2);
   else
       y_mes = [-100, -100];
   end
 end
end