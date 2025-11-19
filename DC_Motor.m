function DC_Motor()

% Parameter
  Ra = 0.7;           % Resistance (Ohm)
  La = 1.3e-3;        % Inductance (H)
  Kt = 0.14;          % Torque Constant (Nm/A)
  Kb = 0.14;          % Back EMF Constant (V/rad/s)
  Jw = 2.9e-7;        % Wheel Inertia (kg m^2)
  zeta_w = 0.01;      % Viscous Friction (Nm/rad/s)
  tspan = [0 10];     % Dari 0 sampai 10 detik

% Struct untuk parameter
  params.Ra = Ra; params.La = La; params.Kt = Kt;
  params.Kb = Kb; params.Jw = Jw; params.zeta_w = zeta_w;

% Differential
    % Case 1: Input +5 Volt
    params.amp = 5;
    [t_pos, y_pos] = ode45(@(t,y) motor_ode(t, y, params), tspan, [0; 0]);
    
    % Case 2: Input -5 Volt
    params.amp = -5;
    [t_neg, y_neg] = ode45(@(t,y) motor_ode(t, y, params), tspan, [0; 0]);

% Hitung Torsi dan Back EMF 
  I_pos = y_pos(:,1); 
  w_pos = y_pos(:,2);
  Torque_pos = Kt * I_pos;
  Bemf_pos = Kb * w_pos;

  I_neg = y_neg(:,1);
  w_neg = y_neg(:,2);
  Torque_neg = Kt * I_neg;
  Bemf_neg = Kb * w_neg;

 % Plot
   figure('Name', 'Simulasi Motor DC');
    
   % Plot Respon Torsi
   subplot(3,1,1);
   plot(t_pos, Torque_pos, 'b', t_neg, Torque_neg, 'r--');
   title('Torsi (Torque)'); ylabel('Nm'); grid on;
   legend('+5V Input', '-5V Input');

   % Plot Respon Kecepatan Sudut
   subplot(3,1,2);
   plot(t_pos, w_pos, 'b', t_neg, w_neg, 'r--');
   title('Kecepatan Sudut (Angular Velocity)'); ylabel('rad/s'); grid on;

   % Plot Respon Back EMF
   subplot(3,1,3);
   plot(t_pos, Bemf_pos, 'b', t_neg, Bemf_neg, 'r--');
   title('Back EMF'); ylabel('Volt'); xlabel('Time (s)'); grid on;
end

%% Persamaan Matematis
function dydt = motor_ode(t, y, p)
         I = y(1);      % y(1) = Arus (I)
         omega = y(2);  % y(2) = Kecepatan Sudut (omega)
    
    % Tentukan Input Tegangan (Pulse 5 detik)
    if t >= 1 && t < 6
        V_in = p.amp;
    else
        V_in = 0;
    end
    
    % Persamaan 2 (Berdasarkan Referensi pada Jurnal)
    dIdt = (V_in - p.Ra * I - p.Kb * omega) / p.La;
    
    % Persamaan 3 (Berdasarkan Referensi pada Jurnal)
    dwdt = (p.Kt * I - p.zeta_w * omega) / p.Jw;
    
    dydt = [dIdt; dwdt];
end