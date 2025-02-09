function q = trajectory(t, q0, qf, v_max, a_max, t0)
    % trajectory_14 calculates the position q(t) at time t for a 
    % time-optimal trajectory.
    %
    % Inputs:
    %   t     - The time at which to calculate the position
    %   q0    - Initial position
    %   qf    - Final position
    %   v_max - Maximum velocity
    %   a_max - Maximum acceleration
    %   t0    - Start time of the trajectory
    %
    % Output:
    %   q     - The position at time t
    
    % Calculate the total distance to travel
    s = qf - q0;

    % Time for acceleration and deceleration
    t_a = v_max / a_max;
    
    % Distance covered during acceleration and deceleration phases
    d_accel = 0.5 * a_max * t_a^2;
    
    % Check if the maximum velocity is reached
    if 2 * d_accel >= abs(s)
        % Triangular profile: only acceleration and deceleration
        t_a = sqrt(abs(s) / a_max);
        t_const = 0;
    else
        % Trapezoidal profile: acceleration, constant velocity, deceleration
        d_const = abs(s) - 2 * d_accel;
        t_const = d_const / v_max;
    end
    
    % Total time for the trajectory
    t_f = t0 + 2 * t_a + t_const;
    
    % Calculate q(t) based on the time intervals
    if t < t0
        % Before the start of the trajectory
        q = q0;
    elseif t <= t0 + t_a
        % Acceleration phase
        q = q0 + 0.5 * a_max * (t - t0)^2;
    elseif t <= t0 + t_a + t_const
        % Constant velocity phase
        q = q0 + d_accel + v_max * (t - t0 - t_a);
    elseif t <= t_f
        % Deceleration phase
        q = qf - 0.5 * a_max * (t_f - t)^2;
    else
        % After the end of the trajectory
        q = qf;
    end
endfunction q = trajectory(t, q0, qf, v_max, a_max, t0)
    % trajectory_14 calculates the position q(t) at time t for a 
    % time-optimal trajectory.
    %
    % Inputs:
    %   t     - The time at which to calculate the position
    %   q0    - Initial position
    %   qf    - Final position
    %   v_max - Maximum velocity
    %   a_max - Maximum acceleration
    %   t0    - Start time of the trajectory
    %
    % Output:
    %   q     - The position at time t
    
    % Calculate the total distance to travel
    s = qf - q0;

    % Time for acceleration and deceleration
    t_a = v_max / a_max;
    
    % Distance covered during acceleration and deceleration phases
    d_accel = 0.5 * a_max * t_a^2;
    
    % Check if the maximum velocity is reached
    if 2 * d_accel >= abs(s)
        % Triangular profile: only acceleration and deceleration
        t_a = sqrt(abs(s) / a_max);
        t_const = 0;
    else
        % Trapezoidal profile: acceleration, constant velocity, deceleration
        d_const = abs(s) - 2 * d_accel;
        t_const = d_const / v_max;
    end
    
    % Total time for the trajectory
    t_f = t0 + 2 * t_a + t_const;
    
    % Calculate q(t) based on the time intervals
    if t < t0
        % Before the start of the trajectory
        q = q0;
    elseif t <= t0 + t_a
        % Acceleration phase
        q = q0 + 0.5 * a_max * (t - t0)^2;
    elseif t <= t0 + t_a + t_const
        % Constant velocity phase
        q = q0 + d_accel + v_max * (t - t0 - t_a);
    elseif t <= t_f
        % Deceleration phase
        q = qf - 0.5 * a_max * (t_f - t)^2;
    else
        % After the end of the trajectory
        q = qf;
    end
end