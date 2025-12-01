function [Aeq, beq] = helperSetDesiredStateConstraints(initial_state, terminal_state, build, plannerparams)
    m = plannerparams.degree;
    N = plannerparams.num_piece;
    dt = plannerparams.PlanningResolution;
    % 제어점 개수
    num_vars_total = N * (m + 1) * 2;
    num_vars_per_piece = (m + 1) * 2;
    
    Aeq = zeros(10, num_vars_total); % 초기 6개 + 종단 4개 = 10개 제약
    beq = zeros(10, 1);
    
    % --- 초기 상태 제약 (Piece #1의 제어점들에 대해) ---
    p_s_indices = 1 : m+1;
    p_d_indices = m+2 : 2*(m+1);

    % --- 종단 상태 제약 (Piece #N의 제어점들에 대해) ---
    start_idx_last_piece = (N-1) * num_vars_per_piece;
    p_s_indices_last = start_idx_last_piece + (1 : m+1);
    p_d_indices_last = start_idx_last_piece + (m+2 : 2*(m+1));
    
    if build
        % [s0, d0, v_s0, v_d0, a_s0, a_d0]
        Aeq(1, p_s_indices(1)) = dt;
        Aeq(2, p_d_indices(1)) = dt;
        Aeq(3, p_s_indices(1:2)) = [-m, m];
        Aeq(4, p_d_indices(1:2)) = [-m, m];
        Aeq(5, p_s_indices(1:3)) = [m*(m-1)/dt, -2*m*(m-1)/dt, m*(m-1)/dt];
        Aeq(6, p_d_indices(1:3)) = [m*(m-1)/dt, -2*m*(m-1)/dt, m*(m-1)/dt];
        
        % [a_sT, d_T, v_dT, a_dT]
        Aeq(7, p_s_indices_last(end-2:end)) = [m*(m-1), -2*m*(m-1), m*(m-1)]/dt;
        Aeq(8,  p_d_indices_last(end)) = dt;
        Aeq(9, p_d_indices_last(end-1:end)) = [-m, m];
        Aeq(10, p_d_indices_last(end-2:end)) = [m*(m-1), -2*m*(m-1), m*(m-1)]/dt;
    else
        beq(1) = initial_state.s;
        beq(2) = initial_state.d;
        beq(3) = initial_state.vs;
        beq(4) = initial_state.vd;
        beq(5) = initial_state.as;
        beq(6) = initial_state.ad;
        beq(7) = terminal_state.as;
        beq(8) = terminal_state.d;
        beq(9) = terminal_state.vd;
        beq(10) = terminal_state.ad;
    end
end