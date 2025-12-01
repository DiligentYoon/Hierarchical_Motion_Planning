function [Aeq_cont, beq_cont] = helperSetContinuityConstraints(build, plannerparams)
    m = plannerparams.degree;
    N = plannerparams.num_piece;
    dt = plannerparams.PlanningResolution;
    % 제어점 개수 (한 Piece 당 degree x 2(종 & 횡)의 개수)
    num_vars_total = N * (m + 1) * 2;
    num_vars_per_piece = (m + 1) * 2;
    % 이음새 개수
    num_seams = N - 1;
    if num_seams < 1
        Aeq_cont = [];
        beq_cont = [];
        return;
    end
    
    num_constraints_per_seam = 8; % s, d에 대한 C0, C1, C2, C3
    Aeq_cont = zeros(num_seams * num_constraints_per_seam, num_vars_total);
    beq_cont = zeros(num_seams * num_constraints_per_seam, 1); % 항상 0

    if build
        % 각 이음새에 대해 제약 조건 생성
        for i = 1:num_seams
            % --- 인덱스 계산 ---
            % i번째 조각과 i+1번째 조각의 제어점에 해당하는 인덱스 계산
            vars_i = (i-1)*num_vars_per_piece+1 : i*num_vars_per_piece;
            vars_i_plus_1 = i*num_vars_per_piece+1 : (i+1)*num_vars_per_piece;
            
            p_s_i_indices = vars_i(1 : m+1);
            p_d_i_indices = vars_i(m+2 : end);
            p_s_i_plus_1_indices = vars_i_plus_1(1 : m+1);
            p_d_i_plus_1_indices = vars_i_plus_1(m+2 : end);
            
            row_offset = (i-1) * num_constraints_per_seam;
            
            % --- C0 (위치) 연속성 ---
            % s_end(i) - s_start(i+1) = 0
            Aeq_cont(row_offset + 1, p_s_i_indices(end)) = dt;
            Aeq_cont(row_offset + 1, p_s_i_plus_1_indices(1)) = -dt;
            % d_end(i) - d_start(i+1) = 0
            Aeq_cont(row_offset + 2, p_d_i_indices(end)) = dt;
            Aeq_cont(row_offset + 2, p_d_i_plus_1_indices(1)) = -dt;
            
            % --- C1 (속도) 연속성 ---
            % v_s_end(i) - v_s_start(i+1) = 0
            Aeq_cont(row_offset + 3, p_s_i_indices(end-1:end)) = [-m, m];
            Aeq_cont(row_offset + 3, p_s_i_plus_1_indices(1:2)) = -[-m, m];
            % v_d_end(i) - v_d_start(i+1) = 0
            Aeq_cont(row_offset + 4, p_d_i_indices(end-1:end)) = [-m, m];
            Aeq_cont(row_offset + 4, p_d_i_plus_1_indices(1:2)) = -[-m, m];

            % --- C2 (가속도) 연속성 ---
            acc_coeffs = [m*(m-1), -2*m*(m-1), m*(m-1)] / dt;
            acc_coeffs_d = [m*(m-1), -2*m*(m-1), m*(m-1)] / dt;
            % a_s_end(i) - a_s_start(i+1) = 0
            Aeq_cont(row_offset + 5, p_s_i_indices(end-2:end)) = acc_coeffs;
            Aeq_cont(row_offset + 5, p_s_i_plus_1_indices(1:3)) = -acc_coeffs;
            % a_d_end(i) - a_d_start(i+1) = 0
            Aeq_cont(row_offset + 6, p_d_i_indices(end-2:end)) = acc_coeffs_d;
            Aeq_cont(row_offset + 6, p_d_i_plus_1_indices(1:3)) = -acc_coeffs_d;

            % --- C3 (저크) 연속성 ---
            % end of piece i: use last 4 ctrl pts, start of piece i+1: first 4
            jerk_coeffs = [m*(m-1)*(m-2), -3*m*(m-1)*(m-2),  3*m*(m-1)*(m-2), -m*(m-1)*(m-2)] / (dt^2);
            jerk_coeffs_d = [m*(m-1)*(m-2), -3*m*(m-1)*(m-2),  3*m*(m-1)*(m-2), -m*(m-1)*(m-2)] / (dt^2);
            Aeq_cont(row_offset + 7, p_s_i_indices(end-3:end))   = jerk_coeffs;
            Aeq_cont(row_offset + 7, p_s_i_plus_1_indices(1:4))       = -jerk_coeffs;
            Aeq_cont(row_offset + 8, p_d_i_indices(end-3:end))   = jerk_coeffs_d;
            Aeq_cont(row_offset + 8, p_d_i_plus_1_indices(1:4))       = -jerk_coeffs_d;
        end
    end
end