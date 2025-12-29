function [A_safe, b_safe] = helperSetSafetyConstraints(gap_sequence, build, plannerparams)
    m = plannerparams.degree;
    N = plannerparams.num_piece;
    dt = plannerparams.PlanningResolution;
    MaxFront = plannerparams.MaxFront;
    ep = 1e-7;
    % 제어점 개수
    num_vars_total = N * (m + 1) * 2;

    % 각 제어점당 4개의 제약 조건 (s_max, -s_min, d_max, -d_min)
    num_constraints = N * (m + 1) * 4;
    A_safe = zeros(num_constraints, num_vars_total);
    b_safe = zeros(num_constraints, 1);

    row_idx = 0;
    vars_per_piece = 2*(m+1);
    dtI  = dt * eye(m+1);

    if build
        % 각 궤적 조각 (piece)에 대해 반복
        for i = 1:N 
            % i번째 조각의 s, d 제어점들의 인덱스 계산
            base = (i-1)*vars_per_piece;
            s_idx = base + (1 : (m+1));
            d_idx = base + (m+2 : 2*(m+1));

            % s방향 제약조건 : 최대
            A_safe(row_idx+(1 : (m+1)), s_idx) = dtI;
            row_idx = row_idx + (m+1);
            % s방향 제약조건 : 최소
            A_safe(row_idx+(1 : (m+1)), s_idx) = -dtI;
            row_idx = row_idx + (m+1);

            % d방향 제약조건 : 최대
            A_safe(row_idx+(1 : (m+1)), d_idx) = dtI;
            row_idx = row_idx + (m+1);
            % d방향 제약조건 : 최소
            A_safe(row_idx+(1 : (m+1)), d_idx) = -dtI;
            row_idx = row_idx + (m+1);
        end
    else
        % 각 궤적 조각 (piece)에 대해 반복
        for i = 1:N
            if i == N || i == 1
                current_gap = gap_sequence(i);
                s_max_s = (current_gap.s + current_gap.l/2);
                s_min_s = max(0, (current_gap.s - current_gap.l/2));
                d_max_s = (current_gap.d + current_gap.w/2);
                d_min_s = (current_gap.d - current_gap.w/2);

                d_max = d_max_s .* ones(m+1, 1);
                d_min = d_min_s .* ones(m+1, 1);
                s_max = s_max_s .* ones(m+1, 1);
                s_min = s_min_s .* ones(m+1, 1);
            else
                current_gap = gap_sequence(i);
                next_gap = gap_sequence(i+1);

                s_max_t1 = current_gap.s + current_gap.l/2;
                s_min_t1 = max(0, current_gap.s - current_gap.l/2);

                d_max_t1 = (current_gap.d + current_gap.w/2);
                d_min_t1 = (current_gap.d - current_gap.w/2);
                d_max_t2 = (next_gap.d + next_gap.w/2);
                d_min_t2 = (next_gap.d - next_gap.w/2);

                u = linspace(0, 1, m+1);
                s_min = s_min_t1 .* ones(m+1, 1);
                s_max = s_max_t1 .* ones(m+1, 1);
                d_min = (1-u) .* d_min_t1 + u .* d_min_t2;
                d_max = (1-u) .* d_max_t1 + u .* d_max_t2;
            end
            % s방향 제약조건 : 최대
            b_safe(row_idx+(1:m+1)) = (s_max+ep);
            row_idx = row_idx + (m+1);
            % s방향 제약조건 : 최소
            b_safe(row_idx+(1:m+1)) = -(s_min-ep);
            row_idx = row_idx + (m+1);

            % d방향 제약조건 : 최대
            b_safe(row_idx+(1:m+1)) = d_max;
            row_idx = row_idx + (m+1);
            % d방향 제약조건 : 최소
            b_safe(row_idx+(1:m+1)) = -d_min;
            row_idx = row_idx + (m+1);
        end
    end
end