function [A_feasible, b_feasible] = helperSetFeasibleConstraints(e1, e2, bounds, build, plannerparams)
    m = plannerparams.degree;
    N = plannerparams.num_piece;
    dt = plannerparams.PlanningResolution;

    % 전체 제어점 개수
    num_vars_total = N * (m + 1) * 2;
    % 8개의 제약 조건 (v_s_min, v_s_max, v_d_min, v_d_max, a_s_min, a_s_max, a_d_min, a_d_max)
    num_constraints = N * 2 * 2 * (m + m-1);
    A_feasible = zeros(num_constraints, num_vars_total);
    b_feasible = zeros(num_constraints, 1);

    D1 = m * e1;
    D2 = (m*(m-1) / dt) * e2;

    row_idx = 0;
    vars_per_piece = 2*(m+1);

    if build
        for i = 1:N
            % 각 제어점마다 제약조건을 AX <= B 꼴로 할당
            base = (i-1)*vars_per_piece;
            s_idx = base + (1 : (m+1));
            d_idx = base + (m+2 : 2*(m+1));

            % s방향 속도 제약조건 : 최대
            A_feasible(row_idx+(1:m), s_idx) = D1;
            row_idx = row_idx + m;
            % s방향 속도 제약조건 : 최소
            A_feasible(row_idx+(1:m), s_idx) = -D1;
            row_idx = row_idx + m;

            % s방향 가속도 제약조건 : 최대
            A_feasible(row_idx+(1:m-1), s_idx) = D2;
            row_idx = row_idx + (m-1);
            % s방향 가속도 제약조건 : 최소
            A_feasible(row_idx+(1:m-1), s_idx) = -D2;
            row_idx = row_idx + (m-1);

            % d방향 속도 제약조건 : 최대
            A_feasible(row_idx+(1:m), d_idx) = D1;
            row_idx = row_idx + m;
            % d방향 속도 제약조건 : 최소
            A_feasible(row_idx+(1:m), d_idx) = -D1;
            row_idx = row_idx + m;

            % d방향 가속도 제약조건 : 최대
            A_feasible(row_idx+(1:m-1), d_idx) = D2;
            row_idx = row_idx + (m-1);
            % d방향 가속도 제약조건 : 최소
            A_feasible(row_idx+(1:m-1), d_idx) = -D2;
            row_idx = row_idx + (m-1);            
        end
    else
        v_s_max = bounds.vs_max + 1e-6; v_s_min = bounds.vs_min - 1e-6;
        v_d_max = bounds.vd_max + 1e-6; v_d_min = bounds.vd_min - 1e-6;
        a_s_max = bounds.as_max + 1e-6; a_s_min = bounds.as_min - 1e-6;
        a_d_max = bounds.ad_max + 1e-6; a_d_min = bounds.ad_min - 1e-6;
        v_s_terminal_min = bounds.vs_terminal_min - 1e-6;
        v_s_terminal_max = bounds.vs_terminal_max + 1e-6;
        for i = 1:N
            % s방향 속도 제약조건 : 최대
            % s방향 속도 제약조건 : 최소
            if i == N
                % 마지막 구간에서는 Terminal State 값 이용
                % b_feasible(row_idx+(1:m-1)) = v_s_max;
                b_feasible(row_idx+(1:m)) = v_s_terminal_max;
                row_idx = row_idx + m;
                % b_feasible(row_idx+(1:m-1)) = -v_s_min;
                b_feasible(row_idx+(1:m)) = -v_s_terminal_min;
                row_idx = row_idx + m;
            else
                b_feasible(row_idx+(1:m)) = v_s_max;
                row_idx = row_idx + m;
                b_feasible(row_idx+(1:m)) = -v_s_min;
                row_idx = row_idx + m;
            end
            % s방향 가속도 제약조건 : 최대
            b_feasible(row_idx+(1:m-1)) = a_s_max;
            row_idx = row_idx + (m-1);
            % s방향 가속도 제약조건 : 최소
            b_feasible(row_idx+(1:m-1)) = -a_s_min;
            row_idx = row_idx + (m-1);

            % d방향 속도 제약조건 : 최대
            b_feasible(row_idx+(1:m)) = v_d_max;
            row_idx = row_idx + m;
            % d방향 속도 제약조건 : 최소
            b_feasible(row_idx+(1:m)) = -v_d_min;
            row_idx = row_idx + m;

            % d방향 가속도 제약조건 : 최대
            b_feasible(row_idx+(1:m-1)) = a_d_max;
            row_idx = row_idx + (m-1);
            % d방향 가속도 제약조건 : 최소
            b_feasible(row_idx+(1:m-1)) = -a_d_min;
            row_idx = row_idx + (m-1);            
        end
    end
end