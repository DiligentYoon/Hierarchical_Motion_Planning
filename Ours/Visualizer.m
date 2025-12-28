classdef Visualizer
    % Visualizer -시각화 전용 유틸리티 클래스

    methods(Static, Access = private)

        function [fig, ax] = ensure_canvas()
            % 1) 전용 Figure 확보 (없으면 생성)
            fig = findobj('Type','figure','Tag','Visualizer3D');
            if isempty(fig) || ~isvalid(fig)
                fig = figure( ...
                    'Name','Dynamic Topology 3D', ...
                    'Tag','Visualizer3D', ...
                    'NumberTitle','off', ...
                    'MenuBar','none', ...
                    'ToolBar','none');
            end

            % 2) 전용 Axes 확보 (없으면 생성)
            ax = findobj(fig,'Type','axes','Tag','Visualizer3D_Axes');
            if isempty(ax) || ~isvalid(ax)
                ax = axes('Parent',fig,'Tag','Visualizer3D_Axes');
            end
            delete(findobj(ax, 'Type','text', 'Tag','GapCostText'));

            % 3) 이 전용 Axes만 초기화
            cla(ax); hold(ax,'on');

            % 4) 현재 Figure/Axes를 명시적으로 설정
            set(0,'CurrentFigure',fig);
            set(fig,'CurrentAxes',ax);
        end
    end
    
    methods(Static)

        function visualize(C_t, T_t, optimal_sequence, planning_horizon, planning_resolution, time_scale_factor, ego, adjacency_mtrx)
            [fig, ax] = Visualizer.ensure_canvas();
            Visualizer.visualize_all_connectivity_3D(C_t, T_t, planning_horizon, planning_resolution, time_scale_factor, ego, fig, ax);
            Visualizer.highlight_optimal_path(optimal_sequence, C_t, T_t, planning_resolution, time_scale_factor, adjacency_mtrx, fig, ax);
        end

        function visualize_all_connectivity_3D(C_t, T_t, planning_horizon, dt, time_scale_factor, ego_info, fig, ax)
            % 모든 시간에 대한 Gap 연결성을 3D로 시각화 (Spatio + Temporal)
            % 
            % 입력:
            %   C_t - SpatioConnectSet 배열 (각 시간별)
            %   T_t - TemporalConnectSet 배열 (시간 간 연결)
            %   planning_horizon - 계획 시간 범위
            %   dt - 시간 간격
            %   time_scale_factor - 시간축 스케일링 팩터 (기본값: 10)
            %   ego_info - Ego 차량 정보 (선택적)

            if isempty(C_t)
                return;
            end

            if class(C_t) ~= "cell"
                C_t = num2cell(C_t);
                T_t = num2cell(T_t);
                for i = 1:length(C_t)
                    C_t{i}.gaps = num2cell(C_t{i}.gaps);
                end
                for i = 1:length(T_t)
                    T_t{i}.current_spatio_set.gaps = num2cell(T_t{i}.current_spatio_set.gaps);
                    T_t{i}.next_spatio_set.gaps = num2cell(T_t{i}.next_spatio_set.gaps);
                end
            end

            
            % time_scale_factor가 제공되지 않으면 기본값 10 사용
            if nargin < 5
                time_scale_factor = 10;
            end
            
            hold on;
            
            % 각 시간 단계별로 Spatio Dimension 연결성 시각화
            for t_idx = 1:length(C_t)
                c_t = C_t{t_idx};
                if nargin >= 6 && ~isempty(ego_info)
                    Visualizer.visualize_spatio_connectivity_3D(c_t, t_idx, dt, time_scale_factor, ego_info, ax);
                else
                    Visualizer.visualize_spatio_connectivity_3D(c_t, t_idx, dt, time_scale_factor, ax);
                end
            end
            
            % TemporalConnectSet을 사용한 시간 차원 연결성 시각화
            if nargin >= 2 && ~isempty(T_t)
                Visualizer.visualize_temporal_connections_with_set(C_t, T_t, dt, time_scale_factor, ax);
            else
                % 기존 방식 (임시 호환성)
                Visualizer.visualize_temporal_connections_legacy(C_t, dt, time_scale_factor, ax);
            end
            
            % 범례 및 축 설정
            Visualizer.setup_3D_plot_aesthetics(C_t, dt, time_scale_factor, ax);
        end
        
        function visualize_spatio_connectivity_3D(c_t, time_idx, dt, time_scale_factor, ego_info, ax)
            % 특정 시간에서의 Gap 연결성을 3D로 시각화
            
            if isempty(c_t.gaps) || c_t.num_gaps == 0
                return;
            end
            
            % time_scale_factor가 제공되지 않으면 기본값 10 사용
            if nargin < 4
                time_scale_factor = 10;
            end
            
            % 현재 시간 계산 (스케일링 적용)
            current_time = (time_idx - 1) * dt * time_scale_factor;
            
            % Gap 위치별 색상 정의 (차선별)
            lane_colors = [1 0 0; 0 0 1; 0 0.7 0]; % 빨강, 파랑, 초록
            
            % Gap 노드들을 3D 점으로 표시
            for i = 1:c_t.num_gaps
                if ~isempty(c_t.gaps{i})
                    gap = c_t.gaps{i};
                    lane_idx = gap.lane_idx;
                    % Ego Gap인지 확인 (t=0이고 ego 위치와 일치하는지)
                    is_ego_gap = false;
                    if time_idx == 1 && nargin >= 5 && ~isempty(ego_info) % t=0이고 ego_info가 제공된 경우
                        % Ego 차량이 현재 Gap 안에 있는지 확인
                        if gap.d == ego_info.y0 && ...
                           ego_info.x0 >= (gap.s - gap.l/2) && ...
                           ego_info.x0 <= (gap.s + gap.l/2)
                            is_ego_gap = true;
                        end
                    end
                    
                    % 색상 결정
                    if is_ego_gap
                        gap_color = [1 1 0]; % 노란색 (Ego Gap)
                        marker_size = 50; % 더 큰 마커
                        edge_color = 'r'; % 빨간 테두리
                    else
                        gap_color = lane_colors(lane_idx, :);
                        marker_size = 25;
                        edge_color = 'k';
                    end
                    
                    % Gap 중심점 표시
                    scatter3(ax, gap.s, gap.d, current_time, marker_size, gap_color, 'filled', 'MarkerEdgeColor', edge_color);
                    
                    % Gap 영역을 반투명 박스로 표시
                    gap_width = gap.w; % 차량 폭
                    gap_length = gap.l;
                    
                    % Gap 박스의 꼭짓점들
                    x_gap = [gap.s - gap_length/2, gap.s + gap_length/2, gap.s + gap_length/2, gap.s - gap_length/2];
                    y_gap = [gap.d - gap_width/2, gap.d - gap_width/2, gap.d + gap_width/2, gap.d + gap_width/2];
                    z_gap = [current_time, current_time, current_time, current_time];
                    
                    % 박스 투명도 조정
                    if is_ego_gap
                        face_alpha = 0.3;
                        edge_width = 1.5;
                    else
                        face_alpha = 0.1;
                        edge_width = 0.5;
                    end
                    
                    patch(x_gap, y_gap, z_gap, gap_color, 'FaceAlpha', face_alpha, 'EdgeColor', edge_color, 'LineWidth', edge_width, 'Parent', ax);
                end
            end
            
            % 연결성 표시 (같은 시간 내 차선 간 연결)
            for i = 1:c_t.num_gaps
                if ~isempty(c_t.gaps{i})
                    for j = 1:c_t.num_gaps
                        if c_t.reachable_mtrx(i, j) && ~isempty(c_t.gaps{j})
                            gap_i = c_t.gaps{i};
                            gap_j = c_t.gaps{j};

                            % if gap_i.time_idx <= 2
                            %     plot3(ax, [gap_i.s, gap_j.s], [gap_i.d, gap_j.d], ...
                            %           [current_time, current_time], '-', 'Color', [0.5, 0.5, 0.5], 'LineWidth', 1.0);
                            % else
                            plot3(ax, [gap_i.s, gap_j.s], [gap_i.d, gap_j.d], ...
                                  [current_time, current_time], 'k-', 'LineWidth', 2.0);
                            % end
                           
                            
                            % % 화살표 방향 표시 (선택적)
                            % % 중점에서 작은 화살표
                            % mid_s = (gap_i.s + gap_j.s) / 2;
                            % mid_d = (gap_i.d + gap_j.d) / 2;
                            % dir_s = (gap_j.s - gap_i.s) * 0.1;
                            % dir_d = (gap_j.d - gap_i.d) * 0.1;
                            % 
                            % quiver3(ax, mid_s, mid_d, current_time, dir_s, dir_d, 0, 0, 'k', 'MaxHeadSize', 0.3);
                        end
                    end
                end
            end
        end
        
        function visualize_temporal_connections_with_set(C_t, T_t, dt, time_scale_factor, ax)
            % TemporalConnectSet을 사용하여 시간 축에서의 Gap 연결을 시각화
            
            if isempty(T_t) || length(T_t) == 0
                return;
            end
            
            % time_scale_factor가 제공되지 않으면 기본값 10 사용
            if nargin < 4
                time_scale_factor = 10;
            end
            
            % 각 TemporalConnectSet에 대해 연결성 시각화
            for t_idx = 1:length(T_t)
                temporal_set = T_t{t_idx};
                
                if isempty(temporal_set) || temporal_set.num_nodes == 0
                    continue;
                end
                
                current_time = (t_idx - 1) * dt * time_scale_factor;
                next_time = t_idx * dt * time_scale_factor;
                
                % temporal_topology_global_graph에서 연결 정보 가져오기
                for i = 1:temporal_set.num_nodes
                    source_gap_id = temporal_set.temporal_topology_global_graph(1, i);
                    target_gap_id = temporal_set.temporal_topology_global_graph(2, i);
                    
                    % 해당 시간의 SpatioConnectSet에서 gap 정보 찾기
                    source_gap = Visualizer.find_gap_by_id(temporal_set.current_spatio_set, source_gap_id);
                    target_gap = Visualizer.find_gap_by_id(temporal_set.next_spatio_set, target_gap_id);
                    
                    if ~isempty(source_gap) && ~isempty(target_gap)
                        % 시간 축 연결선 그리기
                        plot3(ax, [source_gap.s, target_gap.s], [source_gap.d, target_gap.d], ...
                                  [current_time, next_time], 'b-', 'LineWidth', 2.0);
                        
                        % 연결 방향 표시 (화살표)
                        mid_s = (source_gap.s + target_gap.s) / 2;
                        mid_d = (source_gap.d + target_gap.d) / 2;
                        mid_t = (current_time + next_time) / 2;
                        
                        dir_s = (target_gap.s - source_gap.s) * 0.1;
                        dir_d = (target_gap.d - source_gap.d) * 0.1;
                        dir_t = (next_time - current_time) * 0.1;
                        
                        quiver3(ax, mid_s, mid_d, mid_t, dir_s, dir_d, dir_t, 0, 'b', 'MaxHeadSize', 0.2);
                    end
                end
            end
        end
        
        function gap = find_gap_by_id(spatio_set, gap_id)
            % SpatioConnectSet에서 특정 ID의 gap을 찾는 헬퍼 함수
            gap = [];
            
            if isempty(spatio_set) || isempty(spatio_set.gaps)
                return;
            end
            
            for i = 1:spatio_set.num_gaps
                if ~isempty(spatio_set.gaps{i}) && spatio_set.gaps{i}.idx == gap_id
                    gap = spatio_set.gaps{i};
                    return;
                end
            end
        end
        
        function visualize_temporal_connections_legacy(C_t, dt, time_scale_factor, ax)
            % 기존 방식의 시간 축 연결 (임시 호환성)
            % TemporalConnectSet이 없을 때 사용
            
            if length(C_t) < 2
                return;
            end
            
            % time_scale_factor가 제공되지 않으면 기본값 10 사용
            if nargin < 3
                time_scale_factor = 10;
            end
            
            % 연속된 시간 단계에서 비슷한 위치의 Gap들 찾아서 연결
            for t_idx = 1:length(C_t)-1
                c_current = C_t{t_idx};
                c_next = C_t{t_idx + 1};
                
                current_time = (t_idx - 1) * dt * time_scale_factor;
                next_time = t_idx * dt * time_scale_factor;
                
                % 현재 시간과 다음 시간의 Gap들을 비교
                for i = 1:c_current.num_gaps
                    if ~isempty(c_current.gaps{i})
                        gap_current = c_current.gaps{i};
                        
                        % 다음 시간에서 비슷한 위치의 Gap 찾기
                        for j = 1:c_next.num_gaps
                            if ~isempty(c_next.gaps{j})
                                gap_next = c_next.gaps{j};
                                
                                % 같은 차선이고 위치가 비슷한 경우 연결
                                if gap_current.lane_idx == gap_next.lane_idx && ...
                                   abs(gap_current.s - gap_next.s) < 5 && ...
                                   abs(gap_current.d - gap_next.d) < 1
                                    
                                    plot3(ax, [gap_current.s, gap_next.s], [gap_current.d, gap_next.d], ...
                                              [current_time, next_time], 'b--', 'LineWidth', 1);
                                    break; % 하나만 연결
                                end
                            end
                        end
                    end
                end
            end
        end
        
        function setup_3D_plot_aesthetics(C_t, dt, time_scale_factor, ax)
            % 3D 플롯의 범례, 축 레이블, 기타 요소 설정
            
            % 범례 추가
            legend_handles = [];
            legend_labels = {};
            
            % 차선별 범례
            lane_colors = [1 0 0; 0 0 1; 0 0.7 0];
            lane_names = {'Lane 1', 'Lane 2', 'Lane 3'};
            
            for i = 1:3
                h = scatter3(ax, NaN, NaN, NaN, 50, lane_colors(i, :), 'filled');
                legend_handles = [legend_handles, h];
                legend_labels{end+1} = lane_names{i};
            end
            
            % Ego Gap 범례
            h_ego = scatter3(ax, NaN, NaN, NaN, 100, [1 1 0], 'filled', 'MarkerEdgeColor', 'r');
            legend_handles = [legend_handles, h_ego];
            legend_labels{end+1} = 'Ego Gap (t=0)';
            
            % 연결성 범례
            h_spatio = plot3(ax, NaN, NaN, NaN, 'k-', 'LineWidth', 1.5);
            legend_handles = [legend_handles, h_spatio];
            legend_labels{end+1} = 'Lane Change Connection';
            
            h_temporal = plot3(ax, NaN, NaN, NaN, 'b--', 'LineWidth', 1.5);
            legend_handles = [legend_handles, h_temporal];
            legend_labels{end+1} = 'Lane Keeping Connection';
            
            % legend(ax, legend_handles, legend_labels, 'Location', 'southwest');
            
            % 축 레이블 및 제목
            xlabel(ax, 'Longitudinal Distance (m)', 'FontSize', 16);
            ylabel(ax, 'Lateral Distance (m)', 'FontSize', 16);
            zlabel(ax, 'Time (s)', 'FontSize', 16);
            title(ax, 'Spatio-Temporal 3D Decision Graph', 'FontSize', 16);
            
            % 시간 축 틱을 실제 시간 값으로 설정
            max_scaled_time = (length(C_t) - 1) * dt * time_scale_factor;
            actual_time_ticks = 0:dt:(length(C_t) - 1) * dt;
            scaled_time_ticks = actual_time_ticks * time_scale_factor;
            
            if max_scaled_time > 0
                zticks(ax, scaled_time_ticks);
                zticklabels(ax, arrayfun(@(x) sprintf('%.1f', x), actual_time_ticks, 'UniformOutput', false));
            end
            
            % 3D 뷰 설정
            view(ax, 30, 20);
            grid(ax, 'on');
            
            % 축 비율을 수동으로 설정하여 시간 축이 더 길게 보이도록 함
            axis(ax, 'equal')  % 우선 동일 비율로 설정
            ax.DataAspectRatio = [1.5 0.8 0.4];  % [x y z] 비율, z축(시간)을 더 압축하여 상대적으로 길어 보이게
        end

        function draw_gap_nodes_3D(C_t, dt, time_scale_factor, ego_info)
            % Gap 노드들을 3D로 그리기 (기존 로직과 동일)
            
            lane_colors = [1 0 0; 0 0 1; 0 0.7 0]; % 빨강, 파랑, 초록
            
            for t_idx = 1:length(C_t)
                c_t = C_t{t_idx};
                if isempty(c_t.gaps) || c_t.num_gaps == 0
                    continue;
                end
                
                current_time = (t_idx - 1) * dt * time_scale_factor;
                
                for i = 1:c_t.num_gaps
                    if ~isempty(c_t.gaps{i})
                        gap = c_t.gaps{i};
                        lane_idx = gap.lane_idx;
                        
                        % Ego Gap 확인
                        is_ego_gap = false;
                        if t_idx == 1 && nargin >= 4 && ~isempty(ego_info)
                            if gap.d == ego_info.y0 && ...
                               ego_info.x0 >= (gap.s - gap.l/2) && ...
                               ego_info.x0 <= (gap.s + gap.l/2)
                                is_ego_gap = true;
                            end
                        end
                        
                        % 색상 및 크기 결정
                        if is_ego_gap
                            gap_color = [1 1 0]; % 노란색
                            marker_size = 100;
                            edge_color = 'r';
                        else
                            gap_color = lane_colors(lane_idx, :);
                            marker_size = 50;
                            edge_color = 'k';
                        end
                        
                        % Gap 중심점 표시
                        scatter3(gap.s, gap.d, current_time, marker_size, gap_color, 'filled', 'MarkerEdgeColor', edge_color);
                        
                        % Gap 영역 박스
                        gap_width = 1.8;
                        gap_length = gap.l;
                        
                        x_gap = [gap.s - gap_length/2, gap.s + gap_length/2, gap.s + gap_length/2, gap.s - gap_length/2];
                        y_gap = [gap.d - gap_width/2, gap.d - gap_width/2, gap.d + gap_width/2, gap.d + gap_width/2];
                        z_gap = [current_time, current_time, current_time, current_time];
                        
                        face_alpha = is_ego_gap * 0.2 + 0.1;
                        edge_width = is_ego_gap * 1.0 + 0.5;
                        
                        patch(x_gap, y_gap, z_gap, gap_color, 'FaceAlpha', face_alpha, 'EdgeColor', edge_color, 'LineWidth', edge_width);
                    end
                end
            end
        end

        function highlight_optimal_path(optimal_sequence, C_t, T_t, dt, time_scale_factor, adjacency_mtrx, fig, ax)
            % 최적 경로를 굵은 색상으로 강조하고 애니메이션으로 표시
            %
            % 입력:
            %   optimal_sequence - 최적 Gap 시퀀스 (node IDs)
            %   C_t - SpatioConnectSet 배열
            %   T_t - TemporalConnectSet 배열
            %   dt - 시간 간격
            %   time_scale_factor - 시간축 스케일링

            if isempty(optimal_sequence) || length(optimal_sequence) < 2
                return;
            end

            if class(C_t) ~= "cell"
                C_t = num2cell(C_t);
                T_t = num2cell(T_t);
                for i = 1:length(C_t)
                    C_t{i}.gaps = num2cell(C_t{i}.gaps);
                end

                for i = 1:length(T_t)
                    T_t{i}.current_spatio_set.gaps = num2cell(T_t{i}.current_spatio_set.gaps);
                    T_t{i}.next_spatio_set.gaps = num2cell(T_t{i}.next_spatio_set.gaps);
                end
            end

            % 1. 최적 경로의 Gap 정보 수집
            path_gaps = {};
            path_times = [];

            for i = 1:length(optimal_sequence)
                node_id = optimal_sequence(i);
                % 모든 시간 단계에서 해당 node_id를 가진 gap 찾기
                for t_idx = 1:length(C_t)
                    c_t = C_t{t_idx};
                    for j = 1:c_t.num_gaps
                        if ~isempty(c_t.gaps{j}) && c_t.gaps{j}.idx == node_id
                            path_gaps{i} = c_t.gaps{j};
                            path_times(i) = (t_idx - 1) * dt * time_scale_factor;
                            break;
                        end
                    end
                    if ~isempty(path_gaps) && length(path_gaps) >= i && ~isempty(path_gaps{i})
                        break;
                    end
                end
            end

            % 2. 최적 경로 Gap들을 특별한 마커로 표시
            Visualizer.highlight_optimal_nodes(path_gaps, path_times, ax);

            % 3. 최적 경로의 edge들을 굵은 색상으로 표시
            Visualizer.highlight_optimal_edges(optimal_sequence, path_gaps, path_times, ax);

            % % 4. Node Cost 표시
            % Visualizer.draw_node_costs(C_t, dt, time_scale_factor, ax);

            % % 5. Spatio (Lane Change) Cost 표시
            % Visualizer.draw_lane_change_costs(C_t, dt, time_scale_factor, ax, adjacency_mtrx);

        end

        function highlight_optimal_nodes(path_gaps, path_times, ax)
            % 최적 경로의 노드들을 큰 마커로 강조
            % delete(findall(ax, 'Type','text', 'Tag','NodeNumber'));
            delete(findall(ax, 'Tag','OptimalNode'));

            for i = 1:length(path_gaps)
                if ~isempty(path_gaps{i})
                    gap = path_gaps{i};
                    time = path_times(i);

                    % 큰 마커로 표시 (범례에서 제외)
                    scatter3(ax, gap.s, gap.d, time, 100, [1, 0, 0], 'filled', ...
                             'MarkerEdgeColor', 'red', 'LineWidth', 3, ...
                             'HandleVisibility', 'off', ...
                             'Tag', 'OptimalNode');

                    % 노드 번호 표시 (범례에서 제외)
                    % text(gap.s, gap.d, time + 1.5, sprintf('%d', gap.idx), ...
                    %      'FontSize', 10, 'Color', 'red', ...
                    %      'HorizontalAlignment', 'center', 'HandleVisibility', 'off', ...
                    %      'Tag', 'NodeNumber', 'Parent', ax);
                end
            end
        end


        function highlight_optimal_edges(optimal_sequence, path_gaps, path_times, ax)
            % 최적 경로의 edge들을 굵은 색상으로 강조
            delete(findall(ax, 'Tag', 'OptimalEdge'));
            for i = 1:length(optimal_sequence)-1
                source_gap = path_gaps{i};
                target_gap = path_gaps{i+1};
                source_time = path_times(i);
                target_time = path_times(i+1);
                
                if ~isempty(source_gap) && ~isempty(target_gap)
                    % 굵은 빨간색 실선
                    edge_color = [1, 0, 0]; % 빨간색
                    line_style = '-';
                    line_width = 4;
       
                    % 굵은 선으로 연결 (범례에서 제외)
                    if source_gap.s < target_gap.s
                        plot3(ax, [source_gap.s, target_gap.s], [source_gap.d, target_gap.d], ...
                            [source_time, target_time], line_style, 'Color', edge_color, ...
                            'LineWidth', line_width, 'HandleVisibility', 'off', ...
                            'Tag', 'OptimalEdge');
                    else
                        plot3(ax, [source_gap.s, source_gap.s], [source_gap.d, target_gap.d], ...
                            [source_time, target_time], line_style, 'Color', edge_color, ...
                            'LineWidth', line_width, 'HandleVisibility', 'off', ...
                            'Tag', 'OptimalEdge');
                    end
                end
            end
        end

        function draw_node_costs(C_t, dt, time_scale_factor, ax)
            delete(findobj(ax, 'Type','text', 'Tag','GapCostText'));
            for i = 1:length(C_t)
                c_t = C_t{i};
                time = dt * time_scale_factor * (i - 1);
                for j = 1:c_t.num_gaps
                    if ~isempty(c_t.gaps{j})
                        gap = c_t.gaps{j};
                        text(gap.s, gap.d, time, sprintf('%.2f', gap.node_cost), ...
                            'FontSize', 10, ...
                            'Tag', 'GapCostText', ...
                            'Parent', ax);
                    end
                end
            end
        end

        function draw_lane_change_costs(C_t, dt, time_scale_factor, ax, adjacency_matrix)
            delete(findobj(ax, 'Type','text', 'Tag','LaneChangeCostText'));
            for i = 1:length(C_t)
                c_t = C_t{i};
                time = dt * time_scale_factor * (i - 1);
                for j = 1:c_t.num_gaps
                    if ~isempty(c_t.gaps{j})
                        for k = 1:c_t.num_gaps
                            if c_t.reachable_mtrx(j, k) && ~isempty(c_t.gaps{k})
                                gap_j = c_t.gaps{j};
                                gap_k = c_t.gaps{k};

                                source_id = gap_j.idx;
                                target_id = gap_k.idx;

                                mid_s = (gap_j.s + gap_k.s) / 2;
                                mid_d = (gap_j.d + gap_k.d) / 2;

                                text(mid_s, mid_d, time+1.5, sprintf('%.2f', adjacency_matrix(source_id, target_id)), ...
                                    'FontSize', 12, ...
                                    'Tag', 'LaneChangeCostText', ...
                                    'Parent', ax);
                            end
                        end
                    end
                end
            end
        end
    end
end