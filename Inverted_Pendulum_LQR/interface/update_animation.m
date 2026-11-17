function handles = update_animation(ax, t, x, params, handles)
% UPDATE_ANIMATION Renders the 2D kinematic schematic of the cart-pendulum system.
%
% Coordinates:
%   Cart center is at (p, cart_h/2).
%   Pivot is at (p, cart_h).
%   Pendulum COM is at (p + l*sin(theta), cart_h + l*cos(theta)).
%   Pendulum tip is at (p + L*sin(theta), cart_h + L*cos(theta)).

    p = x(1);
    theta = x(3);

    L = params.L;
    cw = params.cart_w;
    ch = params.cart_h;
    rw = params.wheel_r;

    % Pivot coordinate
    pivot_x = p;
    pivot_y = ch;

    % Tip coordinate
    tip_x = pivot_x + L * sin(theta);
    tip_y = pivot_y + L * cos(theta);

    % Cart rectangle vertices [x_left, y_bottom, width, height]
    cart_rect = [p - cw/2, rw, cw, ch - rw];

    % Wheel coordinates
    wheel1_x = p - cw/3;
    wheel2_x = p + cw/3;
    wheel_y = rw;

    if isempty(handles) || ~isvalid(handles.cart)
        % First-time initialization
        cla(ax);
        hold(ax, 'on');

        % Ground and rail track
        track_L = params.track_lim * 1.3;
        plot(ax, [-track_L, track_L], [0, 0], 'k-', 'LineWidth', 2.5);
        % Limit bumpers
        plot(ax, [-params.track_lim, -params.track_lim], [0, ch*1.5], 'r--', 'LineWidth', 1.5);
        plot(ax, [params.track_lim, params.track_lim], [0, ch*1.5], 'r--', 'LineWidth', 1.5);

        % Upright reference line
        handles.ref_line = plot(ax, [p, p], [ch, ch + L*1.1], 'k:', 'LineWidth', 1.0);

        % Cart body
        handles.cart = rectangle(ax, 'Position', cart_rect, 'FaceColor', [0.2, 0.45, 0.85], ...
            'EdgeColor', [0.05, 0.2, 0.5], 'LineWidth', 2, 'Curvature', [0.08, 0.08]);

        % Wheels
        handles.wheel1 = rectangle(ax, 'Position', [wheel1_x - rw, 0, 2*rw, 2*rw], ...
            'Curvature', [1 1], 'FaceColor', [0.2 0.2 0.2], 'EdgeColor', 'k');
        handles.wheel2 = rectangle(ax, 'Position', [wheel2_x - rw, 0, 2*rw, 2*rw], ...
            'Curvature', [1 1], 'FaceColor', [0.2 0.2 0.2], 'EdgeColor', 'k');

        % Pendulum rod
        handles.rod = plot(ax, [pivot_x, tip_x], [pivot_y, tip_y], 'r-', 'LineWidth', 4);

        % Pendulum bob (tip mass indicator)
        handles.bob = plot(ax, tip_x, tip_y, 'ro', 'MarkerFaceColor', [0.85, 0.15, 0.15], ...
            'MarkerSize', 10, 'LineWidth', 1.5);

        % Pivot pin
        handles.pivot = plot(ax, pivot_x, pivot_y, 'ko', 'MarkerFaceColor', 'y', ...
            'MarkerSize', 7, 'LineWidth', 1.5);

        % Info overlay text
        handles.info_txt = text(ax, -track_L * 0.95, L * 1.35, '', 'FontSize', 10, ...
            'FontWeight', 'bold', 'BackgroundColor', [1 1 1 0.75], 'EdgeColor', [0.8 0.8 0.8]);

        axis(ax, 'equal');
        xlim(ax, [-track_L, track_L]);
        ylim(ax, [-0.1, L * 1.5]);
        xlabel(ax, 'Horizontal Position p [m]');
        ylabel(ax, 'Height [m]');
        title(ax, 'Animated Cart-Pendulum Kinematics', 'FontSize', 12, 'FontWeight', 'bold');
        grid(ax, 'on');
    else
        % Efficient object property update for fast animation
        set(handles.cart, 'Position', cart_rect);
        set(handles.wheel1, 'Position', [wheel1_x - rw, 0, 2*rw, 2*rw]);
        set(handles.wheel2, 'Position', [wheel2_x - rw, 0, 2*rw, 2*rw]);
        set(handles.rod, 'XData', [pivot_x, tip_x], 'YData', [pivot_y, tip_y]);
        set(handles.bob, 'XData', tip_x, 'YData', tip_y);
        set(handles.pivot, 'XData', pivot_x, 'YData', pivot_y);
        set(handles.ref_line, 'XData', [p, p]);
    end

    % Update telemetry text
    theta_deg = rad2deg(theta);
    txt = sprintf('Time: %5.2f s | p: %+.3f m | \\theta: %+.2f^\\circ | \\omega: %+.2f rad/s', ...
        t, p, theta_deg, x(4));
    set(handles.info_txt, 'String', txt);
end
