% Plot Ergebnisse Verladebruecke

xsol = xout.Data';
usol = uout.Data';
tsol = xout.Time';

xL = xsol(1,:) + sin(xsol(5,:)).*xsol(3,:);
yL = param.yL0offset - cos(xsol(5,:)).*xsol(3,:);

figure
subplot(2,1,1);
hold on; grid on; box on; 
plot(tsol, param.xLsoll*ones(length(tsol),1), tsol, param.yLsoll*ones(length(tsol),1), 'LineWidth', 3)
plot(tsol, xL, '--', tsol, yL, '--', 'LineWidth', 3)
legend('x_{L, soll}', 'y_{L, soll}', 'x_L', 'y_L')
ylabel('p [m]');


subplot(2,1,2);
hold on; grid on; box on; 
plot(tsol, xsol(2, :), tsol, usol, 'LineWidth', 3)
legend('v_W', 'u_1', 'u_2')
xlabel('t [s]'); ylabel('v [m/s] / a [m/s^2]');

