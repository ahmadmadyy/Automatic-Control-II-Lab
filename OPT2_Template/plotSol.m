% Plot Ergebnisse Ball-auf-Platte / Plot solution 

xsol = xout.Data';
usol = uout.Data';
xdes = xdesout.Data';
tsol = xout.Time';


tmp_fig2=figure(); clf;
maxHeight = 1200;
tmp_fig2.Position = [maxHeight 10 maxHeight/1.3 maxHeight];
hm(1) = subplot(3,1,1);
hold on; grid on; box on; 
plot(tsol, xdes(1,:), tsol, xdes(2, :), 'LineWidth', 3)
plot(tsol, xsol(1,:),'--', tsol, xsol(5, :), '--', 'LineWidth', 3)
legend('x_{K, soll}', 'y_{K, soll}', 'x_K', 'y_K')
ylabel('p [m]');
set(gca,'xticklabel',[])
p1=get(gca,'Position'); set(gca,'Position',[p1(1)*0.7 p1(2)*0.96-0.01 p1(3)*1.15 p1(4)*1.3]);


if param.controlType == 3
    hm(2) = subplot(3,1,2);
    hold on; grid on; box on; 
    plot(tsol, rad2deg(param.amplVphi*sin(param.freqVphi*tsol)), tsol, rad2deg(param.amplVtheta*sin(param.freqVtheta*tsol)), 'LineWidth', 3)
    plot(tsol, rad2deg(xsol(4, :)),'--', tsol, rad2deg(xsol(8, :)),'--', 'LineWidth', 3)
    plot(tsol, rad2deg(xsol(3, :)),'--', tsol, rad2deg(xsol(7, :)),'--', 'LineWidth', 3)
    legend('v_{\phi, soll}', 'v_{\theta, soll}', 'v_{\phi}', 'v_{\theta}', '\phi', '\theta')
    ylabel('\omega [deg/s]');
    set(gca,'xticklabel',[])
    p2=get(gca,'Position'); set(gca,'Position',[p2(1)*0.7 p2(2)*0.965-0.02 p2(3)*1.15 p2(4)*1.3]);
else
    hm(2) = subplot(3,1,2);
    hold on; grid on; box on; 
    % plot(tsol, xsol, 'LineWidth', 3)
    % legend('x_1', 'x_2', 'x_3', 'x_4', 'x_5', 'x_6', 'x_7', 'x_8')
    plot(tsol, sqrt(xsol(2, :).^2 + xsol(6, :).^2), 'LineWidth', 3)
    if param.controlType == 1
        plot(tsol, param.vsoll*ones(length(tsol),1), 'LineWidth', 3)
    end
    if param.controlType == 2
        plot(tsol, usol(3, :), 'LineWidth', 3)
    end
    legend('v_K', 'v_{Pfad}')
    ylabel('v [m/s]');
    set(gca,'xticklabel',[])
    p2=get(gca,'Position'); set(gca,'Position',[p2(1)*0.7 p2(2)*0.965-0.02 p2(3)*1.15 p2(4)*1.3]);
end


hm(3) = subplot(3,1,3);
hold on; grid on; box on; 
plot(tsol, usol, 'LineWidth', 3)
legend('u_1', 'u_2', 'u_3')
xlabel('t [s]'); ylabel('u');
xlabel('t [s]'); ylabel('v [m/s] / a [m/s^2]');
p3=get(gca,'Position'); set(gca,'Position',[p3(1)*0.7 p3(2)*1.0-0.03 p3(3)*1.15 p3(4)*1.3]);




