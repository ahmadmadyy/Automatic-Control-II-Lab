close all;
% Animation Verladebruecke

realtime = 0;

tmp_fig1=figure(); clf;
maxWidth = 1800; %1920;
tmp_fig1.Position = [10 10 maxWidth maxWidth/(1.4/0.9)];
ph = subplot(1,1,1);
axis equal
steps = 150;
xsol = xout.Data';
tsol = xout.Time';
x_temp = xsol(:,floor(1:numel(xsol(1,:))/steps:numel(xsol(1,:))));
t_temp = tsol(:,floor(1:numel(tsol(1,:))/steps:numel(tsol(1,:))));
hold on; grid on; box on; 
xlabel('x [m]'); ylabel('y [m]');
ylim([param.yL0offset-0.8, param.yL0offset+0.2]);
xlim([-0.7, 0.7]);
ptmp = draw_Sollwerte(param, ph);

for i = 1:numel(x_temp(1,:))
    if realtime == 1
        tic;
    end
    
    draw_Verladebruecke(x_temp(:,i), 0.3+0.7/(steps+2)*i,ph, param)
    for k = 1:length(ptmp) 
        delete(ptmp(k)); 
    end
    ptmp = draw_Sollwerte(param, ph);
    drawnow
    
    ylim([param.yL0offset-0.8, param.yL0offset+0.1]);
    set(gca,'FontSize', 20)
    title( sprintf('t = %.1f s', t_temp(i)) );
    
    if realtime == 1
        pause((t_temp(end)/steps) - toc);
    end
end    
% axis(ph,'equal')


function draw_Verladebruecke(x, color, ph, param)
    temp_color = [1-color, 1-color, 1-color];
    rectangle('Position',[x(1)-0.1 param.yL0offset-0.05 0.2 0.1],'Curvature',0.2,'EdgeColor',temp_color,'Parent',ph)
    rectangle('Position',[x(1)-0.08 param.yL0offset-0.075 0.05 0.05],'Curvature',[1 1],'EdgeColor',temp_color,'Parent',ph)
    rectangle('Position',[x(1)+0.03 param.yL0offset-0.075 0.05 0.05],'Curvature',[1 1],'EdgeColor',temp_color,'Parent',ph)
    plot([x(1) x(1)+(x(3)-param.HoeheLast)*sin(x(5))],[param.yL0offset, param.yL0offset-(x(3)-param.HoeheLast)*cos(x(5))],'Color',temp_color,'Parent',ph)
    rectangle('Position',[x(1)+x(3)*sin(x(5))-param.BreiteLast/2 param.yL0offset-x(3)*cos(x(5))+0.00 param.BreiteLast param.HoeheLast],'Curvature',[0 0],'EdgeColor',temp_color, 'FaceColor',min(1, temp_color*1.5), 'Parent',ph)    
end

function p = draw_Sollwerte(param, ph)

    markerSize = 80;
    p = [];
    
    if param.constrainYLast == 1
        yy=-0.5:0.01:0.5;
        yyy = param.p0*yy.^2+param.p1;
        p(1) = plot(yy,yyy,'Parent',ph, 'LineWidth', 2.5, 'Color', [0,155,119]/255);
    %     legend('y-Beschraenkung')
    end
    
    p(length(p)+1) = plot(-0.5, 0.0, '.', 'Parent',ph, 'MarkerSize', markerSize, 'Color', [152,164,174]/255);
    p(length(p)+1) = plot(0.5, 0.0, '.', 'Parent',ph, 'MarkerSize', markerSize, 'Color', [152,164,174]/255);
    p(length(p)+1) = plot(-0.5, 0.35, '.', 'Parent',ph, 'MarkerSize', markerSize, 'Color', [152,164,174]/255);
    p(length(p)+1) = plot(0.5, 0.35, '.', 'Parent',ph, 'MarkerSize', markerSize, 'Color', [152,164,174]/255);

    if param.Sollvorgabe == 1
        p(length(p)+1) = plot(param.xLsoll, param.yLsoll, '.', 'Parent',ph, 'MarkerSize', markerSize, 'Color', [141,20,41]/255);
        p(length(p)+1) = plot(param.xL0, param.yL0, '.', 'Parent',ph, 'MarkerSize', markerSize, 'Color', [141,20,41]/255);
    end
end
