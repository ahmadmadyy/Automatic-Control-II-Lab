close all;
% Animation Ball-Auf-Platte

realtime = 0;
tPast = 5;

tmp_fig1=figure(); clf;
maxHeight = 1200;
tmp_fig1.Position = [10 10 maxHeight/1.3 maxHeight];
ph = subplot(1,1,1);
axis equal
steps = 150;
xsol = xout.Data';
xdes = xdesout.Data';
tsol = xout.Time';
stepsPast = ceil(steps * tPast/tsol(end));
ptmp = cell(2, stepsPast);
x_temp = xsol(:,floor(1:numel(xsol(1,:))/steps:numel(xsol(1,:))));
x_des_temp = xdes(:,floor(1:numel(xdes(1,:))/steps:numel(xdes(1,:))));
t_temp = tsol(:,floor(1:numel(tsol(1,:))/steps:numel(tsol(1,:))));

rectangle('Position',[-0.1775 -0.2365 2*0.1775 2*0.2365],'Curvature',0.0,'EdgeColor','black','LineWidth',10,'Parent',ph)
hold on; 
plot(xdes(1, :), xdes(2, :), 'LineWidth', 1, 'Color', [152,164,174]/255)

box on; xlabel('x [m]'); ylabel('y [m]');
ylim([-0.26, 0.26]);
xlim([-0.2, 0.2]);

for i = 1:numel(x_temp(1,:))
    if realtime == 1
        tic;
    end
    
    idx = mod(i,stepsPast)+1;
    if i > stepsPast
        delete(ptmp{1, idx});
        delete(ptmp{2, idx});
    end
    ptmp(:, idx) = draw_BAB(ptmp(:, idx), x_temp(:,i), x_des_temp(:, i), ph);
    for k = 1:stepsPast-1
        if (~isempty(ptmp{1, mod(idx+k, stepsPast)+1}))
            ptmp{1, mod(idx+k, stepsPast)+1}.Color = min(1, ptmp{1, mod(idx+k, stepsPast)+1}.Color * 1.02); 
            ptmp{2, mod(idx+k, stepsPast)+1}.Color = min(1, ptmp{2, mod(idx+k, stepsPast)+1}.Color * 1.02);
        end
    end
    drawnow
    ylim([-0.26, 0.26]);
    set(gca,'FontSize', 20)
    title( sprintf('t = %.1f s', t_temp(i)) );
    
    if realtime == 1
        pause((t_temp(end)/steps) - toc);
    end
end    
% axis(ph,'equal')


function ptmp = draw_BAB(ptmp, x, xdes, ph)
    ptmp{1} = plot(xdes(1), xdes(2), '.', 'Parent',ph, 'MarkerSize', 60, 'Color', [0,5,119]/255);
    ptmp{2} = plot(x(1), x(5), '.', 'Parent',ph, 'MarkerSize', 50, 'Color', [0,155,119]/255);
end