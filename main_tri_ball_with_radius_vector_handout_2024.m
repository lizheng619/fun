clear
clc
close all
dos('del *.asv')

dt = 0.0001;
t  = 0:dt:50;

% mass of the ball
m = 1;
% damping associated with the velocity of the ball
c = 1.0;
% gravity
g = 9.81;
gravity = m*g*1;

% radius of the ball
radius_1 = 0.2;
radius_2 = 0.2;
radius_3 = 0.2;

% The boundary of the problem
x_left   = -1;
x_righ   =  1;
y_bottom = -1;
y_top    =  1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Initial conditions
% Initial position of the three balls
% ball 1 x,y; ball 2 x,y; ball 3 x,y; 
d = [-0.5;-0.1;0.0;0.0;0.5;0.5];
% Initial velocity of the three balls
% ball 1 vx,vy; ball 2 vx,vy; ball 3 vx,vy; 
v = [100;30;0;0;0;0];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for i=2:length(t)
% for i=2:487
    A  = [m/dt 0     0     0     0     0
          0    m/dt  0     0     0     0
          0    0     m/dt  0     0     0
          0    0     0     m/dt  0     0
          0    0     0     0     m/dt  0
          0    0     0     0     0     m/dt];
    Fg  = [0;-gravity;0;-gravity;0;-gravity];
    C  =  [c 0 0 0 0 0
           0 c 0 0 0 0
           0 0 c 0 0 0
           0 0 0 c 0 0
           0 0 0 0 c 0
           0 0 0 0 0 c];
    Fd  = C*v(:,i-1);
    B   = Fg-Fd;
    dv  = linsolve(A,B);
    % collision with the boundaries
    % 2 4 6 y_bottom
    if d(2,i-1)<=y_bottom+radius_1
        v(2,i-1) = -v(2,i-1);
    end
    if d(4,i-1)<=y_bottom+radius_1
        v(4,i-1) = -v(4,i-1);  
    end
    if d(6,i-1)<=y_bottom+radius_1
        v(6,i-1) = -v(6,i-1);
    end
    % 2 4 6 y_top
    if d(2,i-1)>=y_top-radius_1
        v(2,i-1) = -v(2,i-1);
    end
    if d(4,i-1)>=y_top-radius_1
        v(4,i-1) = -v(4,i-1);
    end
    if d(6,i-1)>=y_top-radius_1
        v(6,i-1) = -v(6,i-1);
    end
    % 1 3 5 x_left
    if d(1,i-1)<=x_left+radius_1
        v(1,i-1) = -v(1,i-1);
    end
    if d(3,i-1)<=x_left+radius_1
        v(3,i-1) = -v(3,i-1);
    end
    if d(5,i-1)<=x_left+radius_1
        v(5,i-1) = -v(5,i-1);
    end
    % 1 3 5 x_righ
    if d(1,i-1)>=x_righ-radius_1
        v(1,i-1) = -v(1,i-1);
    end
    if d(3,i-1)>=x_righ-radius_1
        v(3,i-1) = -v(3,i-1);
    end
    if d(5,i-1)>=x_righ-radius_1
        v(5,i-1) = -v(5,i-1);
    end
        %
    point1 = d(1:2,i-1);
    point2 = d(3:4,i-1);
    point3 = d(5:6,i-1);
    [angle12,distance12] = find_angle_two_point(point1,point2,2);
    [angle13,distance13] = find_angle_two_point(point1,point3,2);
    [angle23,distance23] = find_angle_two_point(point2,point3,2);
    % Collision with other ball
    if distance12<(radius_1+radius_2)
        v1  = v(1,i-1);
        v2  = v(2,i-1);
        v11 = v(3,i-1);
        v22 = v(4,i-1);
        %
        v1_vector = [v1;v2];
        v2_vector = [v11;v22];
        % Calculate the velocity after Collision
        v1_vector_new = v1_vector-(dot(v1_vector-v2_vector,point1-point2)./norm(point1-point2).^2)*(point1-point2);
        v2_vector_new = v2_vector-(dot(v2_vector-v1_vector,point2-point1)./norm(point2-point1).^2)*(point2-point1);
        v1 = v1_vector_new(1);
        v2 = v1_vector_new(2);
        v11 = v2_vector_new(1);
        v22 = v2_vector_new(2);
        %
        v(1,i-1) = v1;
        v(2,i-1) = v2;
        v(3,i-1) = v11;
        v(4,i-1) = v22;
    end
    %
    if distance13<(radius_1+radius_2)
        v1  = v(1,i-1);
        v2  = v(2,i-1);
        v111 = v(5,i-1);
        v222 = v(6,i-1);
        %
        v1_vector = [v1;v2];
        v2_vector = [v111;v222];
        % Calculate the velocity after Collision
        v1_vector_new = v1_vector-(dot(v1_vector-v2_vector,point1-point3)./norm(point1-point3).^2)*(point1-point3);
        v2_vector_new = v2_vector-(dot(v2_vector-v1_vector,point3-point1)./norm(point3-point1).^2)*(point3-point1);
        v1 = v1_vector_new(1);
        v2 = v1_vector_new(2);
        v111 = v2_vector_new(1);
        v222 = v2_vector_new(2);
        %
        v(1,i-1) = v1;
        v(2,i-1) = v2;
        v(5,i-1) = v111;
        v(6,i-1) = v222;
    end
    %
    if distance23<(radius_1+radius_2)
        v11  = v(3,i-1);
        v22  = v(4,i-1);
        v111 = v(5,i-1);
        v222 = v(6,i-1);
        %
        v1_vector = [v11;v22];
        v2_vector = [v111;v222];
        % Calculate the velocity after Collision
        v1_vector_new = v1_vector-(dot(v1_vector-v2_vector,point2-point3)./norm(point2-point3).^2)*(point2-point3);
        v2_vector_new = v2_vector-(dot(v2_vector-v1_vector,point3-point2)./norm(point3-point2).^2)*(point3-point2);
        v11 = v1_vector_new(1);
        v22 = v1_vector_new(2);
        v111 = v2_vector_new(1);
        v222 = v2_vector_new(2);
        %
        v(3,i-1) = v11;
        v(4,i-1) = v22;
        v(5,i-1) = v111;
        v(6,i-1) = v222;
    end
    %
    v(:,i) = v(:,i-1)+dv;
    d(:,i) = d(:,i-1)+v(:,i)*dt;
    %
end
%
animation = 1;
%
 for i=1:10:length(d)
    figure(1)
    set(1,'color','w')
    set(gca,'fontsize',16)
    [xunit1,yunit1] = drw_circle(d(1,i),d(2,i),radius_1);
    [xunit2,yunit2] = drw_circle(d(3,i),d(4,i),radius_2);
    [xunit3,yunit3] = drw_circle(d(5,i),d(6,i),radius_2);
    fill(xunit1,yunit1,'b',xunit2,yunit2,'r',xunit3,yunit3,'g','FaceAlpha',0.2);
%     plot(d(1,i),d(2,i),'o',d(3,i),d(4,i),'o','markerfacecolor','b','markersize',10);
%     plot(point_mid(1),point_mid(2),'o','markerfacecolor','b','markersize',10); hold on
    text(d(1,i),d(2,i),'1');
    text(d(3,i),d(4,i),'2');
    text(d(5,i),d(6,i),'3');
    axis equal
    xlim([x_left   x_righ])
    ylim([y_bottom y_top])
    getframe;
    %
%     filename = 'Three_balls.gif';
%     % Capture the plot as an image
%     frame = getframe;
%     im = frame2im(frame);
%     [imind,cm] = rgb2ind(im,256);
%     % Write to the GIF File
%     if animation == 1
%         imwrite(imind,cm,filename,'gif', 'Loopcount',inf);
%     else
%         imwrite(imind,cm,filename,'gif','DelayTime',0.01,'WriteMode','append');
%     end
    %
    animation = animation+1;
    %
 end