clear
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf = 5;              % 5 segundos 
ts = 0.001;          
t = 0: ts: tf;       
N = length(t);       

%%%%%%%%%%%%%%%%%%%%%%%%%%% CÁLCULO DE VX %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

vx = sqrt(10*pi)/5; 

%%%%%%%%%%%%%%%%%%%%%%%% CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%
x1 = zeros(1,N+1);  
y1 = zeros(1,N+1);  
phi = zeros(1,N+1); 

x1(1) = 0;              
y1(1) = 0;              
phi(1) = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%% PUNTO DE CONTROL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
hx = zeros(1,N+1);  
hy = zeros(1,N+1);  
hx(1) = x1(1);
hy(1) = y1(1);

%%%%%%%%%%%%%%%%%%%%%% VELOCIDADES DE REFERENCIA %%%%%%%%%%%%%%%%%%%%%%%%%%
u = zeros(1,N); 
w = zeros(1,N); 

for k = 1:N
    tk = t(k);
    xk = vx * tk;
    
    % Función: f(x) = 2 * sin(x^2)
    % Derivadas  (respecto a x)
    dy_dx = 4 * xk * cos(xk^2); 
    ddy_dxx = 4 * cos(xk^2) - 8 * (xk^2) * sin(xk^2);
    
 
    % Usamos el avance vx constante en el eje X
    u(k) = vx * sqrt(1 + dy_dx^2);
    w(k) = (ddy_dxx * vx) / (1 + dy_dx^2); 
end

%%%%%%%%%%%%%%%%%%%%%%%%% BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k=1:N 
    % Integración de la orientación
    phi(k+1) = phi(k) + w(k)*ts;
    
    % Modelo cinemático del Puzzlebot
    xp1 = u(k)*cos(phi(k+1)); 
    yp1 = u(k)*sin(phi(k+1));
    
    x1(k+1) = x1(k) + xp1*ts; 
    y1(k+1) = y1(k) + yp1*ts; 
    
    hx(k+1) = x1(k+1); 
    hy(k+1) = y1(k+1);
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACION 3D %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
scene = figure;
set(scene,'Color','white');
sizeScreen = get(0,'ScreenSize');
set(scene,'position',sizeScreen);
axis equal; grid on; box on;
xlabel('x(m)'); ylabel('y(m)');
view([0 90]); 
axis([-0.5 6 -3 3]);

scale = 0.3; 

MobileRobot_5; 
H1 = MobilePlot_4(x1(1),y1(1),phi(1),scale); hold on;
H2 = plot(hx(1),hy(1),'r','lineWidth',2);

% Animación 
for k=1:20:N 
    delete(H1);    
    delete(H2);
    
    H1 = MobilePlot_4(x1(k),y1(k),phi(k),scale);
    H2 = plot(hx(1:k),hy(1:k),'r','lineWidth',2);
    
    drawnow;
end