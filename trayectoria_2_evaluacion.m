clear
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tf = 5;              % Tiempo para completar la vuelta
ts = 0.01;           
t = 0: ts: tf;       
N = length(t);       

%%%%%%%%%%%%%%%%%%%%%%%%% PARÁMETROS DEL CÍRCULO %%%%%%%%%%%%%%%%%%%%%%%%%%
R = 4;               
omega_ref = (2 * pi) / tf; 

%%%%%%%%%%%%%%%%%%%%%%%% CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%
R = 4;
x1(1) = -4;              
y1(1) = 0;

phi(1) = -pi/2;    

%%%%%%%%%%%%%%%%%%%%%%%%%%%% PUNTO DE CONTROL %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
hx = zeros(1,N+1);  
hy = zeros(1,N+1);  
hx(1) = x1(1);
hy(1) = y1(1);

%%%%%%%%%%%%%%%%%%%%%% VELOCIDADES DE REFERENCIA %%%%%%%%%%%%%%%%%%%%%%%%%%
% En un círculo perfecto, u y w son constantes
u = (omega_ref * R) * ones(1,N); % Velocidad lineal (v = omega * R)
w = omega_ref * ones(1,N);       % Velocidad angular constante

%%%%%%%%%%%%%%%%%%%%%%%%% BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
for k=1:N 
    % Integración de la orientación
    phi(k+1) = phi(k) + w(k)*ts;
    
    % Modelo cinemático
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
axis equal; grid on; box on;
xlabel('x(m)'); ylabel('y(m)');
view([0 90]); % Vista superior
axis([-6 6 -6 6]);

scale = 0.5; 
MobileRobot_5; 
H1 = MobilePlot_4(x1(1),y1(1),phi(1),scale); hold on;
H2 = plot(hx(1),hy(1),'r','lineWidth',2);

for k=1:5:N 
    delete(H1);    
    delete(H2);
    
    H1 = MobilePlot_4(x1(k),y1(k),phi(k),scale);
    H2 = plot(hx(1:k),hy(1:k),'r','lineWidth',2);
    
    drawnow;
end